/*
 * Project: High-Precision BNO085 IMU Gateway for ESP32-S3
 * Hardware: ESP32-S3-N16R8 + BNO085 (9DOF)
 * Wiring: BNO SDA -> GPIO 8, BNO SCL -> GPIO 9 (I2C)
 *
 * Boot flow:
 *   1. Load NVS config
 *   2. If WiFi not configured → captive-portal AP with settings page
 *   3. If configured → connect WiFi, start sensor + comms tasks
 *
 * LED Status Codes (built-in RGB NeoPixel):
 *   Flashing BLUE    = Captive portal active
 *   Flashing YELLOW  = BNO085 IMU initializing
 *   Flashing RED     = BNO085 not detected (error)
 *   Pulsing GREEN    = All systems running normally
 */

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Preferences.h>

// ---- Hardware Pins ----
#define SDA_PIN 8
#define SCL_PIN 9
#define RGB_LED_PIN 48
#define RGB_BRIGHTNESS 10

// ---- LED Helpers ----
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(RGB_LED_PIN, r, g, b);
}
void flashLED(uint8_t r, uint8_t g, uint8_t b, int times, int ms) {
  for (int i = 0; i < times; i++) {
    setLED(r, g, b); delay(ms);
    setLED(0, 0, 0); delay(ms);
  }
}

// ---- Global Objects ----
BNO080 myIMU;
WiFiUDP udp;
AsyncWebServer server(80);
DNSServer dnsServer;
AsyncEventSource events("/events");
SemaphoreHandle_t radioMutex;
Preferences prefs;

// BLE
BLECharacteristic *pCharacteristic;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ---- NVS-backed Config ----
struct Config {
  bool     configured = false;
  String   wifiSSID;
  String   wifiPass;
  char     mdnsName[32] = "compass";
  uint16_t signalkPort  = 10110;
  float    compassOffset = 0.0f;   // degrees, added to raw heading
} cfg;

// ---- Heading Stability (runtime-adjustable) ----
#define DRIFT_HISTORY_MAX 120  // max seconds for drift window

static float    filterAlpha       = 0.005f;  // Mag correction rate (0=gyro-only, 1=mag-only)
static int      driftWindowSec    = 60;      // Drift rate measured over this window
static int      magTrustThreshold = 2;       // Min magAcc (0-3) to trust mag heading
static bool     useCompFilter     = true;    // Enable complementary filter (false=game-vector only)
static bool     useMagHeading     = false;   // Use raw mag heading instead of filter

static bool     calibrationActive = false;   // true while user-initiated cal running
static float    gameHeading   = 0.0f;        // Gyro-stabilized heading (no mag)
static float    magHeading    = 0.0f;        // Raw magnetic heading
static float    rotVecHeading = 0.0f;        // BNO085 rotation vector heading (mag-fused)
static float    filteredHeading = 0.0f;      // Blended output
static float    driftRate     = 0.0f;        // degrees/min drift
static float    headingHistory[DRIFT_HISTORY_MAX]; // 1Hz samples for drift calc
static int      historyIdx    = 0;
static bool     historyFull   = false;
static uint32_t lastHistorySample = 0;

void loadConfig() {
  prefs.begin("config", false);
  cfg.configured    = prefs.getBool("configured", false);
  cfg.wifiSSID      = prefs.getString("ssid", "");
  cfg.wifiPass      = prefs.getString("pass", "");
  cfg.signalkPort   = prefs.getUShort("skport", 10110);
  cfg.compassOffset = prefs.getFloat("compoff", 0.0f);
  String name       = prefs.getString("mdns", "compass");
  name.toCharArray(cfg.mdnsName, sizeof(cfg.mdnsName));
  // Compass filter settings
  filterAlpha       = prefs.getFloat("fAlpha", 0.005f);
  driftWindowSec    = prefs.getInt("driftWin", 60);
  magTrustThreshold = prefs.getInt("magTrust", 2);
  useCompFilter     = prefs.getBool("useComp", true);
  useMagHeading     = prefs.getBool("useMag", false);
  if (driftWindowSec < 5) driftWindowSec = 5;
  if (driftWindowSec > DRIFT_HISTORY_MAX) driftWindowSec = DRIFT_HISTORY_MAX;
}
void saveConfig() {
  prefs.putBool("configured", cfg.configured);
  prefs.putString("ssid", cfg.wifiSSID);
  prefs.putString("pass", cfg.wifiPass);
  prefs.putUShort("skport", cfg.signalkPort);
  prefs.putFloat("compoff", cfg.compassOffset);
  prefs.putString("mdns", String(cfg.mdnsName));
  // Compass filter settings
  prefs.putFloat("fAlpha", filterAlpha);
  prefs.putInt("driftWin", driftWindowSec);
  prefs.putInt("magTrust", magTrustThreshold);
  prefs.putBool("useComp", useCompFilter);
  prefs.putBool("useMag", useMagHeading);
}
void resetConfig() {
  prefs.clear();
  cfg = Config{};
  Serial.println("[CFG] Reset — rebooting into setup mode");
}

// ---- Sensor Data ----
struct IMUData {
  float h, p, r;
  float rot;
  float gx, gy, gz;
  float lax, lay, laz;
  byte quatAcc, magAcc;
  float magH;        // raw magnetic heading for diagnostics
  float gameH;       // game rotation heading
  float driftDegMin; // heading drift in deg/min
} currentData;

// =====================================================================
// Captive Portal — Settings Page
// =====================================================================
static const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>IMU Gateway Setup</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:440px;margin:40px auto;
       padding:0 16px;background:#1a1a2e;color:#e0e0e0}
  h1{color:#0fbcf9;text-align:center}
  label{display:block;margin-top:12px;font-weight:600}
  input,select{width:100%%;padding:8px;margin-top:4px;border:1px solid #444;
               border-radius:4px;background:#16213e;color:#e0e0e0;box-sizing:border-box}
  .row{display:flex;gap:6px;margin-top:4px}
  .row select{flex:1} .row button{width:auto;margin:0;padding:8px 14px;font-size:14px}
  button{margin-top:20px;width:100%%;padding:12px;background:#0fbcf9;color:#1a1a2e;
         border:none;border-radius:6px;font-size:16px;font-weight:700;cursor:pointer}
  button:hover{background:#06a0d6}
  .msg{margin-top:12px;padding:10px;border-radius:6px;text-align:center;font-weight:600}
  .msg.ok{background:#2ecc71;color:#1a1a2e} .msg.err{background:#e74c3c;color:#fff}
  .info{color:#888;font-size:0.85em;margin-top:2px}
</style>
</head><body>
<h1>&#9881; IMU Gateway Setup</h1>
<div id="msg"></div>
<form id="sf">
  <label>WiFi Network</label>
  <div class="row">
    <select id="scanList" onchange="document.getElementById('ssid').value=this.value">
      <option value="">— click Scan —</option>
    </select>
    <button type="button" id="scanBtn" onclick="doScan()">Scan</button>
  </div>
  <input name="ssid" id="ssid" required placeholder="Network name">

  <label>WiFi Password</label>
  <input name="pass" type="password" placeholder="Password">

  <label>Device Name (mDNS)</label>
  <input name="mdns" value="compass" pattern="[a-z0-9\-]+" maxlength="24"
         title="Lowercase letters, numbers, and hyphens only">
  <div class="info">Accessible as <em>name</em>.local after connecting</div>

  <label>Signal K UDP Port</label>
  <input name="skport" type="number" value="10110" min="1024" max="65535">
  <div class="info">UDP broadcast port for Signal K delta JSON</div>

  <label>Compass Offset (&deg;)</label>
  <input name="compoff" type="number" value="0" min="-180" max="180" step="0.1">
  <div class="info">Added to raw heading (-180 to +180). Use to align with known reference.</div>

  <button type="submit">Save &amp; Connect</button>
</form>
<script>
document.getElementById('sf').addEventListener('submit',function(e){
  e.preventDefault();
  var fd=new FormData(this);
  fetch('/save',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{
    var m=document.getElementById('msg');
    if(d.ok){m.className='msg ok';m.textContent='Saved! Connecting to WiFi...';
      setTimeout(()=>{m.textContent='Rebooting... reconnect to your WiFi network.'},3000);
    }else{m.className='msg err';m.textContent=d.error||'Save failed';}
  }).catch(()=>{
    var m=document.getElementById('msg');m.className='msg err';m.textContent='Connection lost';
  });
});
function doScan(){
  var btn=document.getElementById('scanBtn');
  var sel=document.getElementById('scanList');
  btn.disabled=true;btn.textContent='...';
  sel.innerHTML='<option value="">Scanning...</option>';
  function poll(){
    fetch('/scan').then(r=>{
      if(r.status===202){setTimeout(poll,1500);return null;}
      return r.json();
    }).then(nets=>{
      if(!nets)return;
      if(nets.length===0){sel.innerHTML='<option value="">No networks found</option>';btn.disabled=false;btn.textContent='Scan';return;}
      sel.innerHTML='<option value="">— select —</option>';
      nets.sort((a,b)=>b.rssi-a.rssi);
      var seen={};
      nets.forEach(n=>{
        if(seen[n.ssid]||!n.ssid)return;seen[n.ssid]=1;
        var o=document.createElement('option');o.value=n.ssid;
        var bars=n.rssi>-50?'\u2587\u2587\u2587\u2587':n.rssi>-65?'\u2587\u2587\u2587':n.rssi>-75?'\u2587\u2587':'\u2587';
        o.textContent=n.ssid+' '+bars+' ('+n.rssi+'dBm)';
        sel.appendChild(o);
      });
      btn.disabled=false;btn.textContent='Scan';
    }).catch(()=>{sel.innerHTML='<option value="">Scan error</option>';btn.disabled=false;btn.textContent='Scan';});
  }
  poll();
}
// Auto-scan on load
window.addEventListener('load', doScan);
</script>
</body></html>
)rawliteral";

// =====================================================================
// Dashboard Page (normal operation)
// =====================================================================
static const char DASH_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BNO085 IMU Gateway</title>
<style>
  body{background:#1a1a2e;color:#e0e0e0;text-align:center;font-family:system-ui,sans-serif;
       max-width:460px;margin:40px auto;padding:0 16px}
  h1{color:#0fbcf9;font-size:1.3em}
  .big{font-size:3em;margin:10px 0;color:#2ecc71}
  table{width:100%%;border-collapse:collapse;margin-top:12px}
  td{padding:6px 10px;border-bottom:1px solid #333;text-align:left}
  td:first-child{font-weight:600;color:#aaa;width:45%%}
  .section{margin:16px auto;padding:12px;border:1px solid #333;border-radius:8px}
  input{padding:6px;border:1px solid #444;border-radius:4px;background:#16213e;
        color:#e0e0e0;width:100px;text-align:center}
  button{padding:8px 16px;border:none;border-radius:6px;cursor:pointer;font-size:14px;
         font-weight:600;margin:4px}
  .btn-blue{background:#0fbcf9;color:#1a1a2e}
  .btn-red{background:#e74c3c;color:#fff}
  .btn-green{background:#2ecc71;color:#1a1a2e}
  .info{color:#888;font-size:0.85em}
  .sw{position:relative;display:inline-block;width:40px;height:22px;vertical-align:middle;margin-right:6px}
  .sw input{opacity:0;width:0;height:0}
  .sl{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#444;border-radius:22px;transition:.3s}
  .sl:before{content:"";position:absolute;height:16px;width:16px;left:3px;bottom:3px;background:#ccc;border-radius:50%%;transition:.3s}
  .sw input:checked+.sl{background:#2ecc71}
  .sw input:checked+.sl:before{transform:translateX(18px)}
</style>
</head><body>
<h1>&#9875; BNO085 IMU Gateway</h1>
<div class="big" id="h">---&deg;</div>
<table>
  <tr><td>Pitch</td><td><span id="p">--</span>&deg;</td></tr>
  <tr><td>Roll</td><td><span id="r">--</span>&deg;</td></tr>
  <tr><td>Rate of Turn</td><td><span id="rot">--</span>&deg;/s</td></tr>
  <tr><td>Quat Accuracy</td><td><span id="qa">0</span>/3</td></tr>
  <tr><td>Mag Accuracy</td><td><span id="ma">0</span>/3</td></tr>
  <tr><td>Mag Heading</td><td><span id="mh">--</span>&deg;</td></tr>
  <tr><td>Game Heading</td><td><span id="gh">--</span>&deg;</td></tr>
  <tr><td>Drift Rate</td><td><span id="dr">--</span>&deg;/min</td></tr>
  <tr><td>Calibration</td><td><span id="cs">Off</span></td></tr>
  <tr><td>IP Address</td><td>%IP%</td></tr>
  <tr><td>mDNS</td><td><a href="http://%MDNS%.local" style="color:#0fbcf9">%MDNS%.local</a></td></tr>
  <tr><td>Signal K Port</td><td>%SKPORT% (UDP)</td></tr>
</table>
<div class="section">
  <b>Device Name</b><br>
  <input id="name" type="text" maxlength="24" value="%MDNS%">
  <span class="info">.local</span>
  <button class="btn-blue" onclick="setName()">Set</button>
  <div id="ns" class="info"></div>
</div>
<div class="section">
  <b>Signal K UDP Port</b><br>
  <input id="port" type="number" min="1024" max="65535" value="%SKPORT%">
  <button class="btn-blue" onclick="setPort()">Set</button>
  <div id="ps" class="info"></div>
</div>
<div class="section">
  <b>Set Heading</b> <span class="info">(enter your real heading after boot)</span><br>
  <input id="realhdg" type="number" min="0" max="360" step="1" value="0">
  <span class="info">&deg;</span>
  <button class="btn-blue" onclick="setHeading()">Set Heading</button>
  <div id="hs" class="info"></div>
</div>
<div class="section">
  <b>Compass Offset</b> <span class="info">(auto-set by Set Heading)</span><br>
  <input id="compoff" type="number" min="-180" max="180" step="0.1" value="%COMPOFF%">
  <span class="info">&deg;</span>
  <button class="btn-blue" onclick="setOffset()">Set</button>
  <div id="os" class="info"></div>
</div>
<div class="section">
  <b>Compass Filter Settings</b>
  <table>
    <tr>
      <td>Comp. Filter</td>
      <td>
        <label class="sw"><input type="checkbox" id="useComp" %USECOMP% onchange="setFilter()"><span class="sl"></span></label>
        <span class="info">Blend mag + gyro</span>
      </td>
    </tr>
    <tr>
      <td>Mag-Only Mode</td>
      <td>
        <label class="sw"><input type="checkbox" id="useMag" %USEMAG% onchange="setFilter()"><span class="sl"></span></label>
        <span class="info">Use raw mag heading</span>
      </td>
    </tr>
    <tr>
      <td>Filter Alpha</td>
      <td>
        <input id="fAlpha" type="number" min="0" max="1" step="0.001" value="%FALPHA%" style="width:80px">
        <span class="info">0=gyro, 1=mag</span>
      </td>
    </tr>
    <tr>
      <td>Mag Trust</td>
      <td>
        <input id="magTrust" type="number" min="0" max="3" step="1" value="%MAGTRUST%" style="width:60px">
        <span class="info">/3 min accuracy</span>
      </td>
    </tr>
    <tr>
      <td>Drift Window</td>
      <td>
        <input id="driftWin" type="number" min="5" max="120" step="1" value="%DRIFTWIN%" style="width:60px">
        <span class="info">sec</span>
      </td>
    </tr>
  </table>
  <button class="btn-blue" onclick="setFilter()">Apply</button>
  <div id="fs" class="info"></div>
</div>
<div class="section">
  <button class="btn-blue" onclick="fetch('/startcal').then(r=>r.text()).then(t=>alert(t))">
    Start Calibration</button>
  <button class="btn-green" onclick="fetch('/savecal').then(r=>r.text()).then(t=>alert(t))">
    Save &amp; Lock Calibration</button>
  <button class="btn-red" onclick="if(confirm('Reset WiFi and settings?'))fetch('/reset')">
    Factory Reset</button>
</div>
<script>
var s=new EventSource('/events');
s.onmessage=function(e){
  var d=JSON.parse(e.data);
  document.getElementById('h').innerHTML=d.h.toFixed(1)+'&deg;';
  document.getElementById('p').innerHTML=d.p.toFixed(1);
  document.getElementById('r').innerHTML=d.r.toFixed(1);
  document.getElementById('rot').innerHTML=d.rot.toFixed(2);
  document.getElementById('qa').innerHTML=d.qacc;
  document.getElementById('ma').innerHTML=d.macc;
  if(d.mh!==undefined) document.getElementById('mh').innerHTML=d.mh.toFixed(1);
  if(d.gh!==undefined) document.getElementById('gh').innerHTML=d.gh.toFixed(1);
  if(d.dr!==undefined){var dr=document.getElementById('dr');dr.innerHTML=d.dr.toFixed(2);dr.style.color=Math.abs(d.dr)>0.5?'#e74c3c':'#2ecc71';}
  if(d.cs!==undefined) document.getElementById('cs').innerHTML=d.cs?'<span style="color:#e74c3c">Active</span>':'Off';
};
function setName(){
  fetch('/setname?name='+document.getElementById('name').value)
    .then(r=>r.text()).then(t=>{document.getElementById('ns').innerHTML=t});
}
function setPort(){
  fetch('/setport?port='+document.getElementById('port').value)
    .then(r=>r.text()).then(t=>{document.getElementById('ps').innerHTML=t;
    setTimeout(()=>document.getElementById('ps').innerHTML='',3000)});
}
function setHeading(){
  fetch('/setheading?hdg='+document.getElementById('realhdg').value)
    .then(r=>r.json()).then(d=>{
      document.getElementById('hs').innerHTML=d.msg;
      document.getElementById('compoff').value=d.offset.toFixed(1);
      setTimeout(()=>document.getElementById('hs').innerHTML='',5000);
    });
}
function setOffset(){
  fetch('/setoffset?val='+document.getElementById('compoff').value)
    .then(r=>r.text()).then(t=>{document.getElementById('os').innerHTML=t;
    setTimeout(()=>document.getElementById('os').innerHTML='',3000)});
}
function setFilter(){
  var p='useComp='+(document.getElementById('useComp').checked?1:0)
    +'&useMag='+(document.getElementById('useMag').checked?1:0)
    +'&alpha='+document.getElementById('fAlpha').value
    +'&magTrust='+document.getElementById('magTrust').value
    +'&driftWin='+document.getElementById('driftWin').value;
  fetch('/setfilter?'+p).then(r=>r.text()).then(t=>{document.getElementById('fs').innerHTML=t;
    setTimeout(()=>document.getElementById('fs').innerHTML='',3000)});
}
</script>
</body></html>
)rawliteral";

static String dashProcessor(const String& var) {
  if (var == "IP")       return WiFi.localIP().toString();
  if (var == "MDNS")     return String(cfg.mdnsName);
  if (var == "SKPORT")   return String(cfg.signalkPort);
  if (var == "COMPOFF")  return String(cfg.compassOffset, 1);
  if (var == "FALPHA")   return String(filterAlpha, 3);
  if (var == "MAGTRUST") return String(magTrustThreshold);
  if (var == "DRIFTWIN") return String(driftWindowSec);
  if (var == "USECOMP")  return useCompFilter ? "checked" : "";
  if (var == "USEMAG")   return useMagHeading ? "checked" : "";
  return String();
}

// =====================================================================
// Sensor Task — Core 1, high priority
// =====================================================================
// Wrap angle to 0-360
static float wrapHeading(float deg) {
  while (deg >= 360.0f) deg -= 360.0f;
  while (deg < 0.0f)    deg += 360.0f;
  return deg;
}

// Shortest signed angular difference (a - b), result in -180..+180
static float angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Update drift rate tracking (called ~1Hz)
static void updateDriftTracking(float heading) {
  uint32_t now = millis();
  if (now - lastHistorySample < 1000) return;
  lastHistorySample = now;

  headingHistory[historyIdx] = heading;
  historyIdx = (historyIdx + 1) % driftWindowSec;
  if (historyIdx == 0) historyFull = true;

  if (historyFull) {
    int oldest = historyIdx;
    float totalDrift = angleDiff(heading, headingHistory[oldest]);
    driftRate = totalDrift / (driftWindowSec / 60.0f); // deg/min
  }
}

// Compute yaw from quaternion components (returns degrees)
static float quatToYaw(float qi, float qj, float qk, float qr) {
  float siny_cosp = 2.0f * (qr * qk + qi * qj);
  float cosy_cosp = 1.0f - 2.0f * (qj * qj + qk * qk);
  return atan2(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// =====================================================================
// BNO085 report enable — called on boot and after any reinit
// =====================================================================
static void initIMUReports() {
  myIMU.enableGameRotationVector(25);         delay(50);  // 40 Hz
  myIMU.enableRotationVector(50);             delay(50);  // 20 Hz (drift-correction reference only)
  myIMU.enableGyro(25);                       delay(50);  // 40 Hz
  myIMU.enableLinearAccelerometer(25);        delay(50);  // 40 Hz
  // Let BNO085 dynamically calibrate magnetometer — do NOT call endCalibration()
}

void sensorTask(void * pvParameters) {
  initIMUReports();

  bool firstReading = true;
  float prevGameHeading = 0.0f;
  float rotVecQuality = 0;       // quatAcc from rotation vector (0-3)
  uint32_t lastSerialPrint = 0;
  uint32_t lastGoodData = millis(); // watchdog: last time we got a valid SHTP packet
  bool calSavedThisBoot = false;   // auto-save BNO085 cal once per boot when quatAcc=3

  for (;;) {
    // BNO085 spontaneous reset (power glitch, brown-out) — re-enable reports without I2C reinit
    if (myIMU.hasReset()) {
      Serial.println("[IMU] BNO085 reset detected — re-enabling reports");
      Serial0.println("[IMU] BNO085 reset detected — re-enabling reports");
      initIMUReports();
      firstReading = true;
      lastGoodData = millis();
    }

    while (myIMU.dataAvailable()) {
      lastGoodData = millis();
      uint8_t reportID = myIMU.shtpData[5];

      if (reportID == SENSOR_REPORTID_ROTATION_VECTOR) {
        // Mag-fused heading — absolute reference (may be noisy initially)
        float qi = myIMU.getQuatI();
        float qj = myIMU.getQuatJ();
        float qk = myIMU.getQuatK();
        float qr = myIMU.getQuatReal();
        float rawYaw = quatToYaw(qi, qj, qk, qr);
        float rawDeg = rawYaw < 0 ? rawYaw + 360.0f : rawYaw;
        rotVecHeading = rawDeg;
        rotVecQuality = myIMU.getQuatAccuracy();
        currentData.quatAcc = rotVecQuality;

        // Auto-save calibration to BNO085 flash once per boot when mag is fully calibrated
        if (rotVecQuality >= 3 && !calSavedThisBoot) {
          myIMU.saveCalibration();
          calSavedThisBoot = true;
          Serial.println("[CAL] quatAcc=3 — calibration auto-saved to BNO085 flash");
          Serial0.println("[CAL] quatAcc=3 — calibration auto-saved to BNO085 flash");
        }
      }
      else if (reportID == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
        // Game vector — smooth tracking, compute delta for complementary filter
        float qi = myIMU.getQuatI();
        float qj = myIMU.getQuatJ();
        float qk = myIMU.getQuatK();
        float qr = myIMU.getQuatReal();
        float rawYaw = quatToYaw(qi, qj, qk, qr);
        float rawDeg = rawYaw < 0 ? rawYaw + 360.0f : rawYaw;
        gameHeading = rawDeg;

        // Pitch/roll from game rotation quaternion
        currentData.p = myIMU.getPitch() * 180.0f / PI;
        currentData.r = myIMU.getRoll()  * 180.0f / PI;

        if (firstReading) {
          filteredHeading = gameHeading;
          prevGameHeading = gameHeading;
          firstReading = false;
        } else {
          // Track rotation via game vector delta (smooth, no mag noise)
          float gameDelta = angleDiff(gameHeading, prevGameHeading);
          prevGameHeading = gameHeading;
          filteredHeading = wrapHeading(filteredHeading + gameDelta);

          // Slowly correct toward rotation vector heading when mag quality is good
          // quatAcc: 0=unreliable, 1=low, 2=medium, 3=high
          if (rotVecQuality >= 2) {
            // Rotation vector heading with same offset as our filtered heading
            float rotRef = wrapHeading(rotVecHeading + cfg.compassOffset);
            float filtered = wrapHeading(filteredHeading + cfg.compassOffset);
            float error = angleDiff(rotRef, filtered);
            // Gentle correction — 0.002 = ~0.2% per 100ms, prevents sudden jumps
            filteredHeading = wrapHeading(filteredHeading + error * 0.002f);
          }
        }

        // Final heading = filtered heading + user offset
        float finalHeading = wrapHeading(filteredHeading + cfg.compassOffset);
        currentData.h     = finalHeading;
        currentData.gameH = gameHeading;

        // Drift tracking (1Hz sampling)
        updateDriftTracking(finalHeading);
        currentData.driftDegMin = driftRate;

        // Serial debug output ~1Hz
        uint32_t now = millis();
        if (now - lastSerialPrint >= 1000) {
          lastSerialPrint = now;
          char dbg[300];
          snprintf(dbg, sizeof(dbg),
            "[HDG] final=%.1f  game=%.1f  rotVec=%.1f  filtered=%.1f  offset=%.1f  "
            "quatAcc=%d  drift=%.2f°/min  pitch=%.1f  roll=%.1f",
            finalHeading, gameHeading, rotVecHeading, filteredHeading,
            cfg.compassOffset, (int)rotVecQuality, driftRate,
            currentData.p, currentData.r);
          Serial.println(dbg);
          Serial0.println(dbg);
        }
      }
      else if (reportID == SENSOR_REPORTID_GYROSCOPE) {
        currentData.gx  = myIMU.getGyroX() * 180.0f / PI;
        currentData.gy  = myIMU.getGyroY() * 180.0f / PI;
        currentData.gz  = myIMU.getGyroZ() * 180.0f / PI;
        currentData.rot = currentData.gz;
      }
      else if (reportID == SENSOR_REPORTID_LINEAR_ACCELERATION) {
        currentData.lax = myIMU.getLinAccelX();
        currentData.lay = myIMU.getLinAccelY();
        currentData.laz = myIMU.getLinAccelZ();
      }
    }

    // Stale-data watchdog: if no valid SHTP packet in 500 ms the I2C bus or BNO085 is stuck.
    // Wire.setTimeOut(100) means requestFrom() has already given up after 100 ms —
    // so reaching here means the library returned false for several consecutive poll cycles.
    if (millis() - lastGoodData > 500) {
      Serial.println("[IMU] No data for 500 ms — reinitializing I2C + BNO085");
      Serial0.println("[IMU] No data for 500 ms — reinitializing I2C + BNO085");
      Wire.end();
      vTaskDelay(pdMS_TO_TICKS(20));
      Wire.begin(SDA_PIN, SCL_PIN);
      Wire.setClock(400000);
      Wire.setTimeOut(100);
      if (myIMU.begin()) {
        initIMUReports();
        firstReading = true;
        Serial.println("[IMU] Reinit OK");
        Serial0.println("[IMU] Reinit OK");
      } else {
        Serial.println("[IMU] Reinit FAILED — will retry in 1 s");
        Serial0.println("[IMU] Reinit FAILED — will retry in 1 s");
        vTaskDelay(pdMS_TO_TICKS(1000));
      }
      lastGoodData = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// =====================================================================
// Comms Task — Core 0
// =====================================================================
void sendSignalKDelta() {
  float headRad  = currentData.h   * DEG_TO_RAD;
  float pitchRad = currentData.p   * DEG_TO_RAD;
  float rollRad  = currentData.r   * DEG_TO_RAD;
  float yawRad   = headRad;
  float rotRad   = currentData.rot * DEG_TO_RAD;
  float magAccNorm  = currentData.magAcc  / 3.0f;
  float quatAccNorm = currentData.quatAcc / 3.0f;

  char delta[600];
  snprintf(delta, sizeof(delta),
    "{\"updates\":[{\"source\":{\"label\":\"BNO085\",\"type\":\"IMU\"},"
    "\"values\":["
    "{\"path\":\"navigation.headingMagnetic\",\"value\":%.6f},"
    "{\"path\":\"navigation.rateOfTurn\",\"value\":%.6f},"
    "{\"path\":\"navigation.attitude\",\"value\":{\"roll\":%.6f,\"pitch\":%.6f,\"yaw\":%.6f}},"
    "{\"path\":\"sensors.imu.compassCalibration\",\"value\":%.2f},"
    "{\"path\":\"sensors.imu.fusionCalibration\",\"value\":%.2f}"
    "]}]}",
    headRad, rotRad, rollRad, pitchRad, yawRad, magAccNorm, quatAccNorm);

  udp.beginPacket("255.255.255.255", cfg.signalkPort);
  udp.print(delta);
  udp.endPacket();
}

void commsTask(void * pvParameters) {
  for (;;) {
    bool wifiOk = (WiFi.status() == WL_CONNECTED);

    if (xSemaphoreTake(radioMutex, portMAX_DELAY)) {
      String payload = "{\"h\":" + String(currentData.h, 1) +
                       ",\"p\":" + String(currentData.p, 1) +
                       ",\"r\":" + String(currentData.r, 1) +
                       ",\"rot\":" + String(currentData.rot, 2) +
                       ",\"gx\":" + String(currentData.gx, 2) +
                       ",\"gy\":" + String(currentData.gy, 2) +
                       ",\"gz\":" + String(currentData.gz, 2) +
                       ",\"lax\":" + String(currentData.lax, 3) +
                       ",\"lay\":" + String(currentData.lay, 3) +
                       ",\"laz\":" + String(currentData.laz, 3) +
                       ",\"qacc\":" + String(currentData.quatAcc) +
                       ",\"macc\":" + String(currentData.magAcc) +
                       ",\"mh\":" + String(currentData.magH, 1) +
                       ",\"gh\":" + String(currentData.gameH, 1) +
                       ",\"dr\":" + String(currentData.driftDegMin, 2) +
                       ",\"cs\":" + String(calibrationActive ? 1 : 0) + "}";

      if (wifiOk) {
        udp.beginPacket("255.255.255.255", 4210);
        udp.print(payload);
        udp.endPacket();
        events.send(payload.c_str(), "message", millis());
        sendSignalKDelta();
      }

      pCharacteristic->setValue(payload.c_str());
      pCharacteristic->notify();
      xSemaphoreGive(radioMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(25));  // 40 Hz
  }
}

// =====================================================================
// LED Pulse Task
// =====================================================================
void ledTask(void * pvParameters) {
  for (;;) {
    for (int i = 0; i <= RGB_BRIGHTNESS; i++) {
      setLED(0, i, 0);
      vTaskDelay(pdMS_TO_TICKS(3000 / (RGB_BRIGHTNESS + 1)));
    }
    for (int i = RGB_BRIGHTNESS; i >= 0; i--) {
      setLED(0, i, 0);
      vTaskDelay(pdMS_TO_TICKS(5000 / (RGB_BRIGHTNESS + 1)));
    }
  }
}

// =====================================================================
// Setup Mode — Captive Portal AP
// =====================================================================
void runSetupMode() {
  Serial.println("[SETUP] Starting captive portal AP...");

  // Generate AP name with MAC suffix
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char apName[32];
  snprintf(apName, sizeof(apName), "IMU-Gateway-%02X%02X", mac[4], mac[5]);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName);
  delay(500);
  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("[SETUP] AP: %s  IP: %s\n", apName, apIP.toString().c_str());

  // DNS: resolve ALL domains to our IP (captive portal redirect)
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP);

  // Captive portal detection endpoints (Android/iOS/Windows)
  auto redirectHandler = [&apIP](AsyncWebServerRequest *req) {
    req->redirect("http://" + apIP.toString() + "/");
  };
  server.on("/generate_204", HTTP_GET, redirectHandler);      // Android
  server.on("/gen_204", HTTP_GET, redirectHandler);            // Android
  server.on("/hotspot-detect.html", HTTP_GET, redirectHandler);// Apple
  server.on("/connecttest.txt", HTTP_GET, redirectHandler);    // Windows
  server.on("/redirect", HTTP_GET, redirectHandler);           // Windows
  server.on("/ncsi.txt", HTTP_GET, redirectHandler);           // Windows
  server.on("/fwlink", HTTP_GET, redirectHandler);             // Windows

  // Settings page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", SETUP_HTML);
  });

  // WiFi scan
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *req) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED) {
      WiFi.scanNetworks(true, false, false, 300, 0, NULL, NULL);
      req->send(202, "application/json", "[]");
      return;
    }
    if (n == WIFI_SCAN_RUNNING) {
      req->send(202, "application/json", "[]");
      return;
    }
    String json = "[";
    for (int i = 0; i < n; i++) {
      if (i > 0) json += ",";
      String ssid = WiFi.SSID(i);
      ssid.replace("\"", "\\\"");
      json += "{\"ssid\":\"" + ssid + "\",\"rssi\":" + String(WiFi.RSSI(i)) + "}";
    }
    json += "]";
    WiFi.scanDelete();
    req->send(200, "application/json", json);
  });

  // Save config
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *req) {
    String ssid;
    if (req->hasParam("ssid", true))
      ssid = req->getParam("ssid", true)->value();
    ssid.trim();
    if (ssid.length() == 0) {
      req->send(200, "application/json", "{\"ok\":false,\"error\":\"SSID required\"}");
      return;
    }
    cfg.wifiSSID = ssid;
    if (req->hasParam("pass", true))
      cfg.wifiPass = req->getParam("pass", true)->value();
    if (req->hasParam("mdns", true)) {
      String name = req->getParam("mdns", true)->value();
      name.trim(); name.toLowerCase();
      if (name.length() > 0 && name.length() <= 24)
        name.toCharArray(cfg.mdnsName, sizeof(cfg.mdnsName));
    }
    if (req->hasParam("skport", true)) {
      uint16_t p = (uint16_t)req->getParam("skport", true)->value().toInt();
      if (p >= 1024) cfg.signalkPort = p;
    }
    if (req->hasParam("compoff", true)) {
      float off = req->getParam("compoff", true)->value().toFloat();
      if (off >= -180.0f && off <= 180.0f) cfg.compassOffset = off;
    }
    cfg.configured = true;
    saveConfig();
    Serial.printf("[SETUP] Saved: ssid='%s' mdns='%s' skport=%u compoff=%.1f\n",
                  cfg.wifiSSID.c_str(), cfg.mdnsName, cfg.signalkPort, cfg.compassOffset);
    req->send(200, "application/json", "{\"ok\":true}");
    delay(2000);
    ESP.restart();
  });

  // Catch-all: redirect unknown paths to setup page
  server.onNotFound([&apIP](AsyncWebServerRequest *req) {
    req->redirect("http://" + apIP.toString() + "/");
  });

  server.begin();
  Serial.println("[SETUP] Captive portal ready — connect to AP and configure WiFi");

  // Blink blue while waiting
  for (;;) {
    dnsServer.processNextRequest();
    flashLED(0, 0, RGB_BRIGHTNESS, 1, 300);
    delay(400);
  }
}

// =====================================================================
// Bridge Mode — Normal Operation
// =====================================================================
void startBridgeMode() {
  // Connect WiFi
  Serial.printf("[WIFI] Connecting to '%s'...\n", cfg.wifiSSID.c_str());
  setLED(0, 0, RGB_BRIGHTNESS);  // Blue
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 40) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WIFI] Connected — IP %s\n", WiFi.localIP().toString().c_str());
    if (MDNS.begin(cfg.mdnsName)) {
      MDNS.addService("http", "tcp", 80);
      MDNS.addServiceTxt("http", "tcp", "swname", "compass-gateway");
      MDNS.addServiceTxt("http", "tcp", "txtvers", "1");
      MDNS.addServiceTxt("http", "tcp", "path", "/");
      Serial.printf("[mDNS] http://%s.local\n", cfg.mdnsName);
    }
    flashLED(0, RGB_BRIGHTNESS, 0, 3, 150);
  } else {
    Serial.println("\n[WIFI] Connection failed — continuing with BLE only");
    flashLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0, 2, 200);
  }

  // Initialize BNO085
  Serial.println("[IMU] Initializing BNO085...");
  setLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0);  // Yellow
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(100); // 100 ms hardware I2C timeout — prevents indefinite blocking if BNO085 holds SDA low
  if (!myIMU.begin()) {
    Serial.println("[IMU] BNO085 not detected! Check SDA=GPIO8, SCL=GPIO9");
    while (1) { flashLED(RGB_BRIGHTNESS, 0, 0, 1, 500); }
  }
  Serial.println("[IMU] BNO085 detected!");
  flashLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS/2, 0, 2, 150);

  // Initialize BLE
  Serial.println("[BLE] Starting...");
  setLED(0, RGB_BRIGHTNESS, RGB_BRIGHTNESS);  // Cyan
  BLEDevice::init("S3_IMU_GATEWAY");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  Serial.println("[BLE] Advertising as S3_IMU_GATEWAY");

  // Web dashboard
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", DASH_HTML, dashProcessor);
  });
  server.on("/setname", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("name")) {
      req->send(400, "text/plain", "Missing name"); return;
    }
    String newName = req->getParam("name")->value();
    newName.trim(); newName.toLowerCase();
    bool valid = newName.length() >= 1 && newName.length() <= 24;
    for (unsigned int i = 0; valid && i < newName.length(); i++) {
      char c = newName.charAt(i);
      if (!isalnum(c) && c != '-') valid = false;
    }
    if (!valid) {
      req->send(400, "text/plain", "Invalid (a-z, 0-9, hyphens, 1-24 chars)"); return;
    }
    newName.toCharArray(cfg.mdnsName, sizeof(cfg.mdnsName));
    saveConfig();
    req->send(200, "text/plain", "Set to " + newName + ".local — rebooting...");
    delay(1000); ESP.restart();
  });
  server.on("/setport", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("port")) {
      req->send(400, "text/plain", "Missing port"); return;
    }
    uint16_t p = req->getParam("port")->value().toInt();
    if (p < 1024 || p > 65535) {
      req->send(400, "text/plain", "Invalid (1024-65535)"); return;
    }
    cfg.signalkPort = p;
    saveConfig();
    req->send(200, "text/plain", "Port set to " + String(p) + " (saved)");
  });
  server.on("/setheading", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("hdg")) {
      req->send(400, "text/plain", "Missing heading"); return;
    }
    float realHdg = req->getParam("hdg")->value().toFloat();
    if (realHdg < 0.0f || realHdg > 360.0f) {
      req->send(400, "text/plain", "Invalid (0-360)"); return;
    }
    float newOffset = realHdg - filteredHeading;
    if (newOffset > 180.0f) newOffset -= 360.0f;
    if (newOffset < -180.0f) newOffset += 360.0f;
    cfg.compassOffset = newOffset;
    saveConfig();
    req->send(200, "application/json", "{\"msg\":\"Heading set to " + String(realHdg, 0) + "\u00B0 (offset=" + String(newOffset, 1) + "\u00B0)\",\"offset\":" + String(newOffset, 1) + "}");
  });
  server.on("/setoffset", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (!req->hasParam("val")) {
      req->send(400, "text/plain", "Missing value"); return;
    }
    float off = req->getParam("val")->value().toFloat();
    if (off < -180.0f || off > 180.0f) {
      req->send(400, "text/plain", "Invalid (-180 to 180)"); return;
    }
    cfg.compassOffset = off;
    saveConfig();
    req->send(200, "text/plain", "Offset set to " + String(off, 1) + "\u00B0 (saved)");
  });
  server.on("/setfilter", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (req->hasParam("useComp"))
      useCompFilter = req->getParam("useComp")->value().toInt() != 0;
    if (req->hasParam("useMag"))
      useMagHeading = req->getParam("useMag")->value().toInt() != 0;
    if (req->hasParam("alpha")) {
      float a = req->getParam("alpha")->value().toFloat();
      if (a >= 0.0f && a <= 1.0f) filterAlpha = a;
    }
    if (req->hasParam("magTrust")) {
      int t = req->getParam("magTrust")->value().toInt();
      if (t >= 0 && t <= 3) magTrustThreshold = t;
    }
    if (req->hasParam("driftWin")) {
      int w = req->getParam("driftWin")->value().toInt();
      if (w >= 5 && w <= DRIFT_HISTORY_MAX) {
        driftWindowSec = w;
        historyIdx = 0; historyFull = false;  // reset drift tracking
      }
    }
    saveConfig();
    req->send(200, "text/plain", "Filter updated (saved)");
  });
  server.on("/startcal", HTTP_GET, [](AsyncWebServerRequest *req) {
    calibrationActive = true;
    myIMU.calibrateAll();
    req->send(200, "text/plain", "Calibration started — rotate the sensor slowly in all directions.\nPress 'Save & Lock' when Mag Accuracy reaches 2 or 3.");
  });
  server.on("/savecal", HTTP_GET, [](AsyncWebServerRequest *req) {
    myIMU.saveCalibration();
    delay(200);
    calibrationActive = false;
    // Do NOT call endCalibration() — it disables dynamic mag recalibration
    req->send(200, "text/plain", "Calibration saved to BNO085 flash.\nDynamic calibration remains active.");
  });
  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/plain", "Resetting...");
    resetConfig();
    delay(1000);
    ESP.restart();
  });
  server.addHandler(&events);
  server.begin();

  radioMutex = xSemaphoreCreateMutex();
  setLED(0, RGB_BRIGHTNESS, 0);  // Green = all go
  Serial.println("[BOOT] All systems running!");

  xTaskCreatePinnedToCore(sensorTask, "Sensor", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(commsTask, "Comms", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ledTask, "LED", 2048, NULL, 0, NULL, 0);
}

// =====================================================================
// Arduino setup() / loop()
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial0.begin(115200);  // CH340 UART for serial monitor
  delay(2000);
  Serial.println("\n=== BNO085 IMU Gateway Boot ===");

  setLED(RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
  delay(300);
  setLED(0, 0, 0);

  loadConfig();
  Serial.printf("[CFG] configured=%d  ssid='%s'  mdns='%s'  skport=%u  compoff=%.1f\n",
                cfg.configured, cfg.wifiSSID.c_str(), cfg.mdnsName, cfg.signalkPort, cfg.compassOffset);

  if (!cfg.configured) {
    runSetupMode();   // blocks forever (reboots after save)
    return;
  }

  startBridgeMode();
}

void loop() {
  // Nothing — all work in FreeRTOS tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}