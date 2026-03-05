// =============================================================================
// setup_mode.cpp — Dual-channel first-boot configuration
//
//   1. WiFi AP captive portal  (ESPAsyncWebServer)
//   2. BLE GATT config service (NimBLE)
//
// After the user submits config via either channel the device saves to NVS
// and reboots into bridge mode.
// =============================================================================
#include "setup_mode.h"
#include "config.h"
#include "config_manager.h"
#include "led_status.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <NimBLEDevice.h>

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void startConfigAP();
static void startConfigBLE();
static void commitAndReboot();

static AsyncWebServer webServer(80);
static DNSServer      dnsServer;
static volatile bool  commitRequested = false;

// Staging area — written by either web form or BLE, then committed together
static String  stgSSID;
static String  stgPass;
static String  stgMode     = "wifi";
static String  stgPort     = String(DEFAULT_TCP_PORT);
static String  stgBaud     = String(MOTOR_BAUD_DEFAULT);
static String  stgHostname = DEFAULT_MDNS_NAME;
static String  stgChannel  = "0";

// =====================================================================
// HTML setup page (served from flash)
// =====================================================================
static const char SETUP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PyPilot Bridge Setup</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:460px;margin:40px auto;
       padding:0 16px;background:#1a1a2e;color:#e0e0e0}
  h1{color:#0fbcf9;text-align:center}
  label{display:block;margin-top:12px;font-weight:600}
  input,select{width:100%;padding:8px;margin-top:4px;border:1px solid #444;
               border-radius:4px;background:#16213e;color:#e0e0e0;box-sizing:border-box}
  button{margin-top:20px;width:100%;padding:12px;background:#0fbcf9;color:#1a1a2e;
         border:none;border-radius:6px;font-size:16px;font-weight:700;cursor:pointer}
  button:hover{background:#06a0d6}
  .info{margin-top:24px;padding:12px;background:#16213e;border-radius:6px;font-size:13px;line-height:1.5}
</style>
</head><body>
<h1>&#9875; PyPilot Bridge Setup</h1>
<form action="/save" method="POST">
  <label>WiFi Network</label>
  <div style="display:flex;gap:6px;margin-top:4px">
    <select id="scanList" style="flex:1" onchange="document.getElementById('ssid').value=this.value">
      <option value="">— click Scan to find networks —</option>
    </select>
    <button type="button" id="scanBtn" onclick="doScan()" style="width:auto;margin:0;padding:8px 14px;font-size:14px">Scan</button>
  </div>
  <input name="ssid" id="ssid" required placeholder="Or type network name manually">

  <label>WiFi Password</label>
  <input name="pass" type="password" placeholder="Network password">

  <label>Bridge Mode</label>
  <select name="mode">
    <option value="wifi" selected>WiFi TCP</option>
    <option value="ble">Bluetooth LE</option>
  </select>

  <label>TCP Port (WiFi mode)</label>
  <input name="port" type="number" value="20220" min="1024" max="65535">

  <label>Motor Baud Rate</label>
  <select name="baud">
    <option value="9600">9600</option>
    <option value="19200">19200</option>
    <option value="38400" selected>38400 (PyPilot default)</option>
    <option value="57600">57600</option>
    <option value="115200">115200</option>
  </select>

  <label>WiFi Channel</label>
  <select name="channel">
    <option value="0" selected>Auto</option>
    <option value="1">1</option><option value="2">2</option><option value="3">3</option>
    <option value="4">4</option><option value="5">5</option><option value="6">6</option>
    <option value="7">7</option><option value="8">8</option><option value="9">9</option>
    <option value="10">10</option><option value="11">11</option>
    <option value="12">12</option><option value="13">13</option>
  </select>

  <label>mDNS Hostname</label>
  <input name="hostname" value="pypilot-bridge" pattern="[a-z0-9\-]+"
         title="Lowercase letters, numbers, and hyphens only">

  <button type="submit">Save &amp; Reboot</button>
</form>
<div class="info">
  <strong>Zero-Config:</strong> After setup the bridge advertises as
  <code>_pypilot._tcp</code> via mDNS/DNS-SD.  PyPilot will auto-discover it.<br>
  Web UI at <code>http://&lt;hostname&gt;.local</code>
</div>
<script>
function doScan(){
  var btn=document.getElementById('scanBtn');
  var sel=document.getElementById('scanList');
  btn.disabled=true; btn.textContent='Scanning...';
  sel.innerHTML='<option value="">Scanning...</option>';
  function poll(){
    fetch('/scan').then(r=>{
      if(r.status===202){setTimeout(poll,1500);return null;}
      return r.json();
    }).then(nets=>{
      if(!nets)return;
      if(nets.length===0){sel.innerHTML='<option value="">No networks found</option>';btn.disabled=false;btn.textContent='Scan';return;}
      sel.innerHTML='<option value="">\u2014 select a network \u2014</option>';
      nets.sort((a,b)=>b.rssi-a.rssi);
      var seen={};
      nets.forEach(n=>{
        if(seen[n.ssid]||!n.ssid)return;seen[n.ssid]=1;
        var o=document.createElement('option');
        o.value=n.ssid;
        var bars=n.rssi>-50?'\u2587\u2587\u2587\u2587':n.rssi>-65?'\u2587\u2587\u2587':n.rssi>-75?'\u2587\u2587':'\u2587';
        o.textContent=n.ssid+' '+bars+' ('+n.rssi+'dBm)'+(n.enc?' \uD83D\uDD12':'');
        sel.appendChild(o);
      });
      btn.disabled=false; btn.textContent='Scan';
    }).catch(e=>{
      sel.innerHTML='<option value="">Scan failed</option>';
      btn.disabled=false; btn.textContent='Scan';
    });
  }
  poll();
}
</script>
</body></html>
)rawliteral";

// ---------------------------------------------------------------------------
// Confirmation page
// ---------------------------------------------------------------------------
static const char SAVED_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Saved</title>
<style>body{font-family:system-ui,sans-serif;text-align:center;margin-top:80px;
             background:#1a1a2e;color:#e0e0e0}h1{color:#0fbcf9}</style>
</head><body>
<h1>&#10004; Configuration Saved</h1>
<p>Rebooting into bridge mode&hellip;</p>
</body></html>
)rawliteral";

// =====================================================================
// WiFi AP + Web Server
// =====================================================================
static void startConfigAP() {
    // Build AP name with MAC suffix
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    char apName[32];
    snprintf(apName, sizeof(apName), "%s%02X%02X",
             AP_NAME_PREFIX, mac[4], mac[5]);

    WiFi.mode(WIFI_AP_STA);   // AP+STA so we can scan while serving
    WiFi.softAP(apName);
    delay(200);                               // let DHCP stabilise
    Serial.printf("[SETUP] AP '%s' started — IP %s\n",
                  apName, WiFi.softAPIP().toString().c_str());

    // DNS server — resolve ALL domains to our IP (captive portal trigger)
    dnsServer.start(53, "*", WiFi.softAPIP());
    Serial.println("[SETUP] DNS captive portal started");

    // mDNS on the AP interface so pypilot-bridge.local → 192.168.4.1
    MDNS.begin(DEFAULT_MDNS_NAME);
    MDNS.addService(HTTP_SERVICE_TYPE, "tcp", 80);

    // --- WiFi scan endpoint -------------------------------------------------
    webServer.on("/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_FAILED) {
            WiFi.scanNetworks(true);  // async scan
            req->send(202, "application/json", "[]");
            return;
        }
        if (n == WIFI_SCAN_RUNNING) {
            req->send(202, "application/json", "[]");
            return;
        }
        String json = "[";
        // Deduplicate and sort by RSSI (strongest first)
        for (int i = 0; i < n; i++) {
            if (i > 0) json += ",";
            String ssid = WiFi.SSID(i);
            ssid.replace("\"", "\\\"");  // escape quotes in SSID
            json += "{\"ssid\":\"" + ssid + "\",";
            json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
            json += "\"enc\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN) + "}";
        }
        json += "]";
        WiFi.scanDelete();
        req->send(200, "application/json", json);
    });

    // --- Captive portal probe handlers --------------------------------------
    // Return the setup page directly (not a redirect) so the OS captive-portal
    // browser sheet opens immediately with our config form.
    auto portalHandler = [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", SETUP_HTML);
    };
    // Android
    webServer.on("/generate_204", HTTP_GET, portalHandler);
    webServer.on("/gen_204", HTTP_GET, portalHandler);
    webServer.on("/connectivitycheck.gstatic.com", HTTP_GET, portalHandler);
    // iOS / macOS
    webServer.on("/hotspot-detect.html", HTTP_GET, portalHandler);
    webServer.on("/library/test/success.html", HTTP_GET, portalHandler);
    // Windows
    webServer.on("/connecttest.txt", HTTP_GET, portalHandler);
    webServer.on("/ncsi.txt", HTTP_GET, portalHandler);
    webServer.on("/redirect", HTTP_GET, portalHandler);
    // Firefox
    webServer.on("/canonical.html", HTTP_GET, portalHandler);
    webServer.on("/success.txt", HTTP_GET, portalHandler);

    // --- Serve setup page ---------------------------------------------------
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", SETUP_HTML);
    });

    // Redirect any other requests to "/"
    webServer.onNotFound([](AsyncWebServerRequest* req) {
        req->redirect("/");
    });

    // --- Handle form submission ---------------------------------------------
    // GET /save → redirect back to form (prevents accidental blank save)
    webServer.on("/save", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->redirect("/");
    });

    webServer.on("/save", HTTP_POST, [](AsyncWebServerRequest* req) {
        // Extract SSID and reject if blank
        String ssid;
        if (req->hasParam("ssid", true))
            ssid = req->getParam("ssid", true)->value();
        ssid.trim();
        if (ssid.length() == 0) {
            req->redirect("/?err=ssid");
            return;
        }
        stgSSID = ssid;
        if (req->hasParam("pass", true))
            stgPass     = req->getParam("pass",     true)->value();
        if (req->hasParam("mode", true))
            stgMode     = req->getParam("mode",     true)->value();
        if (req->hasParam("port", true))
            stgPort     = req->getParam("port",     true)->value();
        if (req->hasParam("baud", true))
            stgBaud     = req->getParam("baud",     true)->value();
        if (req->hasParam("hostname", true))
            stgHostname = req->getParam("hostname", true)->value();
        if (req->hasParam("channel", true))
            stgChannel  = req->getParam("channel", true)->value();

        Serial.printf("[SETUP] Saving config: ssid='%s' mode='%s'\n",
                      stgSSID.c_str(), stgMode.c_str());
        req->send(200, "text/html", SAVED_HTML);
        commitRequested = true;
    });

    webServer.begin();
    Serial.println("[SETUP] Web config server started on port 80");
}

// =====================================================================
// BLE Configuration Service
// =====================================================================

// Write callback — stores the value into the matching staging string
class CfgWriteCB : public NimBLECharacteristicCallbacks {
public:
    CfgWriteCB(String& target) : _target(target) {}
    void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo&) override {
        _target = String(pChar->getValue().c_str());
        Serial.printf("[BLE-CFG] Updated field → '%s'\n", _target.c_str());
    }
private:
    String& _target;
};

// Commit callback — writing any value triggers save + reboot
class CommitCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic*, NimBLEConnInfo&) override {
        Serial.println("[BLE-CFG] Commit requested");
        commitRequested = true;
    }
};

static void startConfigBLE() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    char name[32];
    snprintf(name, sizeof(name), "%s-%02X%02X",
             BLE_DEVICE_PREFIX, mac[4], mac[5]);

    NimBLEDevice::init(name);
    NimBLEDevice::setMTU(247);

    NimBLEServer*  pServer = NimBLEDevice::createServer();
    NimBLEService* pSvc    = pServer->createService(CFG_SERVICE_UUID);

    auto addChar = [&](const char* uuid, String& target) {
        NimBLECharacteristic* c = pSvc->createCharacteristic(
            uuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
        c->setCallbacks(new CfgWriteCB(target));
        c->setValue(target.c_str());
    };

    addChar(CFG_SSID_UUID,     stgSSID);
    addChar(CFG_PASS_UUID,     stgPass);
    addChar(CFG_MODE_UUID,     stgMode);
    addChar(CFG_PORT_UUID,     stgPort);
    addChar(CFG_BAUD_UUID,     stgBaud);
    addChar(CFG_HOSTNAME_UUID, stgHostname);

    // Commit characteristic
    NimBLECharacteristic* commitChar = pSvc->createCharacteristic(
        CFG_COMMIT_UUID, NIMBLE_PROPERTY::WRITE);
    commitChar->setCallbacks(new CommitCB());

    pSvc->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName(name);
    // Note: 128-bit UUID + name exceeds 31-byte adv packet limit on C3,
    // so we omit the UUID from advertising. Clients discover it after connect.
    pAdv->start();

    Serial.printf("[SETUP] BLE config service advertising as '%s'\n", name);
}

// =====================================================================
// Commit — save staging values to NVS and reboot
// =====================================================================
static void commitAndReboot() {
    auto& cfg       = configManager.config();
    cfg.wifiSSID    = stgSSID;
    cfg.wifiPass    = stgPass;
    cfg.mode        = (stgMode == "ble") ? BridgeMode::BLE : BridgeMode::WIFI_TCP;
    cfg.tcpPort     = (uint16_t)stgPort.toInt();
    cfg.baudRate    = (uint32_t)stgBaud.toInt();
    cfg.mdnsName    = stgHostname;
    cfg.wifiChannel = (uint8_t)stgChannel.toInt();
    cfg.configured  = true;

    configManager.save();
    Serial.println("[SETUP] Config saved — rebooting in 2 s …");
    delay(2000);
    ESP.restart();
}

// =====================================================================
// Main entry — blocks until commitRequested
// =====================================================================
void setupModeRun() {
    Serial.println("========================================");
    Serial.println("  SETUP MODE — configure via WiFi or BLE");
    Serial.println("========================================");
    ledStatusSet(LedState::SETUP);

    startConfigAP();
    startConfigBLE();

    // Spin until the user commits config from either channel
    while (!commitRequested) {
        dnsServer.processNextRequest();   // handle captive portal DNS
        delay(10);
    }

    // Tear down
    dnsServer.stop();
    NimBLEDevice::deinit(true);
    delay(200);

    commitAndReboot();  // does not return
}
