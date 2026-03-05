// =============================================================================
// main.cpp — PyPilot Wireless Serial Bridge  (ESP32-C3 HW466AB)
// =============================================================================
//
// Boot sequence:
//   1. Serial + LED init
//   2. Load NVS config
//   3. If not configured → enter Setup Mode (WiFi AP portal + BLE config)
//   4. If configured    → start bridge:
//        - UART1 serial task  (Core 1, priority 2)
//        - WiFi-TCP or BLE bridge task  (Core 0, priority 1)
//        - LED status task  (Core 0, priority 0)
//   5. BOOT button long-press (3 s) resets config → re-enters setup mode
//
// =============================================================================

#include <Arduino.h>
#include "config.h"
#include "config_manager.h"
#include "led_status.h"
#include "serial_bridge.h"
#include "bridge_wifi.h"
#include "bridge_ble.h"
#include "setup_mode.h"
#include "web_console.h"
#include "pypilot_proto.h"

// ---------------------------------------------------------------------------
// Status web page served in bridge mode (lightweight)
// ---------------------------------------------------------------------------
#include <ESPAsyncWebServer.h>
#include <WiFi.h>

static AsyncWebServer statusServer(80);

static const char STATUS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PyPilot Bridge</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:460px;margin:40px auto;
       padding:0 16px;background:#1a1a2e;color:#e0e0e0}
  h1{color:#0fbcf9;text-align:center}
  table{width:100%%;border-collapse:collapse;margin-top:16px}
  td{padding:8px 12px;border-bottom:1px solid #333}
  td:first-child{font-weight:600;color:#aaa;width:40%%}
  .ok{color:#2ecc71} .warn{color:#f39c12}
  button{margin-top:20px;width:100%%;padding:12px;background:#e74c3c;color:#fff;
         border:none;border-radius:6px;font-size:14px;cursor:pointer}
  button:hover{background:#c0392b}
</style>
</head><body>
<h1>&#9875; PyPilot Bridge</h1>
<table>
  <tr><td>Firmware</td><td>%FW%</td></tr>
  <tr><td>Mode</td><td>%MODE%</td></tr>
  <tr><td>WiFi SSID</td><td>%SSID%</td></tr>
  <tr><td>IP Address</td><td>%IP%</td></tr>
  <tr><td>mDNS</td><td><a href="http://%MDNS%.local" style="color:#0fbcf9">%MDNS%.local</a></td></tr>
  <tr><td>TCP Port</td><td>%PORT%</td></tr>
  <tr><td>Motor Baud</td><td>%BAUD%</td></tr>
  <tr><td>UART TX/RX</td><td>GPIO %TX% / GPIO %RX%</td></tr>
  <tr><td>Uptime</td><td>%UP% s</td></tr>
</table>
<div style="display:flex;gap:8px;margin-top:12px">
  <a href="/console" style="flex:1;display:block;padding:12px;background:#1f6feb;color:#fff;
     text-align:center;border-radius:6px;font-weight:700;text-decoration:none;font-size:14px">
    &#9881; Motor Console
  </a>
  <a href="/settings" style="flex:1;display:block;padding:12px;background:#6c5ce7;color:#fff;
     text-align:center;border-radius:6px;font-weight:700;text-decoration:none;font-size:14px">
    &#9881; Settings
  </a>
</div>
<form action="/reset" method="POST">
  <button type="submit" onclick="return confirm('Reset config and reboot into setup mode?')">
    Factory Reset
  </button>
</form>
</body></html>
)rawliteral";

static String templateProcessor(const String& var) {
    const auto& cfg = configManager.config();
    if (var == "FW")    return FW_VERSION;
    if (var == "MODE")  return cfg.passthrough ? "PASSTHROUGH" : (cfg.mode == BridgeMode::WIFI_TCP) ? "WiFi TCP" : "BLE";
    if (var == "SSID")  return cfg.wifiSSID;
    if (var == "IP")    return WiFi.localIP().toString();
    if (var == "MDNS")  return cfg.mdnsName;
    if (var == "PORT")  return String(cfg.tcpPort);
    if (var == "BAUD")  return String(cfg.baudRate);
    if (var == "TX")    return String(UART_TX_PIN);
    if (var == "RX")    return String(UART_RX_PIN);
    if (var == "UP")    return String(millis() / 1000);
    return String();
}

// ---------------------------------------------------------------------------
// Settings page (bridge mode) — change config without factory reset
// ---------------------------------------------------------------------------
static const char SETTINGS_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PyPilot Bridge Settings</title>
<style>
  body{font-family:system-ui,sans-serif;max-width:460px;margin:40px auto;
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
  a.back{display:block;margin-top:12px;text-align:center;color:#0fbcf9;font-size:14px}
  .msg{margin-top:12px;padding:10px;border-radius:6px;text-align:center;font-weight:600}
  .msg.ok{background:#2ecc71;color:#1a1a2e} .msg.err{background:#e74c3c;color:#fff}
</style>
</head><body>
<h1>&#9881; Settings</h1>
<div id="msg"></div>
<form id="sf">
  <label>WiFi Network</label>
  <div class="row">
    <select id="scanList" onchange="document.getElementById('ssid').value=this.value">
      <option value="">— click Scan to find networks —</option>
    </select>
    <button type="button" id="scanBtn" onclick="doScan()">Scan</button>
  </div>
  <input name="ssid" id="ssid" required value="%SSID%" placeholder="Network name">

  <label>WiFi Password</label>
  <input name="pass" type="password" placeholder="Enter new password (leave blank to keep current)">

  <label>WiFi Channel</label>
  <select name="channel" id="channel">
    <option value="0">Auto</option>
    <option value="1">1</option><option value="2">2</option><option value="3">3</option>
    <option value="4">4</option><option value="5">5</option><option value="6">6</option>
    <option value="7">7</option><option value="8">8</option><option value="9">9</option>
    <option value="10">10</option><option value="11">11</option>
    <option value="12">12</option><option value="13">13</option>
  </select>

  <label>Bridge Mode</label>
  <select name="mode" id="mode">
    <option value="wifi">WiFi TCP</option>
    <option value="ble">Bluetooth LE</option>
  </select>

  <label>TCP Port (WiFi mode)</label>
  <input name="port" type="number" value="%PORT%" min="1024" max="65535">

  <label>Motor Baud Rate</label>
  <select name="baud" id="baud">
    <option value="9600">9600</option>
    <option value="19200">19200</option>
    <option value="38400">38400 (PyPilot default)</option>
    <option value="57600">57600</option>
    <option value="115200">115200</option>
  </select>

  <label>mDNS Hostname</label>
  <input name="hostname" value="%MDNS%" pattern="[a-z0-9\-]+"
         title="Lowercase letters, numbers, and hyphens only">

  <label style="margin-top:16px;display:flex;align-items:center;gap:8px">
    <input type="checkbox" name="passthrough" id="passthrough" value="1" %PT_CHK%
           style="width:auto;margin:0">
    USB Passthrough Mode
  </label>
  <div style="font-size:12px;color:#888;margin-top:2px">
    Bypasses WiFi/BLE bridge. Forwards USB serial directly to motor UART for testing.
  </div>

  <label style="margin-top:12px;display:flex;align-items:center;gap:8px">
    <input type="checkbox" name="debug" id="debug" value="1" %DBG_CHK%
           style="width:auto;margin:0">
    Debug Mode
  </label>
  <div style="font-size:12px;color:#888;margin-top:2px">
    Verbose logging, fast heartbeat (400ms), raw serial monitor, 10Hz telemetry.
    Off = quiet mode, 2s heartbeat, 2Hz telemetry, lower power.
  </div>

  <button type="submit">Save &amp; Reboot</button>
</form>
<a class="back" href="/">&#8592; Back to Status</a>
<script>
// Pre-select current mode and baud
(function(){
  var m='%MODE_VAL%',b='%BAUD%',ch='%CHANNEL%';
  document.getElementById('mode').value=m;
  document.getElementById('channel').value=ch;
  var bs=document.getElementById('baud');
  for(var i=0;i<bs.options.length;i++){if(bs.options[i].value===b){bs.selectedIndex=i;break;}}
})();

document.getElementById('sf').addEventListener('submit',function(e){
  e.preventDefault();
  var fd=new FormData(this);
  fetch('/settings/save',{method:'POST',body:fd}).then(r=>r.json()).then(d=>{
    var m=document.getElementById('msg');
    if(d.ok){m.className='msg ok';m.textContent='Saved! Rebooting...';
      setTimeout(()=>{window.location='/';},4000);
    }else{m.className='msg err';m.textContent=d.error||'Save failed';}
  }).catch(()=>{
    var m=document.getElementById('msg');m.className='msg err';m.textContent='Connection lost';
  });
});

function doScan(){
  var btn=document.getElementById('scanBtn');
  var sel=document.getElementById('scanList');
  btn.disabled=true;btn.textContent='Scanning...';
  sel.innerHTML='<option value="">Scanning...</option>';
  function poll(){
    fetch('/settings/scan').then(r=>{
      if(r.status===202){setTimeout(poll,1500);return null;}
      return r.json();
    }).then(nets=>{
      if(!nets)return;
      if(nets.length===0){sel.innerHTML='<option value="">No networks found</option>';btn.disabled=false;btn.textContent='Scan';return;}
      sel.innerHTML='<option value="">— select a network —</option>';
      nets.sort((a,b)=>b.rssi-a.rssi);
      var seen={};
      nets.forEach(n=>{
        if(seen[n.ssid]||!n.ssid)return;seen[n.ssid]=1;
        var o=document.createElement('option');o.value=n.ssid;
        var bars=n.rssi>-50?'\u2587\u2587\u2587\u2587':n.rssi>-65?'\u2587\u2587\u2587':n.rssi>-75?'\u2587\u2587':'\u2587';
        o.textContent=n.ssid+' '+bars+' ('+n.rssi+'dBm)'+(n.enc?' \uD83D\uDD12':'');
        sel.appendChild(o);
      });
      btn.disabled=false;btn.textContent='Scan';
    }).catch(()=>{sel.innerHTML='<option value="">Scan failed</option>';btn.disabled=false;btn.textContent='Scan';});
  }
  poll();
}
</script>
</body></html>
)rawliteral";

static String settingsProcessor(const String& var) {
    const auto& cfg = configManager.config();
    if (var == "SSID")     return cfg.wifiSSID;
    if (var == "PORT")     return String(cfg.tcpPort);
    if (var == "BAUD")     return String(cfg.baudRate);
    if (var == "MDNS")     return cfg.mdnsName;
    if (var == "MODE_VAL") return (cfg.mode == BridgeMode::WIFI_TCP) ? "wifi" : "ble";
    if (var == "PT_CHK")   return cfg.passthrough ? "checked" : "";
    if (var == "DBG_CHK")  return cfg.debugMode ? "checked" : "";
    if (var == "CHANNEL")  return String(cfg.wifiChannel);
    return String();
}

static void startStatusServer() {
    statusServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", STATUS_HTML, templateProcessor);
    });
    statusServer.on("/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
        req->send(200, "text/plain", "Resetting...");
        configManager.reset();
        delay(1000);
        ESP.restart();
    });

    // --- Settings page ------------------------------------------------------
    statusServer.on("/settings", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", SETTINGS_HTML, settingsProcessor);
    });

    // WiFi scan from settings page
    statusServer.on("/settings/scan", HTTP_GET, [](AsyncWebServerRequest* req) {
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_FAILED) {
            WiFi.scanNetworks(true);
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
            json += "{\"ssid\":\"" + ssid + "\",";
            json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
            json += "\"enc\":" + String(WiFi.encryptionType(i) != WIFI_AUTH_OPEN) + "}";
        }
        json += "]";
        WiFi.scanDelete();
        req->send(200, "application/json", json);
    });

    // Save settings
    statusServer.on("/settings/save", HTTP_POST, [](AsyncWebServerRequest* req) {
        String ssid;
        if (req->hasParam("ssid", true))
            ssid = req->getParam("ssid", true)->value();
        ssid.trim();
        if (ssid.length() == 0) {
            req->send(200, "application/json", "{\"ok\":false,\"error\":\"SSID is required\"}");
            return;
        }

        auto& cfg = configManager.config();
        cfg.wifiSSID = ssid;

        // Only update password if a new one was provided
        if (req->hasParam("pass", true)) {
            String pass = req->getParam("pass", true)->value();
            if (pass.length() > 0) cfg.wifiPass = pass;
        }
        if (req->hasParam("mode", true)) {
            String mode = req->getParam("mode", true)->value();
            cfg.mode = (mode == "ble") ? BridgeMode::BLE : BridgeMode::WIFI_TCP;
        }
        if (req->hasParam("port", true))
            cfg.tcpPort = (uint16_t)req->getParam("port", true)->value().toInt();
        if (req->hasParam("baud", true))
            cfg.baudRate = (uint32_t)req->getParam("baud", true)->value().toInt();
        if (req->hasParam("hostname", true))
            cfg.mdnsName = req->getParam("hostname", true)->value();
        if (req->hasParam("channel", true))
            cfg.wifiChannel = (uint8_t)req->getParam("channel", true)->value().toInt();

        // Checkboxes — unchecked means param absent
        cfg.passthrough = req->hasParam("passthrough", true);
        cfg.debugMode   = req->hasParam("debug", true);

        cfg.configured = true;
        configManager.save();
        Serial.printf("[SETTINGS] Updated: ssid='%s' mode=%s port=%u baud=%lu debug=%d\n",
                      cfg.wifiSSID.c_str(),
                      cfg.mode == BridgeMode::WIFI_TCP ? "wifi" : "ble",
                      cfg.tcpPort, cfg.baudRate, cfg.debugMode);

        req->send(200, "application/json", "{\"ok\":true}");
        delay(2000);
        ESP.restart();
    });

    // Attach diagnostic console (WebSocket + /console page)
    webConsoleAttach(statusServer);

    statusServer.begin();
}

// ---------------------------------------------------------------------------
// BOOT button monitor — long-press resets config
// ---------------------------------------------------------------------------
static void checkBootButton() {
    static uint32_t pressStart = 0;
    static bool     wasPressed = false;

    bool pressed = (digitalRead(BOOT_BUTTON_PIN) == LOW);

    if (pressed && !wasPressed) {
        pressStart = millis();
    }
    if (pressed && wasPressed) {
        if (millis() - pressStart >= RESET_HOLD_MS) {
            Serial.println("[BOOT] Long-press detected — factory reset!");
            ledStatusSet(LedState::ERROR);
            configManager.reset();
            delay(1000);
            ESP.restart();
        }
    }
    wasPressed = pressed;
}

// ---------------------------------------------------------------------------
// Button-check task (runs on Core 0, very light)
// ---------------------------------------------------------------------------
static void buttonTask(void* /*param*/) {
    pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
    for (;;) {
        checkBootButton();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// =====================================================================
// Arduino setup()
// =====================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);                              // USB-CDC enumeration time
    Serial.println();
    Serial.println("=============================================");
    Serial.println("  PyPilot Wireless Serial Bridge  v" FW_VERSION);
    Serial.println("  ESP32-C3 HW466AB");
    Serial.println("=============================================");

    // ---- LED ---------------------------------------------------------------
    ledStatusBegin();
    ledStatusSet(LedState::BOOT);
    xTaskCreate(ledStatusTask, "led", LED_TASK_STACK,
                nullptr, LED_TASK_PRIO, nullptr);

    // ---- NVS config --------------------------------------------------------
    configManager.begin();
    const auto& cfg = configManager.config();

    Serial.printf("[CFG] configured=%d  mode=%s  ssid='%s'  port=%u  baud=%lu  mdns='%s'  passthrough=%d  debug=%d\n",
                  cfg.configured,
                  cfg.mode == BridgeMode::WIFI_TCP ? "WiFi" : "BLE",
                  cfg.wifiSSID.c_str(),
                  cfg.tcpPort,
                  cfg.baudRate,
                  cfg.mdnsName.c_str(),
                  cfg.passthrough,
                  cfg.debugMode);

    // ---- Setup mode or Bridge mode -----------------------------------------
    if (!cfg.configured) {
        setupModeRun();                       // blocks → reboots
        return;                               // never reached
    }

    // ---- Bridge mode -------------------------------------------------------
    Serial.println("[BOOT] Entering bridge mode");

    // Start UART to motor driver
    serialBridge.begin(cfg.baudRate, UART_TX_PIN, UART_RX_PIN);

    if (cfg.passthrough) {
        // --- Passthrough mode: USB CDC ↔ UART1 direct forwarding ---
        Serial.println("[BOOT] *** PASSTHROUGH MODE — USB serial forwarded to motor UART ***");
        xTaskCreate(passthroughSerialTask, "passthru", SERIAL_TASK_STACK,
                    nullptr, SERIAL_TASK_PRIO, nullptr);

        // Still start WiFi + web server so settings page is accessible
        if (cfg.mode == BridgeMode::WIFI_TCP) {
            xTaskCreate(bridgeWifiTask, "wifi_br", BRIDGE_TASK_STACK,
                        nullptr, BRIDGE_TASK_PRIO, nullptr);
            delay(5000);
            startStatusServer();
        }
    } else {
        // --- Normal bridge mode ---
        xTaskCreate(serialTask, "serial", SERIAL_TASK_STACK,
                    nullptr, SERIAL_TASK_PRIO, nullptr);

        // Start wireless bridge (WiFi or BLE)
        if (cfg.mode == BridgeMode::WIFI_TCP) {
            xTaskCreate(bridgeWifiTask, "wifi_br", BRIDGE_TASK_STACK,
                        nullptr, BRIDGE_TASK_PRIO, nullptr);
            // Status web page (only in WiFi mode — we have a web server)
            // Delay briefly so WiFi task can connect first
            delay(5000);
            startStatusServer();
        } else {
            xTaskCreate(bridgeBleTask, "ble_br", BRIDGE_TASK_STACK,
                        nullptr, BRIDGE_TASK_PRIO, nullptr);
        }
    }

    // Boot button monitor
    xTaskCreate(buttonTask, "btn", 2048,
                nullptr, 0, nullptr);

    Serial.println("[BOOT] All tasks running ✓");
}

// =====================================================================
// Arduino loop() — unused (all work in FreeRTOS tasks)
// =====================================================================
void loop() {
    // Push telemetry to WebSocket clients
    // Debug: ~10 Hz, Normal: ~2 Hz (power saving)
    webConsolePushTelemetry();
    vTaskDelay(pdMS_TO_TICKS(configManager.config().debugMode ? 100 : 500));
}
