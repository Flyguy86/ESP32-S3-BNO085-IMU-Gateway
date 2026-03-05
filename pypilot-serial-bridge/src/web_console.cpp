// =============================================================================
// web_console.cpp — Diagnostic web console  (WebSocket + HTML/JS)
//
//   GET  /console        → rich single-page app
//   WS   /ws             → bidirectional JSON messages
//
//   WS inbound (browser → ESP):
//     {"cmd":"motor","value":1200}        motor position 0-2000 (1000=stop)
//     {"cmd":"disengage"}                 disengage motor
//     {"cmd":"reset"}                     reset overcurrent fault
//     {"cmd":"max_current","value":2000}  max current in 10 mA units
//     {"cmd":"raw","hex":"C7D004XX"}      send raw 3-byte hex (CRC auto-added)
//
//   WS outbound (ESP → browser):
//     {"t":"telem","cur":1.23,"volt":12.4,"ct":32.1,"mt":28.5,
//      "rud":32000,"flags":9,"pkt":1234,"err":0,"age":42}
//     {"t":"log","msg":"..."}
//     {"t":"hex","dir":"rx","data":"1C3A00E2"}
// =============================================================================
#include "web_console.h"
#include "config.h"
#include "pypilot_proto.h"
#include "serial_bridge.h"

#include <ArduinoJson.h>
#include <driver/uart.h>

static AsyncWebSocket ws("/ws");

// Forward declarations
static void doLoopbackTest(AsyncWebSocketClient* client);

// ---------------------------------------------------------------------------
// Flag name helper
// ---------------------------------------------------------------------------
static String flagsToString(uint16_t f) {
    String s;
    if (f & PP_FLAG_SYNC)               s += "SYNC ";
    if (f & PP_FLAG_OVERTEMP_FAULT)     s += "OVERTEMP ";
    if (f & PP_FLAG_OVERCURRENT_FAULT)  s += "OVERCUR ";
    if (f & PP_FLAG_ENGAGED)            s += "ENGAGED ";
    if (f & PP_FLAG_INVALID)            s += "INVALID ";
    if (f & PP_FLAG_PORT_PIN_FAULT)     s += "PORT_PIN ";
    if (f & PP_FLAG_STARBOARD_PIN_FAULT)s += "STBD_PIN ";
    if (f & PP_FLAG_BADVOLTAGE_FAULT)   s += "BADVOLT ";
    if (f & PP_FLAG_MIN_RUDDER_FAULT)   s += "MIN_RUD ";
    if (f & PP_FLAG_MAX_RUDDER_FAULT)   s += "MAX_RUD ";
    if (f & PP_FLAG_CURRENT_RANGE)      s += "CUR_RNG ";
    if (f & PP_FLAG_BAD_FUSES)          s += "FUSES ";
    if (f & PP_FLAG_REBOOTED)           s += "REBOOT ";
    if (f & PP_FLAG_DRIVER_TIMEOUT)     s += "TIMEOUT ";
    if (s.isEmpty()) s = "NONE";
    s.trim();
    return s;
}

// ---------------------------------------------------------------------------
// Send a command packet to the motor via UART
// ---------------------------------------------------------------------------
static void sendMotorCmd(uint8_t code, uint16_t value) {
    uint8_t pkt[4];
    ppBuildPacket(code, value, pkt);
    serialBridge.uartWrite(pkt, 4);

    // Log to WebSocket clients
    char hex[12];
    snprintf(hex, sizeof(hex), "%02X%02X%02X%02X", pkt[0], pkt[1], pkt[2], pkt[3]);
    String msg = "{\"t\":\"hex\",\"dir\":\"tx\",\"data\":\"" + String(hex) + "\"}";
    ws.textAll(msg);
}

// ---------------------------------------------------------------------------
// Background scan tasks (run in FreeRTOS to avoid blocking WebSocket)
// ---------------------------------------------------------------------------
static volatile bool scanTaskRunning = false;

static void wsLog(const char* text) {
    char msg[300];
    snprintf(msg, sizeof(msg), "{\"t\":\"log\",\"msg\":\"%s\"}", text);
    ws.textAll(msg);
}

static void baudScanTask(void* /*param*/) {
    const uint32_t bauds[] = {9600, 19200, 38400, 57600, 115200};
    const int numBauds = sizeof(bauds) / sizeof(bauds[0]);

    wsLog("Scanning baud rates on BOTH pin orientations...");
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int orient = 0; orient < 2; orient++) {
        int txP = (orient == 0) ? UART_TX_PIN : UART_RX_PIN;
        int rxP = (orient == 0) ? UART_RX_PIN : UART_TX_PIN;

        for (int i = 0; i < numBauds; i++) {
            serialBridge.reinit(bauds[i], rxP, txP);
            vTaskDelay(pdMS_TO_TICKS(20));

            uint32_t rxBefore = serialBridge.rawByteCount;

            const uint8_t wake[] = {0xFF, 0xFF, 0xFF, 0xFF};
            serialBridge.uartWrite(wake, 4);
            vTaskDelay(pdMS_TO_TICKS(50));

            for (int j = 0; j < 5; j++) {
                uint8_t pkt[4];
                ppBuildPacket(PP_DISENGAGE_CODE, 0, pkt);
                serialBridge.uartWrite(pkt, 4);
                vTaskDelay(pdMS_TO_TICKS(30));
                serialBridge.poll();
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            serialBridge.poll();

            uint32_t rxGot = serialBridge.rawByteCount - rxBefore;

            char line[200];
            snprintf(line, sizeof(line),
                "TX=GPIO%d RX=GPIO%d @ %lu baud: RX=%lu bytes %s",
                txP, rxP, bauds[i], rxGot,
                rxGot > 0 ? "** GOT DATA! **" : "");
            wsLog(line);

            if (rxGot > 0) {
                char hexDump[200];
                serialBridge.dbgDrain(hexDump, sizeof(hexDump));
                char hexLine[260];
                snprintf(hexLine, sizeof(hexLine), "Raw hex: %s", hexDump);
                wsLog(hexLine);
            }

            vTaskDelay(pdMS_TO_TICKS(10));  // yield
        }
    }

    serialBridge.reinit(MOTOR_BAUD_DEFAULT, UART_RX_PIN, UART_TX_PIN);
    wsLog("Baud scan complete. UART restored.");
    scanTaskRunning = false;
    vTaskDelete(nullptr);
}

static void gpioScanTask(void* /*param*/) {
    const int testPins[] = {2, 3, 4, 5, 6, 7, 10};
    const int numPins = sizeof(testPins) / sizeof(testPins[0]);

    wsLog("Scanning GPIOs for serial activity (1s each)...");
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int i = 0; i < numPins; i++) {
        int pin = testPins[i];
        pinMode(pin, INPUT_PULLUP);
        vTaskDelay(pdMS_TO_TICKS(10));

        uint32_t transitions = 0;
        int lastState = digitalRead(pin);
        uint32_t start = millis();
        while ((millis() - start) < 1000) {
            int state = digitalRead(pin);
            if (state != lastState) {
                transitions++;
                lastState = state;
            }
            vTaskDelay(0);  // yield to watchdog
        }

        char line[120];
        const char* status = transitions > 100 ? "** ACTIVE **" :
                             transitions > 0 ?   "low" : "dead";
        snprintf(line, sizeof(line), "GPIO%d: %lu transitions (%s)",
                 pin, transitions, status);
        wsLog(line);
    }

    serialBridge.begin(MOTOR_BAUD_DEFAULT, UART_TX_PIN, UART_RX_PIN);
    wsLog("GPIO scan complete. UART restarted.");
    scanTaskRunning = false;
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// WebSocket event handler
// ---------------------------------------------------------------------------
static void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len)
{
    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] Client #%u connected\n", client->id());
        // Send a welcome log
        client->text("{\"t\":\"log\",\"msg\":\"Connected to PyPilot Bridge console\"}");
        // Send UART config info
        char cfgMsg[128];
        snprintf(cfgMsg, sizeof(cfgMsg),
            "{\"t\":\"uart_cfg\",\"tx\":%d,\"rx\":%d,\"baud\":%d}",
            UART_TX_PIN, UART_RX_PIN, MOTOR_BAUD_DEFAULT);
        client->text(cfgMsg);
    }
    else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Client #%u disconnected\n", client->id());
    }
    else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->opcode != WS_TEXT) return;

        // Parse JSON command
        JsonDocument doc;
        DeserializationError err = deserializeJson(doc, data, len);
        if (err) {
            client->text("{\"t\":\"log\",\"msg\":\"Bad JSON\"}");
            return;
        }

        const char* cmd = doc["cmd"] | "";

        if (strcmp(cmd, "motor") == 0) {
            uint16_t val = doc["value"] | 1000;
            if (val > 2000) val = 2000;
            sendMotorCmd(PP_COMMAND_CODE, val);
        }
        else if (strcmp(cmd, "disengage") == 0) {
            sendMotorCmd(PP_DISENGAGE_CODE, 0);
        }
        else if (strcmp(cmd, "reset") == 0) {
            sendMotorCmd(PP_RESET_CODE, 0);
        }
        else if (strcmp(cmd, "max_current") == 0) {
            uint16_t val = doc["value"] | 2000;
            sendMotorCmd(PP_MAX_CURRENT_CODE, val);
        }
        else if (strcmp(cmd, "max_ctrl_temp") == 0) {
            uint16_t val = doc["value"] | 7000;
            sendMotorCmd(PP_MAX_CONTROLLER_TEMP, val);
        }
        else if (strcmp(cmd, "max_motor_temp") == 0) {
            uint16_t val = doc["value"] | 7000;
            sendMotorCmd(PP_MAX_MOTOR_TEMP, val);
        }
        else if (strcmp(cmd, "clutch") == 0) {
            uint8_t pwm   = doc["pwm"]   | 255;
            uint8_t brake = doc["brake"]  | 0;
            uint16_t val  = pwm | ((uint16_t)brake << 8);
            sendMotorCmd(PP_CLUTCH_PWM_BRAKE_CODE, val);
        }
        else if (strcmp(cmd, "raw") == 0) {
            // Parse 3 hex bytes (6 chars), auto-add CRC
            const char* hexStr = doc["hex"] | "";
            size_t slen = strlen(hexStr);
            if (slen >= 6) {
                uint8_t pkt[4];
                for (int i = 0; i < 3; i++) {
                    char tmp[3] = { hexStr[i*2], hexStr[i*2+1], 0 };
                    pkt[i] = (uint8_t)strtol(tmp, nullptr, 16);
                }
                pkt[3] = ppCrc8(pkt, 3);
                serialBridge.uartWrite(pkt, 4);
                char hex[12];
                snprintf(hex, sizeof(hex), "%02X%02X%02X%02X",
                         pkt[0], pkt[1], pkt[2], pkt[3]);
                String msg = "{\"t\":\"hex\",\"dir\":\"tx\",\"data\":\"" + String(hex) + "\"}";
                ws.textAll(msg);
            } else {
                client->text("{\"t\":\"log\",\"msg\":\"Raw hex must be 6+ chars (3 bytes)\"}");
            }
        }
        else if (strcmp(cmd, "loopback") == 0) {
            doLoopbackTest(client);
        }
        else if (strcmp(cmd, "monitor") == 0) {
            serialBridge.rawMonitorEnabled = !serialBridge.rawMonitorEnabled;
            char msg[100];
            snprintf(msg, sizeof(msg),
                "{\"t\":\"log\",\"msg\":\"Raw serial monitor %s\"}",
                serialBridge.rawMonitorEnabled ? "ENABLED - watching TX/RX" : "DISABLED");
            client->text(msg);
            snprintf(msg, sizeof(msg),
                "{\"t\":\"mon_state\",\"on\":%s}",
                serialBridge.rawMonitorEnabled ? "true" : "false");
            client->text(msg);
        }
        else if (strcmp(cmd, "baud_scan") == 0) {
            if (scanTaskRunning) {
                client->text("{\"t\":\"log\",\"msg\":\"Scan already running...\"}");
            } else {
                client->text("{\"t\":\"log\",\"msg\":\"Starting baud scan (runs in background)...\"}");
                scanTaskRunning = true;
                xTaskCreate(baudScanTask, "bscan", 4096, nullptr, 1, nullptr);
            }
        }
        else if (strcmp(cmd, "gpio_test") == 0) {
            if (scanTaskRunning) {
                client->text("{\"t\":\"log\",\"msg\":\"Scan already running...\"}");
            } else {
                client->text("{\"t\":\"log\",\"msg\":\"Starting GPIO scan (runs in background)...\"}");
                scanTaskRunning = true;
                xTaskCreate(gpioScanTask, "gscan", 4096, nullptr, 1, nullptr);
            }
        }
        else {
            client->text("{\"t\":\"log\",\"msg\":\"Unknown command\"}");
        }
    }
}

// ---------------------------------------------------------------------------
// Loopback test: deep diagnostic using raw ESP-IDF UART calls
// Tests HardwareSerial layer AND raw driver independently
// ---------------------------------------------------------------------------
static void doLoopbackTest(AsyncWebSocketClient* client) {
    char msg[300];
    const uart_port_t port = (uart_port_t)UART_NUM;

    // Step 1: Check if UART driver is installed
    bool driverInstalled = uart_is_driver_installed(port);
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[1] UART%d driver installed: %s\"}", UART_NUM,
        driverInstalled ? "YES" : "NO");
    client->text(msg);

    if (!driverInstalled) {
        snprintf(msg, sizeof(msg),
            "{\"t\":\"log\",\"msg\":\"FAIL: UART driver not installed! HardwareSerial::begin() may have failed.\"}");
        client->text(msg);
        return;
    }

    // Step 2: Enable internal loopback
    esp_err_t err = uart_set_loop_back(port, true);
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[2] uart_set_loop_back → %s (err=%d)\"}", 
        err == ESP_OK ? "OK" : "FAIL", err);
    client->text(msg);

    delay(10);

    // Step 3: Flush RX buffer via ESP-IDF
    uart_flush_input(port);
    delay(5);

    // Step 4: Write test data directly via ESP-IDF (bypass HardwareSerial)
    const uint8_t testData[] = {0xAA, 0x55, 0xDE, 0xAD, 0xBE, 0xEF, 0x42, 0x99};
    int written = uart_write_bytes(port, (const char*)testData, sizeof(testData));
    err = uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[3] uart_write_bytes → %d written, tx_done=%s\"}", 
        written, err == ESP_OK ? "OK" : "TIMEOUT");
    client->text(msg);

    delay(20); // wait for loopback

    // Step 5: Check how many bytes available in RX buffer
    size_t buffered = 0;
    uart_get_buffered_data_len(port, &buffered);
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[4] uart_get_buffered_data_len → %u bytes in RX buffer\"}", 
        (unsigned)buffered);
    client->text(msg);

    // Step 6: Read back via ESP-IDF
    uint8_t rxBuf[16] = {0};
    int readLen = uart_read_bytes(port, rxBuf, sizeof(rxBuf), pdMS_TO_TICKS(100));
    
    // Format received bytes as hex
    char hexStr[64] = "none";
    if (readLen > 0) {
        int pos = 0;
        for (int i = 0; i < readLen && pos < 60; i++) {
            if (i > 0) hexStr[pos++] = ' ';
            pos += snprintf(hexStr + pos, 4, "%02X", rxBuf[i]);
        }
    }
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[5] uart_read_bytes → %d bytes: %s\"}", readLen, hexStr);
    client->text(msg);

    // Step 7: Also test HardwareSerial layer
    uart_flush_input(port);
    delay(5);
    uart_write_bytes(port, (const char*)testData, 4);
    uart_wait_tx_done(port, pdMS_TO_TICKS(100));
    delay(20);
    serialBridge.poll();
    int hsAvail = 0; // count via rawByteCount delta
    uint32_t before = serialBridge.rawByteCount;
    serialBridge.poll();
    uint32_t after = serialBridge.rawByteCount;
    hsAvail = after - before;
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"[6] HardwareSerial poll → %d new RX bytes\"}", hsAvail);
    client->text(msg);

    // Restore normal operation
    uart_set_loop_back(port, false);

    // Final verdict
    const char* verdict = (readLen >= (int)sizeof(testData)) ? "PASS: UART HW+driver working" :
                          (readLen > 0) ? "PARTIAL: some bytes received" :
                          "FAIL: UART RX completely dead";
    snprintf(msg, sizeof(msg),
        "{\"t\":\"log\",\"msg\":\"=== LOOPBACK RESULT: %s ===\"}", verdict);
    client->text(msg);
}


// ---------------------------------------------------------------------------
// Push telemetry JSON to all connected WS clients (~10 Hz from main loop)
// ---------------------------------------------------------------------------
void webConsolePushTelemetry() {
    if (ws.count() == 0) return;   // nobody listening

    const auto& t = protoSniffer.telemetry();
    uint32_t age = (t.lastUpdateMs > 0) ? (millis() - t.lastUpdateMs) : 0;

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"t\":\"telem\",\"cur\":%.2f,\"volt\":%.2f,\"ct\":%.1f,\"mt\":%.1f,"
        "\"rud\":%u,\"flags\":%u,\"flagsStr\":\"%s\","
        "\"pkt\":%lu,\"err\":%lu,\"age\":%lu,"
        "\"rawRx\":%lu,\"rawTx\":%lu}",
        t.current_A, t.voltage_V, t.controllerTemp_C, t.motorTemp_C,
        t.rudderRaw, t.flags, flagsToString(t.flags).c_str(),
        t.packetCount, t.errorCount, age,
        serialBridge.rawByteCount, serialBridge.txByteCount);

    ws.textAll(buf);

    // Push raw hex dump of recently received bytes (if any)
    char hexDump[200];
    serialBridge.dbgDrain(hexDump, sizeof(hexDump));
    if (hexDump[0] != '\0') {
        char rxMsg[256];
        snprintf(rxMsg, sizeof(rxMsg),
            "{\"t\":\"hex\",\"dir\":\"rx\",\"data\":\"%s\"}", hexDump);
        ws.textAll(rxMsg);
    }

    // Stream raw monitor data if enabled
    if (serialBridge.rawMonitorEnabled) {
        uint8_t rxBytes[64], txBytes[64];
        size_t rxN = 0, txN = 0;
        serialBridge.monDrain(rxBytes, sizeof(rxBytes), &rxN,
                              txBytes, sizeof(txBytes), &txN);

        // Send TX bytes as hex
        if (txN > 0) {
            char hexStr[200];
            size_t pos = 0;
            for (size_t i = 0; i < txN && (pos + 3) < sizeof(hexStr); i++) {
                if (i > 0) hexStr[pos++] = ' ';
                snprintf(hexStr + pos, 3, "%02X", txBytes[i]);
                pos += 2;
            }
            hexStr[pos] = '\0';
            char msg[260];
            snprintf(msg, sizeof(msg),
                "{\"t\":\"hex\",\"dir\":\"tx\",\"data\":\"%s\"}", hexStr);
            ws.textAll(msg);
        }

        // Send RX bytes as hex
        if (rxN > 0) {
            char hexStr[200];
            size_t pos = 0;
            for (size_t i = 0; i < rxN && (pos + 3) < sizeof(hexStr); i++) {
                if (i > 0) hexStr[pos++] = ' ';
                snprintf(hexStr + pos, 3, "%02X", rxBytes[i]);
                pos += 2;
            }
            hexStr[pos] = '\0';
            char msg[260];
            snprintf(msg, sizeof(msg),
                "{\"t\":\"hex\",\"dir\":\"rx\",\"data\":\"%s\"}", hexStr);
            ws.textAll(msg);
        }
    }
}

// ---------------------------------------------------------------------------
// Console HTML page (served from flash)
// ---------------------------------------------------------------------------
static const char CONSOLE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>PyPilot Console</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#c9d1d9;
     max-width:900px;margin:0 auto;padding:12px}
h1{color:#58a6ff;text-align:center;margin:8px 0;font-size:1.4em}
h2{color:#8b949e;font-size:.95em;margin:12px 0 6px;text-transform:uppercase;
   letter-spacing:.05em;border-bottom:1px solid #21262d;padding-bottom:4px}

/* Telemetry gauges */
.gauges{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));gap:8px;margin-bottom:8px}
.gauge{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px;text-align:center}
.gauge .label{font-size:.7em;color:#8b949e;text-transform:uppercase}
.gauge .value{font-size:1.5em;font-weight:700;color:#58a6ff;font-variant-numeric:tabular-nums}
.gauge .unit{font-size:.7em;color:#8b949e}
.gauge.warn .value{color:#d29922}
.gauge.err  .value{color:#f85149}
.gauge.ok   .value{color:#3fb950}

/* Flags */
.flags{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:8px 12px;
       margin-bottom:8px;font-size:.8em;word-break:break-word}
.flags .tag{display:inline-block;padding:2px 6px;margin:2px;border-radius:4px;font-size:.75em;
            background:#30363d;color:#c9d1d9}
.flags .tag.active{background:#1f6feb;color:#fff}
.flags .tag.fault{background:#da3633;color:#fff}

/* Motor controls */
.controls{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:8px;margin-bottom:8px}
.ctrl-group{background:#161b22;border:1px solid #30363d;border-radius:8px;padding:10px}
.ctrl-group label{display:block;font-size:.75em;color:#8b949e;margin-bottom:4px}
.btn-row{display:flex;gap:4px;flex-wrap:wrap}
button{padding:8px 14px;border:none;border-radius:6px;cursor:pointer;font-size:.85em;
       font-weight:600;transition:background .15s}
.btn-stop{background:#da3633;color:#fff}     .btn-stop:hover{background:#b62324}
.btn-port{background:#1f6feb;color:#fff}     .btn-port:hover{background:#1958b7}
.btn-stbd{background:#238636;color:#fff}     .btn-stbd:hover{background:#196c2e}
.btn-cmd{background:#30363d;color:#c9d1d9}    .btn-cmd:hover{background:#484f58}
.btn-danger{background:#8b0000;color:#fff}   .btn-danger:hover{background:#6e0000}
.btn-warn{background:#9e6a03;color:#fff}     .btn-warn:hover{background:#7d5500}

/* Slider */
.slider-wrap{margin:8px 0}
input[type=range]{width:100%;accent-color:#58a6ff}
.slider-val{text-align:center;font-size:.85em;color:#58a6ff;font-weight:700;margin-top:2px}

/* Raw hex */
.raw-row{display:flex;gap:6px;align-items:center;margin-top:6px}
.raw-row input{flex:1;padding:8px;background:#0d1117;border:1px solid #30363d;border-radius:6px;
               color:#c9d1d9;font-family:monospace;font-size:.9em}

/* Log */
.log{background:#0d1117;border:1px solid #30363d;border-radius:8px;padding:8px;
     height:200px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:.75em;
     line-height:1.5}
.log .rx{color:#3fb950} .log .tx{color:#f0883e} .log .info{color:#8b949e}

/* Status bar */
.status-bar{display:flex;justify-content:space-between;align-items:center;
            font-size:.7em;color:#8b949e;margin-top:8px;padding-top:6px;border-top:1px solid #21262d}
.ws-dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:4px}
.ws-dot.on{background:#3fb950} .ws-dot.off{background:#da3633}

a{color:#58a6ff;text-decoration:none}
</style>
</head>
<body>

<h1>&#9875; PyPilot Motor Console</h1>

<!-- Telemetry Gauges -->
<h2>Motor Telemetry</h2>
<div class="gauges">
  <div class="gauge" id="g-cur"><div class="label">Current</div><div class="value" id="v-cur">--</div><div class="unit">A</div></div>
  <div class="gauge" id="g-volt"><div class="label">Voltage</div><div class="value" id="v-volt">--</div><div class="unit">V</div></div>
  <div class="gauge" id="g-ct"><div class="label">Ctrl Temp</div><div class="value" id="v-ct">--</div><div class="unit">&deg;C</div></div>
  <div class="gauge" id="g-mt"><div class="label">Motor Temp</div><div class="value" id="v-mt">--</div><div class="unit">&deg;C</div></div>
  <div class="gauge" id="g-rud"><div class="label">Rudder</div><div class="value" id="v-rud">--</div><div class="unit">raw</div></div>
  <div class="gauge" id="g-pkt"><div class="label">Packets</div><div class="value" id="v-pkt">0</div><div class="unit">good / <span id="v-err">0</span> err</div></div>
</div>

<!-- UART Debug -->
<h2>UART Debug</h2>
<div class="gauges">
  <div class="gauge" id="g-rxb"><div class="label">RX Bytes</div><div class="value" id="v-rxb">0</div><div class="unit">total raw</div></div>
  <div class="gauge" id="g-txb"><div class="label">TX Bytes</div><div class="value" id="v-txb">0</div><div class="unit">total raw</div></div>
</div>
<div class="flags" style="font-family:monospace;font-size:.75em">
  <strong>UART Config:</strong> TX=GPIO<span id="uart-tx">?</span> RX=GPIO<span id="uart-rx">?</span> @ <span id="uart-baud">?</span> baud
</div>

<!-- Flags -->
<div class="flags" id="flags-box">
  <strong>Flags:</strong> <span id="flags-tags">--</span>
</div>

<!-- Motor Controls -->
<h2>Motor Control</h2>
<div class="controls">
  <div class="ctrl-group">
    <label>Quick Commands</label>
    <div class="btn-row">
      <button class="btn-port" onclick="sendCmd('motor',{value:600})" title="Port fast">&#9664;&#9664; Port</button>
      <button class="btn-port" onclick="sendCmd('motor',{value:800})" title="Port slow">&#9664; Slow</button>
      <button class="btn-stop" onclick="sendCmd('disengage')" title="Disengage motor">&#9724; Stop</button>
      <button class="btn-stbd" onclick="sendCmd('motor',{value:1200})" title="Starboard slow">Slow &#9654;</button>
      <button class="btn-stbd" onclick="sendCmd('motor',{value:1400})" title="Starboard fast">Stbd &#9654;&#9654;</button>
    </div>
  </div>
  <div class="ctrl-group">
    <label>Motor Position (0=full port, 1000=stop, 2000=full stbd)</label>
    <div class="slider-wrap">
      <input type="range" id="motor-slider" min="0" max="2000" value="1000" step="10">
      <div class="slider-val" id="slider-label">1000</div>
    </div>
    <div class="btn-row">
      <button class="btn-cmd" onclick="sendSlider()">Send Position</button>
      <button class="btn-cmd" onclick="resetSlider()">Center (1000)</button>
    </div>
  </div>
</div>

<div class="controls">
  <div class="ctrl-group">
    <label>System Commands</label>
    <div class="btn-row">
      <button class="btn-warn" onclick="sendCmd('reset')" title="Reset overcurrent fault">Reset Fault</button>
      <button class="btn-cmd" onclick="sendCmd('max_current',{value:parseInt(prompt('Max current (10mA units):','2000'))||2000})">Set Max Current</button>
      <button class="btn-cmd" onclick="sendCmd('clutch',{pwm:parseInt(prompt('Clutch PWM 0-255:','255'))||255,brake:0})">Clutch PWM</button>
      <button class="btn-cmd" onclick="sendCmd('loopback')" title="Internal HW loopback test (no jumper needed)">Loopback Test</button>
      <button class="btn-cmd" onclick="sendCmd('gpio_test')" title="Check for any electrical signal on RX pin">GPIO RX Test</button>
      <button class="btn-warn" onclick="sendCmd('baud_scan')" title="Try all baud rates + pin swaps to find motor">Baud Scan</button>
      <button class="btn-cmd" id="mon-btn" onclick="sendCmd('monitor')" title="Toggle raw serial byte monitor">&#128269; Raw Monitor: OFF</button>
    </div>
  </div>
  <div class="ctrl-group">
    <label>Send Raw Hex (3 bytes = 6 hex chars, CRC auto-added)</label>
    <div class="raw-row">
      <input id="raw-hex" type="text" placeholder="e.g. C7D004" maxlength="6" pattern="[0-9a-fA-F]{6}">
      <button class="btn-cmd" onclick="sendRaw()">Send</button>
    </div>
  </div>
</div>

<!-- Packet Log -->
<h2>Packet Log <button class="btn-cmd" onclick="clearLog()" style="float:right;padding:2px 8px;font-size:.75em">Clear</button></h2>
<div class="log" id="log"></div>

<!-- Status bar -->
<div class="status-bar">
  <span><span class="ws-dot off" id="ws-dot"></span><span id="ws-status">Disconnected</span></span>
  <span>Age: <span id="v-age">--</span> ms</span>
  <a href="/">&#8592; Status</a>
</div>

<script>
// --- WebSocket ---------------------------------------------------------------
let ws, wsOk=false;
const MAX_LOG=200;

function connect(){
  const proto=location.protocol==='https:'?'wss':'ws';
  ws=new WebSocket(proto+'://'+location.host+'/ws');
  ws.onopen=()=>{wsOk=true;dot(true);log('info','WebSocket connected')};
  ws.onclose=()=>{wsOk=false;dot(false);log('info','WebSocket closed — reconnecting…');setTimeout(connect,2000)};
  ws.onerror=()=>{ws.close()};
  ws.onmessage=e=>{
    try{const m=JSON.parse(e.data);handle(m)}catch(ex){}
  };
}

function dot(on){
  const d=document.getElementById('ws-dot');
  d.className='ws-dot '+(on?'on':'off');
  document.getElementById('ws-status').textContent=on?'Connected':'Disconnected';
}

// --- Handle inbound messages ------------------------------------------------
const FLAG_NAMES=[
  ['SYNC',0x0001,false],['OVERTEMP',0x0002,true],['OVERCURRENT',0x0004,true],
  ['ENGAGED',0x0008,false],['INVALID',0x0010,true],['PORT_PIN',0x0020,true],
  ['STBD_PIN',0x0040,true],['BADVOLT',0x0080,true],['MIN_RUD',0x0100,true],
  ['MAX_RUD',0x0200,true],['CUR_RNG',0x0400,false],['FUSES',0x0800,true],
  ['REBOOT',0x1000,false],['TIMEOUT',0x2000,true]
];

function handle(m){
  if(m.t==='telem'){
    setVal('v-cur',m.cur.toFixed(2)); gaugeClass('g-cur',m.cur>15?'err':m.cur>10?'warn':'ok');
    setVal('v-volt',m.volt.toFixed(1)); gaugeClass('g-volt',m.volt<10?'err':m.volt<11?'warn':'ok');
    setVal('v-ct',m.ct.toFixed(1)); gaugeClass('g-ct',m.ct>60?'err':m.ct>45?'warn':'ok');
    setVal('v-mt',m.mt.toFixed(1)); gaugeClass('g-mt',m.mt>60?'err':m.mt>45?'warn':'ok');
    setVal('v-rud',m.rud);
    setVal('v-pkt',m.pkt); setVal('v-err',m.err);
    setVal('v-age',m.age);

    // UART debug counters
    if(m.rawRx!==undefined) setVal('v-rxb',m.rawRx);
    if(m.rawTx!==undefined) setVal('v-txb',m.rawTx);
    gaugeClass('g-rxb',m.rawRx>0?'ok':'err');
    gaugeClass('g-txb',m.rawTx>0?'ok':'err');

    // Flags
    let html='';
    FLAG_NAMES.forEach(([name,bit,isFault])=>{
      const active=!!(m.flags&bit);
      let cls='tag';
      if(active) cls+=isFault?' fault':' active';
      html+='<span class="'+cls+'">'+name+'</span> ';
    });
    document.getElementById('flags-tags').innerHTML=html;
  }
  else if(m.t==='hex'){
    log(m.dir, (m.dir==='tx'?'TX → ':'RX ← ')+m.data);
  }  else if(m.t==='uart_cfg'){
    document.getElementById('uart-tx').textContent=m.tx;
    document.getElementById('uart-rx').textContent=m.rx;
    document.getElementById('uart-baud').textContent=m.baud;
    log('info','UART: TX=GPIO'+m.tx+' RX=GPIO'+m.rx+' @ '+m.baud+' baud');
  }  else if(m.t==='mon_state'){
    var btn=document.getElementById('mon-btn');
    if(m.on){btn.textContent='\uD83D\uDD0D Raw Monitor: ON';btn.className='btn-stbd';}
    else{btn.textContent='\uD83D\uDD0D Raw Monitor: OFF';btn.className='btn-cmd';}
  }  else if(m.t==='log'){
    log('info',m.msg);
  }
}

function setVal(id,v){document.getElementById(id).textContent=v}
function gaugeClass(id,cls){
  const el=document.getElementById(id);
  el.classList.remove('ok','warn','err');
  el.classList.add(cls);
}

// --- Send commands ----------------------------------------------------------
function sendCmd(cmd,extra){
  if(!wsOk)return;
  const msg=Object.assign({cmd},extra||{});
  ws.send(JSON.stringify(msg));
}

function sendSlider(){
  const v=parseInt(document.getElementById('motor-slider').value);
  sendCmd('motor',{value:v});
}

function resetSlider(){
  const sl=document.getElementById('motor-slider');
  sl.value=1000;
  document.getElementById('slider-label').textContent='1000';
}

function sendRaw(){
  const hex=document.getElementById('raw-hex').value.trim();
  if(hex.length>=6) sendCmd('raw',{hex:hex.substring(0,6).toUpperCase()});
}

// --- Log --------------------------------------------------------------------
function log(cls,msg){
  const el=document.getElementById('log');
  const line=document.createElement('div');
  line.className=cls;
  const ts=new Date().toLocaleTimeString('en',{hour12:false,hour:'2-digit',minute:'2-digit',second:'2-digit',fractionalSecondDigits:1});
  line.textContent='['+ts+'] '+msg;
  el.appendChild(line);
  while(el.children.length>MAX_LOG) el.removeChild(el.firstChild);
  el.scrollTop=el.scrollHeight;
}

function clearLog(){document.getElementById('log').innerHTML=''}

// --- Slider live label ------------------------------------------------------
document.getElementById('motor-slider').addEventListener('input',e=>{
  document.getElementById('slider-label').textContent=e.target.value;
});

// --- Boot -------------------------------------------------------------------
connect();
</script>
</body>
</html>
)rawliteral";

// ---------------------------------------------------------------------------
// Attach to server
// ---------------------------------------------------------------------------
void webConsoleAttach(AsyncWebServer& server) {
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.on("/console", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html", CONSOLE_HTML);
    });

    Serial.println("[WEB] Console attached at /console  WS at /ws");
}
