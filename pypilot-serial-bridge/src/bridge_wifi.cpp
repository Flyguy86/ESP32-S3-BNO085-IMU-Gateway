// =============================================================================
// bridge_wifi.cpp — WiFi STA connection, TCP server, mDNS advertisement
// =============================================================================
#include "bridge_wifi.h"
#include "config.h"
#include "config_manager.h"
#include "serial_bridge.h"
#include "led_status.h"

#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>

static WiFiServer   tcpServer(DEFAULT_TCP_PORT);
static WiFiClient   tcpClient;
static bool         _clientConnected = false;
static uint32_t     lastReconnect    = 0;
static uint32_t     _idleCount       = 0;

// ---------------------------------------------------------------------------
// Internal: Register mDNS hostname + DNS-SD services with TXT records
// ---------------------------------------------------------------------------
static void registerMDNS() {
    const auto& cfg = configManager.config();
    const char* host = cfg.mdnsName.c_str();

    if (!MDNS.begin(host)) {
        Serial.println("[mDNS] FAILED to start mDNS responder");
        return;
    }
    Serial.printf("[mDNS] Hostname: %s.local\n", host);

    // ---- _pypilot._tcp  — PyPilot auto-discovery ---------------------------
    MDNS.addService(PYPILOT_SERVICE_TYPE, "tcp", cfg.tcpPort);
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "version",  FW_VERSION);
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "type",     "serial-bridge");
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "baud",     String(cfg.baudRate).c_str());
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "uart_tx",  String(UART_TX_PIN).c_str());
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "uart_rx",  String(UART_RX_PIN).c_str());
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "mode",     "wifi-tcp");
    MDNS.addServiceTxt(PYPILOT_SERVICE_TYPE, "tcp", "hostname", host);
    Serial.printf("[mDNS] Advertising _%s._tcp on port %u\n",
                  PYPILOT_SERVICE_TYPE, cfg.tcpPort);

    // ---- _http._tcp — web config page --------------------------------------
    MDNS.addService(HTTP_SERVICE_TYPE, "tcp", 80);
    MDNS.addServiceTxt(HTTP_SERVICE_TYPE, "tcp", "path", "/");
    MDNS.addServiceTxt(HTTP_SERVICE_TYPE, "tcp", "type", "pypilot-bridge");
}

// ---------------------------------------------------------------------------
// Connect to the saved WiFi network
// ---------------------------------------------------------------------------
static bool connectWiFi() {
    const auto& cfg = configManager.config();
    if (cfg.wifiSSID.isEmpty()) {
        Serial.println("[WiFi] No SSID configured");
        return false;
    }

    Serial.printf("[WiFi] Connecting to '%s'%s ...\n", cfg.wifiSSID.c_str(),
                  cfg.wifiChannel ? (" ch" + String(cfg.wifiChannel)).c_str() : " (auto channel)");
    WiFi.mode(WIFI_STA);
    if (cfg.wifiChannel > 0) {
        WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str(), cfg.wifiChannel);
    } else {
        WiFi.begin(cfg.wifiSSID.c_str(), cfg.wifiPass.c_str());
    }

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - t0 > 15000) {          // 15 s timeout
            Serial.println("[WiFi] Connection FAILED");
            return false;
        }
        delay(250);
        Serial.print(".");
    }
    Serial.printf("\n[WiFi] Connected — IP %s\n", WiFi.localIP().toString().c_str());
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void bridgeWifiBegin() {
    if (!connectWiFi()) {
        ledStatusSet(LedState::ERROR);
        return;
    }

    const auto& cfg = configManager.config();
    tcpServer = WiFiServer(cfg.tcpPort);
    tcpServer.begin();
    tcpServer.setNoDelay(true);
    Serial.printf("[TCP] Server listening on port %u\n", cfg.tcpPort);

    registerMDNS();                           // start mDNS + DNS-SD
    ledStatusSet(LedState::WIFI_WAITING);
}

bool bridgeWifiClientConnected() {
    return _clientConnected;
}

// ---------------------------------------------------------------------------
// Main data-pump — called from bridgeWifiTask at high rate
// ---------------------------------------------------------------------------
void bridgeWifiLoop() {
    // ----- Accept new client ------------------------------------------------
    if (!tcpClient || !tcpClient.connected()) {
        if (_clientConnected) {
            Serial.println("[TCP] Client disconnected");
            _clientConnected = false;
            ledStatusSet(LedState::WIFI_WAITING);
        }
        tcpClient = tcpServer.accept();
        if (tcpClient) {
            tcpClient.setNoDelay(true);
            _clientConnected = true;
            Serial.printf("[TCP] Client connected from %s\n",
                          tcpClient.remoteIP().toString().c_str());
            ledStatusSet(LedState::WIFI_BRIDGING);
        }
    }

    if (!_clientConnected) {
        // Handle WiFi reconnection
        if (WiFi.status() != WL_CONNECTED && millis() - lastReconnect > 10000) {
            lastReconnect = millis();
            Serial.println("[WiFi] Lost connection — reconnecting...");
            connectWiFi();
            registerMDNS();
        }
        return;
    }

    // ----- TCP → UART (host → motor) ----------------------------------------
    bool didWork = false;
    while (tcpClient.available()) {
        uint8_t buf[256];
        int n = tcpClient.read(buf, sizeof(buf));
        if (n > 0) {
            serialBridge.uartWrite(buf, n);
            didWork = true;
        }
    }

    // ----- UART → TCP (motor → host) ----------------------------------------
    if (serialBridge.uartAvailable()) {
        uint8_t buf[256];
        size_t n = serialBridge.uartRead(buf, sizeof(buf));
        if (n > 0 && tcpClient.connected()) {
            tcpClient.write(buf, n);
            didWork = true;
        }
    }

    // Yield more CPU when idle so the web server stays responsive
    if (!didWork) {
        _idleCount++;
    } else {
        _idleCount = 0;
    }
}

// ---------------------------------------------------------------------------
// FreeRTOS task — Core 0
// ---------------------------------------------------------------------------
void bridgeWifiTask(void* /*param*/) {
    bridgeWifiBegin();
    for (;;) {
        bridgeWifiLoop();
        // Adaptive polling: 1 ms when data flowing, up to 10 ms when idle.
        // Gives the async web server CPU time on the single-core C3.
        uint32_t delayMs = (_idleCount > 10) ? 10 : 1;
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}
