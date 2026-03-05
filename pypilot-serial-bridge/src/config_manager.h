// =============================================================================
// config_manager.h — NVS-backed persistent configuration
// =============================================================================
#pragma once

#include <Arduino.h>

// Bridge connection mode
enum class BridgeMode : uint8_t {
    WIFI_TCP = 0,
    BLE      = 1
};

struct BridgeConfig {
    // WiFi credentials
    String   wifiSSID;
    String   wifiPass;

    // Operating mode
    BridgeMode mode        = BridgeMode::WIFI_TCP;

    // Network
    uint16_t tcpPort       = 20220;
    String   mdnsName;            // e.g. "pypilot-bridge"
    uint8_t  wifiChannel   = 0;   // 0 = auto, 1-13 = fixed channel

    // Serial
    uint32_t baudRate      = 38400;
    bool     passthrough   = false;   // true = USB-UART passthrough

    // Debug
    bool     debugMode     = false;   // verbose logging, faster telemetry, raw monitor

    // State
    bool     configured    = false;
};

class ConfigManager {
public:
    void         begin();
    void         load();
    void         save();
    void         reset();           // wipe config → next boot enters setup
    bool         isConfigured() const { return _cfg.configured; }
    BridgeConfig&       config()       { return _cfg; }
    const BridgeConfig& config() const { return _cfg; }

private:
    BridgeConfig _cfg;
};

extern ConfigManager configManager;   // singleton
