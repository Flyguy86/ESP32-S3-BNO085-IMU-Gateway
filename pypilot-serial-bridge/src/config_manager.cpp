// =============================================================================
// config_manager.cpp — NVS read / write via Preferences
// =============================================================================
#include "config_manager.h"
#include "config.h"
#include <Preferences.h>

static Preferences prefs;
static const char* NS = "ppbridge";   // NVS namespace (max 15 chars)

ConfigManager configManager;          // singleton instance

// ---------------------------------------------------------------------------
void ConfigManager::begin() {
    prefs.begin(NS, /*readOnly=*/false);
    load();
}

// ---------------------------------------------------------------------------
void ConfigManager::load() {
    _cfg.configured = prefs.getBool("configured", false);
    _cfg.wifiSSID   = prefs.getString("ssid",     "");
    _cfg.wifiPass   = prefs.getString("pass",     "");
    _cfg.mode       = static_cast<BridgeMode>(prefs.getUChar("mode", 0));
    _cfg.tcpPort    = prefs.getUShort("tcpPort",  DEFAULT_TCP_PORT);
    _cfg.mdnsName   = prefs.getString("mdns",     DEFAULT_MDNS_NAME);
    _cfg.baudRate   = prefs.getULong("baud",      MOTOR_BAUD_DEFAULT);
    _cfg.wifiChannel= prefs.getUChar("wifichan",  0);
    _cfg.passthrough= prefs.getBool("passthru",   false);
    _cfg.debugMode  = prefs.getBool("debug",      false);
}

// ---------------------------------------------------------------------------
void ConfigManager::save() {
    prefs.putBool   ("configured", _cfg.configured);
    prefs.putString ("ssid",       _cfg.wifiSSID);
    prefs.putString ("pass",       _cfg.wifiPass);
    prefs.putUChar  ("mode",       static_cast<uint8_t>(_cfg.mode));
    prefs.putUShort ("tcpPort",    _cfg.tcpPort);
    prefs.putString ("mdns",       _cfg.mdnsName);
    prefs.putULong  ("baud",       _cfg.baudRate);
    prefs.putUChar  ("wifichan",   _cfg.wifiChannel);
    prefs.putBool   ("passthru",   _cfg.passthrough);
    prefs.putBool   ("debug",      _cfg.debugMode);
}

// ---------------------------------------------------------------------------
void ConfigManager::reset() {
    prefs.clear();
    _cfg = BridgeConfig{};           // defaults
    Serial.println("[CFG] Config reset — rebooting into setup mode");
}
