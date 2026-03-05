// =============================================================================
// setup_mode.h — First-boot configuration (WiFi captive portal + BLE config)
// =============================================================================
#pragma once

/// Enter setup mode — blocks until configuration is committed and the
/// device reboots.  Call from setup() when configManager.isConfigured()==false.
void setupModeRun();
