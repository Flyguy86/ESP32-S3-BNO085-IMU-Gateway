// =============================================================================
// led_status.h — NeoPixel status indicator
// =============================================================================
#pragma once

#include <cstdint>

enum class LedState : uint8_t {
    OFF,
    BOOT,              // red pulse
    SETUP,             // pulsing blue
    WIFI_WAITING,      // breathing yellow — connected to WiFi, no client
    WIFI_BRIDGING,     // solid green — TCP client connected
    BLE_ADVERTISING,   // breathing cyan
    BLE_BRIDGING,      // solid cyan
    ERROR              // fast red blink
};

void ledStatusBegin();
void ledStatusSet(LedState state);
void ledStatusTask(void* param);   // FreeRTOS task entry
