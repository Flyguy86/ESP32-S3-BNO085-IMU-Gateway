// =============================================================================
// config.h — Hardware pins, defaults, and compile-time constants
// =============================================================================
#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Firmware version
// ---------------------------------------------------------------------------
#define FW_VERSION "0.01"

// ---------------------------------------------------------------------------
// Hardware – UART to motor driver (UART1)
// ---------------------------------------------------------------------------
#define UART_TX_PIN       4         // ESP32-C3 GPIO 4 → motor driver RX
#define UART_RX_PIN       5         // ESP32-C3 GPIO 5 ← motor driver TX
#define UART_NUM          1         // UART1 via GPIO matrix
#define MOTOR_BAUD_DEFAULT 38400    // PyPilot motor-driver default baud

// ---------------------------------------------------------------------------
// Hardware – NeoPixel status LED (WS2812B on ESP32-C3 devkit / HW466AB)
// ---------------------------------------------------------------------------
#define RGB_LED_PIN       8
#define RGB_LED_COUNT     1

// ---------------------------------------------------------------------------
// Hardware – BOOT button (used for factory-reset)
// ---------------------------------------------------------------------------
#define BOOT_BUTTON_PIN   9
#define RESET_HOLD_MS     3000      // 3-second long-press to reset config

// ---------------------------------------------------------------------------
// WiFi defaults
// ---------------------------------------------------------------------------
#define AP_NAME_PREFIX     "PyPilot-Bridge-"   // + last 4 hex of MAC
#define DEFAULT_MDNS_NAME  "pypilot-bridge"    // → pypilot-bridge.local
#define DEFAULT_TCP_PORT   20220               // TCP server listen port

// ---------------------------------------------------------------------------
// mDNS / DNS-SD service types
// ---------------------------------------------------------------------------
#define PYPILOT_SERVICE_TYPE  "pypilot"         // → _pypilot._tcp.local.
#define HTTP_SERVICE_TYPE     "http"            // → _http._tcp.local.

// ---------------------------------------------------------------------------
// BLE
// ---------------------------------------------------------------------------
//  Nordic UART Service (NUS) — bridge mode
#define NUS_SERVICE_UUID   "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX_CHAR_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // write (host→ESP)
#define NUS_TX_CHAR_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // notify (ESP→host)

//  Configuration service — setup mode
#define CFG_SERVICE_UUID   "deadbeef-0001-0000-0000-000000000000"
#define CFG_SSID_UUID      "deadbeef-0001-0001-0000-000000000000"
#define CFG_PASS_UUID      "deadbeef-0001-0002-0000-000000000000"
#define CFG_MODE_UUID      "deadbeef-0001-0003-0000-000000000000"  // "wifi" or "ble"
#define CFG_PORT_UUID      "deadbeef-0001-0004-0000-000000000000"
#define CFG_BAUD_UUID      "deadbeef-0001-0005-0000-000000000000"
#define CFG_HOSTNAME_UUID  "deadbeef-0001-0006-0000-000000000000"
#define CFG_COMMIT_UUID    "deadbeef-0001-00ff-0000-000000000000"  // write 1 → save+reboot

#define BLE_DEVICE_PREFIX  "PyPilot-Bridge"    // BLE advertising device name

// ---------------------------------------------------------------------------
// Ring-buffer sizes (bytes, each direction)
// ---------------------------------------------------------------------------
#define RING_BUF_SIZE      2048

// ---------------------------------------------------------------------------
// FreeRTOS task stack sizes & priorities
// ---------------------------------------------------------------------------
#define SERIAL_TASK_STACK   4096
#define BRIDGE_TASK_STACK   8192
#define LED_TASK_STACK      2048
#define SETUP_TASK_STACK    8192

#define SERIAL_TASK_PRIO    2   // highest, time-critical
#define BRIDGE_TASK_PRIO    1
#define LED_TASK_PRIO       0   // lowest
