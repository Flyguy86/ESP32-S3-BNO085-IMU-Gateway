# 🧭 ESP32-S3 BNO085 Marine IMU Gateway

A high-stability 9-DOF orientation system designed for **marine autopilot** compass heading. Bridges high-fidelity BNO085 sensor fusion data over **BLE**, **WiFi** (UDP + Web Dashboard), and **Signal K** (delta JSON). Built for the ESP32-S3-N16R8 with a dual-core architecture that separates critical sensor sampling from wireless communication.

> **Primary Goal:** Provide a "fit and forget" heading sensor for boats where router reliability might be an issue. By combining the precision of the BNO085 with the dual-radio capabilities of the ESP32-S3, heading data reaches the helm even if the primary WiFi network fails.

---

## Features

- **Dual-Core Execution**
  - Core 1: Dedicated high-speed I2C polling and BNO085 data processing
  - Core 0: WiFi stack, Async Web Server, BLE advertising, and LED animation
- **BNO085 Hardware Fusion** — Hillcrest Labs SH-2 Kalman filtering for maximum heading stability
- **Signal K Integration** — Native Signal K delta JSON broadcast over UDP (no NMEA translation needed)
- **WiFi Captive Portal** — ESPAsyncWiFiManager creates an AP (`S3_IMU_GATEWAY`) for first-time WiFi setup
- **mDNS Discovery** — Access the dashboard at [http://s3imu.local](http://s3imu.local) — no need to find the IP
- **Hybrid Failover Connectivity**
  - Primary: WiFi UDP broadcast (JSON on port 4210 + Signal K delta on configurable port)
  - Always-on: BLE notifications (GATT)
  - Web: Real-time dashboard with Server-Sent Events
- **Configurable Signal K Port** — Change the Signal K UDP port from the web dashboard; saved to NVS flash
- **RGB LED Status Indicators** — NeoPixel on GPIO 48 with slow green breathing pulse during normal operation
- **Calibration Persistence** — Save BNO085 magnetometer calibration to internal flash via web button or `/save` endpoint

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S3-N16R8 (QFN56, rev v0.2) |
| Flash / PSRAM | 16 MB / 8 MB |
| IMU | BNO085 (9-DOF, I2C) |
| LED | Built-in NeoPixel RGB on GPIO 48 |
| USB | USB-Serial/JTAG on `/dev/ttyACM0` |

### Wiring

| BNO085 Pin | ESP32-S3 Pin | Function |
|------------|-------------|----------|
| VCC | 3V3 | Power (3.3V) |
| GND | GND | Common Ground |
| SDA | **GPIO 8** | I2C Data |
| SCL | **GPIO 9** | I2C Clock |
| PS0 / PS1 | GND | I2C mode select |

> **Note:** I2C runs at 400 kHz. The pin assignment (GPIO 8/9) supports a direct "sandwich" solder between the ESP32-S3 devkit and the BNO085 breakout board.

---

## Signal K Integration

The device broadcasts **Signal K delta JSON** via UDP broadcast. Signal K server picks this up natively — no NMEA translation or protocol bridges needed.

### Setup in Signal K Server

1. Go to **Server → Data Connections**
2. Click **Add** → set type to **"Signal K (Delta) over UDP"**
3. Set port to **10110** (default, configurable via web UI)
4. Save & restart the Signal K server

### Signal K Data Paths

| Signal K Path | Value | Unit | Description |
|---------------|-------|------|-------------|
| `navigation.headingMagnetic` | 0–6.28 | radians | Compass heading |
| `navigation.rateOfTurn` | ±n | rad/s | Rate of turn (autopilot PID input) |
| `navigation.attitude` | `{roll, pitch, yaw}` | radians | Full vessel attitude |
| `sensors.imu.compassCalibration` | 0.00–1.00 | ratio | Magnetometer cal status (1.0 = fully calibrated) |
| `sensors.imu.fusionCalibration` | 0.00–1.00 | ratio | Quaternion fusion accuracy (1.0 = highest) |

### Example Delta Broadcast

```json
{
  "updates": [{
    "source": {"label": "BNO085", "type": "IMU"},
    "values": [
      {"path": "navigation.headingMagnetic", "value": 1.415462},
      {"path": "navigation.rateOfTurn", "value": 0.015621},
      {"path": "navigation.attitude", "value": {"roll": 0.020944, "pitch": -0.043633, "yaw": 1.415462}},
      {"path": "sensors.imu.compassCalibration", "value": 0.67},
      {"path": "sensors.imu.fusionCalibration", "value": 1.00}
    ]
  }]
}
```

### Configurable Port

The Signal K UDP port defaults to **10110** and can be changed at any time from the web dashboard at `http://s3imu.local`. The setting is saved to NVS flash and survives reboots. You can also set it via: `GET http://s3imu.local/setport?port=10111`

---

## Web Dashboard

Access at **http://s3imu.local** (or by IP address). The dashboard shows:

- Real-time compass heading (large display)
- Pitch, roll, and rate of turn
- Quaternion and magnetometer accuracy (0–3 scale)
- Signal K UDP port configuration
- Save calibration button
- Reset WiFi credentials button

---

## WiFi Setup (Captive Portal)

On first boot (or after WiFi reset), the device creates a WiFi access point:

1. Connect to AP **`S3_IMU_GATEWAY`** from your phone/laptop
2. A captive portal opens automatically (or navigate to `http://192.168.4.1`)
3. Select your boat's WiFi network and enter the password
4. The device saves credentials and auto-connects on future boots

The portal times out after **3 minutes** and falls back to BLE-only mode if no WiFi is configured.

To clear saved WiFi credentials: visit `http://s3imu.local/resetwifi` or press the "RESET WIFI" button on the dashboard.

---

## Data Streams

### JSON Payload (BLE + UDP port 4210)

```json
{
  "h": 327.4,
  "p": -2.5,
  "r": 1.2,
  "rot": -0.12,
  "gx": 0.50,
  "gy": -0.20,
  "gz": -0.12,
  "lax": 0.013,
  "lay": -0.031,
  "laz": 0.024,
  "qacc": 2,
  "macc": 1
}
```

| Field | Unit | Source | Description |
|-------|------|--------|-------------|
| `h` | degrees | Rotation Vector | Compass heading (0–360°) |
| `p` | degrees | Rotation Vector | Pitch |
| `r` | degrees | Rotation Vector | Roll |
| `rot` | °/s | Gyroscope Z | Rate of turn (critical for autopilot PID) |
| `gx/gy/gz` | °/s | Gyroscope | Angular velocity X/Y/Z |
| `lax/lay/laz` | m/s² | Linear Accelerometer | Motion without gravity (sea state, impacts) |
| `qacc` | 0–3 | Rotation Vector | Quaternion accuracy (3 = highest) |
| `macc` | 0–3 | Magnetometer | Magnetometer calibration status (3 = highest) |

### Output Rate

- **Sensor sampling:** 10 Hz (100 ms reports for rotation vector, gyro, linear accel)
- **Magnetometer:** 2 Hz (500 ms, separate report for calibration tracking)
- **BLE/UDP/Signal K broadcast:** 2 Hz (500 ms)

---

## LED Status Codes

| Color | Pattern | Meaning |
|-------|---------|---------|
| White | Brief flash | Boot |
| Blue | Solid | WiFi captive portal active |
| Yellow | Flashing | BNO085 initializing |
| Red | Flashing (forever) | BNO085 not detected — check wiring |
| Cyan | Flashing | BLE advertising started |
| Green | Slow pulse (3s up, 5s down) | All systems running normally |

---

## BLE Interface

| Property | Value |
|----------|-------|
| Device Name | `S3_IMU_GATEWAY` |
| Service UUID | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Characteristic UUID | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Properties | NOTIFY |

### Python BLE Reader

A companion `ble_reader.py` script is included for receiving data on a laptop:

```bash
# Create virtual environment and install dependencies
python3 -m venv .venv
source .venv/bin/activate
pip install bleak

# Run the reader
python ble_reader.py
```

Example output:
```
HDG:  327.4°  P:   -2.5°  R:    1.2°  ROT:   -0.12°/s  Accel: [ 0.01,-0.03, 0.02]  Q:2 M:1
```

---

## Web API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web dashboard (HTML + SSE) |
| `/events` | GET | Server-Sent Events stream (JSON) |
| `/save` | GET | Save BNO085 calibration to flash |
| `/setport?port=N` | GET | Set Signal K UDP port (1024–65535, saved to NVS) |
| `/resetwifi` | GET | Clear WiFi credentials and reboot |

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-S3-N16R8                        │
│                                                         │
│  ┌──────────────┐         ┌──────────────────────────┐  │
│  │  Core 1      │         │  Core 0                  │  │
│  │              │         │                          │  │
│  │  sensorTask  │ ──────► │  commsTask               │  │
│  │  (priority 2)│  shared │  (priority 1)            │  │
│  │              │  struct │  ├─ BLE Notify            │  │
│  │  BNO085 I2C  │         │  ├─ WiFi UDP :4210       │  │
│  │  10Hz poll   │         │  ├─ Signal K :10110      │  │
│  │  while-drain │         │  └─ SSE /events          │  │
│  └──────────────┘         │                          │  │
│                           │  ledTask                 │  │
│                           │  (priority 0)            │  │
│                           │  └─ Green pulse          │  │
│                           └──────────────────────────┘  │
│                                                         │
│  BNO085 ◄──── I2C 400kHz ────► GPIO 8 (SDA)           │
│                                 GPIO 9 (SCL)           │
│  NeoPixel ◄────────────────── GPIO 48                  │
└─────────────────────────────────────────────────────────┘
```

### Task Details

| Task | Core | Priority | Stack | Rate | Purpose |
|------|------|----------|-------|------|---------|
| `sensorTask` | 1 | 2 | 4096 B | 10 Hz polling | I2C reads from BNO085, updates shared `IMUData` struct |
| `commsTask` | 0 | 1 | 4096 B | 2 Hz output | Formats JSON, sends via BLE + UDP + Signal K + SSE |
| `ledTask` | 0 | 0 | 2048 B | continuous | Non-blocking green pulse animation |

---

## Building & Flashing

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB-C cable to ESP32-S3

### Build & Upload

```bash
# Build and flash
pio run -t upload

# Monitor serial output
pio device monitor --baud 115200
```

### Platform Configuration

The project uses the `esp32s3box` board definition with 16 MB partition table:

```ini
[env:esp32s3-devkitc-1]
platform = espressif32
board = esp32s3box
framework = arduino
monitor_speed = 115200
board_build.flash_size = 16MB
board_build.psram_size = 8MB
board_build.partitions = default_16MB.csv
```

### Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| [ESPAsyncWebServer](https://github.com/mathieucarbou/ESPAsyncWebServer) | 3.6.0 | Async HTTP + SSE |
| [ESPAsyncWiFiManager](https://github.com/alanswx/ESPAsyncWiFiManager) | 0.31 | Captive portal WiFi config |
| [SparkFun BNO080](https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library) | 1.1.12 | BNO085 I2C driver |
| AsyncTCP | 3.3.2 | TCP transport for web server |
| ESP32 BLE Arduino | 2.0.0 | BLE GATT server |
| Preferences | built-in | NVS flash storage for settings |
| ESPmDNS | built-in | mDNS for `s3imu.local` discovery |

---

## Lessons Learned

These are notes from the development process that may save time if you're building something similar:

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| `ESPAsyncWebServer` + `WiFiManager` HTTP enum conflicts | Both define `HTTP_GET`, `HTTP_DELETE`, etc. | Use `ESPAsyncWiFiManager` (alanswx fork) instead of tzapu/WiFiManager |
| Flash overflow (>100% of 1310 KB) | Default 4 MB partition on a 16 MB chip | Add `board_build.partitions = default_16MB.csv` |
| Signal K not receiving pitch/roll via NMEA | Signal K's nmea0183-signalk parser has **no XDR handler** | Switch to Signal K delta JSON over UDP — all paths work natively |
| WiFiManager takes over server routes | `autoConnect()` registers its own handlers | Call `server.reset()` after WiFiManager before registering dashboard routes |
| LED animation blocking data transmission | LED pulse loop ran in `commsTask` (8s cycle) | Move LED to separate `ledTask` FreeRTOS task on Core 0 |
| `getMagAccuracy()` always returns 0 | Only updates from `MAGNETIC_FIELD` report, not `ROTATION_VECTOR` | Use `getQuatAccuracy()` for rotation vector; enable magnetometer separately for `getMagAccuracy()` |
| `getLinAccelX/Y/Z()` always returns 0 | BNO085 silently drops `enableLinearAccelerometer()` if sent immediately after other enables | Add `delay(50)` between each `enable*()` call |
| Only one sensor report processed per loop | `dataAvailable()` reads one I2C packet per call | Use `while (dataAvailable())` to drain all pending reports |
| Serial output not visible on boot | USB-Serial/JTAG needs time to enumerate | Add `delay(2000)` after `Serial.begin()` |

---

## File Structure

```
├── src/
│   └── main.cpp          # Firmware: dual-core IMU gateway
├── ble_reader.py          # Python BLE client (bleak)
├── platformio.ini         # PlatformIO project config
├── CMakeLists.txt         # ESP-IDF cmake (optional)
└── README.md
```

---

## License

MIT