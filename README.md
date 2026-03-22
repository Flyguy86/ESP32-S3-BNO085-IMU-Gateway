# 🧭 ESP32-S3 BNO085 Marine IMU Gateway

> **Version:** v1.5.0 — 2026-03-21

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
| `commsTask` | 0 | 1 | 4096 B | 40 Hz output | Formats JSON, sends via BLE + UDP + Signal K + SSE |
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

## Full System Setup & Operations

This device is one component in a four-layer autopilot chain. This section covers the complete system — from first-time setup through at-sea troubleshooting.

### System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  ESP32-S3 BNO085 (this device)                                  │
│  compass.local  •  40 Hz (25 ms commsTask cycle)                │
│  sensorTask 40 Hz Core 1 → commsTask 40 Hz Core 0              │
│  Signal K delta UDP broadcast → 255.255.255.255:10123  [40 Hz]  │
└───────────────────────┬─────────────────────────────────────────┘
                        │ UDP broadcast  [~1–5 ms WiFi hop]
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  Signal K Server  (signalk.service)                             │
│  http://localhost:3000   •  systemd auto-start                  │
│  Aggregates heading, GPS, wind, AIS, etc.                       │
│  Pushes heading to PyPilot via WebSocket  [25 ms period]        │
└───────────────────────┬─────────────────────────────────────────┘
                        │ WebSocket (zeroconf discovered)  [25 ms — signalk.period=0.025]
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  PyPilot Autopilot  (pypilot.service + pypilot-web.service)     │
│  /home/brian/pypilot  •  TCP port 20220  •  Web UI port 8080   │
│  6-term PID (P I D DD PR FF)  •  imu.source = signalk          │
│  PID loop 40 Hz  [25 ms — imu.rate=40]                         │
│  servo.period=0.1 — motor stiction windup window               │
│  Sends 4-byte motor commands to /dev/pypilot-servo              │
└───────────────────────┬─────────────────────────────────────────┘
                        │ writes to /dev/pypilot-servo (PTY)  [<1 ms]
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  socat  (pypilot-bridge.service)                                │
│  /dev/pypilot-servo  ←→  TCP → pypilot-bridge.local:20220      │
│  retry=2147483647  •  keepalive 10/5/3  •  StartLimitIntervalSec=0│
└───────────────────────┬─────────────────────────────────────────┘
                        │ TCP WiFi  [~2–5 ms]
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  ESP32-C3 Motor Bridge  (pypilot-serial-bridge firmware)        │
│  pypilot-bridge.local  •  TCP port 20220                        │
│  UART1 GPIO 4 TX / GPIO 5 RX  •  38400 baud  [~1 ms per pkt]  │
└───────────────────────┬─────────────────────────────────────────┘
                        │ UART 38400 baud  [~1 ms / 4-byte packet]
                        ▼
┌─────────────────────────────────────────────────────────────────┐
│  PyPilot Motor Controller (arduino_servo)                       │
│  CRC-8 verified 4-byte packets  •  current/voltage/rudder FB   │
└─────────────────────────────────────────────────────────────────┘

  Total end-to-end latency (typical): ~60 ms
  Dominant delays: SK WebSocket push 25 ms + PID cycle 25 ms

  Note: servo.period is the motor stiction windup window, NOT the PID rate.
  PID rate = 1/imu.rate = 1/40 = 25 ms. signalk.period = SK minPeriod = 25 ms.
```

> **System Status Dashboard:** `http://pypilotstatus.local:8083` — live health monitor
> for all layers, served by `pypilot-status.service`

---

### Services Reference

| Service | What it does | Config file |
|---------|-------------|-------------|
| `signalk.service` | Signal K server — aggregates all sensor data | `~/.signalk/settings.json` |
| `pypilot-bridge.service` | socat creates `/dev/pypilot-servo` PTY bridged over WiFi to ESP32-C3 | `/etc/systemd/system/pypilot-bridge.service` |
| `pypilot.service` | Main autopilot process — PID, heading control, servo output | `/etc/systemd/system/pypilot.service` |
| `pypilot-web.service` | Web UI for pypilot (port 8080) | `/etc/systemd/system/pypilot-web.service` |
| `pypilot-status.service` | Status monitor web server — `http://pypilotstatus.local:8083` | `/etc/systemd/system/pypilot-status.service` |

#### Useful service commands

```bash
# View status of all autopilot services at once
systemctl status signalk pypilot-bridge pypilot pypilot-web

# Restart a single service
systemctl restart pypilot-bridge.service

# Follow live log of a service
journalctl -u pypilot-bridge.service -f
journalctl -u pypilot.service -f

# Fix a service stuck in 'failed' state (systemd hit its restart limit)
systemctl reset-failed pypilot-bridge.service
systemctl start pypilot-bridge.service

# Restart everything in dependency order
systemctl restart signalk.service
sleep 3
systemctl restart pypilot-bridge.service
sleep 3
systemctl restart pypilot.service pypilot-web.service
```

---

### First-Time Setup Checklist

#### 1. Signal K UDP Input for BNO085

The ESP32-S3 broadcasts Signal K deltas over UDP. The Signal K server needs a matching UDP input on the same port.

**Step 1** — Check what port the compass is broadcasting on. Visit **http://compass.local** and look for "Signal K Port" in the dashboard table (default: `10110`, configurable).

**Step 2** — Open: **http://localhost:3000/admin/#/serverConfiguration/connections/-**  
If a `Signal K (UDP)` input already exists, note its port. If the port doesn't match the compass, either:
- Change the compass port: `http://compass.local/setport?port=EXISTING_SK_PORT`  
  *(e.g. if Signal K has a delta input on port 101022: `http://compass.local/setport?port=101022`)*
- Or add a new Signal K (UDP) input on the port the compass is using.

> **Note:** If port `10110` is already used by an **NMEA 0183** GPS input, you must use a different port for the Signal K delta feed. Use a port like `10111` or `101022` and point the compass there.

**Step 3** — Save & restart Signal K, then verify at **http://localhost:3000/admin/#/databrowser** — search `headingMagnetic` — should update live at ~40 Hz.

**Quick check from terminal:**
```bash
curl -s "http://localhost:3000/signalk/v1/api/vessels/self/navigation/headingMagnetic"
# Should return JSON with a recent timestamp and value in radians
```

#### 2. PyPilot IMU Source

Tell PyPilot to use Signal K (BNO085) instead of looking for a local RTIMU chip:

```bash
cd /home/brian/pypilot
# One-time run to write the persistent config value
python3 -c "
import sys, time
sys.path.insert(0, '.')
from pypilot.client import pypilotClient
c = pypilotClient('localhost')
c.set('imu.source', 'signalk')
time.sleep(1)
print('Done')
"
```

Verify with:
```bash
curl -s http://localhost:23322/  # then check imu.source in output, or use pypilot web UI
```

#### 3. PyPilot Serial Port

Ensure PyPilot only scans the socat virtual port (prevents slow startup scanning all USB devices):

```bash
mkdir -p ~/.pypilot
echo '/dev/pypilot-servo' > ~/.pypilot/serial_ports
```

#### 4. Install / Reinstall Service Files

```bash
cd pypilot-serial-bridge/pi-setup

# Install or update the bridge service
sudo cp pypilot-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable pypilot-bridge.service
sudo systemctl restart pypilot-bridge.service
```

---

### Status Web Monitor

**`pypilot-status.service`** runs a persistent live dashboard at `http://pypilotstatus.local:8083`
(port 8083). It checks all 6 layers every 10 seconds and shows per-layer status cards with
timing notes and direct links. The mDNS hostname `pypilotstatus.local` is registered via
`/etc/avahi/hosts` and `/etc/avahi/services/pypilot-status.service` (created by `setup-bridge.sh`).

```bash
# Check service status
systemctl status pypilot-status.service

# Follow logs
journalctl -u pypilot-status.service -f

# Restart
systemctl restart pypilot-status.service
```

JSON API for scripting: `http://pypilotstatus.local:8083/api/status`

### Diagnostic Script (CLI)

A manual CLI tool is also available at `pypilot-serial-bridge/pi-setup/check-pypilot.sh`
for terminal use when the web monitor is not accessible.

```bash
# Make executable once
chmod +x pypilot-serial-bridge/pi-setup/check-pypilot.sh

# Run diagnostic
./pypilot-serial-bridge/pi-setup/check-pypilot.sh

# Run diagnostic + attempt automatic fixes
./pypilot-serial-bridge/pi-setup/check-pypilot.sh --fix

# Restart all services then check
./pypilot-serial-bridge/pi-setup/check-pypilot.sh --restart-all
```

Example output when healthy:
```
━━━  LAYER 1 — Signal K Server
  ✓  signalk.service is running
  ✓  Signal K API responding on port 3000
  ✓  navigation.headingMagnetic = 247.3°  (2s ago)

━━━  LAYER 4 — socat Virtual Serial Port (/dev/pypilot-servo)
  ✓  pypilot-bridge.service is running
  ✓  /dev/pypilot-servo exists → /dev/pts/22
  ✓  /dev/pypilot-servo is writable by current user

━━━  LAYER 5 — PyPilot Autopilot
  ✓  pypilot.service is running
  ✓  PyPilot JSON server responding on TCP 23322
  ✓  PyPilot IMU source = signalk
```

---

### Known Failure Modes & Fixes

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| `/dev/pypilot-servo` missing | `pypilot-bridge.service` failed & hit systemd restart limit | `systemctl reset-failed pypilot-bridge.service && systemctl start pypilot-bridge.service` |
| Service shows 1000+ restarts then stops | Old service had `retry=10` — socat quits after 10 failed TCP attempts | Reinstall service file from repo (now uses `retry=2147483647`) |
| Heading stale in SignalK (>60s old) | ESP32-S3 lost WiFi or rebooted | Check `compass.local` reachable; check green LED pulse |
| Heading missing from Signal K entirely | Port mismatch — compass broadcasts to port X, SK listens on port Y | Run `./check-pypilot.sh` — it detects and shows exact fix URL |
| Heading missing from Signal K entirely | Port 10110 taken by NMEA GPS provider, SK can't parse Signal K JSON on same port | Add new SK delta UDP input on different port AND change compass: `http://compass.local/setport?port=NNNN` |
| PyPilot has no IMU data | `imu.source` not set to `signalk` | Run the one-liner above to set `imu.source=signalk` |
| autopilot engaged but rudder not moving | `/dev/pypilot-servo` exists but ESP32-C3 is offline → socat reads EOF | `nc -z pypilot-bridge.local 20220` to check TCP; restart bridge service |
| Motor controller not found by pypilot | `~/.pypilot/serial_ports` missing or wrong path | `echo '/dev/pypilot-servo' > ~/.pypilot/serial_ports` |
| Signal K not receiving UDP | UDP input not configured, or port mismatch | Run `./check-pypilot.sh` for exact diagnosis |

---

### Web UIs Quick Reference

| URL | What it shows |
|-----|--------------|
| `http://pypilotstatus.local:8083` | **Live system status dashboard** — all 6 layers, auto-refreshing |
| `http://compass.local` | ESP32-S3 BNO085 live heading dashboard |
| `http://pypilot-bridge.local` | ESP32-C3 motor controller diagnostics |
| `http://localhost:3000/admin/#/databrowser` | Signal K live data browser |
| `http://localhost:8080` | PyPilot autopilot web control |

---

## Configuration Reference

These are the actual files that must be set up correctly on the Raspberry Pi for the full system to work. They live outside this repo in the Pi's home directory.

---

### `~/.signalk/settings.json` — Signal K Server

The critical section is `pipedProviders`. Two providers must be active:

| ID | Type | Port | Purpose |
|----|------|------|---------|
| `SingalK_UDP_101022` | SignalK/UDP | **101022** | Receives compass heading delta from ESP32-S3 |
| `GPS-dietPi` | NMEA0183/UDP | 10110 | Receives NMEA GPS sentences from the system GPS |

> **Note:** Port 10110 is already used by the NMEA GPS — do **not** put the Signal K compass input on 10110. Use a separate port (101022 is the one currently configured). The `SingalK_UDP_101022` ID has a typo in the name; do not change the ID as it breaks the reference in settings.

Relevant `pipedProviders` entries:

```json
{
  "pipeElements": [{
    "type": "providers/simple",
    "options": {
      "logging": false,
      "type": "SignalK",
      "subOptions": {
        "type": "udp",
        "port": "101022",
        "selfHandling": "noSelf",
        "overrideTimestamp": true
      }
    }
  }],
  "id": "SingalK_UDP_101022",
  "enabled": true
},
{
  "pipeElements": [{
    "type": "providers/simple",
    "options": {
      "logging": true,
      "type": "NMEA0183",
      "subOptions": {
        "validateChecksum": true,
        "type": "udp",
        "host": "127.0.0.1",
        "port": "10110"
      }
    }
  }],
  "id": "GPS-dietPi",
  "enabled": true
}
```

To add the Signal K UDP input from scratch via the Admin UI:
- Go to `http://localhost:3000/admin/#/server/connections`
- Add connection → type: **Signal K (UDP)**, port: `101022`, self-handling: `noSelf`, override timestamp: on

---

### `~/.signalk/baseDeltas.json` — Vessel Identity

Contains the vessel name and dimensions used by Signal K. Edit via:
`http://localhost:3000/admin/#/vessel`

Current values:
```json
{ "name": "Erinistine", "mmsi": "none-yet",
  "design": { "length": { "overall": 11 }, "beam": 3.3, "draft": { "maximum": 2 }, "airHeight": 12 },
  "sensors": { "gps": { "fromBow": 6, "fromCenter": 1.5 } } }
```

---

### `~/.signalk/package.json` — Installed Plugins

These Signal K plugins must be installed (`npm install` is run automatically by the server):

| Package | Purpose |
|---------|---------|
| `pypilot-autopilot-provider` | Connects Signal K ↔ PyPilot (heading, AP commands) |
| `@signalk/freeboard-sk` | Chart plotter webapp |
| `@signalk/course-provider` | Course/routing support |
| `@mxtommy/kip` | Instrument panel dashboard |
| `@signalk/open-meteo-provider` | Weather data |
| `signalk-ais-target-prioritizer` | AIS filtering |

---

### `~/.signalk/plugin-config-data/pypilot-autopilot-provider.json`

```json
{
  "configuration": {
    "pypilot": { "host": "localhost", "port": 8000 }
  },
  "enabled": true,
  "enableLogging": false
}
```

This must point to PyPilot's web/JSON server (port 8000, same host). Configured via `http://localhost:3000/admin/#/plugin-config` → PyPilot Autopilot Provider.

---

### `~/.pypilot/pypilot.conf` — PyPilot Main Configuration

Key values that differ from defaults and must be preserved:

```ini
# IMU source — use Signal K instead of local hardware
imu.source="signalk"

# Signal K server location
signalk.host="localhost"
signalk.port=3000

# Signal K WebSocket minPeriod — SK will push heading no faster than this (seconds)
# 0.025 = 40 Hz, aligned with ESP32-S3 commsTask rate
signalk.period=0.0250

# Signal K auth token UID (auto-generated, must match signalk-token file)
signalk.uid="pypilot-41552038451"

# PID loop rate — how often the autopilot iteration runs (Hz)
# 40 Hz = 25 ms loop, aligned with signalk.period and ESP32-S3 output rate
imu.rate=40

# Autopilot mode and pilot
ap.mode="compass"
ap.pilot="basic"

# Servo parameters (tuned for this motor)
servo.max_current=2.1500
servo.max_controller_temp=60.0000
servo.max_motor_temp=60.0000
servo.position.p=0.1500
servo.position.i=0.0000
servo.position.d=0.0200

# Pilot gains (tuned profile "default")
[profile="default"]
# servo.period = motor stiction windup window (NOT PID rate — see imu.rate above)
# 0.1 = minimum allowed, tightest stiction response
servo.period=0.1000
ap.pilot.basic.P=0.0221
ap.pilot.basic.I=0.0000
ap.pilot.basic.D=0.0900
ap.pilot.basic.DD=0.0750
ap.pilot.basic.PR=0.0050
ap.pilot.basic.FF=0.6000
```

To set `imu.source` via CLI (one-time, writes to this file):
```bash
cd ~/pypilot && python3 -c "
from pypilot.client import pypilotClient
c = pypilotClient('localhost')
import time; time.sleep(1)
c.set('imu.source', 'signalk')
time.sleep(1); c.disconnect()
"
```

---

### `~/.pypilot/serial_ports` — Motor Controller Serial Device

```
/dev/pypilot-servo
```

This file tells PyPilot which serial device to probe for the motor controller. The `pypilot-bridge.service` creates `/dev/pypilot-servo` as a PTY linked via socat to the ESP32-C3 over TCP.

To recreate if missing:
```bash
echo '/dev/pypilot-servo' > ~/.pypilot/serial_ports
```

---

### `~/.pypilot/servodevice` — Last Known Servo Device

```
["/dev/pypilot-servo", 38400]
```

Auto-written by PyPilot after it successfully talks to the motor. Contains the device path and baud rate. If PyPilot fails to find the servo, delete this file to force a fresh probe:
```bash
rm ~/.pypilot/servodevice
```

---

### `~/.pypilot/pypilot_client.conf` — PyPilot Client Connection

```json
{"host": "localhost", "port": 20220}
```

> **Note:** This port (20220) is the ESP32-C3 TCP server port, **not** PyPilot's own server. This file is used by CLI client tools. PyPilot's own server listens on 23322.

---

### `~/.pypilot/web.conf` — PyPilot Web UI

```ini
server="127.0.0.1"
port=8000
```

PyPilot's web dashboard is at `http://localhost:8080` (mapped by service) or directly at port 8000 on localhost.

---

### `~/.pypilot/signalk-token` — Signal K Authentication Token

PyPilot uses a JWT token to authenticate with Signal K. This is auto-generated on first connection but must survive reboot. The current token is stored in `~/.pypilot/signalk-token`.

If PyPilot loses access to Signal K (e.g. after a Signal K security reset), delete this file and restart PyPilot — it will request a new token that you approve in the Signal K admin UI under **Security → Access Requests**.

---

## License

MIT