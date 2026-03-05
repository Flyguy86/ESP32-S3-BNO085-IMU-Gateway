# PyPilot Wireless Serial Bridge

A transparent WiFi TCP / BLE serial bridge for [PyPilot](https://github.com/pypilot/pypilot) autopilot systems, running on an **ESP32-C3** microcontroller. It replaces the physical wired serial connection between a PyPilot host (Raspberry Pi) and a PyPilot motor controller with a wireless link, while providing a rich diagnostic web UI.

**Firmware version:** `0.01`

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Pin Assignments](#pin-assignments)
- [Building & Flashing](#building--flashing)
- [First-Time Setup](#first-time-setup)
  - [WiFi Captive Portal](#wifi-captive-portal)
  - [BLE Configuration](#ble-configuration)
- [Operating Modes](#operating-modes)
- [Web Interface](#web-interface)
  - [Status Page](#status-page)
  - [Settings Page](#settings-page)
  - [Motor Console](#motor-console)
- [LED Status Indicators](#led-status-indicators)
- [Configuration Options](#configuration-options)
- [Network Services](#network-services)
- [Raspberry Pi Integration](#raspberry-pi-integration)
- [Python Utilities](#python-utilities)
- [PyPilot Protocol Reference](#pypilot-protocol-reference)
- [Debug Mode](#debug-mode)
- [Factory Reset](#factory-reset)
- [Troubleshooting](#troubleshooting)
- [Architecture](#architecture)

---

## Features

- **WiFi TCP Bridge** — Transparent bidirectional serial bridge over TCP (default port 20220)
- **BLE Bridge** — Alternative transport using Bluetooth Low Energy (Nordic UART Service)
- **Captive Portal Setup** — First-boot configuration via WiFi AP with DNS redirect (works on Android, iOS, Windows, macOS, Firefox)
- **BLE Configuration** — Alternative first-boot setup over BLE GATT (scriptable with included Python tool)
- **Web Dashboard** — Status page, settings page, and full motor diagnostic console
- **Motor Console** — Real-time telemetry gauges, motor controls, raw packet log, and diagnostic tools via WebSocket
- **mDNS / DNS-SD** — Zero-configuration networking (`pypilot-bridge.local`, `_pypilot._tcp` service advertisement)
- **Passthrough Mode** — Direct USB-CDC ↔ UART forwarding for bench testing with `motor_test.py`
- **Debug Mode** — Verbose logging, fast heartbeat (400 ms), 10 Hz telemetry, raw serial monitor
- **RGB LED Status** — WS2812B NeoPixel with animated states for boot, setup, connecting, bridging, and error
- **Factory Reset** — 3-second BOOT button hold wipes config and reboots into setup mode
- **Pi Deployment** — Included systemd service and setup script for `socat` virtual serial port on Raspberry Pi
- **PyPilot Protocol** — CRC-8 verified packet parsing with telemetry decoding (current, voltage, temperature, rudder, flags)

---

## Hardware Requirements

| Component | Details |
|---|---|
| **Microcontroller** | ESP32-C3 Super Mini "HW466AB" (QFN32, 4 MB flash, built-in USB) |
| **Motor Controller** | PyPilot motor driver (arduino_servo) |
| **Wiring** | 3 wires: TX → motor RX, RX ← motor TX, GND ↔ GND |
| **Power** | USB-C (from computer or charger) or external 3.3 V supply |
| **LED** | WS2812B NeoPixel (usually built into HW466AB on GPIO 8) |

### Wiring Diagram

```
  ESP32-C3 HW466AB              PyPilot Motor Controller
  ┌──────────────┐              ┌───────────────────────┐
  │   GPIO 4 (TX)├─────────────►│ RX                    │
  │   GPIO 5 (RX)│◄─────────────┤ TX                    │
  │          GND ├──────────────┤ GND                   │
  └──────────────┘              └───────────────────────┘
```

---

## Pin Assignments

| Pin | GPIO | Function |
|---|---|---|
| UART TX | 4 | Serial data to motor controller |
| UART RX | 5 | Serial data from motor controller (internal pull-up) |
| NeoPixel LED | 8 | WS2812B status indicator |
| BOOT Button | 9 | Factory reset (3 s hold); active-LOW with internal pull-up |
| USB | — | Built-in USB JTAG/serial CDC (monitor + passthrough) |

---

## Building & Flashing

### Prerequisites

- [PlatformIO](https://platformio.org/) (CLI or VS Code extension)
- USB-C cable

### Build & Upload

```bash
cd pypilot-serial-bridge

# Build
pio run

# Build & flash
pio run --target upload

# Open serial monitor
pio device monitor
```

### Build Configuration

| Setting | Value |
|---|---|
| Platform | `espressif32` |
| Board | `esp32-c3-devkitc-02` |
| Framework | Arduino |
| Flash | 4 MB, DIO |
| Partitions | `default.csv` |
| NimBLE ATT MTU | 247 |
| USB CDC | Enabled on boot |

### Libraries

| Library | Version |
|---|---|
| ESP Async WebServer | ^3.0.6 |
| NimBLE-Arduino | ^2.1.1 |
| ArduinoJson | ^7.0.0 |
| Adafruit NeoPixel | ^1.12.0 |

---

## First-Time Setup

On first boot (or after a factory reset), the bridge enters **Setup Mode** with dual-channel configuration: WiFi captive portal **and** BLE GATT. The LED breathes **blue** during setup.

### WiFi Captive Portal

1. Power on the ESP32-C3 — it creates a WiFi access point named **`PyPilot-Bridge-XXXX`** (last 4 hex digits of MAC)
2. Connect to this AP from your phone, tablet, or laptop
3. A captive portal page should open automatically (if not, browse to `http://192.168.4.1`)
4. The setup page offers:
   - **WiFi Network** — tap "Scan" to see nearby networks (with signal bars and encryption icons), or enter an SSID manually
   - **WiFi Password** — your network password
   - **Bridge Mode** — WiFi TCP (default) or BLE
   - **TCP Port** — default `20220`
   - **Baud Rate** — 9600 / 19200 / **38400** (default) / 57600 / 115200
   - **mDNS Hostname** — default `pypilot-bridge` (→ `pypilot-bridge.local`)
5. Click **Save & Reboot** — the device stores settings in NVS flash and reboots into bridge mode

> **Captive portal detection:** The firmware responds to probe URLs from Android, iOS/macOS, Windows, and Firefox to ensure the portal opens automatically.

### BLE Configuration

As an alternative (or if the captive portal is inconvenient), you can configure the bridge over BLE using the included Python script:

```bash
# Scan for nearby bridges
python pypilot_bridge_config.py --scan

# Configure with arguments
python pypilot_bridge_config.py --ssid "MyNetwork" --pass "secret" \
    --mode wifi --port 20220 --baud 38400 --hostname pypilot-bridge

# Interactive mode
python pypilot_bridge_config.py
```

Requires the `bleak` Python package (`pip install bleak`).

#### BLE Configuration Service UUIDs

| UUID | Characteristic | Properties |
|---|---|---|
| `deadbeef-0001-0000-...` | Config Service | — |
| `deadbeef-0001-0001-...` | SSID | READ, WRITE |
| `deadbeef-0001-0002-...` | Password | READ, WRITE |
| `deadbeef-0001-0003-...` | Mode (`"wifi"` / `"ble"`) | READ, WRITE |
| `deadbeef-0001-0004-...` | TCP Port | READ, WRITE |
| `deadbeef-0001-0005-...` | Baud Rate | READ, WRITE |
| `deadbeef-0001-0006-...` | Hostname | READ, WRITE |
| `deadbeef-0001-00ff-...` | Commit (write `0x01` → save & reboot) | WRITE |

---

## Operating Modes

### WiFi TCP Bridge (default)

The primary mode. The ESP32 connects to your WiFi network as a station and runs a TCP server. PyPilot (via `socat` on the Pi) connects to this server and data flows transparently between the TCP socket and the motor's UART.

**Data flow:**
```
Signal K → pypilot daemon → TCP socket → [WiFi] → ESP32 → UART → Motor Controller
Motor Controller → UART → ESP32 → [WiFi] → TCP socket → pypilot daemon → Signal K
```

### BLE Bridge

Uses the Nordic UART Service (NUS) instead of WiFi/TCP. Select this mode during setup if WiFi isn't available or for lower-power operation.

| UUID | Characteristic | Properties |
|---|---|---|
| `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | NUS Service | — |
| `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | RX (host → ESP) | WRITE, WRITE_NR |
| `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | TX (ESP → host) | NOTIFY |

### Passthrough Mode

Enable via Settings. The ESP32 directly forwards bytes between USB-CDC and UART1 with no protocol processing. WiFi and the web server remain active for settings access.

Useful for bench testing with `motor_test.py`:

```bash
python motor_test.py --port /dev/ttyACM0 --baud 38400
```

Commands: `d`=disengage, `c`=command (stop), `f`=forward, `r`=reverse, `x`=desync, `w`=wake, `q`=quit.

---

## Web Interface

All web pages are served from the ESP32's async web server (port 80). Available in WiFi bridge mode.

### Status Page (`/`)

The landing page shows:

- Firmware version
- Bridge mode (WiFi TCP / BLE)
- WiFi SSID and IP address
- mDNS hostname (clickable link)
- TCP port and baud rate
- UART TX/RX pin assignments
- System uptime
- Links to **Motor Console** and **Settings**
- **Factory Reset** button

### Settings Page (`/settings`)

Change any configuration option without reflashing:

- WiFi SSID & password (with network scanner)
- Bridge mode
- TCP port
- Baud rate
- mDNS hostname
- **Passthrough mode** toggle
- **Debug mode** toggle

Changes are saved to NVS and the device reboots.

### Motor Console (`/console`)

A full single-page diagnostic application with real-time WebSocket communication:

#### Telemetry Gauges
- Motor current (A)
- Supply voltage (V)
- Controller temperature (°C)
- Motor temperature (°C)
- Rudder position (raw ADC)
- Packet count and error count
- UART RX/TX byte counters

#### Motor Controls
- **Port / Starboard** — fast and slow jog buttons
- **Stop** — disengage motor
- **Slider** — direct command value (0–2000, center = 1000)

#### Diagnostic Tools
- Reset fault
- Set max current
- Clutch PWM & brake control
- Loopback test (UART self-test)
- GPIO RX test (scan GPIO pins for serial activity)
- Baud rate scanner
- Raw serial monitor toggle
- Raw hex packet sender (3 bytes, CRC auto-appended)

#### Packet Log
- Scrolling log with color-coded entries:
  - **Green** — RX packets from motor
  - **Orange** — TX packets to motor
  - **Gray** — info/system messages
- Timestamps on each entry
- WebSocket status indicator (green dot = connected, red = disconnected, auto-reconnect)

---

## LED Status Indicators

| Color / Pattern | State | Meaning |
|---|---|---|
| Red breathing (1 s) | `BOOT` | Starting up |
| Blue breathing (2 s) | `SETUP` | Setup mode — captive portal / BLE config active |
| Yellow breathing (2.5 s) | `WIFI_WAITING` | WiFi connected, waiting for TCP client |
| Solid green | `WIFI_BRIDGING` | TCP client connected, actively bridging |
| Cyan breathing (2.5 s) | `BLE_ADVERTISING` | BLE advertising, waiting for client |
| Solid cyan | `BLE_BRIDGING` | BLE client connected, actively bridging |
| Fast red blink (200 ms) | `ERROR` | Error condition |

LED brightness: 40/255.

---

## Configuration Options

All settings are persisted in NVS flash (namespace `ppbridge`) and survive power cycles.

| Setting | Default | NVS Key | Description |
|---|---|---|---|
| WiFi SSID | *(empty)* | `ssid` | Network name |
| WiFi Password | *(empty)* | `pass` | Network password |
| Bridge Mode | WiFi TCP | `mode` | `0` = WiFi TCP, `1` = BLE |
| TCP Port | `20220` | `tcpPort` | TCP server listen port |
| mDNS Hostname | `pypilot-bridge` | `mdns` | Hostname for `.local` resolution |
| Baud Rate | `38400` | `baud` | Motor UART baud rate |
| Passthrough | `false` | `passthru` | USB-CDC ↔ UART direct mode |
| Debug Mode | `false` | `debug` | Verbose logging & fast telemetry |
| Configured | `false` | `configured` | First-time setup completed flag |

---

## Network Services

When running in WiFi TCP mode, the bridge registers:

| Service | Port | Details |
|---|---|---|
| **mDNS** | — | Hostname `<mdnsName>.local` (default: `pypilot-bridge.local`) |
| **DNS-SD** `_pypilot._tcp` | 20220 | TXT records: `version`, `type=serial-bridge`, `baud`, `uart_tx`, `uart_rx`, `mode=wifi-tcp`, `hostname` |
| **DNS-SD** `_http._tcp` | 80 | Web interface |
| **TCP Server** | 20220 | Serial bridge data socket (one client at a time, TCP_NODELAY) |
| **HTTP Server** | 80 | Web UI (status, settings, console) |

### Discovering the Bridge

Use the included discovery script:

```bash
# Browse continuously
python pypilot_bridge_discover.py

# Find first device and exit
python pypilot_bridge_discover.py --once

# Discover and open test TCP session
python pypilot_bridge_discover.py --connect
```

Requires the `zeroconf` Python package (`pip install zeroconf`).

---

## Raspberry Pi Integration

PyPilot expects a local serial port device. The bridge creates the wireless link, and `socat` on the Pi creates a virtual serial port that tunnels to the ESP32 over TCP.

### Quick Setup

```bash
cd pi-setup
chmod +x setup-bridge.sh
sudo ./setup-bridge.sh
```

This script:

1. Installs `socat` (if not already installed)
2. Copies `pypilot-bridge.service` to `/etc/systemd/system/`
3. Enables and starts the service
4. Verifies `/dev/pypilot-servo` was created
5. Adds `/dev/pypilot-servo` to `~/.pypilot/serial_ports` so that pypilot auto-discovers it

### Manual Setup

```bash
# Install socat
sudo apt install socat

# Copy and enable service
sudo cp pi-setup/pypilot-bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pypilot-bridge.service

# Verify the virtual serial port exists
ls -la /dev/pypilot-servo

# Tell pypilot to use it
echo "/dev/pypilot-servo" >> ~/.pypilot/serial_ports
```

### How It Works

The systemd service runs:

```
socat pty,link=/dev/pypilot-servo,raw,echo=0,waitslave,unlink-close=0 \
      tcp:pypilot-bridge.local:20220,retry=forever,interval=5,keepalive,...
```

After `socat` creates the PTY, an `ExecStartPost` step waits for `/dev/pypilot-servo` to appear and runs `chmod 666` on it so that non-root users (e.g., the `pypilot` or `pi` user) can read/write the virtual serial port. Without this, `pypilot_servo` will repeatedly probe the port but fail to open it due to insufficient permissions.

This creates `/dev/pypilot-servo` as a virtual serial device. PyPilot opens this device as a regular serial port, and `socat` transparently tunnels all data to/from the ESP32 bridge over TCP.

The service auto-restarts on failure and retries the connection indefinitely, so it handles network interruptions and ESP32 reboots gracefully.

### Signal K Integration Chain

```
Signal K server
    └─► pypilot plugin (signalk-pypilot)
            └─► pypilot daemon (python)
                    └─► /dev/pypilot-servo (socat virtual port)
                            └─► TCP:20220 [WiFi]
                                    └─► ESP32-C3 bridge
                                            └─► UART @ 38400
                                                    └─► Motor Controller
```

---

## Python Utilities

| Script | Purpose | Dependencies |
|---|---|---|
| `pypilot_bridge_config.py` | Configure bridge over BLE (scan, interactive, or CLI args) | `bleak` |
| `pypilot_bridge_discover.py` | Discover bridge on network via mDNS/DNS-SD | `zeroconf` |
| `motor_test.py` | Direct motor controller testing via USB-serial passthrough | `pyserial` |

Install all dependencies:

```bash
pip install bleak zeroconf pyserial
```

---

## PyPilot Protocol Reference

The bridge implements the pypilot motor protocol — 4-byte binary packets with CRC-8 verification.

### Packet Format

```
┌──────┬──────────┬──────────┬──────┐
│ code │ value_lo │ value_hi │ CRC8 │
└──────┴──────────┴──────────┴──────┘
  1 byte   1 byte    1 byte   1 byte
```

### CRC-8

- Polynomial: x⁸ + x² + x + 1 (0x07)
- Init value: **0xFF**
- 256-byte lookup table (from pypilot `arduino_servo.cpp`)

### Command Codes (Host → Motor)

| Code | Hex | Description |
|---|---|---|
| COMMAND | `0xC7` | Motor position (value = 0–2000, 1000 = center) |
| DISENGAGE | `0x68` | Disengage motor |
| RESET | `0xE7` | Reset overcurrent/fault |
| MAX_CURRENT | `0x1E` | Set max current (10 mA units) |
| MAX_CONTROLLER_TEMP | `0xA4` | Set max controller temperature |
| MAX_MOTOR_TEMP | `0x5A` | Set max motor temperature |
| RUDDER_RANGE | `0xB6` | Set rudder range |
| MAX_SLEW | `0x71` | Set max slew rate |
| CLUTCH_PWM_BRAKE | `0x36` | Clutch PWM + brake control |

### Telemetry Codes (Motor → Host)

| Code | Hex | Description | Units |
|---|---|---|---|
| CURRENT | `0x1C` | Motor current | 10 mA steps → A |
| VOLTAGE | `0xB3` | Supply voltage | 10 mV steps → V |
| CONTROLLER_TEMP | `0xF9` | Controller temperature | hundredths °C |
| MOTOR_TEMP | `0x48` | Motor temperature | hundredths °C |
| RUDDER_SENSE | `0xA7` | Rudder position | raw ADC (0–65535) |
| FLAGS | `0x8F` | Status/fault flags | bitfield |

### Status Flags

| Flag | Bit | Description |
|---|---|---|
| SYNC | 0x0001 | Synchronized with host |
| OVERTEMP_FAULT | 0x0002 | Over-temperature fault |
| OVERCURRENT_FAULT | 0x0004 | Over-current fault |
| ENGAGED | 0x0008 | Motor engaged |
| INVALID | 0x0010 | Invalid state |
| PORT_PIN_FAULT | 0x0020 | Port pin fault |
| STARBOARD_PIN_FAULT | 0x0040 | Starboard pin fault |
| BADVOLTAGE_FAULT | 0x0080 | Bad voltage fault |
| MIN_RUDDER_FAULT | 0x0100 | Min rudder limit reached |
| MAX_RUDDER_FAULT | 0x0200 | Max rudder limit reached |
| CURRENT_RANGE | 0x0400 | Current out of range |
| BAD_FUSES | 0x0800 | Bad fuses detected |
| REBOOTED | 0x1000 | Controller rebooted |
| DRIVER_TIMEOUT | 0x2000 | Driver timeout |

---

## Debug Mode

Enable via the Settings page. Changes take effect after reboot.

| Feature | Normal Mode | Debug Mode |
|---|---|---|
| Heartbeat interval | 2000 ms | 400 ms |
| WebSocket telemetry | ~2 Hz | ~10 Hz |
| UART startup logging | Silent | Pin levels, config, RX idle state |
| Raw byte debug buffer | Disabled | Last 64 bytes captured for hex dump |
| Raw serial monitor | Disabled | TX/RX ring buffers (256 bytes each) |
| Motor init logging | Silent | Desync/wake details, RX byte count |

---

## Factory Reset

Two methods:

1. **Hardware:** Hold the **BOOT button** (GPIO 9) for **3 seconds** while the device is running. The LED flashes rapidly, NVS is wiped, and the device reboots into setup mode.

2. **Web UI:** Click the **Factory Reset** button on the status page (`/`), or send a POST to `/reset`.

---

## Troubleshooting

### Bridge won't connect to WiFi
- Verify SSID and password are correct (use the captive portal's WiFi scanner to confirm the network is visible)
- The ESP32-C3 supports **2.4 GHz only** — 5 GHz networks won't appear
- Check the serial monitor (`pio device monitor`) for connection error messages

### Motor not responding
- Verify wiring: TX→RX, RX←TX, GND↔GND (3 wires)
- Confirm baud rate matches the motor controller (default: 38400)
- Check the motor console's packet log for CRC errors
- Use the **Loopback Test** in the console to verify UART hardware
- Use the **GPIO RX Test** to confirm the correct RX pin is receiving data
- Try **Baud Scan** to auto-detect the motor's baud rate

### Can't reach web interface
- Browse to `http://pypilot-bridge.local` (requires mDNS support, e.g., Avahi on Linux)
- Alternatively, find the IP address from your router's DHCP table and access directly
- Use `pypilot_bridge_discover.py` to locate the bridge on the network

### socat / Pi virtual port not working
- Check service status: `sudo systemctl status pypilot-bridge`
- Verify the bridge is reachable: `ping pypilot-bridge.local`
- Check socat can connect: `socat - tcp:pypilot-bridge.local:20220` (should see binary motor data)
- Ensure `/dev/pypilot-servo` exists: `ls -la /dev/pypilot-servo`

### LED is fast red blink
- Error state — check serial monitor for details
- Try factory reset (hold BOOT button 3 seconds)

---

## Architecture

### Boot Flow

```
Power On
  │
  ├─ Init USB CDC (115200)
  ├─ Init NeoPixel (red breathing)
  ├─ Load NVS config
  │
  ├─ NOT configured? ──► Setup Mode
  │   ├─ Start WiFi AP ("PyPilot-Bridge-XXXX")
  │   ├─ Start DNS server (captive portal redirect)
  │   ├─ Start BLE config GATT service
  │   ├─ Start web server (setup form)
  │   └─ Wait for config submission → save NVS → reboot
  │
  └─ Configured? ──► Bridge Mode
      ├─ Start UART1 (GPIO 4/5 @ configured baud)
      ├─ Passthrough? → Start passthroughSerialTask
      │   Otherwise  → Start serialTask (protocol + heartbeat)
      ├─ WiFi mode? → Start bridgeWifiTask (STA + TCP server + mDNS)
      │   BLE mode? → Start bridgeBleTask (NUS)
      ├─ Start web server (status + settings + console)
      ├─ Start LED status task
      └─ Start button monitor task
```

### FreeRTOS Tasks

| Task | Stack | Priority | Purpose |
|---|---|---|---|
| `serial` | 4096 | 2 (highest) | UART poll, motor init, heartbeat |
| `passthru` | 4096 | 2 | USB ↔ UART direct forwarding |
| `wifi_br` | 8192 | 1 | WiFi STA + TCP data pump |
| `ble_br` | 8192 | 1 | BLE NUS data pump |
| `led` | 2048 | 0 | NeoPixel animations (~50 fps) |
| `btn` | 2048 | 0 | BOOT button monitor (50 ms poll) |

### Source Files

| File | Purpose |
|---|---|
| `config.h` | Compile-time constants: pins, UUIDs, defaults, task config |
| `config_manager.h/.cpp` | NVS-backed persistent config (ESP32 Preferences API) |
| `main.cpp` | Boot flow, web pages, button monitor, task orchestration |
| `bridge_wifi.h/.cpp` | WiFi STA, TCP server, mDNS/DNS-SD, TCP↔UART data pump |
| `bridge_ble.h/.cpp` | BLE NUS service, NimBLE callbacks, BLE↔UART data pump |
| `serial_bridge.h/.cpp` | UART1 driver, ring buffer, serial tasks, diagnostics |
| `setup_mode.h/.cpp` | First-boot config: WiFi AP captive portal + BLE GATT |
| `led_status.h/.cpp` | NeoPixel state-machine animations |
| `pypilot_proto.h/.cpp` | CRC-8 table, packet builder, telemetry decoder |
| `web_console.h/.cpp` | Motor console SPA, WebSocket handler, diagnostic tools |
| `main_passthrough.cpp` | Compile-time minimal USB↔UART passthrough |

---

## License

This project is provided as-is for use with [PyPilot](https://github.com/pypilot/pypilot) open-source autopilot systems.
