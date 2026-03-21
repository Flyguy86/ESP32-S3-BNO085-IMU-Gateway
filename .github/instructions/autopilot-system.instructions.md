---
applyTo: "**"
---

# PyPilot Marine Autopilot System — Copilot Instructions

## Project Overview

This workspace contains firmware and Pi-side software for a **full marine autopilot system**:

- **ESP32-S3** (`src/`) — BNO085 9-axis compass/IMU, broadcasts 40 Hz Signal K delta UDP
- **ESP32-C3** (`pypilot-serial-bridge/src/`) — TCP-to-UART motor bridge
- **Pi-side scripts** (`pypilot-serial-bridge/pi-setup/`) — systemd services, diagnostic tools, status web server

---

## Primary Engineering Goals

**Every change must be evaluated against these two goals:**

### 1. Reduce Latency
The full chain currently has **~60 ms end-to-end latency** (tuned 2026-03-21).

**Latency breakdown:**

| Stage | Latency | Config |
|---|---|---|
| ESP32-S3 WiFi hop | ~2 ms | commsTask 40 Hz |
| Signal K WebSocket push to PyPilot | **25 ms** | `signalk.period = 0.025` |
| PyPilot PID cycle | **25 ms** | `imu.rate = 40` |
| PTY + socat + TCP WiFi + UART | ~8 ms | servo path |
| **Total** | **~60 ms** | |

**Critical clarification — `servo.period` vs `imu.rate`:**
- `imu.rate` = PID loop frequency. `imu.rate=40` → PID runs every 25 ms.
- `servo.period` = motor stiction windup window in `servo.py` — controls how long the motor can slip before forcing a minimum-speed nudge. It does **not** control PID rate.
- `signalk.period` = `minPeriod` in pypilot's WebSocket subscription to Signal K — Signal K will not push heading faster than this value.

**Never confuse `servo.period` with PID rate. They are unrelated.**

When making changes, always ask:
- Does this shorten `imu.rate` or `signalk.period`?
- Does it eliminate an unnecessary hop or buffer?
- Does it reduce heading age by the time it reaches the motor?

Do **not** add delays, extra processing, or polling loops that increase time-to-motor.

### 2. Maximise Stability
The system must keep running at sea with no manual intervention. Stability means:
- **Services never give up** — all systemd units must use `Restart=always` + `StartLimitIntervalSec=0`
- **socat never stops retrying** — `retry=forever` is mandatory in `pypilot-bridge.service`
- **WiFi loss is tolerated** — firmware must reconnect automatically without needing a reset
- **No silent failures** — if a layer stops, `pypilot-status.service` must detect and display it
- **No single point of failure in software** — a crashed service restarts in ≤5 s

Prefer solutions that fail gracefully and self-heal over those that require intervention.

---

## Hardware & Network Resource Budget

**Before increasing any rate or adding any new data path, verify it fits within these ceilings.**
Resource exhaustion causes subtle instability — not immediate crashes — making it hard to diagnose at sea.

### I2C Bus (BNO085 ↔ ESP32-S3)
- Speed: 400 kHz (fast mode). Each SHTP report packet ≈23 bytes → **~0.6 ms per report read.**
- 4 reports enabled (game rotation, rotation vector, gyro, linear accel): peak load ≈ 2.4 ms per sensorTask cycle.
- **Current utilisation at 40 Hz: ~24% (2.4 ms of 10 ms poll window). Safe ceiling: ~200 Hz with 4 reports (48% bus load at 5 ms window). Do not exceed.**
- **BNO085 hardware max rates:** Game/Rotation Vector & Gyro: 400 Hz. Linear Accel: 500 Hz. Magnetometer: 100 Hz. The I2C bus with 4 reports is the real ceiling, not the chip.
- **I2C timeout & recovery (implemented):** `Wire.setTimeOut(100)` is set at boot so `Wire.requestFrom()` aborts after 100 ms if the BNO085 holds the bus (instead of blocking forever). `sensorTask` also tracks `lastGoodData`; if no valid SHTP packet arrives for 500 ms, it performs a full `Wire.end()` / `Wire.begin()` / `myIMU.begin()` / `initIMUReports()` cycle and retries. A spontaneous BNO085 reset (brown-out, power glitch) is detected via `myIMU.hasReset()` and triggers `initIMUReports()` without touching the Wire bus.
- Rule: do NOT add more BNO085 report types without removing an existing one or confirming I2C headroom.

### CPU — ESP32-S3 @ 240 MHz, dual-core
- **Core 1 (sensorTask):** I2C read + quaternion math ≈ ~2.5 ms active per 10 ms poll cycle → **~25% Core 1 utilisation.**
- **Core 0 (commsTask):** snprintf + 2× UDP send + BLE notify + SSE ≈ ~1 ms active per 25 ms cycle → **~4% Core 0 utilisation.**
- **Safe ceiling: commsTask is not CPU-limited until ~250 Hz.**
- Rule: never block Core 0 or Core 1 with a synchronous network call, sleep, or `while(1)` without a FreeRTOS yield — it starves the other tasks on that core.

### WiFi / UDP Bandwidth
- Signal K delta payload ≈ 350 bytes + diagnostic payload ≈ 200 bytes = **~550 bytes per commsTask cycle.**
- At 40 Hz: **~176 kbps of actual data.** 802.11n minimum PHY rate is 6.5 Mbps → <3% of minimum WiFi capacity.
- **Bandwidth is not the risk.** The risk is **802.11 CSMA/CA contention** (DIFS + random backoff per packet):
  - On a quiet home/boat network: backoff ≈ 1–2 ms/packet → negligible.
  - On a crowded marina with many APs: backoff can reach 5–15 ms/packet. At 40 Hz, commsTask will occasionally miss its 25 ms window and the effective UDP rate will drop.
  - Signal K UDP is fire-and-forget — **dropped packets cause momentary heading gaps, not crashes.** This is acceptable graceful degradation.
- Rule: do NOT increase commsTask rate above **100 Hz**. At 100 Hz (10 ms budget), WiFi backoff alone could consume the entire cycle window on a congested network, causing commsTask to fall permanently behind and the WDT to eventually trigger.

### BLE Notify Rate
- iOS default connection interval ≈ 30 ms; Android ≈ 45–60 ms.
- At 40 Hz (25 ms), we send faster than the BLE CI. The ESP32 BLE stack queues notifications (queue depth ≈ 20). Notifications will be delivered in bursts at the connection interval.
- BLE is **not in the critical autopilot path** — Signal K UDP provides heading to PyPilot. BLE drops or delays do not affect autopilot.
- Rule: BLE notify rate can follow commsTask rate freely up to 100 Hz. Do NOT add blocking waits for BLE ack.

### radioMutex (commsTask contention)
- `commsTask` holds `radioMutex` for the entire send cycle (UDP × 2 + BLE + SSE). On a congested WiFi network, `udp.endPacket()` can block for the duration of WiFi backoff.
- If backoff extends the mutex hold time beyond the commsTask period, commsTask falls behind but **does not crash** — `vTaskDelay` still fires after mutex release and the task self-corrects.
- Rule: do NOT add additional blocking I/O inside the `radioMutex` critical section.

---

## ⚠️ MANDATORY: Review Architecture on Every Change

**Before and after every code change, update the system architecture section in `README.md`.**

The architecture section (`## Full System Setup & Operations → ### System Architecture`) must remain accurate. It must show:

1. All active data paths (current ones — no removed features like ESP-NOW)
2. Timing estimates for each hop/stage
3. Service names and ports
4. The current version number in the README header

When you make a change, ask:
- Does this change a data path, rate, port, or service?
- Does it add or remove a communication channel?
- Does it change timing or latency at any layer?

If yes → update the architecture diagram in the README.

---

## Version Tracking

The README header must include a version tag. **Increment the version with every meaningful change:**

```markdown
> **Version:** v1.x.x — YYYY-MM-DD
```

- Patch (v1.0.x): bug fixes, config corrections, service tweaks
- Minor (v1.x.0): new features, new services, added monitoring
- Major (vX.0.0): architecture changes, firmware stack changes

---

## System Architecture (current)

```
ESP32-S3 BNO085
  └─ commsTask 40 Hz (25 ms)
  └─► Signal K delta UDP → 255.255.255.255:101022  [40 Hz, ~1–5 ms WiFi hop]
        └─► Signal K Server (signalk.service) :3000
              └─► WebSocket push to PyPilot  [25 ms — signalk.period=0.025]
                    └─► PyPilot PID loop 40 Hz  [25 ms — imu.rate=40]
                          │  servo.period=0.1 (motor stiction window, NOT PID rate)
                          └─► writes to /dev/pypilot-servo (PTY, <1 ms)
                                └─► socat (pypilot-bridge.service, retry=forever)
                                      └─► TCP to pypilot-bridge.local:20220  [~2–5 ms]
                                            └─► ESP32-C3 TCP→UART bridge
                                                  └─► UART 38400 baud → Motor controller
```

**Total end-to-end latency (typical): ~60 ms**

> `servo.period` is the motor stiction windup window in servo.py — NOT the PID rate. PID rate = `1/imu.rate`.

---

## Key Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | ESP32-S3 firmware: sensor + WiFi + BLE + Signal K |
| `pypilot-serial-bridge/src/main.cpp` | ESP32-C3 firmware: WiFi TCP bridge |
| `pypilot-serial-bridge/src/web_console.cpp` | ESP32-C3 motor diagnostics web UI |
| `pypilot-serial-bridge/pi-setup/pypilot-bridge.service` | socat PTY service (retry=forever) |
| `pypilot-serial-bridge/pi-setup/pypilot-status.py` | Status web server (pypilotstatus.local) |
| `pypilot-serial-bridge/pi-setup/pypilot-status.service` | Systemd unit for status server |
| `pypilot-serial-bridge/pi-setup/check-pypilot.sh` | CLI diagnostic script (legacy) |
| `pypilot-serial-bridge/pi-setup/setup-bridge.sh` | Full Pi-side install script |
| `README.md` | Full documentation including config reference |

---

## Critical Constraints

### socat service MUST use retry=forever
The `pypilot-bridge.service` socat command MUST include `retry=forever` and the unit MUST have `StartLimitIntervalSec=0`. Without these, the service gives up when the ESP32-C3 reboots and `/dev/pypilot-servo` disappears permanently.

**Correct:**
```
tcp:pypilot-bridge.local:20220,retry=forever,interval=5,keepalive,...
```

### Signal K UDP port for compass
The ESP32-S3 broadcasts Signal K delta UDP to a configurable port. Signal K has a `SingalK_UDP_101022` provider enabled on port **101022**. Port 10110 is already used by NMEA GPS — do NOT use it for Signal K delta.

To change the compass's target port: `http://compass.local/setport?port=101022`

### PyPilot IMU source
PyPilot must use `imu.source = "signalk"` to receive heading from the BNO085 via Signal K. If it's `"local"`, it uses non-existent local hardware and autopilot gets no heading.

### Partition layout (ESP32-C3)
The ESP32-C3 firmware uses `min_spiffs.csv` partition scheme (1.875 MB app slots) due to large Wi-Fi + AsyncWebServer stack. Flash usage is currently ~69%. Do not add large dependencies without checking flash budget: `pio run -e esp32c3 --target size`.

---

## Config Files on Pi (must be preserved)

| File | Critical setting |
|------|-----------------|
| `~/.signalk/settings.json` | `SingalK_UDP_101022` provider enabled on port 101022 |
| `~/.pypilot/pypilot.conf` | `imu.source="signalk"`, `signalk.host="localhost"`, `signalk.period=0.025`, `imu.rate=40`, `servo.period=0.1` |
| `~/.pypilot/serial_ports` | `/dev/pypilot-servo` |
| `~/.pypilot/servodevice` | `["/dev/pypilot-servo", 38400]` |

See `README.md` → Configuration Reference for full details.

---

## Build Commands

```bash
# ESP32-S3 compass firmware
cd "ESP-32-S3_9axis sensor"
pio run -e esp32s3-devkitc-1

# ESP32-C3 motor bridge firmware
cd pypilot-serial-bridge
pio run -e esp32c3

# Run diagnostic (CLI)
./pypilot-serial-bridge/pi-setup/check-pypilot.sh

# Status web server (systemd)
systemctl status pypilot-status.service
# Browse: http://pypilotstatus.local
```

---

## After Any Change — Checklist

- [ ] Architecture diagram in README still accurate?
- [ ] Version number incremented in README header?
- [ ] `socat retry=forever` still present in pypilot-bridge.service?
- [ ] Signal K UDP port still 101022?
- [ ] `imu.source = "signalk"` still in pypilot.conf?
- [ ] Both firmwares build without errors? (`pio run`)
- [ ] ESP32-C3 flash < 95%? (`pio run -e esp32c3 --target size`)
- [ ] Does this change increase latency anywhere in the chain? If so, is it justified? (Target: ≤60 ms total)
- [ ] Does this change reduce stability or add a failure mode that doesn't self-heal?
- [ ] If changing a sample rate or adding a data path: does it fit within the I2C / CPU / WiFi resource budget? (see Hardware & Network Resource Budget section)
