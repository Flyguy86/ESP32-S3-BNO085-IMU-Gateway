#!/usr/bin/env python3
"""
pypilot-status.py — PyPilot Autopilot System Status Monitor
============================================================
Runs a live web dashboard at http://pypilotstatus.local:8083
that continuously checks every layer of the autopilot stack.

mDNS hostname 'pypilotstatus.local' is advertised by avahi-daemon via
/etc/avahi/services/pypilot-status.xml (created by setup-bridge.sh).
All checks use only Python stdlib + subprocess calls to system tools.

Install:  see setup-bridge.sh
Service:  pypilot-status.service
"""

import http.server
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from datetime import datetime, timezone

# ── Configuration ──────────────────────────────────────────────────────────
PORT            = 8083
CHECK_INTERVAL  = 10          # seconds between full check cycles
MDNS_HOSTNAME   = "pypilotstatus"
BRIDGE_HOST     = "pypilot-bridge.local"
BRIDGE_PORT     = 20220
COMPASS_HOST    = "compass.local"
PYPILOT_PORT    = 20220
SK_PORT         = 3000
SERVO_DEV       = "/dev/pypilot-servo"
HOME            = os.path.expanduser("~")
PYPILOT_DIR     = os.path.join(HOME, "pypilot")
# Consecutive non-ESTAB check cycles before the watchdog auto-restarts the bridge.
# At CHECK_INTERVAL=10 s, 3 cycles = 30 s of detected disconnection before action.
BRIDGE_WATCHDOG_THRESHOLD = 3
# Minimum seconds between watchdog-triggered restarts (avoid restart storms).
BRIDGE_WATCHDOG_COOLDOWN  = 120
# ───────────────────────────────────────────────────────────────────────────

_lock   = threading.Lock()
_state  = {"layers": [], "summary": {}, "last_check": None, "check_count": 0,
           "bridge_disconnect_count": 0, "last_auto_restart": None}


# ── Helpers ─────────────────────────────────────────────────────────────────

def local_ip():
    """Best-effort local IP address (prefers non-loopback)."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def svc_active(name):
    r = subprocess.run(["systemctl", "is-active", name],
                       capture_output=True, text=True, timeout=5)
    return r.stdout.strip() == "active"


def tcp_open(host, port, timeout=3):
    """Return True if port is open (accepting) OR occupied (timeout = server busy with one client).
    Only return False on ConnectionRefused, which means nothing is listening."""
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True
    except ConnectionRefusedError:
        return False
    except Exception:
        # TimeoutError = server is up but backlog full (single-client bridge already connected)
        # OSError / other = treat as open-but-busy rather than missing
        return True


def http_get(url, timeout=5, token=None):
    try:
        headers = {"User-Agent": "pypilot-status/1"}
        if token:
            headers["Authorization"] = f"Bearer {token}"
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read().decode("utf-8", errors="replace"), r.status
    except Exception:
        return None, None


def ping(host, timeout=3):
    """Check host reachability via TCP port 80 (avoids ICMP permission issues in service context)."""
    return tcp_open(host, 80, timeout=timeout)


def age_secs(timestamp_str):
    try:
        ts = timestamp_str.replace("Z", "+00:00")
        dt = datetime.fromisoformat(ts)
        return int((datetime.now(timezone.utc) - dt).total_seconds())
    except Exception:
        return 9999


def read_file(path):
    try:
        with open(path) as f:
            return f.read()
    except Exception:
        return ""


def pypilot_heading(timeout=2.0):
    """Connect to pypilot IPC (port 20220), read NMEA stream, return heading in degrees or None."""
    try:
        s = socket.create_connection(("localhost", PYPILOT_PORT), timeout=timeout)
        s.settimeout(timeout)
        buf = b""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            try:
                chunk = s.recv(256)
                if not chunk:
                    break
                buf += chunk
                for line in buf.split(b"\n"):
                    line = line.strip()
                    if line.startswith(b"$APHDM,"):
                        parts = line.decode("ascii", errors="ignore").split(",")
                        if len(parts) >= 2 and parts[1]:
                            s.close()
                            return float(parts[1])
            except socket.timeout:
                break
        s.close()
    except Exception:
        pass
    return None


# ── Layer checks ─────────────────────────────────────────────────────────────

def check_layer1():
    """Layer 1: Signal K Server — heading sink & aggregator."""
    checks = []
    failed = False
    warned = False

    # Auth token that pypilot uses for its SK WebSocket subscription
    sk_token = read_file(os.path.join(HOME, ".pypilot", "signalk-token")).strip()

    if svc_active("signalk.service"):
        checks.append(("ok", "signalk.service running"))
    else:
        checks.append(("fail", "signalk.service NOT running"))
        failed = True

    body, _ = http_get(f"http://localhost:{SK_PORT}/signalk")
    if body:
        checks.append(("ok", f"Signal K API responding on :{SK_PORT}"))
    else:
        checks.append(("fail", f"Signal K HTTP not responding on :{SK_PORT}"))
        failed = True

    body, _ = http_get(
        f"http://localhost:{SK_PORT}/signalk/v1/api/vessels/self/navigation/headingMagnetic",
        token=sk_token
    )
    if body:
        try:
            d = json.loads(body)
            ts  = d.get("timestamp", "")
            val = round(math.degrees(d.get("value", 0)), 1)
            age = age_secs(ts)
            if age < 10:
                checks.append(("ok", f"headingMagnetic = {val}° ({age}s ago)"))
            elif age < 60:
                checks.append(("warn", f"headingMagnetic = {val}° ({age}s ago — getting stale)"))
                warned = True
            else:
                checks.append(("fail", f"headingMagnetic last update {age}s ago — data stopped"))
                failed = True
        except Exception as e:
            checks.append(("warn", f"headingMagnetic parse error: {e}"))
            warned = True
    else:
        checks.append(("fail", "navigation.headingMagnetic not in Signal K"))
        failed = True

    body, _ = http_get(
        f"http://localhost:{SK_PORT}/signalk/v1/api/vessels/self/navigation/rateOfTurn",
        token=sk_token
    )
    if body:
        checks.append(("ok", "navigation.rateOfTurn present"))
    else:
        checks.append(("warn", "navigation.rateOfTurn absent (optional)"))
        warned = True

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "Signal K Server",
        "subtitle": "Heading sink &amp; data aggregator",
        "timing": "~200 ms update period (WebSocket push to PyPilot)",
        "status": status,
        "checks": checks,
        "links": [
            ("Data browser", f"http://localhost:{SK_PORT}/admin/#/databrowser"),
            ("Connections", f"http://localhost:{SK_PORT}/admin/#/server/connections"),
        ],
    }


def check_layer2():
    """Layer 2: ESP32-S3 BNO085 Compass."""
    checks = []
    failed = False
    warned = False

    if ping(COMPASS_HOST):
        checks.append(("ok", f"{COMPASS_HOST} reachable"))
        body, _ = http_get(f"http://{COMPASS_HOST}/")
        if body:
            checks.append(("ok", "ESP32-S3 web dashboard responding"))
        else:
            checks.append(("warn", "ESP32-S3 web dashboard not responding"))
            warned = True
    else:
        checks.append(("fail", f"{COMPASS_HOST} not reachable (WiFi down or wrong mDNS)"))
        failed = True

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "ESP32-S3 BNO085 Compass",
        "subtitle": f"40 Hz heading sensor — {COMPASS_HOST}",
        "timing": "25 ms cycle (40 Hz): sensor→WiFi UDP→Signal K (~2–5 ms WiFi hop)",
        "status": status,
        "checks": checks,
        "links": [
            ("Compass dashboard", f"http://{COMPASS_HOST}/"),
        ],
    }


def fetch_bridge_telemetry():
    """Fetch /api/telem from the ESP32-C3 bridge. Returns dict or None."""
    body, _ = http_get(f"http://{BRIDGE_HOST}/api/telem", timeout=3)
    if not body:
        return None
    try:
        return json.loads(body)
    except Exception:
        return None


def check_layer3():
    """Layer 3: ESP32-C3 Motor Bridge."""
    checks = []
    failed = False
    warned = False
    telem_data = None

    if ping(BRIDGE_HOST):
        checks.append(("ok", f"{BRIDGE_HOST} reachable"))
        body, _ = http_get(f"http://{BRIDGE_HOST}/")
        if body:
            checks.append(("ok", "ESP32-C3 web console responding"))
        else:
            checks.append(("warn", "ESP32-C3 web console not responding"))
            warned = True
    else:
        checks.append(("fail", f"{BRIDGE_HOST} not reachable (ESP32-C3 offline or wrong mDNS)"))
        failed = True

    if tcp_open(BRIDGE_HOST, BRIDGE_PORT):
        checks.append(("ok", f"TCP :{BRIDGE_PORT} open — bridge accepting connections"))
    else:
        checks.append(("fail", f"TCP :{BRIDGE_PORT} not open on {BRIDGE_HOST}"))
        failed = True

    # ── Motor telemetry & health diagnostics ───────────────────────────────
    telem = fetch_bridge_telemetry()
    if telem:
        telem_data = telem

        # Packet health — error rate
        pkt_good = telem.get("pkt_good", 0)
        pkt_err  = telem.get("pkt_err",  0)
        total    = pkt_good + pkt_err
        if total > 0:
            err_pct = pkt_err / total * 100
            if pkt_err == 0:
                checks.append(("ok", f"Motor packets: {pkt_good:,} good / 0 err &#8212; clean serial link"))
            elif err_pct < 1:
                checks.append(("warn", f"Motor packets: {pkt_good:,} good / {pkt_err} err ({err_pct:.1f}%) &#8212; minor noise"))
                warned = True
            else:
                checks.append(("fail", f"Motor packets: {pkt_good:,} good / {pkt_err} err ({err_pct:.1f}%) &#8212; high error rate, check UART wiring"))
                failed = True
        else:
            checks.append(("warn", "No motor packets yet &#8212; motor controller may be off or not connected"))
            warned = True

        # Command latency — age of last motor telemetry packet from controller
        age_ms = telem.get("pkt_age_ms", 9999)
        if age_ms < 500:
            checks.append(("ok", f"Command latency: last motor packet {age_ms} ms ago &#8212; responsive"))
        elif age_ms < 5000:
            checks.append(("warn", f"Command latency: last motor packet {age_ms} ms ago &#8212; bridge may be buffering; &#8216;Big Kick&#8217; risk if &gt;500 ms"))
            warned = True
        else:
            checks.append(("fail", f"Command latency: motor controller silent for {age_ms} ms &#8212; buffer stalled or controller off"))
            failed = True

        # Serial saturation — TX byte rate vs 38400 baud capacity (~3800 B/s ceiling)
        # Normal: pypilot sends ~4 B/cmd × 40 Hz ≈ 160 B/s
        tx_bytes = telem.get("tx_bytes", 0)
        uptime_s = max(telem.get("uptime_s", 1), 1)
        tx_rate  = tx_bytes / uptime_s
        if tx_rate < 1500:
            checks.append(("ok", f"Serial TX rate: {tx_rate:.0f} B/s &#8212; well within 38400 baud capacity"))
        elif tx_rate < 3000:
            checks.append(("warn", f"Serial TX rate: {tx_rate:.0f} B/s &#8212; elevated; if Signal K period &lt;0.05 s, bridge may &#8216;Stutter&#8217;"))
            warned = True
        else:
            checks.append(("fail", f"Serial TX rate: {tx_rate:.0f} B/s &#8212; serial saturation risk at 38400 baud (~3800 B/s ceiling)"))
            failed = True

        # Voltage sanity check
        v = telem.get("voltage_V", 0)
        if v >= 11.0:
            checks.append(("ok", f"Motor voltage: {v:.1f} V"))
        elif v >= 9.0:
            checks.append(("warn", f"Motor voltage: {v:.1f} V &#8212; low, check supply"))
            warned = True
        elif v > 0:
            checks.append(("fail", f"Motor voltage: {v:.1f} V &#8212; critically low"))
            failed = True

        # Controller temperature
        ct = telem.get("ctrl_temp_C", 0)
        if ct >= 60:
            checks.append(("fail", f"Controller temp: {ct:.1f}&#176;C &#8212; OVERTEMP risk"))
            failed = True
        elif ct >= 45:
            checks.append(("warn", f"Controller temp: {ct:.1f}&#176;C &#8212; warm, monitor"))
            warned = True
        elif ct > 0:
            checks.append(("ok", f"Controller temp: {ct:.1f}&#176;C"))
    else:
        checks.append(("warn", "Motor telemetry unavailable &#8212; update bridge firmware to enable /api/telem"))
        warned = True

    status = "fail" if failed else ("warn" if warned else "ok")

    result = {
        "name": "ESP32-C3 Motor Bridge",
        "subtitle": f"TCP&#8594;UART bridge &#8212; {BRIDGE_HOST}:{BRIDGE_PORT}",
        "timing": "~2&#8211;5 ms TCP to UART relay (transparent, 38400 baud)",
        "status": status,
        "checks": checks,
        "links": [
            ("Motor bridge console", f"http://{BRIDGE_HOST}/console"),
            ("Motor telemetry API", f"http://{BRIDGE_HOST}/api/telem"),        ],
    }
    if telem_data:
        result["telem"] = {
            "current_A":    round(telem_data.get("current_A",    0), 2),
            "voltage_V":    round(telem_data.get("voltage_V",    0), 1),
            "ctrl_temp_C":  round(telem_data.get("ctrl_temp_C",  0), 1),
            "motor_temp_C": round(telem_data.get("motor_temp_C", 0), 1),
            "rudder_raw":   telem_data.get("rudder_raw",  0),
            "pkt_good":     telem_data.get("pkt_good",    0),
            "pkt_err":      telem_data.get("pkt_err",     0),
            "pkt_age_ms":   telem_data.get("pkt_age_ms",  0),
            "rx_bytes":     telem_data.get("rx_bytes",    0),
            "tx_bytes":     telem_data.get("tx_bytes",    0),
        }
    return result


def socat_tcp_established():
    """Return (established: bool, state_str: str) for socat's TCP connection to BRIDGE_PORT.

    Uses `ss -tnp` which requires no special privileges.  If the socat process
    has an ESTABLISHED connection to port 20220 the bridge is live.  If the
    connection is SYN-SENT / absent, socat is in its retry loop and the bridge
    is DOWN even though the service shows 'active'.
    """
    try:
        r = subprocess.run(["ss", "-tnp"], capture_output=True, text=True, timeout=5)
        port_str = f":{BRIDGE_PORT}"
        for line in r.stdout.splitlines():
            if port_str in line and "socat" in line:
                state = line.split()[0]   # ESTAB, SYN-SENT, etc.
                return state == "ESTAB", state
            elif port_str in line:
                # socat visible in dest address even without process name (non-root)
                state = line.split()[0]
                return state == "ESTAB", state
        # No line at all — socat not yet attempted or DNS failing
        return False, "no-connection"
    except Exception:
        return False, "unknown"


def svc_show(unit, *properties):
    """Return dict of {prop: value} from `systemctl show`."""
    result = {}
    try:
        props = list(properties)
        r = subprocess.run(
            ["systemctl", "show", unit, "--property=" + ",".join(props)],
            capture_output=True, text=True, timeout=5
        )
        for line in r.stdout.splitlines():
            if "=" in line:
                k, _, v = line.partition("=")
                result[k.strip()] = v.strip()
    except Exception:
        pass
    return result


def check_layer4():
    """Layer 4: socat virtual serial port."""
    checks = []
    failed = False
    warned = False

    svc_running = svc_active("pypilot-bridge.service")
    if svc_running:
        checks.append(("ok", "pypilot-bridge.service running"))
    else:
        checks.append(("fail", "pypilot-bridge.service NOT running — run: sudo systemctl start pypilot-bridge"))
        failed = True

    # ── socat process actually present ──────────────────────────────────────
    try:
        r = subprocess.run(["pgrep", "-x", "socat"], capture_output=True, text=True, timeout=5)
        socat_pid = r.stdout.strip()
        if socat_pid:
            checks.append(("ok", f"socat process running (PID {socat_pid})"))
        elif svc_running:
            checks.append(("fail", "pypilot-bridge.service active but socat process NOT found — ExecStart may have crashed"))
            failed = True
    except Exception:
        pass

    # ── TCP connection state — the critical check ────────────────────────────
    # socat 'active' but in retry loop = no ESTAB connection = motor offline
    established, tcp_state = socat_tcp_established()
    if established:
        checks.append(("ok", f"socat TCP → {BRIDGE_HOST}:{BRIDGE_PORT} ESTABLISHED &#8212; bridge live"))
    elif tcp_state == "SYN-SENT":
        checks.append(("fail",
            f"socat TCP → {BRIDGE_HOST}:{BRIDGE_PORT} SYN-SENT &#8212; connecting, ESP32-C3 may be offline; "
            "motor controller will appear missing until connected"))
        failed = True
    elif tcp_state == "no-connection":
        if svc_running:
            checks.append(("warn",
                f"socat has no TCP connection to :{BRIDGE_PORT} yet &#8212; "
                "service just started or DNS resolving; will auto-connect"))
            warned = True
        # else service not running, already failed above
    else:
        checks.append(("warn", f"socat TCP connection state: {tcp_state}"))
        warned = True

    # ── Restart count and uptime ─────────────────────────────────────────────
    show = svc_show(
        "pypilot-bridge.service",
        "NRestarts", "ActiveEnterTimestamp", "ExecMainStartTimestamp"
    )
    n_restarts = int(show.get("NRestarts", 0) or 0)
    if n_restarts == 0:
        checks.append(("ok", "Service restart count: 0 (stable)"))
    elif n_restarts <= 3:
        checks.append(("warn", f"Service has restarted {n_restarts}× since last boot &#8212; intermittent connectivity"))
        warned = True
    else:
        checks.append(("fail", f"Service has restarted {n_restarts}× &#8212; socat is crash-looping; check ESP32-C3 and DNS"))
        failed = True

    active_ts = show.get("ActiveEnterTimestamp", "")
    if active_ts and active_ts != "n/a":
        try:
            from datetime import datetime, timezone
            # systemd timestamp format: "Sat 2026-03-21 14:32:01 UTC"
            dt = datetime.strptime(active_ts.split(" ", 1)[1].rsplit(" ", 1)[0],
                                   "%Y-%m-%d %H:%M:%S")
            dt = dt.replace(tzinfo=timezone.utc)
            uptime_s = int((datetime.now(timezone.utc) - dt).total_seconds())
            if uptime_s < 30:
                checks.append(("warn", f"Service only started {uptime_s}s ago &#8212; recently restarted"))
                warned = True
            elif uptime_s < 300:
                mins = uptime_s // 60
                checks.append(("ok", f"Service uptime: {mins}m {uptime_s % 60}s"))
            else:
                hrs  = uptime_s // 3600
                mins = (uptime_s % 3600) // 60
                checks.append(("ok", f"Service uptime: {hrs}h {mins}m &#8212; stable"))
        except Exception:
            pass

    # ── Service config sanity ────────────────────────────────────────────────
    svc_content = read_file("/etc/systemd/system/pypilot-bridge.service")
    if "retry=2147483647" in svc_content:
        checks.append(("ok", "socat retry=2147483647 &#8212; will never give up reconnecting"))
    elif "retry=forever" in svc_content:
        checks.append(("fail",
            "socat retry=forever is NOT supported on socat 1.8.x &#8212; socat will exit immediately on first disconnect; "
            "change to retry=2147483647"))
        failed = True
    elif svc_content:
        checks.append(("warn", "socat ExecStart missing retry= &#8212; will give up reconnecting after first disconnect"))
        warned = True

    if "StartLimitIntervalSec=0" in svc_content:
        checks.append(("ok", "StartLimitIntervalSec=0 &#8212; service always restarts"))
    elif svc_content:
        checks.append(("warn", "StartLimitIntervalSec=0 missing &#8212; systemd may stop restarting after burst"))
        warned = True

    # ── PTY device ──────────────────────────────────────────────────────────
    if os.path.exists(SERVO_DEV):
        try:
            target = os.path.realpath(SERVO_DEV)
            checks.append(("ok", f"{SERVO_DEV} exists &#8594; {target}"))
        except Exception:
            checks.append(("ok", f"{SERVO_DEV} exists"))
        if os.access(SERVO_DEV, os.W_OK):
            checks.append(("ok", f"{SERVO_DEV} is writable"))
        else:
            checks.append(("fail", f"{SERVO_DEV} not writable &#8212; run: sudo chmod 666 {SERVO_DEV}"))
            failed = True
    else:
        if established:
            checks.append(("fail",
                f"{SERVO_DEV} missing even though TCP is ESTABLISHED &#8212; "
                "waitslave may still be settling; try: sudo systemctl restart pypilot-bridge"))
        else:
            checks.append(("fail",
                f"{SERVO_DEV} does not exist &#8212; socat has not connected yet "
                "(expected: PTY created once TCP reaches ESTABLISHED)"))
        failed = True

    serial_ports_path = os.path.join(HOME, ".pypilot", "serial_ports")
    content = read_file(serial_ports_path)
    if content and SERVO_DEV in content:
        checks.append(("ok", "~/.pypilot/serial_ports configured correctly"))
    elif content:
        checks.append(("warn", f"serial_ports exists but doesn&#8217;t include {SERVO_DEV}"))
        warned = True
    else:
        checks.append(("warn", "~/.pypilot/serial_ports missing &#8212; PyPilot will scan all ports"))
        warned = True

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "socat Virtual Serial Port",
        "subtitle": f"PTY bridge: {SERVO_DEV} &#8596; TCP &#8594; ESP32-C3",
        "timing": "&lt;1 ms PTY relay (kernel space, synchronous write)",
        "status": status,
        "checks": checks,
        "links": [],
        "actions": [
            {"label": "&#9654; Restart bridge", "post": "/api/restart-bridge"},
        ],
    }


def check_layer5():
    """Layer 5: PyPilot Autopilot."""
    checks = []
    failed = False
    warned = False

    if svc_active("pypilot.service"):
        checks.append(("ok", "pypilot.service running"))
    else:
        checks.append(("fail", "pypilot.service NOT running"))
        failed = True

    # Verify pypilot is receiving heading by reading its live NMEA output
    hdg = pypilot_heading()
    if hdg is not None:
        checks.append(("ok", f"imu.heading = {hdg:.1f}° (heading arriving via Signal K)"))
    elif tcp_open("localhost", PYPILOT_PORT, timeout=1):
        checks.append(("warn", f"pypilot TCP :{PYPILOT_PORT} open but no heading in NMEA stream"))
        warned = True
    else:
        checks.append(("fail", f"pypilot TCP :{PYPILOT_PORT} not open"))
        failed = True

    # Read imu.source from conf file (fast, no pypilot client dep)
    conf_path = os.path.join(HOME, ".pypilot", "pypilot.conf")
    imu_src = "unknown"
    for line in read_file(conf_path).splitlines():
        if line.strip().startswith("imu.source"):
            imu_src = line.split("=", 1)[1].strip().strip('"')
            break

    if imu_src == "signalk":
        checks.append(("ok", "imu.source = signalk (using BNO085 via Signal K)"))
    else:
        checks.append(("warn", f"imu.source = '{imu_src}' — not using Signal K heading"))
        warned = True

    # Read servo device
    servo_device = read_file(os.path.join(HOME, ".pypilot", "servodevice")).strip()
    if servo_device:
        checks.append(("ok", f"servodevice = {servo_device}"))

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "PyPilot Autopilot",
        "subtitle": f"6-term PID — TCP :{PYPILOT_PORT} — imu.source = {imu_src}",
        "timing": "25 ms PID cycle (imu.rate=40) — ~60 ms total chain latency",
        "status": status,
        "checks": checks,
        "links": [
            ("PyPilot web UI", "http://localhost:8080"),
        ],
    }


def check_layer6():
    """Layer 6: PyPilot Web Interface."""
    checks = []
    failed = False
    warned = False

    if svc_active("pypilot-web.service"):
        checks.append(("ok", "pypilot-web.service running"))
    else:
        checks.append(("fail", "pypilot-web.service NOT running"))
        failed = True

    web_up = False
    for port in (8080, 80):
        if tcp_open("localhost", port, timeout=2):
            checks.append(("ok", f"PyPilot web UI on :{port}"))
            web_up = True
            break
    if not web_up:
        checks.append(("warn", "PyPilot web UI port not responding"))
        warned = True

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "PyPilot Web UI",
        "subtitle": "Autopilot control panel (port 8080)",
        "timing": "On-demand HTTP — no polling",
        "status": status,
        "checks": checks,
        "links": [
            ("Open PyPilot UI", "http://localhost:8080"),
        ],
    }


def run_all_checks():
    layers = [
        check_layer1(),
        check_layer2(),
        check_layer3(),
        check_layer4(),
        check_layer5(),
        check_layer6(),
    ]
    total  = len(layers)
    failed = sum(1 for L in layers if L["status"] == "fail")
    warned = sum(1 for L in layers if L["status"] == "warn")
    passed = total - failed - warned
    if failed == 0 and warned == 0:
        overall = "ok"
        message = "All systems GO — Full autopilot chain operational"
    elif failed == 0:
        overall = "warn"
        message = f"Operational with {warned} warning(s) — review below"
    else:
        overall = "fail"
        message = f"{failed} layer(s) FAILED — autopilot will NOT function correctly"
    return layers, {
        "overall": overall, "message": message,
        "passed": passed, "warned": warned, "failed": failed, "total": total,
    }


_recheck_event = threading.Event()


def _trigger_recheck():
    """Signal the checker loop to run immediately (called after a restart action)."""
    _recheck_event.set()


def checker_loop():
    while True:
        try:
            layers, summary = run_all_checks()

            # ── Persist results immediately ───────────────────────────────────
            with _lock:
                _state["layers"]      = layers
                _state["summary"]     = summary
                _state["last_check"]  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                _state["check_count"] += 1

            # ── Bridge watchdog ───────────────────────────────────────────────
            # socat never exits when its TCP connection drops (retry=2147483647),
            # so systemd Restart=always never fires.  We detect the stuck state
            # via the ESTAB check in Layer 4 and restart the service ourselves
            # after BRIDGE_WATCHDOG_THRESHOLD consecutive non-ESTAB cycles.
            bridge_connected = any(
                ("ESTABLISHED" in c[1] or "ESTAB" in c[1]) and c[0] == "ok"
                for L in layers if L.get("name", "").startswith("socat")
                for c in L.get("checks", [])
            )
            bridge_service_running = svc_active("pypilot-bridge.service")

            with _lock:
                if bridge_connected or not bridge_service_running:
                    _state["bridge_disconnect_count"] = 0
                else:
                    _state["bridge_disconnect_count"] += 1
                disconnect_count  = _state["bridge_disconnect_count"]
                last_auto_restart = _state["last_auto_restart"]

            if (bridge_service_running
                    and not bridge_connected
                    and disconnect_count >= BRIDGE_WATCHDOG_THRESHOLD):
                now = time.monotonic()
                if last_auto_restart is None or (now - last_auto_restart) > BRIDGE_WATCHDOG_COOLDOWN:
                    print(
                        f"[watchdog] socat TCP not ESTABLISHED for {disconnect_count} cycles "
                        f"({disconnect_count * CHECK_INTERVAL}s) — auto-restarting pypilot-bridge.service",
                        flush=True
                    )
                    try:
                        r = subprocess.run(
                            ["sudo", "systemctl", "restart", "pypilot-bridge.service"],
                            capture_output=True, text=True, timeout=15
                        )
                        if r.returncode == 0:
                            print("[watchdog] pypilot-bridge.service restarted OK", flush=True)
                        else:
                            print(f"[watchdog] restart failed: {r.stderr.strip()}", flush=True)
                    except Exception as e:
                        print(f"[watchdog] restart error: {e}", flush=True)
                    with _lock:
                        _state["bridge_disconnect_count"] = 0
                        _state["last_auto_restart"]       = time.monotonic()
                    # Wait 5 s then loop immediately for a fresh check
                    _recheck_event.wait(timeout=5)
                    _recheck_event.clear()
                    continue

        except Exception as e:
            print(f"[checker] Error: {e}", flush=True)
        # Wait for CHECK_INTERVAL, but wake early if _trigger_recheck() is called
        _recheck_event.wait(timeout=CHECK_INTERVAL)
        _recheck_event.clear()


# ── HTML template ────────────────────────────────────────────────────────────

HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>PyPilot Status</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#0f0f1a;color:#e0e0e0;padding:16px}
h1{color:#0fbcf9;text-align:center;font-size:1.4em;margin-bottom:4px}
.subtitle{text-align:center;color:#888;font-size:.8em;margin-bottom:16px}
.summary{border-radius:8px;padding:14px 18px;margin-bottom:20px;display:flex;align-items:center;gap:16px;flex-wrap:wrap}
.summary.ok  {background:#0d2a1a;border:1px solid #2ecc71}
.summary.warn{background:#2a1f0a;border:1px solid #f39c12}
.summary.fail{background:#2a0a0a;border:1px solid #e74c3c}
.sum-icon{font-size:2em}
.sum-msg{flex:1;font-weight:600;font-size:1em}
.sum-counts{display:flex;gap:10px;font-size:.85em}
.cnt{padding:3px 10px;border-radius:12px;font-weight:700}
.cnt.ok  {background:#1a4a2a;color:#2ecc71}
.cnt.warn{background:#3a2a0a;color:#f39c12}
.cnt.fail{background:#4a0a0a;color:#e74c3c}
.layers{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:12px}
.card{background:#1a1a2e;border-radius:8px;border:1px solid #2a2a4a;overflow:hidden}
.card-head{padding:12px 14px;display:flex;align-items:flex-start;gap:10px}
.card-head.ok  {border-left:4px solid #2ecc71}
.card-head.warn{border-left:4px solid #f39c12}
.card-head.fail{border-left:4px solid #e74c3c}
.card-title{flex:1}
.card-title strong{display:block;font-size:.95em}
.card-title small{color:#888;font-size:.75em}
.badge{font-size:.7em;font-weight:700;padding:2px 8px;border-radius:10px;white-space:nowrap}
.badge.ok  {background:#1a4a2a;color:#2ecc71}
.badge.warn{background:#3a2a0a;color:#f39c12}
.badge.fail{background:#4a0a0a;color:#e74c3c}
.card-body{padding:8px 14px 10px}
.check{display:flex;align-items:flex-start;gap:6px;margin:3px 0;font-size:.8em;line-height:1.3}
.ck-ok  {color:#2ecc71}
.ck-warn{color:#f39c12}
.ck-fail{color:#e74c3c}
.timing{font-size:.72em;color:#556;margin-top:6px;padding-top:6px;border-top:1px solid #2a2a4a;font-style:italic}
.links{margin-top:6px;display:flex;gap:8px;flex-wrap:wrap}
.links a{font-size:.72em;color:#0fbcf9;text-decoration:none}
.links a:hover{text-decoration:underline}
.telem-strip{display:grid;grid-template-columns:repeat(auto-fill,minmax(90px,1fr));gap:5px;
             margin-top:8px;padding-top:7px;border-top:1px solid #2a2a4a}
.tg{background:#12122a;border:1px solid #2a2a4a;border-radius:5px;padding:5px 4px;text-align:center}
.tg .tg-lbl{font-size:.62em;color:#667;text-transform:uppercase;letter-spacing:.03em;display:block}
.tg .tg-val{font-size:1em;font-weight:700;color:#58c4f5;font-variant-numeric:tabular-nums;display:block}
.tg .tg-sub{font-size:.6em;color:#556;display:block}
.act-btn{font-size:.72em;color:#fff;background:#c0392b;border:none;border-radius:4px;
         padding:3px 10px;cursor:pointer;margin-top:2px}
.act-btn:hover{background:#e74c3c}
.act-btn:disabled{opacity:.5;cursor:default}
.footer{text-align:center;font-size:.72em;color:#445;margin-top:18px}
.spinner{display:inline-block;width:8px;height:8px;border:2px solid #333;border-top-color:#0fbcf9;border-radius:50%;animation:spin .8s linear infinite;margin-right:5px}
@keyframes spin{to{transform:rotate(360deg)}}
</style>
</head>
<body>
<h1>&#9881; PyPilot System Status</h1>
<div class="subtitle" id="ts">Loading...</div>

<div class="summary" id="summary">
  <div class="sum-icon">&#9203;</div>
  <div class="sum-msg">Checking...</div>
</div>

<div class="layers" id="layers"></div>

<div class="footer">
  <span class="spinner" id="spin"></span>
  <span id="next-check">Refreshing in 10s</span> &nbsp;|&nbsp;
  pypilotstatus.local &nbsp;|&nbsp;
  <a href="/api/status" style="color:#445">JSON</a>
</div>

<script>
const ICONS = {ok:'&#9989;', warn:'&#9888;&#65039;', fail:'&#10060;'};
const LABELS = {ok:'OK', warn:'WARN', fail:'FAIL'};
let countdown = 5;

function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;'); }

function fmt(v,dec){ return (v===undefined||v===null)?'—':Number(v).toFixed(dec); }

function telemStrip(t){
  if(!t) return '';
  var pktTot = (t.pkt_good||0)+(t.pkt_err||0);
  var errPct = pktTot>0 ? (t.pkt_err/pktTot*100).toFixed(1)+'%' : '—';
  function tg(lbl,val,sub){
    return '<div class="tg"><span class="tg-lbl">'+lbl+'</span>'
          +'<span class="tg-val">'+val+'</span>'
          +(sub?'<span class="tg-sub">'+sub+'</span>':'')+'</div>';
  }
  return '<div class="telem-strip">'
    +tg('Current',  fmt(t.current_A,2),  'A')
    +tg('Voltage',  fmt(t.voltage_V,1),  'V')
    +tg('Ctrl Temp',fmt(t.ctrl_temp_C,1),'&deg;C')
    +tg('Mtr Temp', fmt(t.motor_temp_C,1),'&deg;C')
    +tg('Packets',  (t.pkt_good||0).toLocaleString(), '/ '+errPct+' err')
    +tg('Pkt Age',  (t.pkt_age_ms||0)+' ms','latency')
    +tg('TX Rate',  t.tx_bytes&&t.pkt_age_ms!==undefined?'—':fmt(t.tx_bytes,0),'bytes tx')
    +tg('RX Bytes', (t.rx_bytes||0).toLocaleString(),'raw UART')
    +'</div>';
}

function render(d){
  const s = d.summary;
  const sb = document.getElementById('summary');
  sb.className = 'summary ' + s.overall;
  sb.innerHTML =
    '<div class="sum-icon">' + ICONS[s.overall] + '</div>' +
    '<div class="sum-msg">' + esc(s.message) + '</div>' +
    '<div class="sum-counts">' +
      '<span class="cnt ok">' + s.passed + ' OK</span>' +
      (s.warned ? '<span class="cnt warn">' + s.warned + ' WARN</span>' : '') +
      (s.failed ? '<span class="cnt fail">' + s.failed + ' FAIL</span>' : '') +
    '</div>';

  document.getElementById('ts').textContent =
    'Last checked: ' + d.last_check + '  (#' + d.check_count + ')';

  const el = document.getElementById('layers');
  el.innerHTML = '';
  (d.layers || []).forEach(function(L){
    const checks = (L.checks || []).map(function(c){
      return '<div class="check"><span class="ck-' + c[0] + '">' +
        (c[0]==='ok'?'✓':c[0]==='warn'?'!':'✗') + '</span><span>' + c[1] + '</span></div>';
    }).join('');
    const links = (L.links||[]).map(function(lk){
      return '<a href="' + lk[1] + '" target="_blank">' + esc(lk[0]) + ' &#8594;</a>';
    }).join('');
    const actions = (L.actions||[]).map(function(a){
      return '<button class="act-btn" onclick="doAction(&#39;'+a.post+'&#39;)">' + a.label + '</button>';
    }).join('');
    el.innerHTML +=
      '<div class="card">' +
        '<div class="card-head ' + L.status + '">' +
          '<div class="card-title">' +
            '<strong>' + L.name + '</strong>' +
            '<small>' + (L.subtitle||'') + '</small>' +
          '</div>' +
          '<span class="badge ' + L.status + '">' + LABELS[L.status] + '</span>' +
        '</div>' +
        '<div class="card-body">' + checks +
          (L.telem ? telemStrip(L.telem) : '') +
          (L.timing ? '<div class="timing">&#8987; ' + L.timing + '</div>' : '') +
          (links||actions ? '<div class="links">' + links + actions + '</div>' : '') +
        '</div>' +
      '</div>';
  });
}

function fetchStatus(){
  document.getElementById('spin').style.display='inline-block';
  fetch('/api/status')
    .then(function(r){return r.json();})
    .then(function(d){render(d); countdown=10;})
    .catch(function(){countdown=10;})
    .finally(function(){document.getElementById('spin').style.display='none';});
}

fetchStatus();
setInterval(fetchStatus, 10000);
setInterval(function(){
  countdown--;
  document.getElementById('next-check').textContent = 'Refreshing in ' + Math.max(0,countdown) + 's';
}, 1000);

function doAction(url){
  var btns = document.querySelectorAll('.act-btn');
  btns.forEach(function(b){b.disabled=true;});
  fetch(url, {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if(d.ok){ countdown=5; setTimeout(fetchStatus,3000); }
      else { alert('Action failed: '+(d.msg||'unknown error')); btns.forEach(function(b){b.disabled=false;}); }
    })
    .catch(function(e){ alert('Request failed: '+e); btns.forEach(function(b){b.disabled=false;}); });
}
</script>
</body>
</html>
"""


# ── HTTP handler ──────────────────────────────────────────────────────────────

class StatusHandler(http.server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress access logs to reduce noise

    def do_GET(self):
        if self.path in ("/api/status", "/api/status/"):
            with _lock:
                payload = {
                    "layers":      _state["layers"],
                    "summary":     _state["summary"],
                    "last_check":  _state["last_check"],
                    "check_count": _state["check_count"],
                }
            body = json.dumps(payload, default=str).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)
        elif self.path in ("/", "/index.html"):
            body = HTML_TEMPLATE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path in ("/api/restart-bridge", "/api/restart-bridge/"):
            # Restart the socat bridge service — requires pypilot-status to run
            # with sudo privileges for systemctl, or a sudoers entry.
            try:
                r = subprocess.run(
                    ["sudo", "systemctl", "restart", "pypilot-bridge.service"],
                    capture_output=True, text=True, timeout=15
                )
                if r.returncode == 0:
                    msg = {"ok": True, "msg": "pypilot-bridge.service restarted"}
                    # Force an immediate re-check so the new state shows up quickly
                    threading.Thread(
                        target=lambda: (time.sleep(3), _trigger_recheck()),
                        daemon=True
                    ).start()
                else:
                    msg = {"ok": False, "msg": r.stderr.strip() or "systemctl returned non-zero"}
            except Exception as e:
                msg = {"ok": False, "msg": str(e)}
            body = json.dumps(msg).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)


# ── mDNS via avahi service file ──────────────────────────────────────────────

AVAHI_SERVICE_FILE = "/etc/avahi/services/pypilot-status.service"

def check_mdns():
    """Verify the avahi service file is installed (created by setup-bridge.sh)."""
    if os.path.exists(AVAHI_SERVICE_FILE):
        print(f"[mDNS] {MDNS_HOSTNAME}.local:{PORT} advertised via {AVAHI_SERVICE_FILE}", flush=True)
    else:
        print(f"[mDNS] WARNING: {AVAHI_SERVICE_FILE} not found.", flush=True)
        print(f"[mDNS] Run setup-bridge.sh to create it, or:", flush=True)
        print(f"[mDNS]   sudo avahi-publish -a {MDNS_HOSTNAME} <ip>", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    # Check mDNS service file
    check_mdns()

    # Signal handlers
    def shutdown(sig, frame):
        print("\n[pypilot-status] Shutting down...", flush=True)
        sys.exit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT,  shutdown)

    # Prime the state immediately (before HTTP starts, so first request isn't empty)
    print("[pypilot-status] Running initial check...", flush=True)
    try:
        layers, summary = run_all_checks()
        with _lock:
            _state["layers"]      = layers
            _state["summary"]     = summary
            _state["last_check"]  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            _state["check_count"] = 1
    except Exception as e:
        print(f"[pypilot-status] Initial check error: {e}", flush=True)

    # Start background checker thread
    t = threading.Thread(target=checker_loop, daemon=True)
    t.start()

    # Start HTTP server
    server = http.server.HTTPServer(("", PORT), StatusHandler)
    host_ip = local_ip()
    print(f"[pypilot-status] Serving on http://{MDNS_HOSTNAME}.local:{PORT}  "
          f"(http://{host_ip}:{PORT})", flush=True)
    print(f"[pypilot-status] Checking every {CHECK_INTERVAL}s", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
