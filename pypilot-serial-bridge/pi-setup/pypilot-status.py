#!/usr/bin/env python3
"""
pypilot-status.py — PyPilot Autopilot System Status Monitor
============================================================
Runs a live web dashboard at http://pypilotstatus.local (port 80)
that continuously checks every layer of the autopilot stack.

Registers the mDNS hostname 'pypilotstatus.local' via avahi-publish-address.
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
PORT            = 80
CHECK_INTERVAL  = 10          # seconds between full check cycles
MDNS_HOSTNAME   = "pypilotstatus"
BRIDGE_HOST     = "pypilot-bridge.local"
BRIDGE_PORT     = 20220
COMPASS_HOST    = "compass.local"
PYPILOT_PORT    = 23322
SK_PORT         = 3000
SERVO_DEV       = "/dev/pypilot-servo"
HOME            = os.path.expanduser("~")
PYPILOT_DIR     = os.path.join(HOME, "pypilot")
# ───────────────────────────────────────────────────────────────────────────

_lock   = threading.Lock()
_state  = {"layers": [], "summary": {}, "last_check": None, "check_count": 0}
_avahi  = None   # avahi-publish-address subprocess handle


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
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        return True
    except Exception:
        return False


def http_get(url, timeout=5):
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "pypilot-status/1"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.read().decode("utf-8", errors="replace"), r.status
    except Exception:
        return None, None


def ping(host, timeout=3):
    r = subprocess.run(
        ["ping", "-c1", f"-W{timeout}", host],
        capture_output=True, timeout=timeout + 2,
    )
    return r.returncode == 0


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


# ── Layer checks ─────────────────────────────────────────────────────────────

def check_layer1():
    """Layer 1: Signal K Server — heading sink & aggregator."""
    checks = []
    failed = False
    warned = False

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
        f"http://localhost:{SK_PORT}/signalk/v1/api/vessels/self/navigation/headingMagnetic"
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
        f"http://localhost:{SK_PORT}/signalk/v1/api/vessels/self/navigation/rateOfTurn"
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
        "subtitle": f"20 Hz heading sensor — {COMPASS_HOST}",
        "timing": "50 ms cycle: sensor→WiFi UDP→Signal K (~100 ms end-to-end this hop)",
        "status": status,
        "checks": checks,
        "links": [
            ("Compass dashboard", f"http://{COMPASS_HOST}/"),
        ],
    }


def check_layer3():
    """Layer 3: ESP32-C3 Motor Bridge."""
    checks = []
    failed = False
    warned = False

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

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "ESP32-C3 Motor Bridge",
        "subtitle": f"TCP→UART bridge — {BRIDGE_HOST}:{BRIDGE_PORT}",
        "timing": "~2–5 ms TCP to UART relay (transparent, 38400 baud)",
        "status": status,
        "checks": checks,
        "links": [
            ("Motor bridge console", f"http://{BRIDGE_HOST}/console"),
        ],
    }


def check_layer4():
    """Layer 4: socat virtual serial port."""
    checks = []
    failed = False
    warned = False

    if svc_active("pypilot-bridge.service"):
        checks.append(("ok", "pypilot-bridge.service running"))
    else:
        checks.append(("fail", "pypilot-bridge.service NOT running"))
        failed = True

    svc_content = read_file("/etc/systemd/system/pypilot-bridge.service")
    if "retry=forever" in svc_content:
        checks.append(("ok", "socat configured with retry=forever"))
    elif svc_content:
        checks.append(("warn", "socat NOT using retry=forever — will give up reconnecting"))
        warned = True

    if "StartLimitIntervalSec=0" in svc_content:
        checks.append(("ok", "StartLimitIntervalSec=0 — service will always restart"))

    if os.path.exists(SERVO_DEV):
        try:
            target = os.path.realpath(SERVO_DEV)
            checks.append(("ok", f"{SERVO_DEV} exists → {target}"))
        except Exception:
            checks.append(("ok", f"{SERVO_DEV} exists"))
        if os.access(SERVO_DEV, os.W_OK):
            checks.append(("ok", f"{SERVO_DEV} is writable"))
        else:
            checks.append(("fail", f"{SERVO_DEV} not writable — permission problem"))
            failed = True
    else:
        checks.append(("fail", f"{SERVO_DEV} does not exist"))
        failed = True

    serial_ports_path = os.path.join(HOME, ".pypilot", "serial_ports")
    content = read_file(serial_ports_path)
    if content and SERVO_DEV in content:
        checks.append(("ok", "~/.pypilot/serial_ports configured correctly"))
    elif content:
        checks.append(("warn", f"serial_ports exists but doesn't include {SERVO_DEV}"))
        warned = True
    else:
        checks.append(("warn", "~/.pypilot/serial_ports missing — PyPilot will scan all ports"))
        warned = True

    status = "fail" if failed else ("warn" if warned else "ok")
    return {
        "name": "socat Virtual Serial Port",
        "subtitle": f"PTY bridge: {SERVO_DEV} ↔ TCP → ESP32-C3",
        "timing": "&lt;1 ms PTY relay (kernel space, synchronous write)",
        "status": status,
        "checks": checks,
        "links": [],
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

    if tcp_open("localhost", PYPILOT_PORT):
        checks.append(("ok", f"PyPilot JSON server on TCP :{PYPILOT_PORT}"))
    else:
        checks.append(("fail", f"PyPilot TCP :{PYPILOT_PORT} not open"))
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
        "timing": "~545 ms command cycle (servo.period = 0.5455 s)",
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


def checker_loop():
    while True:
        try:
            layers, summary = run_all_checks()
            with _lock:
                _state["layers"]      = layers
                _state["summary"]     = summary
                _state["last_check"]  = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                _state["check_count"] += 1
        except Exception as e:
            print(f"[checker] Error: {e}", flush=True)
        time.sleep(CHECK_INTERVAL)


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
          (L.timing ? '<div class="timing">&#8987; ' + L.timing + '</div>' : '') +
          (links ? '<div class="links">' + links + '</div>' : '') +
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


# ── mDNS via avahi-publish-address ───────────────────────────────────────────

def start_mdns():
    """Register pypilotstatus.local via avahi-publish-address (runs as child process)."""
    ip = local_ip()
    print(f"[mDNS] Registering {MDNS_HOSTNAME}.local → {ip}", flush=True)
    try:
        proc = subprocess.Popen(
            ["avahi-publish-address", "-R", MDNS_HOSTNAME, ip],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return proc
    except FileNotFoundError:
        print("[mDNS] avahi-publish-address not found — install avahi-utils", flush=True)
        return None
    except Exception as e:
        print(f"[mDNS] Failed to start: {e}", flush=True)
        return None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _avahi

    # Start mDNS registration
    _avahi = start_mdns()

    # Signal handlers to clean up mDNS on exit
    def shutdown(sig, frame):
        print("\n[pypilot-status] Shutting down...", flush=True)
        if _avahi and _avahi.poll() is None:
            _avahi.terminate()
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
    print(f"[pypilot-status] Serving on http://{MDNS_HOSTNAME}.local  "
          f"(http://{host_ip}:{PORT})", flush=True)
    print(f"[pypilot-status] Checking every {CHECK_INTERVAL}s", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
