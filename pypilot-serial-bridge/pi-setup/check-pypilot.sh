#!/usr/bin/env bash
# =============================================================================
#  check-pypilot.sh  —  PyPilot Autopilot System Diagnostic
#
#  Tests every layer of the communication stack in order.
#  Run this whenever something stops working.
#
#  Usage:
#    ./check-pypilot.sh          # diagnose only
#    ./check-pypilot.sh --fix    # diagnose + attempt automatic fixes
#    ./check-pypilot.sh --restart-all   # restart all services then check
# =============================================================================
set -euo pipefail

# ---- Config (edit if your setup differs) ------------------------------------
BRIDGE_HOST="pypilot-bridge.local"
BRIDGE_PORT="20220"
COMPASS_HOST="compass.local"
PYPILOT_PORT="23322"
SK_PORT="3000"
SERVO_DEV="/dev/pypilot-servo"
PYPILOT_DIR="/home/brian/pypilot"
# -----------------------------------------------------------------------------

FIX=false
RESTART_ALL=false
for arg in "$@"; do
    [[ "$arg" == "--fix" ]]         && FIX=true
    [[ "$arg" == "--restart-all" ]] && RESTART_ALL=true
done

# ---- Colours ----------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

PASS=0; FAIL=0; WARN=0

pass()  { echo -e "  ${GREEN}✓${RESET}  $*"; ((PASS++))  || true; }
fail()  { echo -e "  ${RED}✗${RESET}  $*"; ((FAIL++))   || true; }
warn()  { echo -e "  ${YELLOW}!${RESET}  $*"; ((WARN++)) || true; }
info()  { echo -e "  ${CYAN}→${RESET}  $*"; }
layer() { echo; echo -e "${BOLD}━━━  $* ${RESET}"; }
fix()   { if $FIX; then info "FIX: $*"; eval "$*"; else info "To fix: $*"; fi; }

# ---- Helpers ----------------------------------------------------------------
svc_is_active() { systemctl is-active --quiet "$1" 2>/dev/null; }
svc_is_failed() { systemctl is-failed --quiet "$1" 2>/dev/null; }
tcp_open()      { nc -z -w3 "$1" "$2" 2>/dev/null; }
ping_host()     { ping -c1 -W3 "$1" &>/dev/null; }
http_ok()       { curl -sf --max-time 5 "$1" &>/dev/null; }
age_secs() {
    # Return seconds since a timestamp string (ISO 8601 or epoch)
    local ts="$1"
    local epoch
    epoch=$(date -d "$ts" +%s 2>/dev/null) || epoch=0
    echo $(( $(date +%s) - epoch ))
}

# =============================================================================
echo
echo -e "${BOLD}╔══════════════════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}║        PyPilot Autopilot System — Diagnostic Tool       ║${RESET}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════════╝${RESET}"
echo -e "  Date : $(date)"
echo -e "  Mode : $(if $FIX; then echo 'DIAGNOSE + AUTO-FIX'; else echo 'DIAGNOSE only  (add --fix to auto-repair)'; fi)"

# =============================================================================
#  Optional: restart everything first
# =============================================================================
if $RESTART_ALL; then
    echo
    echo -e "${BOLD}⟳  Restarting all services...${RESET}"
    for svc in signalk.service pypilot-bridge.service pypilot.service pypilot-web.service; do
        systemctl reset-failed "$svc" 2>/dev/null || true
        systemctl restart "$svc" 2>/dev/null && echo -e "  Restarted $svc" || echo -e "  Failed to restart $svc (needs sudo?)"
    done
    sleep 5
fi

# =============================================================================
layer "LAYER 1 — Signal K Server (heading sink + aggregator)"
# =============================================================================

if svc_is_active signalk.service; then
    pass "signalk.service is running"
else
    fail "signalk.service is NOT running"
    if svc_is_failed signalk.service; then
        info "Service is in failed state. Log:"
        journalctl -u signalk.service -n 5 --no-pager 2>/dev/null | sed 's/^/       /'
    fi
    fix "systemctl reset-failed signalk.service; systemctl start signalk.service"
fi

if http_ok "http://localhost:${SK_PORT}/signalk"; then
    pass "Signal K API responding on port ${SK_PORT}"
else
    fail "Signal K HTTP not responding on port ${SK_PORT}"
    info "Try: systemctl restart signalk.service"
fi

# Check for heading data and how stale it is
HDG_JSON=$(curl -sf --max-time 5 "http://localhost:${SK_PORT}/signalk/v1/api/vessels/self/navigation/headingMagnetic" 2>/dev/null || true)
if [[ -n "$HDG_JSON" ]]; then
    TIMESTAMP=$(echo "$HDG_JSON" | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('timestamp',''))" 2>/dev/null || true)
    VALUE=$(echo "$HDG_JSON" | python3 -c "import sys,json,math; d=json.load(sys.stdin); print(round(math.degrees(d.get('value',0)),1))" 2>/dev/null || true)
    if [[ -n "$TIMESTAMP" ]]; then
        AGE=$(age_secs "$TIMESTAMP")
        if (( AGE < 10 )); then
            pass "navigation.headingMagnetic = ${VALUE}°  (${AGE}s ago)"
        elif (( AGE < 60 )); then
            warn "navigation.headingMagnetic = ${VALUE}°  (${AGE}s ago — getting stale)"
        else
            fail "navigation.headingMagnetic last updated ${AGE}s ago — compass data has stopped"
            info "Check ESP32-S3 compass at http://${COMPASS_HOST}"
        fi
    else
        warn "navigation.headingMagnetic exists but has no timestamp"
    fi
else
    fail "navigation.headingMagnetic not in Signal K — compass data not arriving"
    # Diagnose the specific cause
    # Check what ports Signal K is listening on for Signal K delta
    SK_DELTA_PORTS=$(python3 -c "
import json
with open('${HOME}/.signalk/settings.json') as f:
    s = json.load(f)
ports = []
for p in s.get('pipedProviders', []):
    if not p.get('enabled', False):
        continue
    for el in p.get('pipeElements', []):
        opts = el.get('options', {})
        sub = opts.get('subOptions', opts)
        if opts.get('type') == 'SignalK' and sub.get('type') == 'udp':
            ports.append(str(sub.get('port','')))
print(','.join(ports) if ports else 'none')
" 2>/dev/null || echo "unknown")
    info "Signal K delta UDP inputs active on port(s): ${SK_DELTA_PORTS}"
    # Try to read the compass's configured port from its web API
    COMPASS_SK_PORT=$(python3 -c "
import urllib.request, json
try:
    r = urllib.request.urlopen('http://${COMPASS_HOST}/api/config', timeout=3)
    data = json.load(r)
    print(data.get('skport', data.get('signalkPort', '?')))
except:
    # Try to parse from dashboard HTML
    import re
    r2 = urllib.request.urlopen('http://${COMPASS_HOST}/', timeout=3)
    m = re.search(r'Signal K Port.*?(\d+)', r2.read().decode(), re.S)
    print(m.group(1) if m else '?')
" 2>/dev/null || echo "?")
    info "ESP32-S3 is broadcasting to port: ${COMPASS_SK_PORT}"
    if [[ "$SK_DELTA_PORTS" != "unknown" && "$COMPASS_SK_PORT" != "?" ]]; then
        if echo "$SK_DELTA_PORTS" | grep -q "$COMPASS_SK_PORT"; then
            warn "Ports match — data should be flowing. Check ESP32-S3 is on WiFi and headings are valid."
        else
            fail "PORT MISMATCH: ESP32-S3 sends to port ${COMPASS_SK_PORT} but Signal K listens on ${SK_DELTA_PORTS}"
            info "Fix A (change compass): http://${COMPASS_HOST}/setport?port=${SK_DELTA_PORTS%%,*}"
            info "Fix B (add SK input):   http://localhost:${SK_PORT}/admin/#/serverConfiguration/connections/-"
            info "   → Add Signal K (UDP) input on port ${COMPASS_SK_PORT}"
        fi
    else
        info "Signal K admin: http://localhost:${SK_PORT}/admin/#/serverConfiguration/connections/-"
        info "Ensure a 'Signal K (UDP)' input exists on the same port the compass is broadcasting to"
        info "Change compass port: http://${COMPASS_HOST}/setport?port=NNNN"
    fi
fi

# ROT check
ROT_JSON=$(curl -sf --max-time 5 "http://localhost:${SK_PORT}/signalk/v1/api/vessels/self/navigation/rateOfTurn" 2>/dev/null || true)
[[ -n "$ROT_JSON" ]] && pass "navigation.rateOfTurn present in Signal K" || warn "navigation.rateOfTurn absent (optional, used by autopilot)"

# =============================================================================
layer "LAYER 2 — ESP32-S3 BNO085 Compass (${COMPASS_HOST})"
# =============================================================================

if ping_host "$COMPASS_HOST"; then
    COMPASS_IP=$(ping -c1 -W3 "$COMPASS_HOST" 2>/dev/null | grep -oP '\(\K[0-9.]+(?=\))' | head -1)
    pass "${COMPASS_HOST} reachable at ${COMPASS_IP}"
else
    fail "${COMPASS_HOST} not reachable — compass ESP32-S3 offline or wrong mDNS name"
    warn "Check: is the ESP32-S3 plugged in and on this WiFi network?"
    info "LED should be pulsing green. If red/yellow, check USB power."

fi

if http_ok "http://${COMPASS_HOST}"; then
    pass "ESP32-S3 web dashboard responding"
else
    warn "ESP32-S3 web dashboard not responding (WiFi may still be fine)"
fi

# =============================================================================
layer "LAYER 3 — ESP32-C3 Motor Bridge (${BRIDGE_HOST})"
# =============================================================================

if ping_host "$BRIDGE_HOST"; then
    BRIDGE_IP=$(ping -c1 -W3 "$BRIDGE_HOST" 2>/dev/null | grep -oP '\(\K[0-9.]+(?=\))' | head -1)
    pass "${BRIDGE_HOST} reachable at ${BRIDGE_IP}"
    if http_ok "http://${BRIDGE_HOST}"; then
        pass "ESP32-C3 web dashboard responding  →  http://${BRIDGE_HOST}"
    else
        warn "ESP32-C3 web dashboard not responding"
    fi
else
    fail "${BRIDGE_HOST} NOT reachable — ESP32-C3 offline or wrong mDNS name"
    warn "Check: is the ESP32-C3 plugged in and on this WiFi network?"
    info "LED should be solid blue. If flashing cyan, it is in BLE setup mode."
    info "Status LEDs: blue=bridging, cyan=BLE, red=error, yellow=connecting"
fi

if tcp_open "$BRIDGE_HOST" "$BRIDGE_PORT"; then
    pass "TCP port ${BRIDGE_PORT} open on ${BRIDGE_HOST} — ESP32-C3 bridge accepting connections"
else
    fail "TCP port ${BRIDGE_PORT} NOT open — ESP32-C3 is online but bridge not listening"
    info "Check ESP32-C3 debug log: pio device monitor (GPIO USB) or web console"
fi

# =============================================================================
layer "LAYER 4 — socat Virtual Serial Port (${SERVO_DEV})"
# =============================================================================

if svc_is_active pypilot-bridge.service; then
    pass "pypilot-bridge.service is running"
else
    fail "pypilot-bridge.service is NOT running"
    if svc_is_failed pypilot-bridge.service; then
        info "Service is in FAILED state (hit restart limit). Last log:"
        journalctl -u pypilot-bridge.service -n 8 --no-pager 2>/dev/null | tail -8 | sed 's/^/       /'
        info "This happens when the ESP32-C3 was offline and socat gave up."
        fix "systemctl reset-failed pypilot-bridge.service && systemctl start pypilot-bridge.service"
    else
        info "Service stopped. Starting..."
        fix "systemctl start pypilot-bridge.service"
    fi

    # Check if the installed service has the old retry=10 (known bug)
    if grep -q "retry=10" /etc/systemd/system/pypilot-bridge.service 2>/dev/null; then
        warn "KNOWN BUG in installed service: uses retry=10 (socat quits after 10 failed attempts)"
        info "This is why it keeps dying whenever the ESP32-C3 reboots."
        fix "sed -i 's/retry=forever/retry=2147483647/' /etc/systemd/system/pypilot-bridge.service && sed -i 's/retry=10/retry=2147483647/' /etc/systemd/system/pypilot-bridge.service && systemctl daemon-reload"
    fi

    if ! grep -q "StartLimitIntervalSec=0" /etc/systemd/system/pypilot-bridge.service 2>/dev/null; then
        warn "Missing StartLimitIntervalSec=0 — systemd will stop restarting after repeated failures"
        info "Fix: add 'StartLimitIntervalSec=0' under [Unit] in /etc/systemd/system/pypilot-bridge.service"
    fi
fi

if [[ -e "$SERVO_DEV" ]]; then
    if [[ -c "$SERVO_DEV" || -L "$SERVO_DEV" ]]; then
        TARGET=$(readlink -f "$SERVO_DEV" 2>/dev/null || echo "?")
        pass "${SERVO_DEV} exists → ${TARGET}"
    else
        warn "${SERVO_DEV} exists but is not a character device"
    fi

    if [[ -w "$SERVO_DEV" ]]; then
        pass "${SERVO_DEV} is writable by current user"
    else
        fail "${SERVO_DEV} is NOT writable — permission problem"
        info "Permissions: $(ls -la "$SERVO_DEV" 2>/dev/null)"
        fix "sudo chmod 666 ${SERVO_DEV}"
    fi
else
    fail "${SERVO_DEV} does not exist"
    if svc_is_active pypilot-bridge.service; then
        warn "Service is active but PTY not created yet — waiting..."
        sleep 3
        if [[ -e "$SERVO_DEV" ]]; then pass "${SERVO_DEV} appeared after waiting"; fi
    else
        info "Start pypilot-bridge.service to create it"
    fi
fi

# Check that pypilot's serial_ports config points here
SERIAL_PORTS_FILE="$HOME/.pypilot/serial_ports"
if [[ -f "$SERIAL_PORTS_FILE" ]]; then
    if grep -q "$SERVO_DEV" "$SERIAL_PORTS_FILE"; then
        pass "PyPilot serial_ports configured to use ${SERVO_DEV}"
    else
        fail "PyPilot serial_ports does NOT include ${SERVO_DEV}"
        info "Contents: $(cat "$SERIAL_PORTS_FILE")"
        fix "echo '${SERVO_DEV}' >> ${SERIAL_PORTS_FILE}"
    fi
else
    fail "~/.pypilot/serial_ports not found — PyPilot will scan all serial ports (slow)"
    info "Create it with: echo '${SERVO_DEV}' > ~/.pypilot/serial_ports"
    fix "mkdir -p ~/.pypilot && echo '${SERVO_DEV}' > ~/.pypilot/serial_ports"
fi

# =============================================================================
layer "LAYER 5 — PyPilot Autopilot (pypilot.service)"
# =============================================================================

if svc_is_active pypilot.service; then
    pass "pypilot.service is running"
else
    fail "pypilot.service is NOT running"
    svc_is_failed pypilot.service && journalctl -u pypilot.service -n 5 --no-pager 2>/dev/null | tail -5 | sed 's/^/       /'
    fix "systemctl reset-failed pypilot.service; systemctl start pypilot.service"
fi

if tcp_open localhost "$PYPILOT_PORT"; then
    pass "PyPilot JSON server responding on TCP ${PYPILOT_PORT}"
else
    fail "PyPilot TCP port ${PYPILOT_PORT} not open — autopilot not ready yet"
    info "Try: systemctl restart pypilot.service  (takes ~5s to start)"
fi

# Check IMU source
IMU_SRC=$(python3 -c "
from pypilot.client import pypilotClient
import time, sys
sys.path.insert(0, '${PYPILOT_DIR}')
try:
    c = pypilotClient('localhost')
    c.watch('imu.source')
    time.sleep(1.5)
    msgs = c.receive()
    print(msgs.get('imu.source', 'unknown'))
except:
    print('error')
" 2>/dev/null)

if [[ "$IMU_SRC" == "signalk" ]]; then
    pass "PyPilot IMU source = signalk (using BNO085 via Signal K)"
elif [[ "$IMU_SRC" == "local" || "$IMU_SRC" == "unknown" ]]; then
    warn "PyPilot IMU source = '${IMU_SRC}' — not using BNO085 from Signal K"
    info "Run: python3 ${PYPILOT_DIR}/pypilot/client.py imu.source=signalk"
    info "Or add a config entry so pypilot remembers this on restart"
elif [[ "$IMU_SRC" == "error" ]]; then
    warn "Could not query PyPilot IMU source (pypilot may still be starting)"
fi

# =============================================================================
layer "LAYER 6 — PyPilot Web Interface (pypilot-web.service)"
# =============================================================================

if svc_is_active pypilot-web.service; then
    pass "pypilot-web.service is running"
else
    fail "pypilot-web.service is NOT running"
    fix "systemctl reset-failed pypilot-web.service; systemctl start pypilot-web.service"
fi

for port in 8080 80; do
    if tcp_open localhost "$port" 2>/dev/null; then
        pass "PyPilot web UI responding on port ${port}  →  http://localhost:${port}"
        break
    fi
done

# =============================================================================
layer "LAYER 7 — End-to-End Motor Controller Connectivity"
# =============================================================================

if [[ -e "$SERVO_DEV" ]] && [[ -w "$SERVO_DEV" ]] && svc_is_active pypilot.service; then
    # Try a quick read — if socat is truly bridged we should see traffic within 2s
    BYTES=$(timeout 2 bash -c "cat $SERVO_DEV 2>/dev/null | wc -c" 2>/dev/null || echo "0")
    if (( BYTES > 0 )); then
        pass "Motor controller traffic flowing on ${SERVO_DEV}  (${BYTES} bytes in 2s)"
    else
        warn "No bytes seen on ${SERVO_DEV} in 2s — motor controller may be idle or disconnected"
        info "Motor controller only transmits when PyPilot sends a command or requests telemetry"
        info "Enable autopilot or use pypilot_web to send a test command"
    fi
else
    warn "Skipping motor traffic test — preconditions not met"
fi

# =============================================================================
#  SUMMARY
# =============================================================================
echo
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
echo -e "  ${GREEN}PASS: ${PASS}${RESET}   ${RED}FAIL: ${FAIL}${RESET}   ${YELLOW}WARN: ${WARN}${RESET}"
echo
if (( FAIL == 0 && WARN == 0 )); then
    echo -e "  ${GREEN}${BOLD}All systems GO. Full autopilot chain is operational.${RESET}"
elif (( FAIL == 0 )); then
    echo -e "  ${YELLOW}${BOLD}System is operational but has warnings — review above.${RESET}"
else
    echo -e "  ${RED}${BOLD}${FAIL} layer(s) FAILED — autopilot will NOT function correctly.${RESET}"
    echo -e "  Re-run with ${BOLD}--fix${RESET} to attempt automatic repairs."
fi
echo
echo -e "  Full system: ${CYAN}http://localhost:${SK_PORT}/admin/#/databrowser${RESET}  (Signal K)"
echo -e "  Autopilot:   ${CYAN}http://localhost:8080${RESET}  (PyPilot web)"
echo -e "  Bridge log:  ${CYAN}journalctl -u pypilot-bridge.service -f${RESET}"
echo
