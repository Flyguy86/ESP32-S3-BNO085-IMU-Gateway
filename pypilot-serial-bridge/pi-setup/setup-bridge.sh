#!/bin/bash
# =============================================================================
# setup-bridge.sh — Install on the Raspberry Pi running pypilot
#
# Creates a virtual serial port at /dev/pypilot-servo that tunnels
# to the ESP32-C3 WiFi serial bridge via TCP.
#
# Usage:  scp -r pi-setup/ pi@pypilot.local:~/
#         ssh pi@pypilot.local 'bash ~/pi-setup/setup-bridge.sh'
# =============================================================================
set -e

BRIDGE_HOST="${1:-pypilot-bridge.local}"
BRIDGE_PORT="${2:-20220}"
VIRT_PORT="/dev/pypilot-servo"
SERVICE_FILE="pypilot-bridge.service"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo "  PyPilot TCP Bridge Setup"
echo "  Bridge: ${BRIDGE_HOST}:${BRIDGE_PORT}"
echo "  Virtual serial: ${VIRT_PORT}"
echo "============================================"

# 1. Install dependencies
echo "[1/6] Installing dependencies..."
NEED_UPDATE=false
for pkg in socat avahi-utils; do
    if ! dpkg -l "$pkg" &>/dev/null; then
        NEED_UPDATE=true
    fi
done
$NEED_UPDATE && sudo apt-get update -qq
for pkg in socat avahi-utils; do
    if ! command -v "${pkg%%-*}" &>/dev/null && ! dpkg -l "$pkg" &>/dev/null; then
        sudo apt-get install -y -qq "$pkg" && echo "  Installed $pkg"
    else
        echo "  $pkg already installed ✓"
    fi
done

# 2. Update service file with provided host/port
echo "[2/6] Installing pypilot-bridge systemd service..."
sed "s|pypilot-bridge.local:20220|${BRIDGE_HOST}:${BRIDGE_PORT}|g" \
    "${SCRIPT_DIR}/${SERVICE_FILE}" | sudo tee /etc/systemd/system/${SERVICE_FILE} > /dev/null

# 3. Enable and start the bridge service
echo "[3/6] Enabling pypilot-bridge service..."
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_FILE}
sudo systemctl restart ${SERVICE_FILE}
sleep 2

# 4. Verify the virtual port appeared
echo "[4/6] Checking virtual serial port..."
if [ -e "${VIRT_PORT}" ]; then
    echo "  ✓ ${VIRT_PORT} exists"
    ls -la "${VIRT_PORT}"
else
    echo "  ⚠ ${VIRT_PORT} not created yet (bridge may not be reachable)"
    echo "  Check: sudo journalctl -u ${SERVICE_FILE} -f"
fi

# 5. Configure pypilot to use the virtual port
echo "[5/6] Configuring pypilot serial_ports..."
PYPILOT_DIR="${HOME}/.pypilot"
mkdir -p "${PYPILOT_DIR}"

SERIAL_PORTS_FILE="${PYPILOT_DIR}/serial_ports"
if [ -f "${SERIAL_PORTS_FILE}" ]; then
    if grep -q "${VIRT_PORT}" "${SERIAL_PORTS_FILE}"; then
        echo "  ✓ ${VIRT_PORT} already in serial_ports"
    else
        echo "  Adding ${VIRT_PORT} to existing serial_ports file"
        echo "${VIRT_PORT}" >> "${SERIAL_PORTS_FILE}"
    fi
else
    echo "${VIRT_PORT}" > "${SERIAL_PORTS_FILE}"
    echo "  Created ${SERIAL_PORTS_FILE}"
fi
echo "  serial_ports: $(cat "${SERIAL_PORTS_FILE}")"

echo ""
echo "============================================"
echo "  Setup complete!"
echo ""

# 6. Install pypilot-status web monitor
echo "[6/6] Installing pypilot-status web monitor..."
STATUS_SCRIPT="${SCRIPT_DIR}/pypilot-status.py"
STATUS_SERVICE="${SCRIPT_DIR}/pypilot-status.service"
STATUS_INSTALL_DIR="/opt/pypilot-status"

if [ -f "${STATUS_SCRIPT}" ]; then
    sudo mkdir -p "${STATUS_INSTALL_DIR}"
    sudo cp "${STATUS_SCRIPT}" "${STATUS_INSTALL_DIR}/pypilot-status.py"
    sudo chmod 755 "${STATUS_INSTALL_DIR}/pypilot-status.py"
    echo "  Installed ${STATUS_INSTALL_DIR}/pypilot-status.py"
else
    echo "  ⚠ pypilot-status.py not found in ${SCRIPT_DIR} — skipping"
fi

if [ -f "${STATUS_SERVICE}" ]; then
    # Patch the User= field to match the current user
    sed "s|^User=.*|User=${USER}|; s|^Environment=HOME=.*|Environment=HOME=${HOME}|" \
        "${STATUS_SERVICE}" | sudo tee /etc/systemd/system/pypilot-status.service > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable pypilot-status.service
    sudo systemctl restart pypilot-status.service
    sleep 2
    if systemctl is-active --quiet pypilot-status.service; then
        echo "  ✓ pypilot-status.service running → http://pypilotstatus.local"
    else
        echo "  ⚠ pypilot-status.service failed to start (check: journalctl -u pypilot-status.service)"
    fi
else
    echo "  ⚠ pypilot-status.service not found in ${SCRIPT_DIR} — skipping"
fi

echo ""
echo "============================================"
echo "  Service status:"
sudo systemctl status ${SERVICE_FILE} --no-pager -l 2>/dev/null || true
echo ""
echo "  Next steps:"
echo "    sudo systemctl restart pypilot"
echo ""
echo "  Useful commands:"
echo "    sudo journalctl -u pypilot-bridge -f    # watch bridge logs"
echo "    sudo systemctl status pypilot-bridge     # check status"
echo "    pypilot_servo ${VIRT_PORT}               # test motor comms"
echo "============================================"
