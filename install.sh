#!/bin/bash
# Install the lidar daemon as a system service on this Pi.
#
#   ./install.sh
#
# Run it as the user the service should run as, not with sudo: the unit is
# written for whoever runs this, and the serial port needs that user in the
# dialout group.
set -e

SERVICE_NAME="lidar"
SCRIPT_NAME="lidar_server.py"

if [ "$EUID" -eq 0 ]; then
    echo "Run as a regular user, not sudo — the unit runs as whoever installs it."
    exit 1
fi

INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER=$(whoami)
PORT_DEV="${LIDAR_PORT:-/dev/ttyAMA0}"

echo "Install directory: ${INSTALL_DIR}"
echo "User:              ${RUN_USER}"
echo "Serial port:       ${PORT_DEV}"

# ── preflight ────────────────────────────────────────────────────────────
if ! /usr/bin/python3 -c "import serial" >/dev/null 2>&1; then
    echo "pyserial missing. Installing…"
    sudo apt-get install -y python3-serial || \
        /usr/bin/python3 -m pip install --break-system-packages pyserial
fi

if [ ! -e "${PORT_DEV}" ]; then
    echo "Warning: ${PORT_DEV} does not exist."
    echo "         On a Pi 5 the primary UART needs 'enable_uart=1' and the serial"
    echo "         console off — see README, Wiring."
fi

# The port is root:dialout. Without this the daemon opens nothing and says so
# once every two seconds in the journal.
if ! id -nG "${RUN_USER}" | tr ' ' '\n' | grep -qx dialout; then
    echo "Adding ${RUN_USER} to the dialout group…"
    sudo usermod -aG dialout "${RUN_USER}"
    echo "NOTE: group membership only applies to NEW logins. Log out and back in,"
    echo "      or the service will still be denied the port until the next boot."
fi

# ── config ───────────────────────────────────────────────────────────────
# Settings live outside the tree so a deploy (rsync --delete) cannot wipe them.
if [ ! -f /etc/default/lidar ]; then
    echo "Installing /etc/default/lidar from config/lidar.env.example"
    sudo install -m 644 "${INSTALL_DIR}/config/lidar.env.example" /etc/default/lidar
else
    echo "/etc/default/lidar exists — left alone"
fi

# ── unit ─────────────────────────────────────────────────────────────────
echo "Installing ${SERVICE_NAME}.service"
sed -e "s#^User=.*#User=${RUN_USER}#" \
    -e "s#^WorkingDirectory=.*#WorkingDirectory=${INSTALL_DIR}#" \
    -e "s#^ExecStart=.*#ExecStart=/usr/bin/python3 ${INSTALL_DIR}/${SCRIPT_NAME}#" \
    "${INSTALL_DIR}/systemd/${SERVICE_NAME}.service" | sudo tee /etc/systemd/system/${SERVICE_NAME}.service >/dev/null

sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}
sudo systemctl restart ${SERVICE_NAME}
sleep 1
systemctl --no-pager -l status ${SERVICE_NAME} | head -15

echo
echo "  http://$(hostname):$(grep -E '^LIDAR_HTTP_PORT=' /etc/default/lidar 2>/dev/null | cut -d= -f2 | tr -d '\"' || echo 8081)/health"
