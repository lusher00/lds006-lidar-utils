#!/bin/bash
# Remove the lidar service from this Pi. Leaves the tree and /etc/default/lidar
# alone — this stops it running, it does not delete your settings.
set -e
sudo systemctl disable --now lidar 2>/dev/null || true
sudo rm -f /etc/systemd/system/lidar.service
sudo systemctl daemon-reload
echo "lidar service removed. /etc/default/lidar and $(pwd) untouched."
echo "The motor is stopped by the unit's ExecStopPost; if it is still spinning:"
echo "  python3 -c \"import serial; s=serial.Serial('/dev/ttyAMA0',115200); s.write(b'stoplds\$')\""
