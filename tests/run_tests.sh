#!/bin/bash
# Off-device test harness. Fakes the serial port and runs the daemon for real,
# so it needs no scanner and does not fight the service for the port.
#
#   ./tests/run_tests.sh
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python3 tests/test_lidar_server.py "$@"
