#!/bin/bash
# Deploy lds006-lidar-utils from this Mac to the Raspberry Pi.
#
#   tools/deploy.sh [install|sync|test|status|stop|start]
#
#   install  (default) rsync, run ./install.sh on the Pi, show its status
#   sync     rsync the tree only; the running service is not touched
#   test     rsync, then run the test suite on the Pi
#   status   service state and /health; no sync
#   stop     stop the service (and the motor with it)
#   start    start the service
#
# Git lives on the Mac. The tree lands in ~/lds006-lidar-utils on the Pi as plain files;
# --delete keeps it an exact copy, which is why the Pi's own settings live in
# /etc/default/lidar and not in the tree.
#
# Host is an ssh alias (see ~/.ssh/config); override with
#   LIDAR_PI_SSH=... tools/deploy.sh ...
set -euo pipefail

PI=${LIDAR_PI_SSH:-pi5-0}
DIR=lds006-lidar-utils

cd "$(dirname "${BASH_SOURCE[0]}")/.."

action=${1:-install}

EXCLUDES=(--exclude .git --exclude __pycache__ --exclude '*.pyc' --exclude .DS_Store
          --exclude .vscode --exclude '*.swp' --exclude logs)

do_sync() {
    echo "== sync -> $PI:~/$DIR"
    rsync -az --delete "${EXCLUDES[@]}" ./ "$PI:$DIR/"
}

case "$action" in
  sync)   do_sync ;;
  install)
    do_sync
    echo "== install on $PI"
    ssh -t "$PI" "cd ~/$DIR && ./install.sh"
    ;;
  test)
    do_sync
    echo "== test on $PI"
    # The suite fakes the serial port, so it does not need the scanner and does
    # not fight the running service for it.
    ssh "$PI" "cd ~/$DIR && ./tests/run_tests.sh"
    ;;
  status)
    ssh "$PI" "systemctl status -n 0 --no-pager lidar; echo; curl -s localhost:8081/health | python3 -m json.tool"
    ;;
  stop)   ssh -t "$PI" "sudo systemctl stop lidar" ;;
  start)  ssh -t "$PI" "sudo systemctl start lidar" ;;
  *)      sed -n '2,20p' "$0"; exit 2 ;;
esac
