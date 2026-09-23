# lidar_aliases.sh — lidar service control, sourced from ~/.bashrc on the Pi.
#
# Same shape as tracker_aliases.sh next door: project-specific names only,
# nothing that belongs in a general shell. Sourced from the repo, so editing
# this file takes effect in the next shell with no reinstall.
#
#   [ -f ~/lds006-lidar-utils/config/lidar_aliases.sh ] && . ~/lds006-lidar-utils/config/lidar_aliases.sh

LIDAR_DIR="${LIDAR_DIR:-$HOME/lds006-lidar-utils}"
LIDAR_URL="${LIDAR_URL:-http://localhost:8081}"

#:: Lidar service
alias ld="cd $LIDAR_DIR"                                      #: cd to the project
alias lds='systemctl status -n 0 --no-pager lidar'            #: status
alias sld='sudo systemctl stop lidar'                         #: stop
alias rld='sudo systemctl restart lidar'                      #: restart
alias eld='sudo systemctl enable --now lidar'                 #: enable at boot and start
alias dld='sudo systemctl disable --now lidar'                #: disable at boot and stop

#:: Lidar logs
alias ldlog='journalctl -u lidar -f'                                  #: follow the service
alias lderr='journalctl -u lidar -p warning -n 200 --no-pager'        #: last 200 warnings and errors

#:: Lidar data
#: health as JSON, formatted
ldhealth() { curl -s "$LIDAR_URL/health" | python3 -m json.tool; }
#: one scan, as "angle distance_mm" lines, skipping stale and no-return bins
ldscan() { curl -s "$LIDAR_URL/scan" | python3 -c "
import json,sys
d=json.load(sys.stdin)
for a,mm in enumerate(d['dist']):
    if mm>0: print(f'{a:3d} {mm:6d}')
"; }
#: motor on / off without opening the dashboard
ldon()  { curl -s -X POST -H 'Content-Type: application/json' -d '{\"motor\":\"start\"}' "$LIDAR_URL/control"; echo; }
ldoff() { curl -s -X POST -H 'Content-Type: application/json' -d '{\"motor\":\"stop\"}'  "$LIDAR_URL/control"; echo; }

#: stop the service, then run the curses viewer (the port is single-owner)
ldview() { sudo systemctl stop lidar && python3 "$LIDAR_DIR/lidar_view.py"; }
