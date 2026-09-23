# lds006-lidar-utils

**An LDS-006 laser scanner, salvaged from an Ecovacs Deebot Ozmo 920 robot
vacuum, driven from a Raspberry Pi.** A small daemon owns the serial port,
decodes the scan, and serves it as JSON over HTTP — so the Pi's own web page
and the balance bot's dashboard both get a live 360° picture without either of
them going near `/dev/ttyAMA0`.

    LDS-006  ──UART 115200──▶  /dev/ttyAMA0  ──▶  lidar_server.py  ──HTTP :8081──▶  browsers
                                                        │
                                          360 bins of distance + signal,
                                          health counters, motor control

The LDS-006 is the spinning laser turret in the lid of an **Ecovacs Deebot
Ozmo 920**: 360 samples a revolution, roughly 5 revolutions a second, a few
metres of usable range, a 5-pin header, and no datasheet. It is the same
protocol family as the XV-11 / Neato units — close enough to recognise, and
different enough to catch you out. Most of this repo is the work of
establishing what it actually sends, so see **Protocol** below, including the
one thing that is still open.

Written for a Pi 5 running Ubuntu, alongside
[hailo-tracker](https://github.com/lusher00/hailo-tracker); the scan also shows
up on the [balance_bot](https://github.com/lusher00/balance_bot) dashboard. It
needs nothing from either of them.

**Status:** bring-up. The scan is displayed and its health is measured. Nothing
steers a robot with it yet.

---

## Quick start

    sudo apt install -y python3-serial
    ./install.sh                       # service, config, dialout group
    curl -s localhost:8081/health | python3 -m json.tool

Or, without installing anything:

    python3 lidar_server.py            # foreground, Ctrl-C to stop

The serial port is single-owner. The daemon and the bench tools cannot both
have it — `sudo systemctl stop lidar` before using `lidar_view.py` and friends.

## Wiring

| Scanner | Pi |
|---|---|
| TX | GPIO15 / RXD (pin 10) |
| RX | GPIO14 / TXD (pin 8) — only needed for `startlds$` / `stoplds$` |
| GND | GND |
| 5 V motor + 3V3 logic | per the module's own pinout, **not** from the Pi's 3V3 if the motor draws from it |

The primary UART has to be free: `enable_uart=1` in `/boot/firmware/config.txt`,
the serial console off (`sudo raspi-config` → Interface → Serial: login shell
**no**, hardware **yes**), and Bluetooth off the UART if you are on a model
where it steals it. `/dev/ttyAMA0` exists when that is right.

## The daemon

| Endpoint | What it gives |
|---|---|
| `GET /scan` | `dist[360]` in mm and `sig[360]`, one entry per degree, plus everything in `/health` |
| `GET /health` | motor state, `speed_raw`, silence, fresh-bin count, cumulative counters and their per-second rates |
| `POST /control` | `{"motor":"start"}` or `{"motor":"stop"}` |

`dist[a]` is `-1` where the bin has not been written in the last 1.5 s, and `0`
where the scanner reported no return. Both are "nothing there", for different
reasons, and neither should be plotted as a distance.

Every response carries `Access-Control-Allow-Origin: *`, because the bot
dashboard is served from the BeagleBone and is therefore always cross-origin.
This is a read-only scan and a motor switch on a robot's own network.

Settings live in `/etc/default/lidar` (see `config/lidar.env.example`), outside
the tree so a deploy cannot wipe them:

| Variable | Default | |
|---|---|---|
| `LIDAR_PORT` | `/dev/ttyAMA0` | serial device |
| `LIDAR_BAUD` | `115200` | fixed on this unit |
| `LIDAR_HTTP_PORT` | `8081` | 8080 is hailo-tracker on the same Pi |
| `LIDAR_AUTOSTART` | `1` | spin the motor as soon as the port opens |

## Protocol

Established on this unit, with the scripts in this repo:

* 115200 8N1. `startlds$` spins the motor up, `stoplds$` stops it. Nothing is
  streamed until it is started.
* 22-byte packets: `0xFA`, index, speed_lo, speed_hi, four 4-byte samples,
  checksum_lo, checksum_hi.
* index runs `0xA0`–`0xF9`: 90 packets × 4 samples = 360 samples a revolution.
  `angle = (index - 0xA0) * 4 + sample`.
* checksum = the arithmetic sum of bytes 0–19, 16-bit little endian. A packet
  that fails it is a framing slip, not a bad reading — which is why the daemon
  checks it *before* accepting a byte position as a packet boundary.
* Each sample is distance_lo, distance_hi, signal_lo, signal_hi.
* `speed_raw / 64` is the documented RPM. On this unit a healthy raw value sits
  near **21500**, which comes out as ~336 — not a believable scan rate for a
  unit doing about 5 revolutions a second. Treat it as a stability indicator
  until it has been checked against something physical; the daemon counts the
  real revolution rate from index wraps instead.

### The one thing that is not settled

`lidar_view.py` reads distance as the full 16-bit value in mm. The XV-11 /
Neato family this protocol descends from instead uses bit 15 as *invalid*,
bit 14 as *signal warning*, and the low 14 bits as the distance — which is
what `lidar2.py` and `lidar_table.py` assume. Both readings agree on everything
under 16 384 mm with clean returns, and disagree exactly where it matters.

The daemon reports the full 16-bit value and separately counts how often those
two top bits are set (`flag_hi`, `flag_warn` in `/health`):

* `flag_hi` stays at **0** while the scanner faces real surfaces → the
  full-16-bit reading is right, and the flag interpretation can be dropped.
* `flag_hi` climbs whenever something is out of range or absorbing → the flags
  are real, and distance should be masked to 14 bits.

Until that is answered, the viewers treat `0` as "no return" and clamp
anything past the chosen range, which is correct under either reading.

## Health, and what the numbers mean

`/health` exists because "the lidar is not working" has several distinct
causes that look identical from a plot:

| Field | Reads |
|---|---|
| `silence_s` | seconds since any byte arrived — wiring, power, or a stopped motor |
| `rates.packets` | good packets a second; zero with bytes arriving means framing, not connection |
| `rates.revs` | revolutions a second, counted from index wraps. The honest scan rate |
| `rates.checksum_bad` | rejected candidate boundaries a second. Non-zero = a dirty stream |
| `counts.index_bad` | packets whose index is outside `0xA0`–`0xF9` (`lidar_health.py` calls these speed errors) |
| `points_fresh` | how many of the 360 bins were written in the last 1.5 s. A stuck motor shows here first |
| `speed_raw` | the raw speed field, undivided — see above |

## Where the scan shows up

* **hailo-tracker page** (`http://<pi>:8080`) — a Lidar card: polar plot, range
  rings every metre, nearest return ahead, the health line, motor buttons. It
  talks to `<same host>:8081`, so the two services restart independently.
* **balance_bot dashboard** (served from the BeagleBone) — the System tab shows
  the nearest return in each 45° sector plus the health summary, polled from
  the Pi directly. No lidar data crosses robot-link, and none of it reaches the
  balance loop.

## Deploying from the Mac

Git lives on the Mac; the Pi gets plain files.

    tools/deploy.sh              # rsync + ./install.sh on the Pi
    tools/deploy.sh sync         # files only, service untouched
    tools/deploy.sh test         # rsync, then run the suite on the Pi
    tools/deploy.sh status       # service state + /health

Host defaults to the ssh alias `pi5-0`; override with `LIDAR_PI_SSH`. The same
four are VS Code tasks in `.vscode/tasks.json`.

Shell helpers for the Pi are in `config/lidar_aliases.sh` — service control,
`ldhealth`, `ldscan`, `ldon` / `ldoff`, and `ldview`, which stops the service
first because the port is single-owner. Source it from `~/.bashrc`.

## Testing

    ./tests/run_tests.sh

The suite fakes the serial port and runs the daemon for real — the framer, the
counters, the HTTP surface — so it needs no scanner, and it is safe to run on
the Pi while the service has the real port. It covers packet decoding and angle
mapping, staleness, the flag counters, the endpoints and their CORS headers, a
deliberately dirty stream (bad checksums, stray indices, garbage between
packets), and a missing serial port.

## Files

| File | |
|---|---|
| `lidar_server.py` | the daemon: serial → 360 bins → HTTP |
| `install.sh` / `uninstall.sh` | systemd unit, config file, dialout group |
| `systemd/lidar.service` | the unit; sends `stoplds$` on the way out |
| `config/lidar.env.example` | every setting, with its default |
| `config/lidar_aliases.sh` | Pi shell helpers |
| `tools/deploy.sh` | Mac → Pi |
| `tests/` | off-device suite and the fake serial port |

### Bench tools

These predate the daemon and are kept because they are still the fastest way
to answer one question each. All of them need the service stopped.

| Script | |
|---|---|
| `lidar_view.py` | curses polar plot, range rings, freeze on SPACE |
| `lidar_health.py` | packets/s, checksum errors, speed-error rate, speed bar |
| `lidar_table.py` | distances every 10°, as text |
| `check_stream.py` | stream-level sanity checking |
| `show_frames.py` | raw frames |
| `rawtest.py` | is anything arriving at all — bytes/s and silence |
| `lidar2.py` | the 14-bit-distance + flags reading of the protocol |
| `lidar.py` | a `5A A5` framing this unit does not use — reference only |

## Troubleshooting

**`/health` says `serial_error`** — the port is missing (`enable_uart`, serial
console) or the user is not in `dialout`. Group changes only apply to new
logins; reboot or log out and back in.

**Bytes arrive, no packets** — `silence_s` near 0 with `rates.packets` at 0 is
a framing problem, not a wiring one: wrong baud, or something else holding the
port.

**Packets arrive, `points_fresh` is low** — the motor is not turning, or is
turning far too slowly. Check `rates.revs` and the raw speed.

**Nothing at all after a reboot** — the motor needs `startlds$` every time the
scanner is powered up. `LIDAR_AUTOSTART=1` does that when the port opens.

**Dashboard says "daemon not reachable"** — the Pi's host or port in that
card's field, or the service is down. `/health` from the Pi itself settles
which.

## License

PolyForm Noncommercial 1.0.0 — free for personal, educational and open-source
use. Commercial use requires written permission: ryan.lush@gmail.com
