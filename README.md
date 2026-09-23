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
where the scanner reported no return (bit 15 set, or a genuine zero). Both are
"nothing there", for different reasons, and neither should be plotted as a
distance. Everything else is millimetres, 14-bit, so the ceiling is 16 383.

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
* Each sample is distance_lo, distance_hi, signal_lo, signal_hi, and the
  distance word carries flags in its top two bits, XV-11 style: **bit 15 = no
  valid return, bit 14 = weak signal, low 14 bits = millimetres.**
* **`speed_raw / 100` is RPM on this unit**, not the `/64` the XV-11 family
  uses. The daemon also counts the revolution rate from index wraps and
  reports both, so the claim can be re-checked instead of believed.

### How those last two were settled

Both were open questions when the daemon was written — `lidar_view.py` read
the full 16-bit distance, `lidar2.py` and `lidar_table.py` masked to 14 bits
and used the flags, and `speed_raw / 64` produced a number nothing else
agreed with. Rather than pick one, the daemon was made to count the evidence.
Fifty seconds of the real scanner running in a room:

    packets    22347        ->  4.97 rev/s  =  298 RPM
    speed_raw  29946        ->  /64 = 468   /100 = 299.5
    bit 15 set on 52% of samples
    zero distances            0

**Distance is 14-bit with flags.** A unit that never once reports a zero, but
sets bit 15 on half its samples, is using the flag to say "no return" — and
read as a full 16-bit value those samples would be 32–65 m, which this module
cannot see. `lidar_view.py` was wrong; `lidar2.py` was right.

**The speed divisor is 100, not 64.** The revolution rate counted from index
wraps is the check: 298 RPM measured against 299.5 from `/100`, while `/64`
claims 468.

`/health` still reports both: `rpm` is counted from index wraps, `rpm_reported`
is the module's own figure. If they ever disagree on a different unit, the
counted one is the one to believe.

## Health, and what the numbers mean

`/health` exists because "the lidar is not working" has several distinct
causes that look identical from a plot:

| Field | Reads |
|---|---|
| `silence_s` | seconds since any byte arrived — wiring, power, or a stopped motor |
| `rates.packets` | good packets a second; zero with bytes arriving means framing, not connection |
| `rates.revs` | revolutions a second, counted from index wraps. The honest scan rate |
| `rates.checksum_bad` | rejected candidate boundaries a second. Non-zero = a dirty stream |
| `rates.invalid` | samples a second with bit 15 set — no return. Half of them indoors is normal; all of them means the laser is blocked or the room is beyond range |
| `rates.weak` | samples with bit 14 set — a return the module distrusts. Dark or glancing surfaces |
| `spinning` | whether wraps are still arriving. `motor` is only what the daemon last *told* it to do — an unpowered scanner accepts `startlds$` in silence |
| `counts.index_bad` | packets whose index is outside `0xA0`–`0xF9` (`lidar_health.py` calls these speed errors) |
| `points_fresh` | how many of the 360 bins were written in the last 1.5 s. A stuck motor shows here first |
| `rpm` | counted from index wraps. The honest one |
| `rpm_reported` | `speed_raw / 100`, the module's own figure |
| `speed_raw` | the raw field, undivided |

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
| `lidar2.py` | the 14-bit-distance + flags reading — the one that turned out to be right |
| `lidar.py` | a `5A A5` framing this unit does not use — reference only |

`lidar_view.py`, `lidar_health.py` and `lidar_table.py` predate the findings
above: the first two read the full 16-bit distance, so a no-return sample
plots as tens of metres rather than disappearing, and all three still divide
speed by 64. They are useful for what they were written for — is it spinning,
is the stream clean, what is the range at this bearing — but the daemon is the
one that decodes this unit correctly.

## Troubleshooting

**`/health` says `serial_error`** — the port is missing (`enable_uart`, serial
console) or the user is not in `dialout`. Group changes only apply to new
logins; reboot or log out and back in.

**Bytes arrive, no packets** — `silence_s` near 0 with `rates.packets` at 0 is
a framing problem, not a wiring one: wrong baud, or something else holding the
port.

**Packets arrive, `points_fresh` is low** — the motor is not turning, or is
turning far too slowly. Check `rpm` and `rates.revs`.

**Everything looks healthy but every distance is 0** — `rates.invalid` will be
running at the full sample rate: the laser is blocked, or nothing is within
range. `motor: "running"` with `spinning: false` means the scanner is not
powered; the daemon can only say it sent the command.

**Nothing at all after a reboot** — the motor needs `startlds$` every time the
scanner is powered up. `LIDAR_AUTOSTART=1` does that when the port opens.

**Dashboard says "daemon not reachable"** — the Pi's host or port in that
card's field, or the service is down. `/health` from the Pi itself settles
which.

## License

PolyForm Noncommercial 1.0.0 — free for personal, educational and open-source
use. Commercial use requires written permission: ryan.lush@gmail.com
