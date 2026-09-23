#!/usr/bin/env python3
"""LDS-006 lidar daemon — reads the scanner, serves the latest scan over HTTP.

The scanner is the laser turret out of an Ecovacs Deebot Ozmo 920 vacuum.

Everything the curses tools in this folder do, done once, in a process that
outlives a terminal and can be read by a browser. The viewer in the Pi's
hailo-tracker page and the Lidar tab on the bot dashboard are both clients of
this; neither of them touches the serial port.

Protocol, as established by lidar_view.py / lidar_health.py on this unit:

  * 115200 8N1 on /dev/ttyAMA0. "startlds$" spins the motor up, "stoplds$"
    stops it. The scanner streams nothing until it is started.
  * 22-byte packets: 0xFA, index, speed_lo, speed_hi, then four 4-byte
    samples, then a 16-bit checksum.
  * index runs 0xA0..0xF9 — 90 packets, 4 samples each, 360 samples a
    revolution. angle = (index - 0xA0) * 4 + sample.
  * checksum = sum of bytes 0..19, 16-bit, little endian. Anything else is a
    framing slip, not a packet.
  * speed_raw / 100 is RPM on this unit, NOT the /64 the XV-11 family uses.
    Measured against the revolution rate counted from index wraps: 4.97 rev/s
    = 298 RPM with speed_raw at 29946. /64 would claim 468. The daemon
    reports the rate it counted alongside the raw field, so this can be
    re-checked rather than believed.
  * Each sample is distance_lo, distance_hi, signal_lo, signal_hi, and the
    distance word carries flags in its top two bits, XV-11 style:
    bit 15 = no valid return, bit 14 = weak signal, low 14 bits = mm.
    Established by measurement, not assumption: running in a room, bit 15 was
    set on 52% of samples while a zero distance never occurred once. Read as
    a full 16-bit value those samples are 32-65 m, which this module cannot
    see — so the flag reading is the right one, and a flagged sample is the
    unit's way of saying "nothing there".

HTTP, all JSON, all CORS-open so a dashboard served from the Bone can read it:

  GET  /scan     latest scan: 360 distances, 360 signal values, health
  GET  /health   health only — small enough to poll hard
  POST /control  {"motor":"start"|"stop"}
  GET  /         one line of plain text, so hitting it in a browser says what
                 this is

Env: LIDAR_PORT (serial, default /dev/ttyAMA0), LIDAR_BAUD, LIDAR_HTTP_PORT
(default 8081), LIDAR_AUTOSTART (default 1 — spin the motor at boot).
"""

import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

SERIAL_PORT = os.environ.get("LIDAR_PORT", "/dev/ttyAMA0")
BAUD = int(os.environ.get("LIDAR_BAUD", "115200"))
HTTP_PORT = int(os.environ.get("LIDAR_HTTP_PORT", "8081"))
AUTOSTART = os.environ.get("LIDAR_AUTOSTART", "1") not in ("0", "no", "false")

PACKET_LEN = 22
IDX_FIRST = 0xA0
IDX_LAST = 0xF9

# A bin older than this is reported as -1 rather than as a live reading: a
# stopped motor leaves a complete, plausible, and entirely stale scan behind.
STALE_S = 1.5


class Scanner:
    """Owns the serial port. One reader thread, one lock, no callers on it."""

    def __init__(self):
        self.lock = threading.Lock()
        self.dist = [0] * 360          # mm, 0 = no return
        self.sig = [0] * 360
        self.stamp = [0.0] * 360       # monotonic time each bin was written
        self.rev = 0
        self.speed_raw = 0
        self.motor = "unknown"
        self.ser = None
        self.serial_error = None
        # Event counters. Rates are derived from these by the /health handler,
        # which is what makes them cheap to poll: the reader never computes a
        # rate, it only counts.
        self.c = dict(bytes=0, packets=0, checksum_bad=0, index_bad=0,
                      revs=0, zero=0, invalid=0, weak=0)
        # Timestamps of the last few index wraps. The revolution rate counted
        # from these is the honest scan rate; everything else about speed on
        # this unit is the module's own word for it.
        self.rev_times = deque(maxlen=11)
        self.last_byte = 0.0
        self.started = time.monotonic()

    # ── serial ──────────────────────────────────────────────────────────
    def open(self):
        self.ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.2)
        self.ser.reset_input_buffer()
        self.serial_error = None

    def command(self, word):
        """startlds$ / stoplds$. Returns an error string, or None."""
        with self.lock:
            if not self.ser:
                return "serial port not open"
            try:
                self.ser.write(word.encode())
                self.ser.flush()
            except Exception as e:                      # noqa: BLE001
                return str(e)
            self.motor = "running" if word.startswith("start") else "stopped"
            return None

    def run(self):
        buf = bytearray()
        while True:
            if not self.ser:
                try:
                    self.open()
                    if AUTOSTART:
                        self.command("startlds$")
                except Exception as e:                  # noqa: BLE001
                    self.serial_error = str(e)
                    time.sleep(2)
                    continue

            try:
                chunk = self.ser.read(512)
            except Exception as e:                      # noqa: BLE001
                self.serial_error = str(e)
                try:
                    self.ser.close()
                except Exception:                       # noqa: BLE001
                    pass
                self.ser = None
                continue

            now = time.monotonic()
            if chunk:
                self.c["bytes"] += len(chunk)
                self.last_byte = now
                buf += chunk

            # Resynchronising framer. The curses tools read a byte at a time
            # looking for 0xFA, which costs a syscall per byte; this walks a
            # buffer instead and drops only what it has to.
            i = 0
            n = len(buf)
            while n - i >= PACKET_LEN:
                if buf[i] != 0xFA:
                    i += 1
                    continue
                pkt = buf[i:i + PACKET_LEN]
                if not self.checksum_ok(pkt):
                    # Counts rejected CANDIDATES, not bad packets: after a
                    # slip the framer steps one byte at a time, and a 0xFA
                    # inside a payload is a candidate too. Treat it as "the
                    # stream is not clean", not as a packet loss count.
                    self.c["checksum_bad"] += 1
                    i += 1
                    continue
                self.ingest(pkt, now)
                i += PACKET_LEN
            del buf[:i]
            if len(buf) > 4096:          # nothing parseable in here
                del buf[:-PACKET_LEN]

    @staticmethod
    def checksum_ok(pkt):
        return (pkt[20] | (pkt[21] << 8)) == (sum(pkt[:20]) & 0xFFFF)

    def ingest(self, pkt, now):
        idx = pkt[1]
        if not IDX_FIRST <= idx <= IDX_LAST:
            self.c["index_bad"] += 1     # the "FA FB speed error" in lidar_health
            return
        self.c["packets"] += 1
        base = (idx - IDX_FIRST) * 4
        speed_raw = pkt[2] | (pkt[3] << 8)

        with self.lock:
            self.speed_raw = speed_raw
            if idx == IDX_FIRST:
                self.rev += 1
                self.c["revs"] += 1
                self.rev_times.append(now)
            for s in range(4):
                off = 4 + s * 4
                word = pkt[off] | (pkt[off + 1] << 8)
                invalid = bool(word & 0x8000)
                weak = bool(word & 0x4000)
                dist = word & 0x3FFF
                if invalid:
                    self.c["invalid"] += 1
                    dist = 0          # no return, which is what 0 means here
                if weak:
                    self.c["weak"] += 1
                if dist == 0:
                    self.c["zero"] += 1
                a = (base + s) % 360
                self.dist[a] = dist
                self.sig[a] = pkt[off + 2] | (pkt[off + 3] << 8)
                self.stamp[a] = now

    # ── readers ─────────────────────────────────────────────────────────
    def health(self):
        now = time.monotonic()
        with self.lock:
            c = dict(self.c)
            speed_raw = self.speed_raw
            motor = self.motor
            err = self.serial_error
            fresh = sum(1 for t in self.stamp if now - t < STALE_S)
            revs = list(self.rev_times)
        # Counted, not claimed. Stale wraps are ignored so a motor that has
        # stopped reads 0 rather than the rate it had before it stopped.
        rpm = 0.0
        if len(revs) > 1 and now - revs[-1] < 2.0:
            span = revs[-1] - revs[0]
            if span > 0:
                rpm = round((len(revs) - 1) / span * 60.0, 1)
        return {
            "ok": err is None,
            "serial_error": err,
            "port": SERIAL_PORT,
            # What the daemon last TOLD the motor to do. Whether anything is
            # actually turning is `spinning`: an unpowered scanner swallows
            # startlds$ without complaint and would otherwise read as running.
            "motor": motor,
            "spinning": rpm > 0,
            "uptime_s": round(now - self.started, 1),
            "silence_s": round(now - self.last_byte, 2) if self.last_byte else None,
            "speed_raw": speed_raw,
            # Measured from index wraps. speed_raw / 100 is the module's own
            # figure and agrees with it; /64, as the XV-11 family uses, does
            # not — see the note at the top of this file.
            "rpm": rpm,
            "rpm_reported": round(speed_raw / 100.0, 1),
            "points_fresh": fresh,
            "counts": c,
        }

    def scan(self):
        now = time.monotonic()
        with self.lock:
            dist = [d if now - t < STALE_S else -1
                    for d, t in zip(self.dist, self.stamp)]
            sig = [s if now - t < STALE_S else -1
                   for s, t in zip(self.sig, self.stamp)]
            rev = self.rev
        out = self.health()
        out.update({"rev": rev, "dist": dist, "sig": sig,
                    "ts": round(time.time(), 3)})
        return out


SCANNER = Scanner()

# Rates, sampled once a second from the counters, so every client sees the same
# numbers and nobody has to difference them itself.
RATES = {}


def rate_thread():
    prev = dict(SCANNER.c)
    while True:
        time.sleep(1.0)
        cur = dict(SCANNER.c)
        RATES.update({k: cur.get(k, 0) - prev.get(k, 0) for k in cur})
        prev = cur


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        pass                       # journald does not need a line per poll

    def _send(self, code, body, ctype="application/json"):
        raw = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(raw)))
        # The bot dashboard is served from the Bone, so every useful client is
        # cross-origin. This daemon exposes one read-only scan and a motor
        # switch on a robot's own network; it is not worth an origin list.
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(raw)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj))

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", "0")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/scan":
            d = SCANNER.scan()
            d["rates"] = dict(RATES)
            self._json(d)
        elif path == "/health":
            d = SCANNER.health()
            d["rates"] = dict(RATES)
            self._json(d)
        elif path in ("/", "/index.html"):
            self._send(200, f"LDS-006 lidar daemon on {SERIAL_PORT}. "
                            f"GET /scan, GET /health, POST /control\n",
                       "text/plain")
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        if self.path.split("?")[0] != "/control":
            self._json({"error": "not found"}, 404)
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}")
        except Exception as e:                          # noqa: BLE001
            self._json({"error": f"bad body: {e}"}, 400)
            return
        want = str(body.get("motor", "")).lower()
        if want not in ("start", "stop"):
            self._json({"error": 'motor must be "start" or "stop"'}, 400)
            return
        err = SCANNER.command("startlds$" if want == "start" else "stoplds$")
        self._json({"ok": err is None, "error": err, "motor": SCANNER.motor},
                   200 if err is None else 500)


def main():
    threading.Thread(target=SCANNER.run, daemon=True).start()
    threading.Thread(target=rate_thread, daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    print(f"lidar daemon: {SERIAL_PORT} @ {BAUD} -> http://0.0.0.0:{HTTP_PORT}",
          flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
