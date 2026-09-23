#!/usr/bin/env python3

import serial
import time
import curses
from collections import deque

PORT = "/dev/ttyAMA0"
BAUD = 115200
PACKET_LEN = 22

def start_lidar(ser):
    ser.write(b"startlds$")
    ser.flush()
    time.sleep(1)
    ser.reset_input_buffer()

def read_frame(ser):
    while True:
        b = ser.read(1)
        if not b:
            return None

        if b[0] != 0xFA:
            continue

        rest = ser.read(21)
        if len(rest) != 21:
            continue

        return b + rest

def checksum_ok(pkt):
    # LDS-006 checksum = arithmetic sum of bytes 0..19
    calc = sum(pkt[:20]) & 0xFFFF
    received = pkt[20] | (pkt[21] << 8)
    return calc == received, calc, received

def safe(stdscr, y, x, text, attr=0):
    try:
        stdscr.addstr(y, x, text, attr)
    except curses.error:
        pass

def draw(stdscr, stats):
    stdscr.erase()

    total = stats["scan"] + stats["speederr"]

    err_pct = (
        100.0 * stats["speederr"] / total
        if total else 0
    )

    raw = stats["speed_raw"]

    safe(stdscr, 0, 0,
         "LDS-006 LIVE HEALTH", curses.A_BOLD)

    safe(stdscr, 2, 0,
         f"Scan packets/sec:     {stats['scan']:6.1f}")

    safe(stdscr, 3, 0,
         f"FA FB speed errors:   {stats['speederr']:6.1f}/sec")

    safe(stdscr, 4, 0,
         f"Speed-error rate:     {err_pct:6.1f}%")

    safe(stdscr, 6, 0,
         f"Speed raw:            {raw:6d}")

    # Show /64 only as a reference. We are NOT assuming
    # this is trustworthy physical RPM yet.
    safe(stdscr, 7, 0,
         f"raw / 64:             {raw/64:6.1f}")

    safe(stdscr, 9, 0,
         f"Checksum good/sec:    {stats['checksum_good']:6.1f}")

    safe(stdscr, 10, 0,
         f"Checksum bad/sec:     {stats['checksum_bad']:6.1f}")

    safe(stdscr, 12, 0,
         "Expected raw speed is roughly ~21500")

    delta = raw - 21500

    safe(stdscr, 13, 0,
         f"Difference from 21500: {delta:+6d}")

    # crude speed bar centered around 21500
    safe(stdscr, 15, 0, "Speed:")

    lo = 15000
    hi = 28000
    width = 50

    pos = int((raw - lo) / (hi - lo) * width)
    pos = max(0, min(width - 1, pos))

    target = int((21500 - lo) / (hi - lo) * width)

    bar = ["-"] * width

    if 0 <= target < width:
        bar[target] = "|"

    if 0 <= pos < width:
        bar[pos] = "#"

    safe(stdscr, 16, 0,
         "[" + "".join(bar) + "]")

    safe(stdscr, 17, 0,
         "                 target |")

    if stats["speederr"] == 0:
        status = "NO SPEED ERRORS"
    elif err_pct < 1:
        status = "OCCASIONAL SPEED ERROR"
    else:
        status = "SPEED ERROR ACTIVE"

    safe(stdscr, 19, 0,
         f"STATUS: {status}", curses.A_BOLD)

    safe(stdscr, 21, 0,
         "q = quit")

    stdscr.refresh()

def run(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.05
    )

    start_lidar(ser)

    events = deque()
    speed_raw = 0

    try:
        while True:
            key = stdscr.getch()

            if key in (ord("q"), ord("Q")):
                break

            pkt = read_frame(ser)

            now = time.monotonic()

            if pkt:
                idx = pkt[1]

                ok, calc, received = checksum_ok(pkt)

                if 0xA0 <= idx <= 0xF9:
                    kind = "scan"

                    speed_raw = (
                        pkt[2] |
                        (pkt[3] << 8)
                    )

                elif idx == 0xFB:
                    kind = "speederr"

                    # Still display its speed bytes
                    speed_raw = (
                        pkt[2] |
                        (pkt[3] << 8)
                    )

                else:
                    kind = "other"

                events.append(
                    (now, kind, ok)
                )

            # Keep only last second
            cutoff = now - 1.0

            while events and events[0][0] < cutoff:
                events.popleft()

            scan = 0
            speederr = 0
            good = 0
            bad = 0

            for _, kind, ok in events:

                if kind == "scan":
                    scan += 1

                elif kind == "speederr":
                    speederr += 1

                if ok:
                    good += 1
                else:
                    bad += 1

            stats = {
                "scan": float(scan),
                "speederr": float(speederr),
                "checksum_good": float(good),
                "checksum_bad": float(bad),
                "speed_raw": speed_raw,
            }

            draw(stdscr, stats)

    finally:
        ser.write(b"stoplds$")
        ser.flush()
        ser.close()

def main():
    curses.wrapper(run)

if __name__ == "__main__":
    main()
