#!/usr/bin/env python3

import serial
import time
import curses

PORT = "/dev/ttyAMA0"
BAUD = 115200
PACKET_LEN = 22

# Display every N degrees.
STEP = 10


def start_lidar(ser):
    ser.write(b"startlds$")
    ser.flush()
    time.sleep(1)


def read_packet(ser):
    while True:
        b = ser.read(1)

        if not b:
            return None

        if b[0] != 0xFA:
            continue

        rest = ser.read(PACKET_LEN - 1)

        if len(rest) != PACKET_LEN - 1:
            continue

        pkt = b + rest

        if 0xA0 <= pkt[1] <= 0xF9:
            return pkt


def parse_packet(pkt):
    index = pkt[1]
    base_angle = (index - 0xA0) * 4

    speed_raw = pkt[2] | (pkt[3] << 8)
    rpm = speed_raw / 64.0

    points = []

    for i in range(4):
        off = 4 + i * 4

        lo = pkt[off]
        hi = pkt[off + 1]

        invalid = bool(hi & 0x80)
        warning = bool(hi & 0x40)

        distance = lo | ((hi & 0x3F) << 8)
        strength = pkt[off + 2] | (pkt[off + 3] << 8)

        angle = base_angle + i

        points.append(
            (angle, distance, strength, invalid, warning)
        )

    return rpm, points


def safe_addstr(stdscr, y, x, text, attr=0):
    h, w = stdscr.getmaxyx()

    if 0 <= y < h and x < w:
        try:
            stdscr.addstr(y, x, text[:max(0, w-x-1)], attr)
        except curses.error:
            pass


def draw(stdscr, scan, rpm):
    stdscr.erase()

    h, w = stdscr.getmaxyx()

    safe_addstr(
        stdscr, 0, 0,
        f"LDS-006    RPM: {rpm:6.1f}    q=quit",
        curses.A_BOLD
    )

    safe_addstr(
        stdscr, 2, 0,
        " Angle    Distance    Strength"
    )

    safe_addstr(
        stdscr, 3, 0,
        " -----    --------    --------"
    )

    row = 4

    for angle in range(0, 360, STEP):

        if row >= h - 1:
            break

        p = scan[angle]

        if p is None:
            text = f"{angle:4d}°        ---         ---"
        else:
            distance, strength, invalid, warning = p

            if invalid:
                text = f"{angle:4d}°    INVALID"
            else:
                flag = " !" if warning else ""

                text = (
                    f"{angle:4d}°"
                    f"    {distance:6d} mm"
                    f"    {strength:8d}{flag}"
                )

        safe_addstr(stdscr, row, 0, text)
        row += 1

    safe_addstr(
        stdscr,
        h - 1,
        0,
        f"Showing every {STEP} degrees"
    )

    stdscr.refresh()


def run(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.05
    )

    ser.reset_input_buffer()

    scan = [None] * 360
    rpm = 0.0

    start_lidar(ser)

    last_draw = 0

    try:
        while True:

            key = stdscr.getch()

            if key in (ord('q'), ord('Q')):
                break

            pkt = read_packet(ser)

            if pkt:
                rpm, points = parse_packet(pkt)

                for angle, distance, strength, invalid, warning in points:
                    scan[angle] = (
                        distance,
                        strength,
                        invalid,
                        warning
                    )

            # Screen only updates 5 times/sec.
            now = time.monotonic()

            if now - last_draw >= 0.2:
                draw(stdscr, scan, rpm)
                last_draw = now

    finally:
        ser.write(b"stoplds$")
        ser.flush()
        ser.close()


def main():
    curses.wrapper(run)


if __name__ == "__main__":
    main()
