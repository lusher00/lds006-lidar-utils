#!/usr/bin/env python3

import serial
import time
import curses
import math

PORT = "/dev/ttyAMA0"
BAUD = 115200

MAX_RANGE_MM = 5000
REFRESH_HZ = 10

# Change this later once we establish physical forward.
ANGLE_OFFSET = 0


def checksum_ok(pkt):
    received = pkt[20] | (pkt[21] << 8)
    calculated = sum(pkt[:20]) & 0xFFFF
    return received == calculated


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

        pkt = b + rest

        if not checksum_ok(pkt):
            continue

        return pkt


def decode(pkt):
    idx = pkt[1]

    if not 0xA0 <= idx <= 0xF9:
        return None

    base = (idx - 0xA0) * 4

    points = []

    for i in range(4):

        off = 4 + i * 4

        # LDS-006:
        # distance = full 16-bit little-endian value
        distance = (
            pkt[off] |
            (pkt[off + 1] << 8)
        )

        reflectivity = (
            pkt[off + 2] |
            (pkt[off + 3] << 8)
        )

        angle = (base + i + ANGLE_OFFSET) % 360

        points.append(
            (angle, distance, reflectivity)
        )

    return points


def safe(stdscr, y, x, text, attr=0):
    h, w = stdscr.getmaxyx()

    if not (0 <= y < h and 0 <= x < w):
        return

    try:
        stdscr.addstr(
            y,
            x,
            text[:max(0, w-x-1)],
            attr
        )
    except curses.error:
        pass


def draw(stdscr, scan, rpm, raw_speed, frozen):

    stdscr.erase()

    h, w = stdscr.getmaxyx()

    # Leave room for table when terminal is wide.
    table_width = 27 if w >= 90 else 0
    map_w = w - table_width

    top = 2
    bottom = h - 2

    cx = map_w // 2
    cy = (top + bottom) // 2

    rx = max(2, map_w // 2 - 4)
    ry = max(2, (bottom - top) // 2 - 1)

    safe(
        stdscr,
        0,
        0,
        f"LDS-006   "
        f"RPM {rpm:5.1f}   "
        f"raw-speed {raw_speed:5d}   "
        f"{'FROZEN' if frozen else 'LIVE'}   "
        f"SPACE freeze   q quit",
        curses.A_BOLD
    )

    # Range rings
    for radius_mm in (1000, 2000, 3000, 4000, 5000):

        for deg in range(0, 360, 3):

            t = math.radians(deg)

            x = cx + int(
                math.sin(t) *
                radius_mm /
                MAX_RANGE_MM *
                rx
            )

            y = cy - int(
                math.cos(t) *
                radius_mm /
                MAX_RANGE_MM *
                ry
            )

            safe(stdscr, y, x, ".")

    # Sensor
    safe(stdscr, cy, cx, "O", curses.A_BOLD)

    safe(stdscr, top, cx - 1, "0")
    safe(stdscr, cy, map_w - 4, "90")
    safe(stdscr, bottom, cx - 2, "180")
    safe(stdscr, cy, 0, "270")

    plotted = 0
    out_of_range = 0

    for angle in range(360):

        p = scan[angle]

        if p is None:
            continue

        distance, reflectivity = p

        # Don't invent protocol validity flags.
        # Only reject values that cannot sensibly be plotted.
        # No usable range return: show it close to the lidar
        # instead of making the point disappear.
        if distance == 0 or distance > MAX_RANGE_MM:
            out_of_range += 1
            distance = 150
            no_range = True
        else:
            no_range = False

        t = math.radians(angle)

        x = cx + int(
            math.sin(t) *
            distance /
            MAX_RANGE_MM *
            rx
        )

        y = cy - int(
            math.cos(t) *
            distance /
            MAX_RANGE_MM *
            ry
        )

        if no_range:
            safe(stdscr, y, x, "x", curses.A_BOLD)
        else:
            safe(stdscr, y, x, "*", curses.A_BOLD)

        plotted += 1

    # Right-side numerical sanity table
    if table_width:

        x0 = map_w + 1

        safe(
            stdscr,
            2,
            x0,
            " ANGLE    DIST    REFL",
            curses.A_BOLD
        )

        row = 3

        # every 15 degrees
        for angle in range(0, 360, 15):

            if row >= h - 2:
                break

            p = scan[angle]

            if p is None:
                text = f"{angle:4d}°     ---     ---"

            else:
                d, r = p

                text = (
                    f"{angle:4d}° "
                    f"{d:7d} "
                    f"{r:7d}"
                )

            safe(stdscr, row, x0, text)

            row += 1

    safe(
        stdscr,
        h - 1,
        0,
        f"plotted={plotted}   "
        f">5m/zero={out_of_range}   "
        f"range={MAX_RANGE_MM/1000:.1f}m"
    )

    stdscr.refresh()


def run(stdscr):

    curses.curs_set(0)
    stdscr.nodelay(True)

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.03
    )

    start_lidar(ser)

    scan = [None] * 360

    frozen = False

    raw_speed = 0

    frame_count = 0
    frame_timer = time.monotonic()

    rpm = 0.0

    last_draw = 0

    try:

        while True:

            key = stdscr.getch()

            if key in (ord("q"), ord("Q")):
                break

            if key == ord(" "):
                frozen = not frozen

            pkt = read_frame(ser)

            now = time.monotonic()

            if pkt is not None:

                idx = pkt[1]

                # Ignore FB/status frames for mapping.
                if 0xA0 <= idx <= 0xF9:

                    frame_count += 1

                    raw_speed = (
                        pkt[2] |
                        (pkt[3] << 8)
                    )

                    points = decode(pkt)

                    if points and not frozen:

                        for angle, distance, reflectivity in points:

                            scan[angle] = (
                                distance,
                                reflectivity
                            )

            # RPM derived from actual scan frame rate:
            # 90 frames = one revolution.
            elapsed = now - frame_timer

            if elapsed >= 1.0:

                fps = frame_count / elapsed

                rpm = (
                    fps /
                    90.0 *
                    60.0
                )

                frame_count = 0
                frame_timer = now

            if now - last_draw >= 1.0 / REFRESH_HZ:

                draw(
                    stdscr,
                    scan,
                    rpm,
                    raw_speed,
                    frozen
                )

                last_draw = now

    finally:

        ser.write(b"stoplds$")
        ser.flush()
        ser.close()


def main():
    curses.wrapper(run)


if __name__ == "__main__":
    main()
