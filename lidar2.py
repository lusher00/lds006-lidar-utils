#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200
PACKET_LEN = 22


def start_lidar(ser):
    ser.write(b"startlds$")
    ser.flush()
    time.sleep(1)


def read_packet(ser):
    while True:
        # Find normal-scan packet header
        b = ser.read(1)

        if not b:
            continue

        if b[0] != 0xFA:
            continue

        rest = ser.read(PACKET_LEN - 1)

        if len(rest) != PACKET_LEN - 1:
            continue

        pkt = b + rest

        # A0-F9 = 90 packets * 4 degrees
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
        strength_warning = bool(hi & 0x40)

        distance = lo | ((hi & 0x3F) << 8)

        strength = (
            pkt[off + 2] |
            (pkt[off + 3] << 8)
        )

        angle = base_angle + i

        points.append(
            (
                angle,
                distance,
                strength,
                invalid,
                strength_warning
            )
        )

    return rpm, points


def main():

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=0.2
    )

    ser.reset_input_buffer()

    print("Starting LDS...")
    start_lidar(ser)

    last_angle = None

    try:

        while True:

            pkt = read_packet(ser)

            rpm, points = parse_packet(pkt)

            for angle, dist, strength, invalid, warning in points:

                # Print revolution separator
                if last_angle is not None and angle < last_angle:
                    print(
                        f"\n--- revolution  RPM={rpm:.1f} ---"
                    )

                last_angle = angle

                if not invalid:
                    print(
                        f"{angle:3d}°  "
                        f"{dist:5d} mm  "
                        f"strength={strength:5d}"
                    )

    except KeyboardInterrupt:
        print("\nStopping LDS...")
        ser.write(b"stoplds$")
        ser.flush()

    finally:
        ser.close()


if __name__ == "__main__":
    main()
