#!/usr/bin/env python3

import serial
import time
import struct

PORT = "/dev/ttyAMA0"
BAUD = 115200

HEADER = b'\x5a\xa5'

def start_lidar(ser):
    ser.write(b"startlds$")
    time.sleep(0.5)

def read_packets(ser):
    buf = bytearray()
    while True:
        buf += ser.read(512)

        while True:
            i = buf.find(HEADER)
            if i < 0:
                if len(buf) > 2048:
                    buf = buf[-64:]
                break

            if len(buf) < i + 3:
                break

            length = buf[i+2]
            total = 3 + length

            if len(buf) < i + total:
                break

            pkt = buf[i:i+total]
            del buf[:i+total]
            yield pkt

def parse_packet(pkt):
    payload = pkt[3:]
    if len(payload) < 4:
        return []

    start_angle = struct.unpack("<H", payload[0:2])[0] / 100.0
    end_angle   = struct.unpack("<H", payload[2:4])[0] / 100.0

    data = payload[4:]
    count = len(data) // 2
    if count == 0:
        return []

    step = (end_angle - start_angle) / count

    samples = []
    for i in range(count):
        dist = struct.unpack("<H", data[i*2:i*2+2])[0]
        angle = start_angle + step * i
        samples.append((angle, dist))

    return samples

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    ser.reset_input_buffer()

    print("Starting LDS...")
    start_lidar(ser)

    for pkt in read_packets(ser):
        pts = parse_packet(pkt)
        for angle, dist in pts:
            print(f"{angle:.1f} {dist}")

if __name__ == "__main__":
    main()
