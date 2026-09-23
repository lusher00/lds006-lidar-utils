#!/usr/bin/env python3

import serial
import time
from collections import Counter

PORT = "/dev/ttyAMA0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.2)

ser.reset_input_buffer()
ser.write(b"startlds$")
ser.flush()

time.sleep(1)
ser.reset_input_buffer()

good = 0
other = 0
other_types = Counter()
examples = {}

start = time.monotonic()

while time.monotonic() - start < 5:

    b = ser.read(1)

    if not b:
        continue

    if b[0] != 0xFA:
        continue

    rest = ser.read(21)

    if len(rest) != 21:
        continue

    pkt = b + rest
    idx = pkt[1]

    if 0xA0 <= idx <= 0xF9:
        good += 1
        continue

    other += 1
    other_types[idx] += 1

    if idx not in examples:
        examples[idx] = pkt


ser.write(b"stoplds$")
ser.flush()
ser.close()


print()
print("GOOD A0-F9:", good)
print("OTHER:     ", other)

print()
print("OTHER SECOND-BYTE VALUES:")

for value, count in other_types.most_common():
    print(f"  0x{value:02X}: {count}")


print()
print("EXAMPLES:")

for value, pkt in examples.items():
    print()
    print(f"second byte = 0x{value:02X}")
    print(" ".join(f"{x:02X}" for x in pkt))
