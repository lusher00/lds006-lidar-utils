#!/usr/bin/env python3

import serial
import time
from collections import Counter

PORT = "/dev/ttyAMA0"
BAUD = 115200
SECONDS = 10


def xv11_checksum(pkt):
    """XV-11/LDS checksum over first 20 bytes."""
    data = pkt[:20]

    chk32 = 0
    for i in range(10):
        word = data[2*i] | (data[2*i+1] << 8)
        chk32 = (chk32 << 1) + word

    checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
    checksum = checksum & 0x7FFF

    return checksum


def read_packet(ser):
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


ser = serial.Serial(
    PORT,
    BAUD,
    timeout=0.2
)

ser.reset_input_buffer()

print("Starting LDS...")
ser.write(b"startlds$")
ser.flush()

time.sleep(1)
ser.reset_input_buffer()

start = time.monotonic()

total = 0
good_checksum = 0
bad_checksum = 0
good_index = 0
bad_index = 0

indices = Counter()
speeds = []

valid_distances = []
invalid_samples = 0
warning_samples = 0

last_index = None
sequence_good = 0
sequence_bad = 0

examples_bad = []

while time.monotonic() - start < SECONDS:

    pkt = read_packet(ser)

    if pkt is None:
        continue

    total += 1

    idx = pkt[1]

    if 0xA0 <= idx <= 0xF9:
        good_index += 1
        indices[idx] += 1
    else:
        bad_index += 1
        continue

    # Check packet sequence A0,A1,...F9,A0...
    if last_index is not None:
        expected = 0xA0 if last_index == 0xF9 else last_index + 1

        if idx == expected:
            sequence_good += 1
        else:
            sequence_bad += 1

    last_index = idx

    # Checksum
    received_checksum = pkt[20] | (pkt[21] << 8)
    calculated_checksum = xv11_checksum(pkt)

    if received_checksum == calculated_checksum:
        good_checksum += 1
    else:
        bad_checksum += 1

        if len(examples_bad) < 5:
            examples_bad.append(
                (
                    pkt.hex(" "),
                    received_checksum,
                    calculated_checksum
                )
            )

        # DON'T use corrupt packets for distance statistics
        continue

    # Speed
    speed_raw = pkt[2] | (pkt[3] << 8)
    speeds.append(speed_raw)

    # Four samples
    for i in range(4):

        off = 4 + i * 4

        lo = pkt[off]
        hi = pkt[off + 1]

        invalid = bool(hi & 0x80)
        warning = bool(hi & 0x40)

        distance = lo | ((hi & 0x3F) << 8)

        if invalid:
            invalid_samples += 1
        else:
            valid_distances.append(distance)

        if warning:
            warning_samples += 1


ser.write(b"stoplds$")
ser.flush()
ser.close()


print()
print("========== LDS-006 STREAM CHECK ==========")
print()

print(f"Packets seen:          {total}")
print(f"Valid A0-F9 index:     {good_index}")
print(f"Bad packet index:      {bad_index}")
print()

print(f"GOOD checksum:         {good_checksum}")
print(f"BAD checksum:          {bad_checksum}")
print()

print(f"Sequential packets:    {sequence_good}")
print(f"Sequence errors:       {sequence_bad}")
print()


if speeds:

    avg_raw = sum(speeds) / len(speeds)

    print("SPEED")
    print(f"  raw min:             {min(speeds)}")
    print(f"  raw max:             {max(speeds)}")
    print(f"  raw average:         {avg_raw:.1f}")
    print()
    print(f"  raw/64 RPM:          {avg_raw/64:.1f}")

else:

    print("NO CHECKSUM-VALID SPEED DATA")


print()

total_samples = len(valid_distances) + invalid_samples

print("DISTANCE DATA")

print(f"  total samples:       {total_samples}")
print(f"  valid samples:       {len(valid_distances)}")
print(f"  invalid flag:        {invalid_samples}")
print(f"  warning flag:        {warning_samples}")

if total_samples:
    print(
        f"  valid percentage:    "
        f"{100.0*len(valid_distances)/total_samples:.1f}%"
    )

if valid_distances:
    print()
    print(f"  distance min:        {min(valid_distances)} mm")
    print(f"  distance max:        {max(valid_distances)} mm")
    print(
        f"  distance average:    "
        f"{sum(valid_distances)/len(valid_distances):.0f} mm"
    )


print()
print("INDEX COVERAGE")

missing = []

for idx in range(0xA0, 0xFA):
    if indices[idx] == 0:
        missing.append(idx)

if not missing:
    print("  ALL A0-F9 packet indices received")
else:
    print(
        "  Missing:",
        " ".join(f"{x:02X}" for x in missing)
    )


if examples_bad:

    print()
    print("EXAMPLE BAD CHECKSUM PACKETS")

    for raw, received, calculated in examples_bad:
        print()
        print(raw)
        print(
            f"received={received:04X} "
            f"calculated={calculated:04X}"
        )

print()
print("==========================================")
