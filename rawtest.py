#!/usr/bin/env python3

import serial
import time

PORT = "/dev/ttyAMA0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()

print("Sending startlds$")
ser.write(b"startlds$")
ser.flush()

start = time.monotonic()
last_data = start
total = 0
last_report = start

while True:
    data = ser.read(4096)
    now = time.monotonic()

    if data:
        total += len(data)
        last_data = now

    if now - last_report >= 0.5:
        print(
            f"{now-start:6.1f}s  "
            f"bytes={total:8d}  "
            f"rate={(total/(now-start)):8.0f} B/s  "
            f"silence={now-last_data:.2f}s"
        )
        last_report = now

    if now - last_data > 10:
        print("\nNO DATA FOR 10 SECONDS")
        break

ser.close()
