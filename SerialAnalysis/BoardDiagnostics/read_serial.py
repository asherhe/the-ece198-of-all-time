"""Read the diagnostic sketch's repeated I2C scan from a serial port."""

import sys
import time

import serial


port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
with serial.Serial(port, 115200, timeout=1) as connection:
    end = time.monotonic() + 8
    while time.monotonic() < end:
        line = connection.readline().decode("ascii", errors="replace").strip()
        if line:
            print(line, flush=True)
