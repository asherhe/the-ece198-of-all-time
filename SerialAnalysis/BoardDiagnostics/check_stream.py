"""Count framed board samples without starting a recording session."""

from collections import Counter
from pathlib import Path
import struct
import sys
import time

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from SerialAnalysis.Live.protocol import Decoder  # noqa: E402


port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
decoder = Decoder()
counts = Counter()
first_optical = None
nonzero_accel = 0

with serial.Serial(port, 115200, timeout=0.5) as connection:
    end = time.monotonic() + 5
    while time.monotonic() < end:
        for frame in decoder.feed(connection.read(1024)):
            counts[frame.type] += frame.count
            if frame.type == 2:
                nonzero_accel += sum(
                    any(struct.unpack_from("<hhh", frame.payload, i * 6))
                    for i in range(frame.count)
                )
            if frame.type == 3 and first_optical is None:
                first_optical = struct.unpack_from("<III", frame.payload)

print(f"ECG={counts[1]}, accelerometer={counts[2]}, optical={counts[3]}")
print(f"Nonzero accelerometer samples={nonzero_accel}, bad frames={decoder.bad}")
print(f"First optical sample (time, red, IR)={first_optical}")
