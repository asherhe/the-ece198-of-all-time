"""Measure board-side sample continuity from frame timestamps.

Host arrival timing is unreliable (USB buffering + read timeout make data look
bursty even when the board streams steadily). Instead, inspect each frame's
board microsecond timestamp per channel: if the board never stalls, consecutive
same-type frame timestamps advance by the expected batch duration with no large
jumps.
"""

from pathlib import Path
import struct
import sys
import time

import serial

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from SerialAnalysis.Live.protocol import Decoder  # noqa: E402

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
decoder = Decoder()

# Expected board-time advance per frame = count * per-sample period (us).
PERIOD_US = {1: 5000, 2: 20000}  # ECG 200 Hz, accel 50 Hz
last_ts = {}
last_seq = {}
jumps = {1: [], 2: []}
seq_gaps = {1: 0, 2: 0}
counts = {1: 0, 2: 0, 3: 0}

with serial.Serial(port, 115200, timeout=0.2) as connection:
    end = time.monotonic() + 6
    while time.monotonic() < end:
        for frame in decoder.feed(connection.read(1024)):
            counts[frame.type] = counts.get(frame.type, 0) + frame.count
            if frame.type in PERIOD_US:
                expected = frame.count * PERIOD_US[frame.type]
                if frame.type in last_ts:
                    advance = (frame.timestamp_us - last_ts[frame.type]) & 0xFFFFFFFF
                    # Board-time jump beyond the previous batch = a real stall.
                    jumps[frame.type].append(advance)
                    missed = (frame.sequence - last_seq[frame.type] - 1) & 0xFFFF
                    seq_gaps[frame.type] += missed
                last_ts[frame.type] = frame.timestamp_us
                last_seq[frame.type] = frame.sequence

for t, label in ((1, "ECG"), (2, "ACCEL")):
    js = sorted(jumps[t])
    if not js:
        print(f"{label}: no data")
        continue
    nominal = None
    # Nominal per-frame board-time advance: ECG 8*5000=40000, accel 5*20000=100000
    worst = js[-3:]
    big = sum(1 for j in js if j > 300000)  # >300 ms board-time jump = real stall
    print(f"{label}: frames_compared={len(js)} seq_gaps={seq_gaps[t]} "
          f"median_advance_us={js[len(js)//2]} worst_us={worst} stalls>300ms={big}")

print(f"counts (5-6s): ECG={counts[1]}, accel={counts[2]}, optical={counts[3]}")
