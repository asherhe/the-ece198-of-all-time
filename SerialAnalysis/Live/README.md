# Python live recorder

This is a Python port of `SerialAnalysis/CLI`. It decodes the same board frames,
applies the same ECG, acceleration, and provisional pulse analysis, counts
rehab reps from the accelerometer, sends heart rate and remaining time to the
board once per second, and writes the same four session files. The notebook
shows a BPM / SpO₂ / rep-count readout and a rolling heart-rate and SpO₂ trend
plot above one-second raw and filtered ECG plots and five-second acceleration
and optical plots. The pulse and SpO₂ estimates are prototype
measurements, not clinically validated.

The rep counter is a port of the Arduino rehab sketch: it high-pass filters out
gravity, limits jerk, low-pass filters tremor, tracks the dominant 3D motion
direction, and counts a rep after a sustained push and return. `vitals.csv`
gains a `rep_count` column, and `metadata.json` records the final `rep_count`.

From the repository root:

```powershell
python -m pip install -r SerialAnalysis/Live/requirements.txt
jupyter lab SerialAnalysis/Live/live_signals.ipynb
```

In the notebook, run the `%pip` cell first to install packages into the active
notebook kernel. Then set `PORT = "COM3"` to the board's port and run the setup
cell. The receive-check cell listens for ten seconds and prints every incoming
binary chunk in hex, plus decoded frame summaries and a final byte count. Run
the live graph cell separately; both cells open the same serial port. A terminal
`pip` command or notebook `!pip` may use a different Python environment. Stop
the live cell with **Interrupt Kernel**; its `finally` block closes the port and
session files. Close Arduino Serial Monitor first, because the port cannot be
shared. On Linux, use a device such as `/dev/ttyACM0`.

The Python recorder asserts DTR when opening the port. This is needed for
native USB CDC boards such as the UNO R4 Minima to transmit. On some older
Arduino boards it can reset the sketch once; acquisition starts as the sketch
restarts.

OLED updates are best effort. If a host-to-board write times out, recording
continues and `display_write_failures` increases in the plot title and session
metadata. Restart the notebook kernel, or rerun the setup cell, after changing
the Python source so the notebook uses the current version.

To record without a notebook:

```powershell
python -m SerialAnalysis.Live --port COM3 --output sessions --duration-min 30
```

The output folder defaults to `sessions` relative to the notebook kernel's
working directory or the CLI's working directory. All plots use the same host
receive clock, so they scroll together over the latest five seconds. Samples
within a frame are spaced by their nominal sample period; serial delivery can
add timing uncertainty. The CSV files retain the board timestamps. The notebook targets a signal refresh every 0.1 seconds. Plots are drawn once
and then only their lines are redrawn (blitting) and sent as PNG images, so a
frame costs roughly 25 ms; the measured rate is shown in the vitals panel.
An axis rescale forces one slower full redraw, which hysteresis keeps rare. The vitals view updates
once per second, while recording continues in a background thread. The red
dashed markers on acceleration indicate completed reps.

## Command-line loss diagnostics

For a one-minute capture with a report every five seconds:

```powershell
python -m SerialAnalysis.Live --port COM8 --duration-min 1 --status-interval 5
```

The rates are received samples per wall-clock second (nominally 400 Hz ECG,
50 Hz acceleration, and 100 Hz optical). `sequence_gaps` counts missing frame
sequence numbers across all streams, not missing samples; `bad_frames` counts
invalid frame lengths or checksums. `timestamp_discontinuities` and
`estimated_missing_samples` are per-stream board-time estimates, not a direct
measurement of loss. Short-term rates vary with batching. The recorder saves
these counters and the latest board diagnostics in `metadata.json`.

The real firmware sends cumulative diagnostics once a second: missed ECG and
acceleration deadlines, I2C errors, optical FIFO overflow, dropped samples per
transmit stream, the maximum loop and optical service durations in microseconds,
and whether the optical sensor initialized. A dropped diagnostics frame can
also cause a global sequence gap. If `optical_overflow_counter_reliable` is 0,
the sensor's overflow register failed a clear check: its overflow count is not
reliable and FIFO loss cannot be ruled out by that counter. Use rates and
observed timestamp gaps as additional evidence. Optical timestamps are estimated
from the FIFO sample rate; a long service gap causes them to be re-anchored.

The board updates the OLED framebuffer in short I2C chunks so that full-screen
transfers do not block sensor sampling. A display update can therefore appear
progressively. The physical display and sensor signal quality still need a
visual and hardware check.
