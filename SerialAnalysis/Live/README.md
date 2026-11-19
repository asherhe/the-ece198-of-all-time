# Python live recorder

This is a Python port of `SerialAnalysis/CLI`. It decodes the same board frames,
applies the same ECG, acceleration, and provisional pulse analysis, sends heart
rate and remaining time to the board once per second, and writes the same four
session files. The notebook also shows a five-second rolling window of raw and
filtered ECG, acceleration, and raw optical signals. The pulse and SpO₂
estimates are prototype measurements, not clinically validated.

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

To record without a notebook:

```powershell
python -m SerialAnalysis.Live --port COM3 --output sessions --duration-min 30
```

The output folder defaults to `sessions` relative to the notebook kernel's
working directory or the CLI's working directory. The live window uses board
microsecond timestamps, so it represents the latest five seconds of acquired
samples even if serial delivery is uneven. The plot refreshes about five times
per second. Only the latest five seconds of samples are kept for plotting;
recording continues for the chosen duration.
