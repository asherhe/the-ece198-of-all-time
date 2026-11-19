"""Matplotlib view for the recorder's five-second signal buffers."""

from matplotlib import pyplot as plt


PLOTS = (
    ("Raw ECG (ADC counts)", ("diff_raw", "wilson_raw")),
    ("Filtered ECG (counts)", ("diff_filtered", "wilson_filtered")),
    ("Acceleration (m/s²)", ("accel_magnitude", "accel_smoothed")),
    ("Optical (counts)", ("red", "infrared")),
)


class LivePlot:
    def __init__(self, window_s: float = 5.0):
        self.window_s = window_s
        self.figure, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True,
                                         constrained_layout=True)
        self.axes = axes
        self.lines = {}
        for axis, (title, channels) in zip(axes, PLOTS):
            axis.set_ylabel(title)
            axis.grid(alpha=0.25)
            axis.set_xlim(-window_s, 0)
            for channel in channels:
                (self.lines[channel],) = axis.plot([], [], label=channel, linewidth=1)
            axis.legend(loc="upper left", ncol=2, fontsize="small")
        axes[-1].set_xlabel("Seconds before newest board sample")

    def update(self, recorder) -> None:
        snapshot = recorder.snapshot()
        newest = recorder.latest_time_s
        for axis, (_, channels) in zip(self.axes, PLOTS):
            values = []
            for channel in channels:
                samples = snapshot[channel]
                x = [t - newest for t, _ in samples] if newest is not None else []
                y = [value for _, value in samples]
                self.lines[channel].set_data(x, y)
                values.extend(y)
            if values:
                lo, hi = min(values), max(values)
                margin = max((hi - lo)*0.08, abs(hi)*0.001, 1e-6)
                axis.set_ylim(lo - margin, hi + margin)
            axis.set_xlim(-self.window_s, 0)
        vitals = recorder.pulse.vitals
        hr = str(vitals.bpm) if vitals.heart_rate_valid else "—"
        spo2 = str(vitals.spo2) if vitals.spo2_valid else "—"
        stats = recorder.stats()
        self.figure.suptitle(
            f"Live signals  |  HR {hr} bpm  |  SpO₂ {spo2}%  |  "
            f"frames {stats['frames']}  gaps {stats['sequence_gaps']}  "
            f"bad CRC {stats['bad_frames']}")
