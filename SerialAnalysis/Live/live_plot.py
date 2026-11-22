"""Matplotlib view for the recorder's five-second signal buffers."""

from __future__ import annotations

import threading
import time
from typing import Any, Iterable

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np


CHANNEL_SPECS: dict[str, tuple[str, float, float]] = {
    # channel_name: (subplot_title, expected_dt_s, gap_factor)
    "diff_raw": ("ECG Diff (Raw)", 0.005, 1.5),
    "wilson_raw": ("ECG Wilson (Raw)", 0.005, 1.5),
    "diff_filtered": ("ECG Diff (Filtered)", 0.005, 1.5),
    "wilson_filtered": ("ECG Wilson (Filtered)", 0.005, 1.5),
    "accel_magnitude": ("Accel Magnitude", 0.020, 1.5),
    "accel_smoothed": ("Accel Smoothed", 0.020, 1.5),
    "red": ("Optical Red", 0.010, 1.5),
    "infrared": ("Optical IR", 0.010, 1.5),
}


def build_gapped_xy(
    samples: Iterable[tuple[float, float]],
    now_s: float,
    window_s: float = 5.0,
    expected_dt_s: float = 0.01,
    gap_factor: float = 3.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return x (seconds relative to now_s) and y arrays with NaNs across gaps."""
    min_t = now_s - window_s
    max_t = now_s + 0.05
    gap_threshold = expected_dt_s * gap_factor

    xs: list[float] = []
    ys: list[float] = []
    prev_t: float | None = None

    for t_s, value in samples:
        if t_s < min_t or t_s > max_t:
            continue
        if prev_t is not None and (t_s - prev_t) > gap_threshold:
            # Break the line right after the last valid point before the gap.
            xs.append(min(now_s, prev_t + 1e-6) - now_s)
            ys.append(np.nan)
        xs.append(t_s - now_s)
        ys.append(float(value))
        prev_t = t_s

    if not xs:
        return np.empty(0, dtype=np.float64), np.empty(0, dtype=np.float64)
    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)


class LivePlot:
    """Rolling multi-channel plot that breaks lines across gaps and avoids render lag."""

    def __init__(
        self,
        channels: Iterable[str] | None = None,
        window_s: float = 5.0,
        figsize: tuple[float, float] = (10.0, 6.0),
        dpi: int = 80,
        channel_windows: dict[str, float] | None = None,
    ) -> None:
        self.window_s = float(window_s)
        self.channel_windows = {name: float(value) for name, value in (channel_windows or {}).items()}
        if self.window_s <= 0 or any(value <= 0 for value in self.channel_windows.values()):
            raise ValueError("plot windows must be positive")
        selected = list(channels) if channels is not None else list(CHANNEL_SPECS.keys())
        self.channels = [ch for ch in selected if ch in CHANNEL_SPECS]
        if not self.channels:
            raise ValueError("No valid channels specified for LivePlot")

        n = len(self.channels)
        cols = 2 if n > 1 else 1
        span_accel = (n % 2 == 1 and {"accel_magnitude", "red", "infrared"}
                      <= set(self.channels))
        positions = []
        row = col = 0
        for ch in self.channels:
            if span_accel and ch == "accel_magnitude":
                if col:
                    row += 1
                    col = 0
                positions.append((row, slice(None)))
                row += 1
            else:
                positions.append((row, col))
                col += 1
                if col == cols:
                    row += 1
                    col = 0
        rows = row + bool(col)
        self.figure = plt.figure(figsize=figsize, dpi=dpi)
        grid = self.figure.add_gridspec(rows, cols)

        self.axes: dict[str, Any] = {}
        self.lines: dict[str, Any] = {}

        for ch, (row, col) in zip(self.channels, positions):
            ax = self.figure.add_subplot(grid[row, col])
            title, _, _ = CHANNEL_SPECS[ch]
            if ch == "accel_magnitude":
                title += " (red dashed: rep completed)"
            ax.set_title(title, fontsize=9)
            ax.set_xlim(-self.channel_windows.get(ch, self.window_s), 0.0)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.tick_params(labelsize=8)
            ax.grid(True, alpha=0.25)
            (line,) = ax.plot([], [], lw=1.1)
            self.axes[ch] = ax
            self.lines[ch] = line

        self.rep_markers = None
        if "accel_magnitude" in self.axes:
            ax = self.axes["accel_magnitude"]
            self.rep_markers = LineCollection([], colors="tab:red", linewidths=1.2,
                                              linestyles="--", alpha=0.85,
                                              transform=ax.get_xaxis_transform())
            ax.add_collection(self.rep_markers)

        self.figure.tight_layout()

    def update(self, recorder: Any, *, now_s: float | None = None) -> None:
        ref_s = time.monotonic() if now_s is None else now_s
        snap = recorder.snapshot(now_s=ref_s)

        if self.rep_markers is not None:
            events = recorder.rep_events_snapshot(now_s=ref_s)
            accel_window = self.channel_windows.get("accel_magnitude", self.window_s)
            self.rep_markers.set_segments([[(t - ref_s, 0), (t - ref_s, 1)]
                                           for t in events if ref_s - accel_window <= t <= ref_s])

        for ch in self.channels:
            _, expected_dt, gap_factor = CHANNEL_SPECS[ch]
            window = self.channel_windows.get(ch, self.window_s)
            x, y = build_gapped_xy(
                snap.get(ch, ()),
                now_s=ref_s,
                window_s=window,
                expected_dt_s=expected_dt,
                gap_factor=gap_factor,
            )
            line = self.lines[ch]
            ax = self.axes[ch]
            line.set_data(x, y)
            ax.set_xlim(-window, 0.0)

            valid_y = y[np.isfinite(y)]
            if valid_y.size > 0:
                y_min = float(np.min(valid_y))
                y_max = float(np.max(valid_y))
                if y_min == y_max:
                    pad = max(abs(y_min) * 0.05, 1.0)
                else:
                    pad = (y_max - y_min) * 0.08
                ax.set_ylim(y_min - pad, y_max + pad)


class VitalsView:
    """Text readout of BPM/SpO2/reps plus a rolling BPM and SpO2 trend plot."""

    def __init__(self, window_s: float = 300.0, figsize: tuple[float, float] = (10.0, 3.0),
                 dpi: int = 80) -> None:
        self.window_s = float(window_s)
        self.figure, (self.text_ax, self.plot_ax) = plt.subplots(
            1, 2, figsize=figsize, dpi=dpi, gridspec_kw={"width_ratios": [1, 3]})

        self.text_ax.axis("off")
        self._readout = self.text_ax.text(
            0.02, 0.5, "", transform=self.text_ax.transAxes,
            fontsize=13, va="center", family="monospace")

        self.plot_ax.set_title("Heart rate and SpO\u2082", fontsize=9)
        self.plot_ax.set_xlabel("Time (s)", fontsize=8)
        self.plot_ax.set_xlim(-self.window_s, 0.0)
        self.plot_ax.tick_params(labelsize=8)
        self.plot_ax.grid(True, alpha=0.25)
        (self._bpm_line,) = self.plot_ax.plot([], [], lw=1.3, color="tab:red", label="BPM")
        (self._spo2_line,) = self.plot_ax.plot([], [], lw=1.3, color="tab:blue", label="SpO\u2082 %")
        self.plot_ax.legend(loc="upper left", fontsize=8)

        self.figure.tight_layout()

    def update(self, recorder: Any, *, now_s: float | None = None) -> None:
        ref_s = time.monotonic() if now_s is None else now_s
        v = recorder.vitals_snapshot()

        bpm = v["bpm"]
        spo2 = v["spo2"]
        self._readout.set_text(
            f"BPM:  {bpm if bpm is not None else '--'}\n"
            f"SpO2: {str(spo2) + '%' if spo2 is not None else '--'}\n"
            f"Reps: {v['reps']}")

        history = v["history"]
        bpm_x = [t - ref_s for t, b, _ in history if b is not None]
        bpm_y = [b for _, b, _ in history if b is not None]
        spo2_x = [t - ref_s for t, _, s in history if s is not None]
        spo2_y = [s for _, _, s in history if s is not None]
        self._bpm_line.set_data(bpm_x, bpm_y)
        self._spo2_line.set_data(spo2_x, spo2_y)

        self.plot_ax.set_xlim(-self.window_s, 0.0)
        ys = bpm_y + spo2_y
        if ys:
            y_min, y_max = float(min(ys)), float(max(ys))
            pad = max((y_max - y_min) * 0.08, 2.0)
            self.plot_ax.set_ylim(y_min - pad, y_max + pad)


class BackgroundRecorderRunner:
    """Runs recorder.poll() on a background thread so Jupyter renders stay responsive."""

    def __init__(self, recorder: Any, poll_interval_s: float = 0.005) -> None:
        self.recorder = recorder
        self.poll_interval_s = poll_interval_s
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.last_error: BaseException | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self.last_error = None
        self._thread = threading.Thread(target=self._loop, name="recorder-poll", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)

    def _loop(self) -> None:
        while not self._stop.is_set() and not getattr(self.recorder, "done", False):
            if getattr(self.recorder, "closed", False):
                break
            try:
                self.recorder.poll()
            except BaseException as exc:
                self.last_error = exc
                break
            time.sleep(self.poll_interval_s)


def run_jupyter_live_plot(
    recorder: Any,
    plot: LivePlot,
    display_handle: Any,
    *,
    refresh_s: float = 0.5,
    vitals_view: Any = None,
    vitals_handle: Any = None,
) -> None:
    """Drive Jupyter display updates while polling serial in the background.

    If ``vitals_view`` and ``vitals_handle`` are given, the BPM/SpO2/reps
    readout and trend plot refresh once per second.
    """
    runner = BackgroundRecorderRunner(recorder, poll_interval_s=0.005)
    runner.start()
    try:
        last_refresh = 0.0
        last_vitals_refresh = 0.0
        while not getattr(recorder, "done", False) and not getattr(recorder, "closed", False):
            if runner.last_error is not None:
                raise runner.last_error
            now = time.monotonic()
            if now - last_refresh >= refresh_s:
                plot.update(recorder, now_s=now)
                display_handle.update(plot.figure)
                if (vitals_view is not None and vitals_handle is not None
                        and now - last_vitals_refresh >= 1.0):
                    vitals_view.update(recorder, now_s=now)
                    vitals_handle.update(vitals_view.figure)
                    last_vitals_refresh = now
                last_refresh = now
            time.sleep(0.02)
    finally:
        runner.stop()
