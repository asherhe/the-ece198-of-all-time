"""Matplotlib view for the recorder's five-second signal buffers."""

from __future__ import annotations

import threading
import time
from typing import Any, Iterable

import matplotlib.pyplot as plt
import numpy as np


CHANNEL_SPECS: dict[str, tuple[str, float, float]] = {
    # channel_name: (subplot_title, expected_dt_s, gap_factor)
    "diff_raw": ("ECG Diff (Raw)", 0.005, 3.0),
    "wilson_raw": ("ECG Wilson (Raw)", 0.005, 3.0),
    "diff_filtered": ("ECG Diff (Filtered)", 0.005, 3.0),
    "wilson_filtered": ("ECG Wilson (Filtered)", 0.005, 3.0),
    "accel_magnitude": ("Accel Magnitude", 0.020, 3.0),
    "accel_smoothed": ("Accel Smoothed", 0.020, 3.0),
    "red": ("Optical Red", 0.010, 3.0),
    "infrared": ("Optical IR", 0.010, 3.0),
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
    gap_threshold = max(expected_dt_s * gap_factor, 0.05)

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
    ) -> None:
        self.window_s = float(window_s)
        selected = list(channels) if channels is not None else list(CHANNEL_SPECS.keys())
        self.channels = [ch for ch in selected if ch in CHANNEL_SPECS]
        if not self.channels:
            raise ValueError("No valid channels specified for LivePlot")

        n = len(self.channels)
        cols = 2 if n > 1 else 1
        rows = (n + cols - 1) // cols

        self.figure, axes_arr = plt.subplots(rows, cols, figsize=figsize, dpi=dpi, sharex=False)
        if isinstance(axes_arr, np.ndarray):
            axes_list = list(axes_arr.flat)
        else:
            axes_list = [axes_arr]

        self.axes: dict[str, Any] = {}
        self.lines: dict[str, Any] = {}

        for idx, ch in enumerate(self.channels):
            ax = axes_list[idx]
            title, _, _ = CHANNEL_SPECS[ch]
            ax.set_title(title, fontsize=9)
            ax.set_xlim(-self.window_s, 0.0)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.tick_params(labelsize=8)
            ax.grid(True, alpha=0.25)
            (line,) = ax.plot([], [], lw=1.1)
            self.axes[ch] = ax
            self.lines[ch] = line

        for extra_ax in axes_list[n:]:
            extra_ax.set_visible(False)

        self.figure.tight_layout()

    def update(self, recorder: Any, *, now_s: float | None = None) -> None:
        ref_s = time.monotonic() if now_s is None else now_s
        snap = recorder.snapshot(now_s=ref_s)

        for ch in self.channels:
            _, expected_dt, gap_factor = CHANNEL_SPECS[ch]
            x, y = build_gapped_xy(
                snap.get(ch, ()),
                now_s=ref_s,
                window_s=self.window_s,
                expected_dt_s=expected_dt,
                gap_factor=gap_factor,
            )
            line = self.lines[ch]
            ax = self.axes[ch]
            line.set_data(x, y)
            ax.set_xlim(-self.window_s, 0.0)

            valid_y = y[np.isfinite(y)]
            if valid_y.size > 0:
                y_min = float(np.min(valid_y))
                y_max = float(np.max(valid_y))
                if y_min == y_max:
                    pad = max(abs(y_min) * 0.05, 1.0)
                else:
                    pad = (y_max - y_min) * 0.08
                ax.set_ylim(y_min - pad, y_max + pad)


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
) -> None:
    """Drive Jupyter display updates while polling serial in the background."""
    runner = BackgroundRecorderRunner(recorder, poll_interval_s=0.005)
    runner.start()
    try:
        last_refresh = 0.0
        while not getattr(recorder, "done", False) and not getattr(recorder, "closed", False):
            if runner.last_error is not None:
                raise runner.last_error
            now = time.monotonic()
            if now - last_refresh >= refresh_s:
                plot.update(recorder, now_s=now)
                display_handle.update(plot.figure)
                last_refresh = time.monotonic()
            time.sleep(0.02)
    finally:
        runner.stop()