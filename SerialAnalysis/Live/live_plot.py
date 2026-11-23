"""Matplotlib view for the recorder's rolling signal buffers.

Rendering is the expensive part of a live plot. A full matplotlib draw of the
ten-panel notebook figure costs ~130 ms and ``display(figure)`` then draws it
again to encode a PNG, which capped the old viewer at about 2-3 frames per
second no matter what ``refresh_s`` asked for. This module instead

* draws the static parts (axes, ticks, grid, titles) once and caches them,
* redraws only the data lines on top of the cached background (blitting),
* only repeats the full draw when an axis limit actually has to change
  (limits use hysteresis so this is rare), and
* encodes the resulting pixel buffer directly (a few milliseconds) instead of
  asking matplotlib to render the figure a second time.
"""

from __future__ import annotations

import io
import threading
import time
from typing import Any, Iterable

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure

from .protocol import SAMPLE_PERIOD_US


def _dt(kind: int) -> float:
    return SAMPLE_PERIOD_US[kind] / 1e6


CHANNEL_SPECS: dict[str, tuple[str, float, float]] = {
    # channel_name: (subplot_title, expected_dt_s, gap_factor)
    "diff_raw": ("ECG Diff (Raw)", _dt(1), 1.5),
    "wilson_raw": ("ECG Wilson (Raw)", _dt(1), 1.5),
    "diff_filtered": ("ECG Diff (Filtered)", _dt(1), 1.5),
    "wilson_filtered": ("ECG Wilson (Filtered)", _dt(1), 1.5),
    "accel_magnitude": ("Accel Magnitude", _dt(2), 1.5),
    "accel_smoothed": ("Accel Smoothed", _dt(2), 1.5),
    "red": ("Optical Red", _dt(3), 1.5),
    "infrared": ("Optical IR", _dt(3), 1.5),
}


def build_gapped_xy(
    samples: Iterable[tuple[float, float]],
    now_s: float,
    window_s: float = 5.0,
    expected_dt_s: float = 0.01,
    gap_factor: float = 3.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return x (seconds relative to now_s) and y arrays with NaNs across gaps."""
    empty = np.empty(0, dtype=np.float64)
    data = np.asarray(samples if isinstance(samples, (list, tuple, np.ndarray)) else list(samples),
                      dtype=np.float64)
    if data.size == 0:
        return empty, empty
    data = data.reshape(-1, 2)
    times, values = data[:, 0], data[:, 1]
    keep = (times >= now_s - window_s) & (times <= now_s + 0.05)
    times, values = times[keep], values[keep]
    if times.size == 0:
        return empty, empty

    # Break the line right after the last valid point before each gap.
    gaps = np.flatnonzero(np.diff(times) > expected_dt_s * gap_factor)
    if gaps.size:
        break_times = np.minimum(now_s, times[gaps] + 1e-6)
        times = np.insert(times, gaps + 1, break_times)
        values = np.insert(values, gaps + 1, np.nan)
    return times - now_s, values


def encode_rgba(rgba: np.ndarray, fmt: str = "png", quality: int = 85) -> bytes:
    """Encode an RGBA pixel buffer. PNG uses the fastest zlib level."""
    fmt = fmt.lower()
    if fmt not in ("png", "jpeg", "jpg"):
        raise ValueError("fmt must be 'png' or 'jpeg'")
    from PIL import Image  # installed with matplotlib

    image = Image.fromarray(np.ascontiguousarray(np.asarray(rgba)[..., :3]))
    out = io.BytesIO()
    if fmt == "png":
        image.save(out, format="PNG", compress_level=1)
    else:
        image.save(out, format="JPEG", quality=quality)
    return out.getvalue()


class _BlitFigure:
    """Figure + Agg canvas that caches everything except the animated lines."""

    SHRINK_AFTER_S = 1.5   # how long data must under-fill an axis before it shrinks
    SHRINK_FILL = 0.4      # ... under-fill meaning data span < 40 % of the axis span
    GROW_HEADROOM = 0.12   # extra room added when the data outgrow an axis

    def __init__(self, figsize: tuple[float, float], dpi: int) -> None:
        # Not a pyplot figure: no global figure manager, nothing to close, and
        # it always has an Agg canvas whatever notebook backend is active.
        # Layout engines re-run on every draw (+200 ms), so the layout is
        # solved once in freeze_layout() and then switched off.
        self.figure = Figure(figsize=figsize, dpi=dpi)
        self.canvas = FigureCanvasAgg(self.figure)
        self._background: Any = None
        self._animated: list[tuple[Any, Any]] = []
        self._dirty = True
        self._ylim_changed_at: dict[Any, float] = {}
        self.last_render_ms = 0.0
        self.full_draws = 0
        self.frames = 0
        self.fps = 0.0

    # -- artists ---------------------------------------------------------
    def animate(self, axis: Any, artist: Any) -> None:
        artist.set_animated(True)
        self._animated.append((axis, artist))
        self._dirty = True

    def invalidate(self) -> None:
        self._dirty = True

    def freeze_layout(self, widest_limits: dict[Any, tuple[float, float]] | None = None) -> None:
        """Solve a tight layout once, using worst-case tick labels, and keep it.

        ``widest_limits`` maps axes to the widest y-range they will show so
        later, wider tick labels are not clipped by the fixed margins.
        """
        saved = {}
        for axis, limits in (widest_limits or {}).items():
            saved[axis] = axis.get_ylim()
            axis.set_ylim(*limits)
        self.figure.set_layout_engine("tight")
        self.canvas.draw()
        self.figure.set_layout_engine("none")
        for axis, limits in saved.items():
            axis.set_ylim(*limits)
        self._dirty = True

    # -- limits ----------------------------------------------------------
    def fit_ylim(self, axis: Any, values: np.ndarray, now: float) -> None:
        """Change y-limits only when needed so cached backgrounds stay valid."""
        if values.size == 0:
            return
        low, high = float(values.min()), float(values.max())
        span = high - low
        pad = max(abs(low) * 0.05, 1.0) if span == 0 else span * 0.08
        want = (low - pad, high + pad)
        first = axis not in self._ylim_changed_at
        cur_low, cur_high = axis.get_ylim()
        cur_span = cur_high - cur_low
        if first:
            new = want
        elif low < cur_low or high > cur_high:
            room = max(span, 1.0) * self.GROW_HEADROOM
            new = (min(want[0], low - room) if low < cur_low else cur_low,
                   max(want[1], high + room) if high > cur_high else cur_high)
        elif (span < cur_span * self.SHRINK_FILL
              and now - self._ylim_changed_at[axis] >= self.SHRINK_AFTER_S):
            new = want
        else:
            if span >= cur_span * self.SHRINK_FILL:
                self._ylim_changed_at[axis] = max(self._ylim_changed_at[axis], now)
            return
        if new != (cur_low, cur_high):
            axis.set_ylim(*new)
            self._dirty = True
        self._ylim_changed_at[axis] = now

    # -- rendering -------------------------------------------------------
    def render_rgba(self) -> np.ndarray:
        started = time.perf_counter()
        if self._dirty or self._background is None:
            self.canvas.draw()
            self._background = self.canvas.copy_from_bbox(self.figure.bbox)
            self._dirty = False
            self.full_draws += 1
        else:
            self.canvas.restore_region(self._background)
        for axis, artist in self._animated:
            axis.draw_artist(artist)
        rgba = np.asarray(self.canvas.buffer_rgba())
        self.frames += 1
        self.last_render_ms = (time.perf_counter() - started) * 1000.0
        return rgba

    def image_bytes(self, fmt: str = "png") -> bytes:
        return encode_rgba(self.render_rgba(), fmt)

    def image(self, fmt: str = "png") -> Any:
        """An ``IPython.display.Image`` for ``display_handle.update(...)``."""
        from IPython.display import Image

        fmt = "jpeg" if fmt.lower() == "jpg" else fmt.lower()
        return Image(data=self.image_bytes(fmt), format=fmt)


class LivePlot(_BlitFigure):
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
        super().__init__(figsize, dpi)

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
            self.animate(ax, line)

        self.rep_markers = None
        if "accel_magnitude" in self.axes:
            ax = self.axes["accel_magnitude"]
            self.rep_markers = LineCollection([], colors="tab:red", linewidths=1.2,
                                              linestyles="--", alpha=0.85,
                                              transform=ax.get_xaxis_transform())
            ax.add_collection(self.rep_markers)
            self.animate(ax, self.rep_markers)

        widest = {"red": (0.0, 262143.0), "infrared": (0.0, 262143.0),
                  "accel_magnitude": (0.0, 30.0), "accel_smoothed": (0.0, 30.0)}
        self.freeze_layout({self.axes[ch]: widest.get(ch, (-20000.0, 20000.0))
                            for ch in self.channels})

    def update(self, recorder: Any, *, now_s: float | None = None) -> None:
        """Load the recorder's latest samples into the lines (cheap; no drawing)."""
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
            self.lines[ch].set_data(x, y)
            ax = self.axes[ch]
            if tuple(ax.get_xlim()) != (-window, 0.0):
                ax.set_xlim(-window, 0.0)
                self.invalidate()
            self.fit_ylim(ax, y[np.isfinite(y)], ref_s)


class VitalsView(_BlitFigure):
    """Text readout of BPM/SpO2/reps plus a rolling BPM and SpO2 trend plot.

    It changes once a second, so it is simply redrawn in full each time.
    """

    def __init__(self, window_s: float = 300.0, figsize: tuple[float, float] = (10.0, 3.0),
                 dpi: int = 80) -> None:
        super().__init__(figsize, dpi)
        self.window_s = float(window_s)
        self.status_text = ""
        grid = self.figure.add_gridspec(1, 2, width_ratios=[1, 3])
        self.text_ax = self.figure.add_subplot(grid[0, 0])
        self.plot_ax = self.figure.add_subplot(grid[0, 1])

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
        self.freeze_layout({self.plot_ax: (0.0, 200.0)})

    def update(self, recorder: Any, *, now_s: float | None = None) -> None:
        ref_s = time.monotonic() if now_s is None else now_s
        v = recorder.vitals_snapshot()

        bpm = v["bpm"]
        spo2 = v["spo2"]
        text = (f"BPM:  {bpm if bpm is not None else '--'}\n"
                f"SpO2: {str(spo2) + '%' if spo2 is not None else '--'}\n"
                f"Reps: {v['reps']}")
        if self.status_text:
            text += f"\n\n{self.status_text}"
        self._readout.set_text(text)

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
        self.invalidate()


class BackgroundRecorderRunner:
    """Runs recorder.poll() on a background thread so Jupyter renders stay responsive."""

    def __init__(self, recorder: Any, poll_interval_s: float = 0.002) -> None:
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
            # poll() already waits for data; this only yields the GIL.
            self._stop.wait(self.poll_interval_s)


def _show(handle: Any, view: Any, fmt: str) -> None:
    """Update a Jupyter display handle with the fastest available image."""
    if hasattr(view, "image"):
        handle.update(view.image(fmt))
    else:  # any object IPython can display, e.g. a plain matplotlib Figure
        handle.update(getattr(view, "figure", view))


def run_jupyter_live_plot(
    recorder: Any,
    plot: Any,
    display_handle: Any,
    *,
    refresh_s: float = 0.1,
    vitals_view: Any = None,
    vitals_handle: Any = None,
    vitals_refresh_s: float = 1.0,
    image_format: str = "png",
) -> None:
    """Drive Jupyter display updates while polling serial in the background.

    Frames are scheduled on a fixed cadence and never queue up: if a frame
    takes longer than ``refresh_s`` the next one starts immediately instead
    of several being rendered back to back. If ``vitals_view`` and
    ``vitals_handle`` are given, the BPM/SpO2/reps readout refreshes every
    ``vitals_refresh_s`` seconds. ``plot.fps`` holds the measured frame rate.
    """
    runner = BackgroundRecorderRunner(recorder)
    runner.start()
    try:
        next_frame = next_vitals = time.monotonic()
        fps_started, fps_frames = next_frame, 0
        while not getattr(recorder, "done", False) and not getattr(recorder, "closed", False):
            if runner.last_error is not None:
                raise runner.last_error
            now = time.monotonic()
            if now >= next_frame:
                plot.update(recorder, now_s=now)
                _show(display_handle, plot, image_format)
                fps_frames += 1
                if now - fps_started >= 1.0:
                    plot.fps = fps_frames / (now - fps_started)
                    fps_started, fps_frames = now, 0
                if vitals_view is not None and vitals_handle is not None and now >= next_vitals:
                    vitals_view.status_text = f"plot {getattr(plot, 'fps', 0.0):4.1f} fps"
                    vitals_view.update(recorder, now_s=now)
                    _show(vitals_handle, vitals_view, image_format)
                    next_vitals = now + vitals_refresh_s
                next_frame = max(next_frame + refresh_s, time.monotonic())
            else:
                time.sleep(min(0.005, next_frame - now))
    finally:
        runner.stop()
