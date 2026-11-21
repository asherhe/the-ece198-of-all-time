"""Serial acquisition, session files, and five-second plot buffers."""

from collections import deque
from datetime import datetime, timezone
import csv
import json
from pathlib import Path
import struct
import threading
import time
from typing import Any

from .analysis import AccelerationAnalysis, EcgFilter, Pulse
from .protocol import Decoder, Frame, encode_frame


CHANNELS = (
    "diff_raw", "wilson_raw", "diff_filtered", "wilson_filtered",
    "accel_magnitude", "accel_smoothed", "red", "infrared",
)
SAMPLE_PERIOD_S = {1: 0.005, 2: 0.020, 3: 0.010}


def open_serial(port_name: str) -> Any:
    """Open 115200 8N1 with DTR asserted for native USB CDC boards."""
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "Install pyserial in this Python environment "
            "(Jupyter: %pip install pyserial; terminal: python -m pip install pyserial)"
        ) from exc
    port = serial.Serial(port=None, baudrate=115200, bytesize=8,
                         parity="N", stopbits=1, timeout=0.1,
                         write_timeout=0.2, xonxoff=False, rtscts=False,
                         dsrdtr=False)
    # UNO R4 Minima's USB CDC write path requires DTR from the host.
    port.dtr = True
    port.rts = True
    port.port = port_name
    port.open()
    return port


class LiveRecorder:
    """Poll from a serial port and maintain a five-second host-time plot window.

    Call ``poll`` repeatedly. ``port`` can be a serial-like object for tests
    or replay; if omitted, pyserial opens ``port_name`` on entry.
    """

    def __init__(self, port_name: str, output: str | Path = "sessions",
                 duration_min: float = 30, *, port: Any = None,
                 window_s: float = 5.0):
        if duration_min <= 0 or window_s <= 0:
            raise ValueError("duration_min and window_s must be positive")
        self.port_name = port_name
        self.output = Path(output)
        self.duration_min = duration_min
        self.window_s = window_s
        self.port = port
        self.owns_port = port is None
        self.decoder = Decoder()
        self.diff = EcgFilter()
        self.wilson = EcgFilter()
        self.motion = AccelerationAnalysis()
        self.pulse = Pulse()
        self.buffers = {name: deque() for name in CHANNELS}
        self._last_plot_time: dict[int, float] = {}
        self.prev_seq: int | None = None
        self._clocks: dict[int, tuple[int, int]] = {}
        self.latest_board_us = 0
        self.frames = self.gaps = 0
        self.ecg_samples = self.accel_samples = self.optical_samples = 0
        self.display_write_failures = 0
        self._files = []
        self.started = False
        self.closed = False
        self._data_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._prev_seq_by_type: dict[int, int] = {}

    def __enter__(self) -> "LiveRecorder":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def start(self) -> None:
        if self.started:
            raise RuntimeError("recorder already started")
        if self.port is None:
            self.port = open_serial(self.port_name)
        try:
            now = datetime.now(timezone.utc)
            self.started_utc = now.strftime("%Y-%m-%dT%H:%M:%SZ")
            base = self.output / now.strftime("%Y-%m-%dT%H-%M-%SZ")
            self.session_dir = base
            suffix = 1
            while self.session_dir.exists():
                self.session_dir = Path(f"{base}-{suffix}")
                suffix += 1
            self.session_dir.mkdir(parents=True)
            self._ecg = self._csv("ecg.csv", ["board_us", "diff_raw", "wilson_raw",
                                               "diff_filtered", "wilson_filtered"])
            self._accel = self._csv("acceleration.csv", ["board_us", "magnitude_m_s2",
                                                         "smoothed_m_s2"])
            self._vitals = self._csv("vitals.csv", ["board_us", "heart_rate_bpm",
                                                       "heart_rate_valid", "spo2_percent",
                                                       "spo2_valid"])
            self.start_time = self.last_display = time.monotonic()
            self.started = True
        except BaseException:
            for file in self._files:
                file.close()
            if self.owns_port:
                self.port.close()
            raise

    def _csv(self, name: str, header: list[str]) -> csv.writer:
        file = (self.session_dir / name).open("w", newline="", encoding="utf-8")
        self._files.append(file)
        writer = csv.writer(file)
        writer.writerow(header)
        return writer

    def _unwrap(self, timestamp: int, kind: int) -> int:
        previous, epoch = self._clocks.get(kind, (None, 0))
        if previous is not None and timestamp < previous:
            if previous - timestamp > 0x80000000:
                epoch += 1 << 32
            elif kind == 1:
                self.diff.reset()
                self.wilson.reset()
                for name in ("diff_raw", "wilson_raw", "diff_filtered", "wilson_filtered"):
                    self.buffers[name].clear()
            elif kind == 2:
                for name in ("accel_magnitude", "accel_smoothed"):
                    self.buffers[name].clear()
            else:
                for name in ("red", "infrared"):
                    self.buffers[name].clear()
                self.pulse.data.clear()
        self._clocks[kind] = (timestamp, epoch)
        unwrapped = epoch + timestamp
        self.latest_board_us = max(self.latest_board_us, unwrapped)
        return unwrapped

    def clear_window(self) -> None:
        for buffer in self.buffers.values():
            buffer.clear()
        self._last_plot_time.clear()

    def _sample(self, channel: str, received_s: float, value: float) -> None:
        self.buffers[channel].append((received_s, value))

    def _trim(self, now_s: float) -> None:
        cutoff = now_s - self.window_s
        for buffer in self.buffers.values():
            while buffer and buffer[0][0] < cutoff:
                buffer.popleft()

    @staticmethod
    def _sample_width(frame: Frame) -> int:
        width = {1: 4, 2: 6, 3: 12}.get(frame.type, 0)
        return width if frame.count and len(frame.payload) == frame.count * width else 0

    def feed(self, data: bytes, *, received_at: float | None = None) -> None:
        """Decode bytes and process complete frames (also useful for replay)."""
        if not self.started or self.closed:
            raise RuntimeError("recorder is not active")
        now_s = time.monotonic() if received_at is None else received_at
        frames = [frame for frame in self.decoder.feed(data) if self._sample_width(frame)]
        if not frames:
            with self._data_lock:
                self._trim(now_s)
            return

        counts = {
            kind: sum(frame.count for frame in frames if frame.type == kind)
            for kind in SAMPLE_PERIOD_S
        }
        plot_times: dict[int, Any] = {}
        for kind, count in counts.items():
            if not count:
                continue
            period = SAMPLE_PERIOD_S[kind]
            # Place the newest sample at now_s and step backward by nominal period
            # so bursty serial reads do not squash samples into a single instant.
            first = now_s - (count - 1) * period
            prev_last = self._last_plot_time.get(kind)
            if prev_last is not None and first <= prev_last:
                # Avoid overlapping timestamps if a previous batch landed slightly late
                first = min(now_s, prev_last + min(period * 0.25, 0.002))
            times = [first + i * period for i in range(count)]
            plot_times[kind] = iter(times)
            self._last_plot_time[kind] = times[-1]

        with self._data_lock:
            for frame in frames:
                self._frame(frame, plot_times[frame.type])
            self._trim(now_s)

    def _frame(self, frame: Frame, plot_times) -> None:
        width = self._sample_width(frame)
        prev_seq = self._prev_seq_by_type.get(frame.type)
        if prev_seq is not None:
            missed = (frame.sequence - prev_seq - 1) & 0xFFFF
            if missed:
                self.gaps += missed
                if frame.type == 1:
                    self.diff.reset()
                    self.wilson.reset()
        self._prev_seq_by_type[frame.type] = frame.sequence
        self.prev_seq = frame.sequence
        self.frames += 1
        t0 = self._unwrap(frame.timestamp_us, frame.type)
        for i in range(frame.count):
            payload = frame.payload[i*width:(i+1)*width]
            plot_time = next(plot_times)
            if frame.type == 1:
                t = t0 + i*5000
                diff_raw, wilson_raw = struct.unpack("<HH", payload)
                diff = self.diff.run(diff_raw)
                wilson = self.wilson.run(wilson_raw)
                self._ecg.writerow([t, diff_raw, wilson_raw, diff, wilson])
                for name, value in (("diff_raw", diff_raw), ("wilson_raw", wilson_raw),
                                    ("diff_filtered", diff), ("wilson_filtered", wilson)):
                    self._sample(name, plot_time, value)
                self.ecg_samples += 1
            elif frame.type == 2:
                t = t0 + i*20000
                magnitude, smoothed = self.motion.run(*struct.unpack("<hhh", payload))
                self._accel.writerow([t, magnitude, smoothed])
                self._sample("accel_magnitude", plot_time, magnitude)
                self._sample("accel_smoothed", plot_time, smoothed)
                self.accel_samples += 1
            else:
                raw_t, red, infrared = struct.unpack("<III", payload)
                t = self._unwrap(raw_t, frame.type)
                self.pulse.add(t/1e6, red, infrared)
                self._sample("red", plot_time, red)
                self._sample("infrared", plot_time, infrared)
                self.optical_samples += 1

    def poll(self) -> None:
        if not self.started or self.closed:
            raise RuntimeError("recorder is not active")
        with self._io_lock:
            data = self.port.read(1024)
        if data:
            self.feed(data)
        now = time.monotonic()
        if now - self.last_display >= 1:
            with self._io_lock:
                self._display_update(now)

    def _display_update(self, now: float) -> None:
        vitals = self.pulse.calculate()
        remaining = max(0, int(self.duration_min*60 - (now - self.start_time)))
        payload = struct.pack("<HIB", vitals.bpm, remaining,
                              int(vitals.heart_rate_valid))
        packet = encode_frame(Frame(0x80, 0, 0, 1, payload))
        # The OLED update is optional. USB CDC can stall on a host-to-board
        # write even while sensor frames continue arriving in the other direction.
        try:
            if self.port.write(packet) != len(packet):
                self.display_write_failures += 1
        except OSError:
            self.display_write_failures += 1
        self._vitals.writerow([self.latest_board_us, vitals.bpm, int(vitals.heart_rate_valid),
                               vitals.spo2, int(vitals.spo2_valid)])
        for file in self._files:
            file.flush()
        self.last_display = now

    @property
    def done(self) -> bool:
        return self.started and time.monotonic() - self.start_time >= self.duration_min*60

    def snapshot(self, *, now_s: float | None = None) -> dict[str, list[tuple[float, float]]]:
        ref_s = time.monotonic() if now_s is None else now_s
        with self._data_lock:
            self._trim(ref_s)
            return {name: list(buffer) for name, buffer in self.buffers.items()}

    def stats(self) -> dict[str, int]:
        return {"frames": self.frames, "sequence_gaps": self.gaps,
                "bad_frames": self.decoder.bad,
                "display_write_failures": self.display_write_failures,
                "ecg_samples": self.ecg_samples,
                "accel_samples": self.accel_samples,
                "optical_samples": self.optical_samples}

    def close(self) -> None:
        if self.closed:
            return
        if not self.started:
            if self.owns_port and self.port is not None:
                self.port.close()
            self.closed = True
            return
        for file in self._files:
            file.close()
        metadata = {"protocol_version": 1, "started_utc": self.started_utc,
                    "port": self.port_name, "duration_minutes": self.duration_min,
                    **self.stats()}
        (self.session_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        if self.owns_port:
            self.port.close()
        self.closed = True
