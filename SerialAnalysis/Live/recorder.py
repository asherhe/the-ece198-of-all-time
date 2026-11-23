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

from .analysis import AccelerationAnalysis, EcgFilter, Pulse, RepCounter
from .protocol import SAMPLE_PERIOD_US, Decoder, Frame, encode_frame


CHANNELS = (
    "diff_raw", "wilson_raw", "diff_filtered", "wilson_filtered",
    "accel_magnitude", "accel_smoothed", "red", "infrared",
)
SAMPLE_PERIOD_S = {kind: period / 1e6 for kind, period in SAMPLE_PERIOD_US.items()}
# Largest single read; the serial port is polled for whatever has arrived.
READ_CHUNK = 4096


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
                 window_s: float = 5.0, vitals_window_s: float = 300.0):
        if duration_min <= 0 or window_s <= 0:
            raise ValueError("duration_min and window_s must be positive")
        self.port_name = port_name
        self.output = Path(output)
        self.duration_min = duration_min
        self.window_s = window_s
        self.vitals_window_s = vitals_window_s
        self.port = port
        self.owns_port = port is None
        self.decoder = Decoder()
        self.diff = EcgFilter()
        self.wilson = EcgFilter()
        self.motion = AccelerationAnalysis()
        self.reps = RepCounter()
        self.pulse = Pulse()
        self.buffers = {name: deque() for name in CHANNELS}
        # Rolling host-time history of per-second vitals for the notebook's
        # BPM/SpO2 trend plot. Each entry is (host_s, bpm, spo2) with the value
        # left as None when that reading is not currently valid.
        self.vitals_history: deque[tuple[float, int | None, int | None]] = deque()
        self.rep_count = 0
        self.rep_events: deque[float] = deque()
        # Maps unwrapped board microseconds onto host-monotonic seconds so the
        # plot timeline follows the board's own clock instead of host arrival
        # time. Kept per frame type because the sensors can run on independent
        # clocks (optical has its own); anchored on the first sample of each
        # type and re-anchored on drift.
        self._board_to_host_offset: dict[int, float] = {}
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
        self._last_sample_us: dict[int, int] = {}
        self.timestamp_discontinuities = {1: 0, 2: 0, 3: 0}
        self.estimated_missing_samples = {1: 0, 2: 0, 3: 0}
        self.board_diagnostics: dict[str, int] = {}

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
                                                       "spo2_valid", "rep_count"])
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
                self.rep_events.clear()
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
        self.rep_events.clear()
        self._board_to_host_offset.clear()

    def _sample(self, channel: str, received_s: float, value: float) -> None:
        self.buffers[channel].append((received_s, value))

    def _trim(self, now_s: float) -> None:
        cutoff = now_s - self.window_s
        for buffer in self.buffers.values():
            while buffer and buffer[0][0] < cutoff:
                buffer.popleft()
        while self.rep_events and self.rep_events[0] < cutoff:
            self.rep_events.popleft()

    @staticmethod
    def _sample_width(frame: Frame) -> int:
        width = {1: 4, 2: 6, 3: 12}.get(frame.type, 0)
        return width if frame.count and len(frame.payload) == frame.count * width else 0

    def _plot_time(self, board_us: int, kind: int, now_s: float) -> float:
        """Map an unwrapped board timestamp (microseconds) to host-monotonic
        seconds for the given frame type. Anchor per type on first use and
        re-anchor on drift so the newest sample stays near ``now_s`` and the
        5-second window math holds."""
        board_s = board_us / 1e6
        offset = self._board_to_host_offset.get(kind)
        if offset is None:
            offset = self._board_to_host_offset[kind] = now_s - board_s
        plot_time = board_s + offset
        # Re-anchor if this type's board clock has drifted ahead of host time or
        # fallen behind by more than the plot window; keeps the newest sample
        # near now so the rolling window keeps showing live data.
        if plot_time > now_s + 0.05 or plot_time < now_s - self.window_s:
            offset = self._board_to_host_offset[kind] = now_s - board_s
            plot_time = now_s
        return plot_time

    def feed(self, data: bytes, *, received_at: float | None = None) -> None:
        """Decode bytes and process complete frames (also useful for replay)."""
        if not self.started or self.closed:
            raise RuntimeError("recorder is not active")
        now_s = time.monotonic() if received_at is None else received_at
        frames = self.decoder.feed(data)
        if not frames:
            with self._data_lock:
                self._trim(now_s)
            return

        with self._data_lock:
            for frame in frames:
                self._frame(frame, now_s)
            self._trim(now_s)

    def _frame(self, frame: Frame, now_s: float) -> None:
        if self.prev_seq is not None:
            missed = (frame.sequence - self.prev_seq - 1) & 0xFFFF
            # Ignore duplicates/backward jumps rather than treating them as 65535 losses.
            if missed < 0x8000:
                self.gaps += missed
        self.prev_seq = frame.sequence
        self.frames += 1
        if frame.type == 4:
            if frame.count == 1 and len(frame.payload) in (40, 44, 48):
                names = ("missed_ecg_deadlines", "missed_accel_deadlines", "i2c_errors",
                         "optical_fifo_overflows", "tx_dropped_ecg", "tx_dropped_accel",
                         "tx_dropped_optical", "max_loop_us", "max_optical_us", "optical_ready")
                self.board_diagnostics = dict(zip(names, struct.unpack("<10I", frame.payload[:40])))
                if len(frame.payload) >= 44:
                    self.board_diagnostics["optical_fifo_state"] = struct.unpack_from("<I", frame.payload, 40)[0]
                if len(frame.payload) >= 48:
                    self.board_diagnostics["optical_overflow_counter_reliable"] = struct.unpack_from("<I", frame.payload, 44)[0]
            return
        width = self._sample_width(frame)
        if not width:
            return
        t0 = self._unwrap(frame.timestamp_us, frame.type) if frame.type != 3 else 0
        for i in range(frame.count):
            payload = frame.payload[i*width:(i+1)*width]
            if frame.type == 1:
                t = t0 + i * SAMPLE_PERIOD_US[1]
                self._track_sample_time(frame.type, t)
                plot_time = self._plot_time(t, frame.type, now_s)
                diff_raw, wilson_raw = struct.unpack("<HH", payload)
                diff = self.diff.run(diff_raw)
                wilson = self.wilson.run(wilson_raw)
                self._ecg.writerow([t, diff_raw, wilson_raw, diff, wilson])
                for name, value in (("diff_raw", diff_raw), ("wilson_raw", wilson_raw),
                                    ("diff_filtered", diff), ("wilson_filtered", wilson)):
                    self._sample(name, plot_time, value)
                self.ecg_samples += 1
            elif frame.type == 2:
                t = t0 + i * SAMPLE_PERIOD_US[2]
                self._track_sample_time(frame.type, t)
                plot_time = self._plot_time(t, frame.type, now_s)
                raw_x, raw_y, raw_z = struct.unpack("<hhh", payload)
                magnitude, smoothed = self.motion.run(raw_x, raw_y, raw_z)
                dynamic = magnitude - smoothed
                self.diff.update_motion(dynamic)
                self.wilson.update_motion(dynamic)
                previous_reps = self.rep_count
                self.rep_count = self.reps.run(raw_x, raw_y, raw_z, t // 1000)
                if self.rep_count > previous_reps:
                    self.rep_events.append(plot_time)
                self._accel.writerow([t, magnitude, smoothed])
                self._sample("accel_magnitude", plot_time, magnitude)
                self._sample("accel_smoothed", plot_time, smoothed)
                self.accel_samples += 1
            else:
                raw_t, red, infrared = struct.unpack("<III", payload)
                # Unwrap the optical clock once; reuse it for both the pulse
                # analysis buffer and the plot sample time.
                t = self._unwrap(raw_t, frame.type)
                self._track_sample_time(frame.type, t)
                plot_time = self._plot_time(t, frame.type, now_s)
                self.pulse.add(t/1e6, red, infrared)
                self._sample("red", plot_time, red)
                self._sample("infrared", plot_time, infrared)
                self.optical_samples += 1

    def _track_sample_time(self, kind: int, timestamp_us: int) -> None:
        previous = self._last_sample_us.get(kind)
        period = SAMPLE_PERIOD_US[kind]
        if previous is not None:
            delta = timestamp_us - previous
            if delta <= 0 or delta > period * 1.5:
                self.timestamp_discontinuities[kind] += 1
                if delta > period * 1.5:
                    self.estimated_missing_samples[kind] += max(0, round(delta / period) - 1)
                if kind == 1:
                    self.diff.reset()
                    self.wilson.reset()
                elif kind == 3:
                    self.pulse.data.clear()
        self._last_sample_us[kind] = timestamp_us

    def _read_available(self) -> bytes:
        """Return the bytes already waiting on the port.

        ``read(n)`` blocks until ``n`` bytes arrive or the port timeout
        expires. Asking for a fixed 1024 bytes at the board's ~4 KB/s made
        every poll wait for the 100 ms timeout, so samples reached the plot
        in bursts. Reading only what is waiting (at least one byte, which
        still honours the timeout while the line is idle) removes that delay.
        """
        waiting = getattr(self.port, "in_waiting", None)
        if waiting is None:  # serial-like test/replay objects
            return self.port.read(1024)
        return self.port.read(max(1, min(READ_CHUNK, waiting)))

    def poll(self) -> None:
        if not self.started or self.closed:
            raise RuntimeError("recorder is not active")
        with self._io_lock:
            data = self._read_available()
        if data:
            self.feed(data)
        now = time.monotonic()
        if now - self.last_display >= 1:
            self._display_update(now)

    def _display_update(self, now: float) -> None:
        vitals = self.pulse.calculate()
        remaining = max(0, int(self.duration_min*60 - (now - self.start_time)))
        payload = struct.pack("<HIB", vitals.bpm, remaining,
                              int(vitals.heart_rate_valid))
        packet = encode_frame(Frame(0x80, 0, 0, 1, payload))
        # The OLED update is optional. USB CDC can stall on a host-to-board
        # write even while sensor frames continue arriving in the other direction.
        # Hold the IO lock only around the write itself so a slow host-to-board
        # write cannot block the read path in poll() for up to a second.
        try:
            with self._io_lock:
                written = self.port.write(packet)
            if written != len(packet):
                self.display_write_failures += 1
        except OSError:
            self.display_write_failures += 1
        self._vitals.writerow([self.latest_board_us, vitals.bpm, int(vitals.heart_rate_valid),
                               vitals.spo2, int(vitals.spo2_valid), self.rep_count])
        with self._data_lock:
            self.vitals_history.append((
                now,
                vitals.bpm if vitals.heart_rate_valid else None,
                vitals.spo2 if vitals.spo2_valid else None,
            ))
            cutoff = now - self.vitals_window_s
            while self.vitals_history and self.vitals_history[0][0] < cutoff:
                self.vitals_history.popleft()
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

    def rep_events_snapshot(self, *, now_s: float | None = None) -> list[float]:
        ref_s = time.monotonic() if now_s is None else now_s
        with self._data_lock:
            self._trim(ref_s)
            return list(self.rep_events)

    def vitals_snapshot(self) -> dict[str, Any]:
        """Latest vitals readout plus rolling BPM/SpO2 history for plotting.

        ``bpm`` and ``spo2`` are the most recent valid readings (or None if
        none is valid yet); ``reps`` is the running rep count. ``history`` lists
        (host_s, bpm, spo2) points with None where a reading was invalid.
        """
        vitals = self.pulse.vitals
        with self._data_lock:
            history = list(self.vitals_history)
        last_bpm = next((b for _, b, _ in reversed(history) if b is not None), None)
        last_spo2 = next((s for _, _, s in reversed(history) if s is not None), None)
        return {
            "bpm": last_bpm if last_bpm is not None else (vitals.bpm if vitals.heart_rate_valid else None),
            "spo2": last_spo2 if last_spo2 is not None else (vitals.spo2 if vitals.spo2_valid else None),
            "reps": self.rep_count,
            "history": history,
        }

    def stats(self) -> dict[str, Any]:
        return {"frames": self.frames, "sequence_gaps": self.gaps,
                "bad_frames": self.decoder.bad,
                "display_write_failures": self.display_write_failures,
                "ecg_samples": self.ecg_samples,
                "accel_samples": self.accel_samples,
                "optical_samples": self.optical_samples,
                "rep_count": self.rep_count,
                "timestamp_discontinuities": dict(self.timestamp_discontinuities),
                "estimated_missing_samples": dict(self.estimated_missing_samples),
                "board_diagnostics": dict(self.board_diagnostics)}

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
