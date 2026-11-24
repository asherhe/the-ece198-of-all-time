"""Offline protocol and recorder checks; no serial hardware required."""

import csv
import json
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from SerialAnalysis.Live.protocol import Decoder, Frame, crc16, encode_frame
from SerialAnalysis.Live.recorder import LiveRecorder, open_serial
from SerialAnalysis.Live.analysis import EcgFilter, RepCounter


class FakePort:
    def __init__(self):
        self.writes = []

    def read(self, _size):
        return b""

    def write(self, data):
        self.writes.append(data)
        return len(data)


class ProtocolTests(unittest.TestCase):
    def test_crc_and_split_frame_recovery(self):
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        frame = Frame(1, 42, 123456, 1, struct.pack("<HH", 8192, 9000))
        good = encode_frame(frame)
        damaged = bytearray(good)
        damaged[12] ^= 1
        decoder = Decoder()
        self.assertEqual(decoder.feed(b"noise" + good[:5]), [])
        self.assertEqual(decoder.feed(good[5:] + damaged + good), [frame, frame])
        self.assertEqual(decoder.bad, 1)


class EcgFilterTests(unittest.TestCase):
    def test_adc_rails_and_slew_limit(self):
        ecg = EcgFilter(16383.0)
        ecg.run(16000)
        ecg.run(16383)
        self.assertEqual(ecg.prev_clean_raw, 16383)
        ecg.run(0)
        self.assertEqual(ecg.prev_clean_raw, 16383 - 16383 * 0.25)
        for _ in range(3):
            ecg.run(0)
        self.assertEqual(ecg.prev_clean_raw, 0)

    def test_default_and_recorder_adc_ranges(self):
        self.assertEqual(EcgFilter().max_slew, 1023 * 0.25)
        recorder = LiveRecorder("FAKE", port=FakePort())
        self.assertEqual(recorder.diff.max_slew, 16383 * 0.25)
        self.assertEqual(recorder.wilson.max_slew, 16383 * 0.25)


class RecorderTests(unittest.TestCase):
    def test_open_asserts_dtr_before_connecting(self):
        class SerialStub:
            def __init__(self, **kwargs):
                self.baudrate = kwargs["baudrate"]
                self.dtr = None

            def open(self):
                self.dtr_at_open = self.dtr

        with patch.dict(sys.modules, {"serial": SimpleNamespace(Serial=SerialStub)}):
            port = open_serial("COM8")
        self.assertEqual(port.baudrate, 115200)
        self.assertEqual(port.port, "COM8")
        self.assertTrue(port.dtr_at_open)

    def test_display_write_timeout_does_not_stop_recording(self):
        class TimeoutPort(FakePort):
            def write(self, _data):
                raise TimeoutError("Write timeout")

        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=TimeoutPort()) as recorder:
                recorder._display_update(recorder.start_time + 1.1)
                recorder.feed(encode_frame(Frame(1, 0, 1_000_000, 1,
                    struct.pack("<HH", 8192, 9000))))
                self.assertEqual(recorder.stats()["display_write_failures"], 1)
                self.assertEqual(recorder.ecg_samples, 1)
                session = recorder.session_dir
            with (session / "vitals.csv").open(newline="") as file:
                self.assertEqual(len(list(csv.DictReader(file))), 1)

    def test_optical_stream_is_continuous_and_fills_window(self):
        # At the real 100 Hz optical rate the plot line must be continuous
        # (no interior NaN breaks) and the five-second window must retain the
        # whole run, not just the most recent sample.
        from SerialAnalysis.Live.live_plot import build_gapped_xy, CHANNEL_SPECS

        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                seq = 0
                base_us = 2_000_000
                now = 200.0
                # 2 seconds of optical samples, 10 ms apart, batched 4 per frame.
                sample = 0
                total = 200
                while sample < total:
                    payload = b""
                    for _ in range(4):
                        t_us = base_us + sample * 10_000
                        payload += struct.pack("<III", t_us, 90_000 + sample,
                                               100_000 + sample)
                        sample += 1
                    first_us = base_us + (sample - 4) * 10_000
                    recorder.feed(encode_frame(Frame(3, seq & 0xFFFF, first_us, 4, payload)),
                                  received_at=now)
                    seq += 1
                    now += 0.04
                snap = recorder.snapshot(now_s=now)
                red = snap["red"]
                # A five-second window at 100 Hz holds far more than one point.
                self.assertGreater(len(red), 100)

                _, expected_dt, gap_factor = CHANNEL_SPECS["red"]
                x, y = build_gapped_xy(red, now_s=now, window_s=5.0,
                                       expected_dt_s=expected_dt, gap_factor=gap_factor)
                self.assertGreater(x.size, 100)
                self.assertFalse(np.isnan(y).any(),
                                 "optical line should have no interior gap breaks")
        # The per-second display update must not hold the serial IO lock while
        # flushing CSV files; otherwise the read path in poll() stalls ~1 Hz.
        import threading

        class LockSpy:
            def __init__(self):
                self._lock = threading.Lock()
                self.held = False

            def __enter__(self):
                self._lock.acquire()
                self.held = True
                return self

            def __exit__(self, *exc):
                self.held = False
                self._lock.release()

        flushed_while_locked = []

        class FlushSpyPort(FakePort):
            pass

        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FlushSpyPort()) as recorder:
                spy = LockSpy()
                recorder._io_lock = spy
                real_files = recorder._files

                class SpyFile:
                    def __init__(self, inner):
                        self._inner = inner

                    def flush(self):
                        flushed_while_locked.append(spy.held)
                        return self._inner.flush()

                    def __getattr__(self, name):
                        return getattr(self._inner, name)

                recorder._files = [SpyFile(f) for f in real_files]
                recorder._display_update(recorder.start_time + 1.1)
                recorder._files = real_files

            self.assertTrue(flushed_while_locked)
            self.assertFalse(any(flushed_while_locked),
                             "CSV flush must not run while the IO lock is held")

    def test_csv_timing_window_and_display_packet(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakePort()
            with LiveRecorder("FAKE", directory, port=port) as recorder:
                recorder.feed(encode_frame(Frame(1, 0, 1_000_000, 2,
                    struct.pack("<HHHH", 8192, 9000, 8200, 9010))), received_at=100)
                recorder.feed(encode_frame(Frame(2, 1, 1_020_000, 1,
                    struct.pack("<hhh", 0, 0, 16384))), received_at=100.1)
                recorder.feed(encode_frame(Frame(3, 3, 1_040_000, 1,
                    struct.pack("<III", 1_040_000, 10000, 12000))), received_at=100.2)
                # The board uses one sequence across all frame types.
                self.assertEqual(recorder.gaps, 1)
                self.assertEqual(len(recorder.snapshot(now_s=100.2)["diff_raw"]), 2)
                self.assertEqual(len(recorder.snapshot(now_s=100.2)["red"]), 1)
                recorder.rep_count = 123
                recorder._display_update(recorder.start_time + 1.1)
                packet = Decoder().feed(port.writes[0])[0]
                self.assertEqual(packet.type, 0x80)
                self.assertEqual(len(packet.payload), 9)
                self.assertEqual(struct.unpack("<HIBH", packet.payload)[3], 123)
                recorder.feed(encode_frame(Frame(1, 4, 7_000_000, 1,
                    struct.pack("<HH", 8192, 9000))), received_at=106.1)
                self.assertEqual(len(recorder.snapshot(now_s=106.1)["red"]), 0)
                self.assertEqual(len(recorder.snapshot(now_s=106.1)["diff_raw"]), 1)
                session = recorder.session_dir
            with (session / "ecg.csv").open(newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertEqual([int(row["board_us"]) for row in rows],
                             [1_000_000, 1_005_000, 7_000_000])
            with (session / "acceleration.csv").open(newline="") as file:
                accel = list(csv.DictReader(file))
            self.assertAlmostEqual(float(accel[0]["magnitude_m_s2"]), 9.81)
            metadata = json.loads((session / "metadata.json").read_text())
            self.assertEqual(metadata["sequence_gaps"], 1)
            self.assertEqual(metadata["frames"], 4)

    def test_interleaved_sequences_and_sample_gaps(self):
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                ecg = struct.pack("<HH", 8192, 9000)
                accel = struct.pack("<hhh", 0, 0, 16384)
                recorder.feed(encode_frame(Frame(1, 65534, 1000000, 1, ecg)), received_at=100)
                recorder.feed(encode_frame(Frame(2, 65535, 1000000, 1, accel)), received_at=100)
                recorder.feed(encode_frame(Frame(1, 0, 1005000, 1, ecg)), received_at=100.005)
                self.assertEqual(recorder.gaps, 0)
                recorder.feed(encode_frame(Frame(1, 2, 1020000, 1, ecg)), received_at=100.020)
                self.assertEqual(recorder.gaps, 1)
                self.assertEqual(recorder.estimated_missing_samples[1], 2)
                self.assertEqual(recorder.timestamp_discontinuities[1], 1)

    def test_optical_batches_and_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                for seq, timestamps in enumerate(([0xffffd8f0, 0xfffffff0], [0x2700, 0x4e10])):
                    payload = b"".join(struct.pack("<III", t, 90000, 100000) for t in timestamps)
                    recorder.feed(encode_frame(Frame(3, seq, timestamps[0], 2, payload)), received_at=100 + seq * .02)
                self.assertEqual(recorder.timestamp_discontinuities[3], 0)
                self.assertEqual(len(recorder.snapshot(now_s=100.04)["red"]), 4)
                diag = struct.pack("<10I", *range(10))
                recorder.feed(encode_frame(Frame(4, 2, 0, 1, diag)), received_at=100.04)
                self.assertEqual(recorder.board_diagnostics["i2c_errors"], 2)
                self.assertEqual(recorder.gaps, 0)

    def test_plot_breaks_for_one_missing_sample(self):
        from SerialAnalysis.Live.live_plot import build_gapped_xy, CHANNEL_SPECS
        _, expected, factor = CHANNEL_SPECS["diff_raw"]
        _, y = build_gapped_xy([(100.0, 1), (100.005, 2), (100.015, 3)],
                               now_s=100.02, expected_dt_s=expected, gap_factor=factor)
        self.assertEqual(int(np.isnan(y).sum()), 1)

    def test_rep_markers_follow_acceleration_timestamps_and_expire(self):
        from matplotlib import pyplot as plt
        from SerialAnalysis.Live.live_plot import LivePlot

        plt.switch_backend("Agg")
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort(), window_s=5) as recorder:
                with patch.object(recorder.reps, "run", side_effect=[0, 1, 1]):
                    payload = struct.pack("<hhhhhhhhh", 0, 0, 16384, 0, 0, 16384,
                                          0, 0, 16384)
                    recorder.feed(encode_frame(Frame(2, 0, 1_000_000, 3, payload)),
                                  received_at=100.0)
                self.assertEqual(recorder.rep_events_snapshot(now_s=100.1), [100.02])
                plot = LivePlot(channels=["diff_filtered", "wilson_filtered",
                                          "accel_magnitude", "red", "infrared"],
                                window_s=1)
                try:
                    plot.update(recorder, now_s=100.1)
                    accel = plot.axes["accel_magnitude"].get_subplotspec()
                    red = plot.axes["red"].get_subplotspec()
                    infrared = plot.axes["infrared"].get_subplotspec()
                    self.assertEqual((accel.colspan.start, accel.colspan.stop), (0, 2))
                    self.assertEqual(red.rowspan.start, infrared.rowspan.start)
                    self.assertEqual(len(plot.rep_markers.get_segments()), 1)
                    self.assertAlmostEqual(plot.rep_markers.get_segments()[0][0, 0], -0.08)
                    plot.update(recorder, now_s=101.1)
                    self.assertEqual(len(plot.rep_markers.get_segments()), 0)
                    self.assertEqual(recorder.rep_events_snapshot(now_s=106), [])
                finally:
                    plt.close(plot.figure)

    def test_signal_groups_use_independent_windows(self):
        from matplotlib import pyplot as plt
        from SerialAnalysis.Live.live_plot import LivePlot

        plt.switch_backend("Agg")
        channels = ["red", "infrared", "accel_magnitude", "diff_filtered",
                    "wilson_filtered", "diff_raw", "wilson_raw"]
        ecg = channels[3:]
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort(), window_s=5) as recorder:
                recorder.rep_events.append(96.5)
                plot = LivePlot(channels=channels, window_s=5,
                                channel_windows={name: 1 for name in ecg})
                try:
                    plot.update(recorder, now_s=100)
                    for name in channels[:3]:
                        self.assertEqual(plot.axes[name].get_xlim(), (-5, 0))
                    for name in ecg:
                        self.assertEqual(plot.axes[name].get_xlim(), (-1, 0))
                    self.assertEqual(plot.rep_markers.get_segments()[0][0, 0], -3.5)
                    accel = plot.axes["accel_magnitude"].get_subplotspec()
                    self.assertEqual((accel.colspan.start, accel.colspan.stop), (0, 2))
                    self.assertLess(plot.axes["red"].get_subplotspec().rowspan.start,
                                    plot.axes["diff_filtered"].get_subplotspec().rowspan.start)
                finally:
                    plt.close(plot.figure)

    def test_timestamp_rollover_is_unwrapped(self):
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                recorder.feed(encode_frame(Frame(1, 65535, 0xFFFFF000, 1,
                    struct.pack("<HH", 8192, 9000))))
                recorder.feed(encode_frame(Frame(1, 0, 0x00001000, 1,
                    struct.pack("<HH", 8192, 9000))))
                self.assertEqual(recorder.gaps, 0)
                session = recorder.session_dir
            with (session / "ecg.csv").open(newline="") as file:
                times = [int(row["board_us"]) for row in csv.DictReader(file)]
            self.assertEqual(times[1] - times[0], 0x2000)

    def test_skewed_sensor_clocks_keep_five_second_plot_windows(self):
        # ECG and acceleration are reported on independent board clocks here
        # (1 MHz-ish vs 10 MHz-ish base). A per-type board->host mapping must
        # keep each channel's samples inside the rolling five-second window.
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                recorder.feed(encode_frame(Frame(1, 0, 1_000_000, 1,
                    struct.pack("<HH", 8192, 9000))), received_at=100)
                recorder.feed(encode_frame(Frame(2, 1, 10_000_000, 1,
                    struct.pack("<hhh", 0, 0, 16384))), received_at=100.5)
                recorder.feed(encode_frame(Frame(1, 2, 1_500_000, 1,
                    struct.pack("<HH", 8200, 9010))), received_at=101)
                recorder.feed(encode_frame(Frame(2, 3, 12_000_000, 1,
                    struct.pack("<hhh", 0, 0, 16384))), received_at=101.5)
                snapshot = recorder.snapshot(now_s=102)
                self.assertEqual(len(snapshot["diff_raw"]), 2)
                self.assertEqual(len(snapshot["accel_magnitude"]), 2)
                # Each channel's newest sample is mapped near its latest host
                # arrival time, regardless of the raw board clock scale.
                self.assertAlmostEqual(snapshot["diff_raw"][-1][0], 100.5)
                self.assertAlmostEqual(snapshot["accel_magnitude"][-1][0], 101.5)

                try:
                    from matplotlib import pyplot as plt
                    from SerialAnalysis.Live.live_plot import LivePlot
                except ImportError:
                    self.skipTest("matplotlib is not installed")
                plt.switch_backend("Agg")
                plot = LivePlot()
                try:
                    plot.update(recorder, now_s=102)
                    first_ax = plot.axes[plot.channels[0]]
                    self.assertEqual(tuple(first_ax.get_xlim()), (-5.0, 0.0))
                    # The real sample x-values (where y is finite) are the
                    # board-derived times expressed relative to now_s=102.
                    # Samples 0.5 s apart exceed the gap threshold, so a NaN
                    # break separates them; ignore those break points here.
                    def real_x(channel: str) -> list[float]:
                        line = plot.lines[channel]
                        xs = line.get_xdata()
                        ys = line.get_ydata()
                        return [float(x) for x, y in zip(xs, ys) if np.isfinite(y)]

                    self.assertEqual(real_x("diff_raw"), [-2.0, -1.5])
                    self.assertEqual(real_x("accel_magnitude"), [-1.5, -0.5])
                    self.assertEqual(len(recorder.snapshot(now_s=106.6)["diff_raw"]), 0)
                    self.assertEqual(len(recorder.snapshot(now_s=106.6)["accel_magnitude"]), 0)
                finally:
                    plt.close(plot.figure)

    def test_batched_frames_get_board_derived_plot_times(self):
        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                first = encode_frame(Frame(1, 0, 1_000_000, 2,
                    struct.pack("<HHHH", 8192, 9000, 8200, 9010)))
                second = encode_frame(Frame(1, 1, 1_010_000, 2,
                    struct.pack("<HHHH", 8210, 9020, 8220, 9030)))
                third = encode_frame(Frame(1, 2, 1_020_000, 2,
                    struct.pack("<HHHH", 8230, 9040, 8240, 9050)))
                recorder.feed(first + second, received_at=100)
                recorder.feed(third, received_at=100.001)
                times = [t for t, _ in recorder.snapshot(now_s=100.030)["diff_raw"]]
                self.assertEqual(len(times), 6)
                # Plot times follow the board clock: anchored so the first
                # sample lands at the first host arrival, then spaced by the
                # real 5 ms board period, not squashed into a ~2 ms sliver.
                self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
                for a, b in zip(times, times[1:]):
                    self.assertAlmostEqual(b - a, 0.005)
                self.assertAlmostEqual(times[0], 100.0)
                self.assertAlmostEqual(times[-1], 100.025)

    def test_replayed_stream_is_continuous_except_at_injected_gap(self):
        # Replay several seconds of ECG+accel+optical on board clocks with one
        # deliberately dropped second of ECG. The ECG plot must be continuous
        # everywhere except a single break at the injected gap.
        from SerialAnalysis.Live.live_plot import build_gapped_xy, CHANNEL_SPECS

        with tempfile.TemporaryDirectory() as directory:
            with LiveRecorder("FAKE", directory, port=FakePort()) as recorder:
                ecg_seq = accel_seq = opt_seq = 0
                ecg_us = accel_us = opt_us = 0
                now = 300.0
                # 4 seconds of data; ECG at 200 Hz (8/frame), gap during [2s,3s).
                for frame_idx in range(200):  # 200 * 20 ms = 4 s of wall time
                    block_start_s = frame_idx * 0.02
                    in_gap = 2.0 <= block_start_s < 3.0

                    if not in_gap:
                        payload = b""
                        for _ in range(4):
                            payload += struct.pack("<HH", 8192, 9000)
                            ecg_us += 5000
                        recorder.feed(encode_frame(Frame(1, ecg_seq & 0xFFFF,
                            ecg_us - 4 * 5000, 4, payload)), received_at=now)
                        ecg_seq += 1
                    else:
                        # Keep the board clock advancing through the gap so the
                        # resumed samples land at their true (later) time.
                        ecg_us += 4 * 5000

                    # Accel every other frame (50 Hz, 1/frame here).
                    if frame_idx % 1 == 0:
                        recorder.feed(encode_frame(Frame(2, accel_seq & 0xFFFF,
                            accel_us, 1, struct.pack("<hhh", 0, 0, 16384))),
                            received_at=now)
                        accel_seq += 1
                        accel_us += 20000

                    now += 0.02

                final_now = now
                snap = recorder.snapshot(now_s=final_now)
                # The five-second window ends at 4 s of data; the gap sat at
                # [2s,3s), i.e. 1-2 s before the end, so it is inside the window.
                _, expected_dt, gap_factor = CHANNEL_SPECS["diff_raw"]
                x, y = build_gapped_xy(snap["diff_raw"], now_s=final_now, window_s=5.0,
                                       expected_dt_s=expected_dt, gap_factor=gap_factor)
                nan_breaks = int(np.isnan(y).sum())
                self.assertEqual(nan_breaks, 1,
                                 "exactly one break expected, at the injected gap")
                # Data exists on both sides of the break.
                nan_index = int(np.where(np.isnan(y))[0][0])
                self.assertGreater(nan_index, 0)
                self.assertLess(nan_index, y.size - 1)


class RepCounterTests(unittest.TestCase):
    @staticmethod
    def _rep(rc, t, polarity, amp_g):
        """Feed one slow push-and-return along x, 20 ms per sample, 1 g on z."""
        push = polarity * amp_g
        n = 20  # 400 ms outward, 400 ms return: both exceed MIN_PHASE_MS (220)
        for i in range(n):
            rc.run(int(push * 16384 * (i + 1) / n), 0, 16384, t); t += 20
        for _ in range(10):
            rc.run(int(push * 16384), 0, 16384, t); t += 20
        for i in range(n):
            frac = (i + 1) / n
            rc.run(int(push * 16384 * (1 - 2 * frac)), 0, 16384, t); t += 20
        for _ in range(25):  # settle back to rest under threshold*0.35
            rc.run(0, 0, 16384, t); t += 20
        return t

    def test_sustained_motion_counts_reps(self):
        rc = RepCounter()
        t = 0
        for _ in range(60):  # warm up filters at rest
            rc.run(0, 0, 16384, t); t += 20
        before = rc.rep_count
        for _ in range(5):
            t = self._rep(rc, t, 1, 0.5)
        self.assertGreater(rc.rep_count, before,
                           "sustained push-and-return reps should be counted")

    def test_quick_twitch_is_not_counted(self):
        rc = RepCounter()
        t = 0
        for _ in range(60):
            rc.run(0, 0, 16384, t); t += 20
        # A single 60 ms spike is shorter than MIN_PHASE_MS and must be ignored.
        for _ in range(3):
            rc.run(int(0.6 * 16384), 0, 16384, t); t += 20
        for _ in range(40):
            rc.run(0, 0, 16384, t); t += 20
        self.assertEqual(rc.rep_count, 0)


if __name__ == "__main__":
    unittest.main()
