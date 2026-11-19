"""Offline protocol and recorder checks; no serial hardware required."""

import csv
import json
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from SerialAnalysis.Live.protocol import Decoder, Frame, crc16, encode_frame
from SerialAnalysis.Live.recorder import LiveRecorder, open_serial


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
                self.assertEqual(recorder.gaps, 1)
                self.assertEqual(len(recorder.snapshot(now_s=100.2)["diff_raw"]), 2)
                self.assertEqual(len(recorder.snapshot(now_s=100.2)["red"]), 1)
                recorder._display_update(recorder.start_time + 1.1)
                packet = Decoder().feed(port.writes[0])[0]
                self.assertEqual(packet.type, 0x80)
                self.assertEqual(len(packet.payload), 7)
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

                try:
                    from matplotlib import pyplot as plt
                    from SerialAnalysis.Live.live_plot import LivePlot
                except ImportError:
                    self.skipTest("matplotlib is not installed")
                plt.switch_backend("Agg")
                plot = LivePlot()
                try:
                    plot.update(recorder, now_s=102)
                    self.assertEqual(tuple(plot.axes[0].get_xlim()), (-5.0, 0.0))
                    self.assertEqual(list(plot.lines["diff_raw"].get_xdata()),
                                     [-2.0, -1.0])
                    self.assertEqual(list(plot.lines["accel_magnitude"].get_xdata()),
                                     [-1.5, -0.5])
                    self.assertEqual(len(recorder.snapshot(now_s=106.6)["diff_raw"]), 0)
                    self.assertEqual(len(recorder.snapshot(now_s=106.6)["accel_magnitude"]), 0)
                finally:
                    plt.close(plot.figure)

    def test_batched_frames_get_ordered_host_plot_times(self):
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
                times = [t for t, _ in recorder.snapshot(now_s=100.001)["diff_raw"]]
                self.assertEqual(len(times), 6)
                self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
                self.assertAlmostEqual(times[-1], 100.001)


if __name__ == "__main__":
    unittest.main()
