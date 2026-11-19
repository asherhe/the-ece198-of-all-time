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

    def test_csv_timing_window_and_display_packet(self):
        with tempfile.TemporaryDirectory() as directory:
            port = FakePort()
            with LiveRecorder("FAKE", directory, port=port) as recorder:
                recorder.feed(encode_frame(Frame(1, 0, 1_000_000, 2,
                    struct.pack("<HHHH", 8192, 9000, 8200, 9010))))
                recorder.feed(encode_frame(Frame(2, 1, 1_020_000, 1,
                    struct.pack("<hhh", 0, 0, 16384))))
                recorder.feed(encode_frame(Frame(3, 3, 1_040_000, 1,
                    struct.pack("<III", 1_040_000, 10000, 12000))))
                self.assertEqual(recorder.gaps, 1)
                self.assertEqual(len(recorder.snapshot()["diff_raw"]), 2)
                self.assertEqual(len(recorder.snapshot()["red"]), 1)
                recorder._display_update(recorder.start_time + 1.1)
                packet = Decoder().feed(port.writes[0])[0]
                self.assertEqual(packet.type, 0x80)
                self.assertEqual(len(packet.payload), 7)
                recorder.feed(encode_frame(Frame(1, 4, 7_000_000, 1,
                    struct.pack("<HH", 8192, 9000))))
                self.assertEqual(len(recorder.snapshot()["red"]), 0)
                self.assertEqual(len(recorder.snapshot()["diff_raw"]), 1)
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
                times = [t for t, _ in recorder.snapshot()["diff_raw"]]
                self.assertAlmostEqual(times[1] - times[0], 0x2000 / 1e6)


if __name__ == "__main__":
    unittest.main()
