"""Version 1 wire format shared with SerialAnalysis/CLI/protocol.hpp."""

from dataclasses import dataclass
import struct


MARKER = b"\xa5\x5a"
MAX_PAYLOAD = 120
HEADER_SIZE = 12   # marker + type + sequence + timestamp + count + payload length
FRAME_OVERHEAD = 14  # header plus CRC16

# Nominal sample periods in microseconds. These must match Board/Board.ino
# (ECG_PERIOD_US, ACCEL_PERIOD_US, OPTICAL_PERIOD_US) and the CLI recorder.
ECG_PERIOD_US = 2500       # 400 Hz
ACCEL_PERIOD_US = 20000    # 50 Hz
OPTICAL_PERIOD_US = 10000  # 100 Hz
SAMPLE_PERIOD_US = {1: ECG_PERIOD_US, 2: ACCEL_PERIOD_US, 3: OPTICAL_PERIOD_US}
ECG_RATE_HZ = 1_000_000 / ECG_PERIOD_US


def _make_crc_table() -> tuple[int, ...]:
    table = []
    for byte in range(256):
        crc = byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc16(data: bytes | bytearray | memoryview) -> int:
    """CRC16-CCITT-FALSE (initial 0xFFFF, polynomial 0x1021)."""
    crc = 0xFFFF
    for byte in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ byte]
    return crc


@dataclass(frozen=True)
class Frame:
    type: int
    sequence: int
    timestamp_us: int
    count: int
    payload: bytes


def encode_frame(frame: Frame) -> bytes:
    if len(frame.payload) > MAX_PAYLOAD:
        raise ValueError("payload exceeds 120 bytes")
    body = struct.pack("<BHIBH", frame.type, frame.sequence, frame.timestamp_us,
                       frame.count, len(frame.payload)) + frame.payload
    return MARKER + body + struct.pack("<H", crc16(body))


class Decoder:
    """Incremental frame decoder that resynchronises on the next marker."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad = 0

    def _skip_to_marker(self) -> None:
        """Drop bytes before the next possible marker, keeping a trailing
        0xA5 that may be the first half of a marker split across reads."""
        index = self.buffer.find(MARKER)
        if index < 0:
            keep = 1 if self.buffer.endswith(MARKER[:1]) else 0
            del self.buffer[:len(self.buffer) - keep]
        else:
            del self.buffer[:index]

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= FRAME_OVERHEAD:
            if self.buffer[:2] != MARKER:
                self._skip_to_marker()
                continue
            length = struct.unpack_from("<H", self.buffer, 10)[0]
            if length > MAX_PAYLOAD:
                self.bad += 1
                del self.buffer[0]
                continue
            size = FRAME_OVERHEAD + length
            if len(self.buffer) < size:
                break
            received = struct.unpack_from("<H", self.buffer, 12 + length)[0]
            if crc16(self.buffer[2:12 + length]) != received:
                self.bad += 1
                del self.buffer[0]
                continue
            kind, sequence, timestamp, count, _ = struct.unpack_from("<BHIBH", self.buffer, 2)
            frames.append(Frame(kind, sequence, timestamp, count, bytes(self.buffer[12:12 + length])))
            del self.buffer[:size]
        return frames
