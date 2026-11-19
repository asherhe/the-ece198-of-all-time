"""Version 1 wire format shared with SerialAnalysis/CLI/protocol.hpp."""

from dataclasses import dataclass
import struct


MARKER = b"\xa5\x5a"
MAX_PAYLOAD = 120


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
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
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 14:
            if self.buffer[:2] != MARKER:
                del self.buffer[0]
                continue
            length = struct.unpack_from("<H", self.buffer, 10)[0]
            if length > MAX_PAYLOAD:
                self.bad += 1
                del self.buffer[0]
                continue
            size = 14 + length
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
