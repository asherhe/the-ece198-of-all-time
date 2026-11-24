#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

// Nominal sample periods in microseconds. These must match Board/Board.ino and
// Live/protocol.py; Live/test_live.py checks all three against each other.
constexpr uint32_t kEcgPeriodUs = 2500;      // 400 Hz
constexpr uint32_t kAccelPeriodUs = 20000;   // 50 Hz
constexpr uint32_t kOpticalPeriodUs = 10000; // 100 Hz

constexpr size_t kMaxPayload = 120;
constexpr size_t kFrameOverhead = 14; // marker, header and CRC16

inline uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
inline uint32_t u32(const uint8_t* p) { return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16); }
inline void w16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
inline void w32(uint8_t* p, uint32_t v) { w16(p, uint16_t(v)); w16(p + 2, uint16_t(v >> 16)); }
inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t c = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    c ^= uint16_t(uint16_t(p[i]) << 8);
    for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
  }
  return c;
}

struct Frame {
  uint8_t type, count;
  uint16_t seq;
  uint32_t t;
  std::vector<uint8_t> data;
};

// Incremental decoder. Bad frames are counted and the decoder resynchronises
// on the next 0xA5 0x5A marker. Consumed bytes are tracked by an offset and
// compacted once per feed(), so a burst of garbage costs O(n), not O(n^2).
class Decoder {
  std::vector<uint8_t> bytes;
  size_t head = 0;

 public:
  uint64_t bad = 0;

  template <class F>
  void feed(const uint8_t* p, size_t n, F emit) {
    if (p != nullptr && n > 0) bytes.insert(bytes.end(), p, p + n);
    while (bytes.size() - head >= kFrameOverhead) {
      const uint8_t* b = bytes.data() + head;
      const size_t avail = bytes.size() - head;
      if (b[0] != 0xa5 || b[1] != 0x5a) {
        size_t skip = 1;
        while (skip < avail && b[skip] != 0xa5) ++skip;
        head += skip;
        continue;
      }
      const size_t len = u16(b + 10);
      if (len > kMaxPayload) {
        ++bad;
        ++head;
        continue;
      }
      if (avail < kFrameOverhead + len) break;
      if (crc16(b + 2, 10 + len) != u16(b + 12 + len)) {
        ++bad;
        ++head;
        continue;
      }
      Frame f{b[2], b[9], u16(b + 3), u32(b + 5), std::vector<uint8_t>(b + 12, b + 12 + len)};
      head += kFrameOverhead + len;
      emit(f);
    }
    if (head > 0) {
      bytes.erase(bytes.begin(), bytes.begin() + std::ptrdiff_t(head));
      head = 0;
    }
  }
};
