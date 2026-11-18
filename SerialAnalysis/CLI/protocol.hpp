#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

static uint16_t u16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
static uint32_t u32(const uint8_t* p) { return uint32_t(u16(p)) | (uint32_t(u16(p + 2)) << 16); }
static void w16(uint8_t* p, uint16_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }
static void w32(uint8_t* p, uint32_t v) { w16(p, uint16_t(v)); w16(p + 2, uint16_t(v >> 16)); }
static uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t c = 0xffff;
  for (size_t i = 0; i < n; ++i) { c ^= uint16_t(p[i]) << 8;
    for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? uint16_t((c << 1) ^ 0x1021) : uint16_t(c << 1);
  } return c;
}
struct Frame { uint8_t type, count; uint16_t seq; uint32_t t; std::vector<uint8_t> data; };
class Decoder {
  std::vector<uint8_t> bytes;
public:
  uint64_t bad = 0;
  template<class F> void feed(const uint8_t* p, size_t n, F emit) {
    bytes.insert(bytes.end(), p, p + n);
    while (bytes.size() >= 14) {
      if (bytes[0] != 0xa5 || bytes[1] != 0x5a) { bytes.erase(bytes.begin()); continue; }
      auto len = u16(bytes.data() + 10);
      if (len > 120) { ++bad; bytes.erase(bytes.begin()); continue; }
      if (bytes.size() < size_t(14 + len)) break;
      if (crc16(bytes.data() + 2, 10 + len) != u16(bytes.data() + 12 + len)) {
        ++bad; bytes.erase(bytes.begin()); continue;
      }
      Frame f{bytes[2], bytes[9], u16(bytes.data() + 3), u32(bytes.data() + 5),
              std::vector<uint8_t>(bytes.begin() + 12, bytes.begin() + 12 + len)};
      emit(f); bytes.erase(bytes.begin(), bytes.begin() + 14 + len);
    }
  }
};
