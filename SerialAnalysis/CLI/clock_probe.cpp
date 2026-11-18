#include "protocol.hpp"
#include "serial_port.hpp"
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: clock_probe COM8 seconds\n";
    return 2;
  }
  try {
    const auto duration = std::chrono::seconds(std::stoi(argv[2]));
    Port port(argv[1]);
    Decoder decoder;
    const auto start = std::chrono::steady_clock::now();
    uint32_t lastTime[4]{};
    bool seenTime[4]{};
    uint16_t lastSequence = 0;
    bool seenSequence = false;
    uint64_t frames = 0;
    while (std::chrono::steady_clock::now() - start < duration) {
      uint8_t buffer[1024];
      int count = port.read(buffer, sizeof(buffer));
      decoder.feed(buffer, count, [&](const Frame& frame) {
        const auto hostMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        const bool backward = frame.type < 4 && seenTime[frame.type] &&
                              frame.t + 1000000 < lastTime[frame.type];
        const bool seqReset = seenSequence && frame.seq < lastSequence &&
                              lastSequence - frame.seq < 32768;
        if (frames < 20 || backward || seqReset) {
          std::cout << "host_ms=" << hostMs << " seq=" << frame.seq
                    << " type=" << unsigned(frame.type) << " board_us=" << frame.t
                    << " count=" << unsigned(frame.count)
                    << (backward ? " CLOCK_BACKWARD" : "")
                    << (seqReset ? " SEQUENCE_RESET" : "") << '\n';
        }
        if (frame.type < 4) {
          lastTime[frame.type] = frame.t;
          seenTime[frame.type] = true;
        }
        lastSequence = frame.seq;
        seenSequence = true;
        ++frames;
      });
    }
    std::cout << "frames=" << frames << " bad_crc=" << decoder.bad << '\n';
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
