#include "acceleration_analysis.hpp"
#include "ecg_filter.hpp"
#include "protocol.hpp"
#include "pulse_analysis.hpp"
#include "serial_port.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

static_assert(EcgFilter::FS * double(kEcgPeriodUs) == 1e6,
              "EcgFilter::FS must match kEcgPeriodUs in protocol.hpp");

static std::atomic<bool> stopping{false};
static void stopSignal(int) { stopping = true; }

static const char* const kUsage =
    "Usage: serial-analysis --port COM3 [--output sessions] [--duration-min 30]\n"
    "       serial-analysis --self-test\n";

static std::string isoUtc() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  std::ostringstream s;
  s << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ");
  return s.str();
}

static std::string jsonEscape(const std::string& in) {
  std::string out;
  for (const char c : in) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", unsigned(static_cast<unsigned char>(c)));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

// Each frame type has its own board clock phase, so each needs its own wrap
// tracking. Sharing one clock made an optical timestamp a few milliseconds
// older than the last ECG frame look like a clock restart (and reset the ECG filters).
struct StreamClock {
  bool have = false;
  uint32_t prev = 0;
  uint64_t epoch = 0;

  // Returns the unwrapped time. `restarted` is set when the clock stepped
  // backwards by less than half a wrap (a board reset, not a 32-bit rollover).
  uint64_t unwrap(uint32_t t, bool& restarted) {
    restarted = false;
    if (have && t < prev) {
      if (prev - t > 0x80000000u)
        epoch += 1ull << 32;
      else
        restarted = true;
    }
    prev = t;
    have = true;
    return epoch + t;
  }
};

// Detects missing or repeated samples from the nominal period of a stream.
struct SampleTrack {
  bool have = false;
  uint64_t last = 0;
  uint64_t discontinuities = 0;
  uint64_t missing = 0;

  bool next(uint64_t t, uint32_t periodUs) {
    bool broken = false;
    if (have) {
      const int64_t delta = int64_t(t) - int64_t(last);
      const bool late = delta * 2 > int64_t(periodUs) * 3;
      if (delta <= 0 || late) {
        broken = true;
        ++discontinuities;
        if (late) missing += uint64_t(std::max<int64_t>(0, std::llround(double(delta) / periodUs) - 1));
      }
    }
    last = t;
    have = true;
    return broken;
  }
};

struct Diagnostic {
  std::string name;
  uint32_t value;
};

static int selfTest() {
  uint8_t frame[18]{};
  frame[0] = 0xa5;
  frame[1] = 0x5a;
  frame[2] = 1;
  w16(frame + 3, 42);
  w32(frame + 5, 123456);
  frame[9] = 1;
  w16(frame + 10, 4);
  w16(frame + 12, 8192);
  w16(frame + 14, 9000);
  w16(frame + 16, crc16(frame + 2, 14));
  Decoder d;
  int accepted = 0;
  d.feed(frame, 5, [&](const Frame&) { ++accepted; });
  d.feed(frame + 5, 13, [&](const Frame& f) {
    if (f.seq == 42 && f.t == 123456 && f.data.size() == 4) ++accepted;
  });
  uint8_t damaged[18];
  std::copy(frame, frame + 18, damaged);
  damaged[17] ^= 1;
  d.feed(damaged, 18, [&](const Frame&) { ++accepted; });
  std::vector<uint8_t> noisy(2000, 0xa5);  // worst case: every byte looks like a marker start
  noisy.insert(noisy.end(), frame, frame + 18);
  d.feed(noisy.data(), noisy.size(), [&](const Frame& f) {
    if (f.seq == 42) ++accepted;
  });
  if (accepted != 2 || d.bad != 1) throw std::runtime_error("self-test failed: frame decoding");

  StreamClock clock;
  bool restarted = false;
  clock.unwrap(0xfffffff0u, restarted);
  const uint64_t wrapped = clock.unwrap(0x10u, restarted);
  if (restarted || wrapped != (1ull << 32) + 0x10u) throw std::runtime_error("self-test failed: rollover");
  clock.unwrap(0x00001000u, restarted);
  clock.unwrap(0x00000100u, restarted);
  if (!restarted) throw std::runtime_error("self-test failed: restart");

  SampleTrack track;
  track.next(1000000, kEcgPeriodUs);
  track.next(1000000 + kEcgPeriodUs, kEcgPeriodUs);
  if (!track.next(1000000 + 4 * kEcgPeriodUs, kEcgPeriodUs) || track.missing != 2)
    throw std::runtime_error("self-test failed: sample tracking");

  EcgFilter e;
  e.updateMotion(0.05);
  double result = 0;
  for (int i = 0; i < 2000; ++i) result = e.run(9000);
  if (!std::isfinite(result) || std::abs(result) > 1.0) throw std::runtime_error("self-test failed: ecg filter");

  // Synthetic 75 BPM optical signal, as BoardDummy emits.
  Pulse pulse;
  for (int i = 0; i < 600; ++i) {
    const double ms = i * 10.0;
    const double dist = std::abs(std::fmod(ms, 800.0) - 190.0);
    const double beat = dist < 145.0 ? 5000.0 * (145.0 - dist) / 145.0 : 0.0;
    pulse.add(ms / 1000.0, uint32_t(90000 + beat * 0.54), uint32_t(100000 + beat));
  }
  pulse.calculate();
  if (!pulse.validHr || std::abs(pulse.bpm - 75) > 2 || !pulse.validSpo2 || std::abs(pulse.spo2 - 95) > 2)
    throw std::runtime_error("self-test failed: pulse analysis");
  std::cout << "self-test passed\n";
  return 0;
}

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << kUsage;
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--self-test") return selfTest();

    std::string portName, output = "sessions";
    double duration = 30;
    for (int i = 1; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--port" && i + 1 < argc) {
        portName = argv[++i];
      } else if (a == "--output" && i + 1 < argc) {
        output = argv[++i];
      } else if (a == "--duration-min" && i + 1 < argc) {
        const std::string text = argv[++i];
        std::size_t used = 0;
        try {
          duration = std::stod(text, &used);
        } catch (const std::exception&) {
          used = 0;
        }
        if (used != text.size() || !std::isfinite(duration))
          throw std::runtime_error(std::string("Invalid --duration-min '") + text + "'.\n" + kUsage);
      } else {
        throw std::runtime_error(kUsage);
      }
    }
    if (portName.empty()) throw std::runtime_error(std::string("Missing --port.\n") + kUsage);
    if (duration <= 0) throw std::runtime_error(std::string("--duration-min must be positive.\n") + kUsage);

    Port port(portName);  // open first so a bad port leaves no empty session folder

    const std::string startedUtc = isoUtc();
    fs::create_directories(output);
    std::string folder = startedUtc;
    std::replace(folder.begin(), folder.end(), ':', '-');
    fs::path dir = fs::path(output) / folder;
    for (int suffix = 1; fs::exists(dir); ++suffix) dir = fs::path(output) / (folder + "-" + std::to_string(suffix));
    fs::create_directories(dir);

    std::ofstream ecg(dir / "ecg.csv"), accel(dir / "acceleration.csv"), vitals(dir / "vitals.csv");
    if (!ecg || !accel || !vitals) throw std::runtime_error("cannot create session files in " + dir.string());
    ecg << std::setprecision(9);
    accel << std::setprecision(9);
    ecg << "board_us,diff_raw,wilson_raw,diff_filtered,wilson_filtered\n";
    accel << "board_us,magnitude_m_s2,smoothed_m_s2\n";
    vitals << "board_us,heart_rate_bpm,heart_rate_valid,spo2_percent,spo2_valid\n";

    Decoder decoder;
    EcgFilter diff, wilson;
    Pulse pulse;
    AccelerationAnalysis motion;
    StreamClock clocks[4];
    SampleTrack tracks[4];
    std::vector<Diagnostic> diagnostics;

    const auto start = Clock::now();
    auto lastDisplay = start, lastStatus = start;
    const std::chrono::duration<double> limit(duration * 60.0);
    bool haveSeq = false;
    uint16_t prevSeq = 0;
    uint64_t gaps = 0, frames = 0, ecgSamples = 0, accelSamples = 0, opticalSamples = 0;
    uint64_t displayWriteFailures = 0, latestBoardUs = 0;
    std::string error;

    auto resetEcg = [&] {
      diff.reset();
      wilson.reset();
    };

    auto handleFrame = [&](const Frame& f) {
      if (haveSeq) {
        // Ignore duplicates and backward jumps instead of counting ~65535 losses.
        const uint16_t missed = uint16_t(f.seq - prevSeq - 1);
        if (missed < 0x8000) gaps += missed;
      }
      prevSeq = f.seq;
      haveSeq = true;
      ++frames;

      if (f.type == 4) {  // cumulative board counters, once per second
        if (f.count == 1 && (f.data.size() == 40 || f.data.size() == 44 || f.data.size() == 48)) {
          static const char* const names[] = {"missed_ecg_deadlines", "missed_accel_deadlines", "i2c_errors",
                                              "optical_fifo_overflows", "tx_dropped_ecg", "tx_dropped_accel",
                                              "tx_dropped_optical", "max_loop_us", "max_optical_us", "optical_ready",
                                              "optical_fifo_state", "optical_overflow_counter_reliable"};
          diagnostics.clear();
          for (size_t i = 0; i < f.data.size() / 4; ++i)
            diagnostics.push_back({names[i], u32(f.data.data() + 4 * i)});
        }
        return;
      }

      const int width = f.type == 1 ? 4 : f.type == 2 ? 6 : f.type == 3 ? 12 : 0;
      if (!width || f.count == 0 || f.data.size() != size_t(f.count) * size_t(width)) return;

      bool restarted = false;
      const uint64_t t0 = f.type == 3 ? 0 : clocks[f.type].unwrap(f.t, restarted);
      if (restarted && f.type == 1) resetEcg();
      if (f.type != 3) latestBoardUs = std::max(latestBoardUs, t0);

      for (int i = 0; i < f.count; ++i) {
        const uint8_t* p = f.data.data() + size_t(i) * size_t(width);
        if (f.type == 1) {
          const uint64_t t = t0 + uint64_t(i) * kEcgPeriodUs;
          if (tracks[1].next(t, kEcgPeriodUs)) resetEcg();
          const double d = diff.run(u16(p)), w = wilson.run(u16(p + 2));
          ecg << t << ',' << u16(p) << ',' << u16(p + 2) << ',' << d << ',' << w << '\n';
          ++ecgSamples;
        } else if (f.type == 2) {
          const uint64_t t = t0 + uint64_t(i) * kAccelPeriodUs;
          tracks[2].next(t, kAccelPeriodUs);
          const auto result = motion.run(int16_t(u16(p)), int16_t(u16(p + 2)), int16_t(u16(p + 4)));
          // Dynamic acceleration (m/s^2) drives both ECG NLMS motion cancellers.
          const double dynamicMotion = result.magnitude - result.smoothed;
          diff.updateMotion(dynamicMotion);
          wilson.updateMotion(dynamicMotion);
          accel << t << ',' << result.magnitude << ',' << result.smoothed << '\n';
          ++accelSamples;
        } else {
          bool opticalRestart = false;
          const uint64_t t = clocks[3].unwrap(u32(p), opticalRestart);
          if (opticalRestart) pulse.data.clear();
          if (tracks[3].next(t, kOpticalPeriodUs)) pulse.data.clear();
          latestBoardUs = std::max(latestBoardUs, t);
          pulse.add(double(t) / 1e6, u32(p + 4), u32(p + 8));
          ++opticalSamples;
        }
      }
    };

    std::cout << "Recording to " << dir << "\n";
    try {
      while (!stopping && Clock::now() - start < limit) {
        uint8_t buf[1024];
        const int n = port.read(buf, sizeof(buf));
        decoder.feed(buf, size_t(std::max(n, 0)), handleFrame);

        const auto now = Clock::now();
        if (now - lastDisplay >= std::chrono::seconds(1)) {
          pulse.calculate();
          const auto left = std::chrono::duration_cast<std::chrono::seconds>(limit - (now - start)).count();
          const uint32_t remaining = uint32_t(std::max<int64_t>(0, left));
          uint8_t packet[21]{};
          packet[0] = 0xa5;
          packet[1] = 0x5a;
          packet[2] = 0x80;
          w32(packet + 5, 0);
          packet[9] = 1;
          w16(packet + 10, 7);
          w16(packet + 12, uint16_t(pulse.bpm));
          w32(packet + 14, remaining);
          packet[18] = pulse.validHr ? 1 : 0;
          w16(packet + 19, crc16(packet + 2, 17));
          try {
            port.write(packet, sizeof(packet));
          } catch (const std::exception&) {
            ++displayWriteFailures;  // the OLED is optional; keep recording
          }
          vitals << latestBoardUs << ',' << pulse.bpm << ',' << pulse.validHr << ',' << pulse.spo2 << ','
                 << pulse.validSpo2 << '\n';
          ecg.flush();
          accel.flush();
          vitals.flush();
          lastDisplay = now;
        }
        if (now - lastStatus >= std::chrono::seconds(10)) {
          std::cout << "frames=" << frames << " gaps=" << gaps << " bad_crc=" << decoder.bad << " ECG=" << ecgSamples
                    << " accel=" << accelSamples << " optical=" << opticalSamples << '\n';
          lastStatus = now;
        }
      }
    } catch (const std::exception& e) {
      error = e.what();  // still close the session cleanly below
    }

    ecg.close();
    accel.close();
    vitals.close();
    std::ofstream meta(dir / "metadata.json");
    meta << "{\n  \"protocol_version\": 1,\n  \"started_utc\": \"" << startedUtc << "\",\n  \"port\": \""
         << jsonEscape(portName) << "\",\n  \"duration_minutes\": " << duration << ",\n  \"frames\": " << frames
         << ",\n  \"sequence_gaps\": " << gaps << ",\n  \"bad_frames\": " << decoder.bad
         << ",\n  \"display_write_failures\": " << displayWriteFailures << ",\n  \"ecg_samples\": " << ecgSamples
         << ",\n  \"accel_samples\": " << accelSamples << ",\n  \"optical_samples\": " << opticalSamples
         << ",\n  \"timestamp_discontinuities\": {\"1\": " << tracks[1].discontinuities
         << ", \"2\": " << tracks[2].discontinuities << ", \"3\": " << tracks[3].discontinuities
         << "},\n  \"estimated_missing_samples\": {\"1\": " << tracks[1].missing << ", \"2\": " << tracks[2].missing
         << ", \"3\": " << tracks[3].missing << "},\n  \"board_diagnostics\": {";
    for (size_t i = 0; i < diagnostics.size(); ++i)
      meta << (i ? ", " : "") << '"' << diagnostics[i].name << "\": " << diagnostics[i].value;
    meta << "}";
    if (!error.empty()) meta << ",\n  \"error\": \"" << jsonEscape(error) << "\"";
    meta << "\n}\n";
    std::cout << "Finished: " << dir << '\n';
    if (!error.empty()) {
      std::cerr << error << '\n';
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
