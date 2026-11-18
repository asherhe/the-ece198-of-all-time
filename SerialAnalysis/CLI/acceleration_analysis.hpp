#pragma once
#include <cmath>
#include <cstdint>

struct AccelerationResult {
  double magnitude;
  double smoothed;
};

struct AccelerationAnalysis {
  double smoothed = 0;
  AccelerationResult run(int16_t rawX, int16_t rawY, int16_t rawZ) {
    constexpr double scale = 9.81 / 16384.0;
    const double x = rawX * scale, y = rawY * scale, z = rawZ * scale;
    const double magnitude = std::sqrt(x*x + y*y + z*z);
    smoothed = 0.2 * magnitude + 0.8 * smoothed;
    return {magnitude, smoothed};
  }
};
