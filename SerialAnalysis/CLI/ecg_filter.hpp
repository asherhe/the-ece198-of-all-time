#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

struct Biquad {
  double b0, b1, b2, a1, a2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  Biquad(double B0, double B1, double B2, double A1, double A2)
      : b0(B0), b1(B1), b2(B2), a1(A1), a2(A2) {}
  double run(double x) {
    double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }
  void reset(double val = 0.0) {
    x1 = x2 = y1 = y2 = val;
  }
};

struct EcgFilter {
  // 60 Hz IIR Notch filter at 200 Hz sample rate (Q = 30)
  Biquad notch{0.9695312529087462, 0.5992023379391097, 0.9695312529087462,
               0.5992023379391097, 0.9390625058174924};
  // Two biquads, 40 Hz low-pass at 200 Hz
  Biquad a{24511874.0 / 268435456, 49023748.0 / 268435456, 24511874.0 / 268435456,
           -263375836.0 / 268435456, 92986877.0 / 268435456};
  Biquad b{40044810.0 / 268435456, 80089620.0 / 268435456, 40044810.0 / 268435456,
           -143201402.0 / 268435456, 34946300.0 / 268435456};

  std::array<double, 201> taps{}, history{};
  size_t cursor = 0;
  bool ready = false;

  double maxSlewPerSample = 250.0;
  double prevCleanRaw = 0.0;
  double dcEstimate = 0.0;

  EcgFilter(double maxAdcCounts = 1023.0) {
    maxSlewPerSample = maxAdcCounts * 0.25;
    const double pi = std::acos(-1.0), wc = 2 * pi * 0.5 / 200;
    for (int k = 0; k < 201; ++k) {
      int m = k - 100;
      taps[k] = (m == 0 ? 1 - wc / pi : -std::sin(wc * m) / (pi * m)) *
                (0.54 - 0.46 * std::cos(2 * pi * k / 200));
    }
  }

  void reset() {
    ready = false;
    cursor = 0;
  }

  double run(double raw) {
    if (!ready) {
      prevCleanRaw = raw;
      dcEstimate = raw;
      notch.reset(0.0);
      a.reset(0.0);
      b.reset(0.0);
      history.fill(0.0);
      ready = true;
    }

    // Only clamp extreme single-sample glitches (>25% full scale in 5ms)
    double delta = raw - prevCleanRaw;
    if (delta > maxSlewPerSample) {
      raw = prevCleanRaw + maxSlewPerSample;
    } else if (delta < -maxSlewPerSample) {
      raw = prevCleanRaw - maxSlewPerSample;
    }
    prevCleanRaw = raw;

    // Track and subtract DC baseline offset
    dcEstimate += (raw - dcEstimate) * 0.02;
    double centered = raw - dcEstimate;

    // 60 Hz Notch + 40 Hz Butterworth Low-Pass
    double lp = b.run(a.run(notch.run(centered)));

    // 201-tap Linear-Phase 0.5 Hz High-Pass FIR
    history[cursor] = lp;
    double y = 0;
    for (size_t k = 0; k < 201; ++k) {
      y += taps[k] * history[(cursor + 201 - k) % 201];
    }
    cursor = (cursor + 1) % 201;
    return y;
  }
};
