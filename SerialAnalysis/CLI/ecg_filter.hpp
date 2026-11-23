#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

struct Biquad {
  double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  double x1 = 0, x2 = 0, y1 = 0, y2 = 0;

  Biquad() = default;
  Biquad(double B0, double B1, double B2, double A1, double A2)
      : b0(B0), b1(B1), b2(B2), a1(A1), a2(A2) {}

  void setNotch(double f0, double Q, double fs) {
    const double pi = std::acos(-1.0);
    double w0 = 2.0 * pi * f0 / fs;
    double alpha = std::sin(w0) / (2.0 * Q);
    double a0 = 1.0 + alpha;
    b0 = 1.0 / a0;
    b1 = (-2.0 * std::cos(w0)) / a0;
    b2 = 1.0 / a0;
    a1 = (-2.0 * std::cos(w0)) / a0;
    a2 = (1.0 - alpha) / a0;
  }

  void setHighPass(double fc, double Q, double fs) {
    const double pi = std::acos(-1.0);
    double w0 = 2.0 * pi * fc / fs;
    double alpha = std::sin(w0) / (2.0 * Q);
    double cs = std::cos(w0);
    double a0 = 1.0 + alpha;
    b0 = ((1.0 + cs) * 0.5) / a0;
    b1 = (-(1.0 + cs)) / a0;
    b2 = ((1.0 + cs) * 0.5) / a0;
    a1 = (-2.0 * cs) / a0;
    a2 = (1.0 - alpha) / a0;
  }

  void setLowPass(double fc, double Q, double fs) {
    const double pi = std::acos(-1.0);
    double w0 = 2.0 * pi * fc / fs;
    double alpha = std::sin(w0) / (2.0 * Q);
    double cs = std::cos(w0);
    double a0 = 1.0 + alpha;
    b0 = ((1.0 - cs) * 0.5) / a0;
    b1 = (1.0 - cs) / a0;
    b2 = ((1.0 - cs) * 0.5) / a0;
    a1 = (-2.0 * cs) / a0;
    a2 = (1.0 - alpha) / a0;
  }

  double run(double x) {
    double y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = x;
    y2 = y1;
    y1 = y;
    return y;
  }

  void reset() { x1 = x2 = y1 = y2 = 0.0; }
};

// Keep in sync with EcgFilter in Live/analysis.py (verified to ~1e-8 on a shared stream).
struct EcgFilter {
  // Must equal 1e6 / kEcgPeriodUs from protocol.hpp (checked by a static_assert in main.cpp).
  static constexpr double FS = 400.0;

  // Stage 1: Non-linear spike & abrupt DC step-jump removal
  std::array<double, 5> glitchWin{};
  size_t glitchPos = 0;

  static constexpr size_t W1 = 61;   // 152.5 ms median (erases QRS spikes)
  static constexpr size_t W2 = 201;  // 502.5 ms median (erases P/T waves & step jumps)
  std::array<double, W1> medBuf1{};
  std::array<double, W2> medBuf2{};
  size_t medPos1 = 0, medPos2 = 0;

  // Stage 2: NLMS Accelerometer Motion-Artifact Canceller (16-tap adaptive FIR)
  static constexpr size_t NLMS_TAPS = 16;
  std::array<double, NLMS_TAPS> motionHist{};
  std::array<double, NLMS_TAPS> nlmsWeights{};
  size_t motionPos = 0;
  double latestDynamicAccel = 0.0;

  // Stage 3: Harmonic Notch Bank (60 Hz + 120 Hz mains interference)
  Biquad notch60, notch120;

  // Stage 4: 8th-Order Brick-Wall Bandpass Fences (1.0 Hz HP + 35.0 Hz LP)
  std::array<Biquad, 4> hpSections;
  std::array<Biquad, 4> lpSections;

  // Stage 5: 128-point Sliding FFT Spectral Equalizer & Adaptive Noise Gate
  static constexpr size_t FFT_N = 128;
  static constexpr size_t FFT_HOP = 32;
  std::array<double, FFT_N> fftInBuf{};
  std::array<double, FFT_N> fftOutAccum{};
  std::array<double, FFT_N> hannWin{};
  std::array<double, FFT_N / 2 + 1> eqCurve{};
  std::array<double, FFT_N / 2 + 1> noiseFloor{};
  size_t fftCursor = 0;
  size_t hopCounter = 0;

  // Stage 6: Adaptive Gain Control (AGC) Peak Envelope Tracker
  double peakEnvelope = 50.0;
  double targetAmplitude = 300.0;

  double adcMax = 16383.0;
  double prevCleanRaw = 0.0;
  bool ready = false;

  EcgFilter(double maxAdcCounts = 16383.0, double targetDisplayPeak = 300.0)
      : targetAmplitude(targetDisplayPeak), adcMax(maxAdcCounts) {
    const double pi = std::acos(-1.0);

    notch60.setNotch(60.0, 28.0, FS);
    notch120.setNotch(120.0, 28.0, FS);

    const double q8[4] = {0.50979558, 0.60134489, 0.89997622, 2.56291545};
    for (int i = 0; i < 4; ++i) {
      hpSections[i].setHighPass(1.0, q8[i], FS);
      lpSections[i].setLowPass(35.0, q8[i], FS);
    }

    for (size_t n = 0; n < FFT_N; ++n) {
      hannWin[n] = 0.5 * (1.0 - std::cos(2.0 * pi * n / FFT_N));
    }
    for (size_t k = 0; k <= FFT_N / 2; ++k) {
      double freq = (double(k) * FS) / double(FFT_N);
      if (freq < 1.0 || freq > 35.0) {
        eqCurve[k] = 0.05;
      } else if (freq >= 5.0 && freq <= 20.0) {
        eqCurve[k] = 1.65;
      } else {
        eqCurve[k] = 1.0;
      }
      noiseFloor[k] = 5.0;
    }
  }

  void reset() {
    ready = false;
    glitchPos = medPos1 = medPos2 = 0;
    motionPos = 0;
    latestDynamicAccel = 0.0;
    motionHist.fill(0.0);
    nlmsWeights.fill(0.0);
    fftCursor = hopCounter = 0;
    peakEnvelope = 50.0;
    notch60.reset();
    notch120.reset();
    for (auto &s : hpSections) s.reset();
    for (auto &s : lpSections) s.reset();
    fftInBuf.fill(0.0);
    fftOutAccum.fill(0.0);
  }

  // Feed dynamic acceleration (magnitude - smoothed in m/s^2) whenever an accel sample arrives
  void updateMotion(double dynamicAccelMs2) {
    latestDynamicAccel = dynamicAccelMs2;
  }

  template <size_t N>
  static double rollingMedian(const std::array<double, N> &buf) {
    std::array<double, N> tmp = buf;
    std::nth_element(tmp.begin(), tmp.begin() + N / 2, tmp.end());
    return tmp[N / 2];
  }

  static void fftRadix2(std::vector<std::complex<double>> &a, bool invert) {
    const size_t n = a.size();
    const double pi = std::acos(-1.0);
    for (size_t i = 1, j = 0; i < n; ++i) {
      size_t bit = n >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
      double ang = 2.0 * pi / len * (invert ? 1.0 : -1.0);
      std::complex<double> wlen(std::cos(ang), std::sin(ang));
      for (size_t i = 0; i < n; i += len) {
        std::complex<double> w(1.0, 0.0);
        for (size_t j = 0; j < len / 2; ++j) {
          std::complex<double> u = a[i + j];
          std::complex<double> v = a[i + j + len / 2] * w;
          a[i + j] = u + v;
          a[i + j + len / 2] = u - v;
          w *= wlen;
        }
      }
    }
    if (invert) {
      for (auto &x : a) x /= double(n);
    }
  }

  double runNlmsMotionCancel(double ecgSignal) {
    motionHist[motionPos] = latestDynamicAccel;

    double estArtifact = 0.0;
    double normSq = 1e-4; // Regularization constant to prevent divide-by-zero when still
    for (size_t k = 0; k < NLMS_TAPS; ++k) {
      double m = motionHist[(motionPos + NLMS_TAPS - k) % NLMS_TAPS];
      estArtifact += nlmsWeights[k] * m;
      normSq += m * m;
    }

    double err = ecgSignal - estArtifact;

    // Conservative step size (mu = 0.015) adapts to exercise movement without erasing QRS spikes
    const double mu = 0.015;
    double step = (mu * err) / normSq;
    for (size_t k = 0; k < NLMS_TAPS; ++k) {
      double m = motionHist[(motionPos + NLMS_TAPS - k) % NLMS_TAPS];
      nlmsWeights[k] = std::clamp(nlmsWeights[k] + step * m, -500.0, 500.0);
    }

    motionPos = (motionPos + 1) % NLMS_TAPS;
    return err;
  }

  double processSpectralBlock(double sample) {
    fftInBuf[fftCursor] = sample;
    double outSample = fftOutAccum[fftCursor];
    fftOutAccum[fftCursor] = 0.0;
    fftCursor = (fftCursor + 1) % FFT_N;

    if (++hopCounter >= FFT_HOP) {
      hopCounter = 0;
      std::vector<std::complex<double>> spec(FFT_N);
      for (size_t i = 0; i < FFT_N; ++i) {
        size_t idx = (fftCursor + i) % FFT_N;
        spec[i] = std::complex<double>(fftInBuf[idx] * hannWin[i], 0.0);
      }

      fftRadix2(spec, false);

      for (size_t k = 0; k <= FFT_N / 2; ++k) {
        double mag = std::abs(spec[k]);
        if (mag < noiseFloor[k] * 1.5) {
          noiseFloor[k] = 0.96 * noiseFloor[k] + 0.04 * mag;
        } else {
          noiseFloor[k] = 0.998 * noiseFloor[k] + 0.002 * mag;
        }

        double snr = mag / std::max(noiseFloor[k], 1e-6);
        double gate = (snr > 1.2) ? (1.0 - 1.0 / (snr * snr)) : 0.1;
        double totalGain = eqCurve[k] * gate;

        spec[k] *= totalGain;
        if (k > 0 && k < FFT_N / 2) {
          spec[FFT_N - k] *= totalGain;
        }
      }

      fftRadix2(spec, true);

      for (size_t i = 0; i < FFT_N; ++i) {
        size_t idx = (fftCursor + i) % FFT_N;
        fftOutAccum[idx] += spec[i].real() * (2.0 / 3.0);
      }
    }
    return outSample;
  }

  double run(double raw) {
    if (!ready) {
      prevCleanRaw = raw;
      glitchWin.fill(raw);
      medBuf1.fill(raw);
      medBuf2.fill(raw);
      ready = true;
    }

    double maxSlew = adcMax * 0.25;
    double delta = raw - prevCleanRaw;
    if (delta > maxSlew) raw = prevCleanRaw + maxSlew;
    else if (delta < -maxSlew) raw = prevCleanRaw - maxSlew;
    prevCleanRaw = raw;

    // 1. 5-tap median de-glitcher + two-stage median step-jump baseline subtraction
    glitchWin[glitchPos] = raw;
    glitchPos = (glitchPos + 1) % 5;
    double deglitched = rollingMedian(glitchWin);

    medBuf1[medPos1] = deglitched;
    medPos1 = (medPos1 + 1) % W1;
    double m1 = rollingMedian(medBuf1);

    medBuf2[medPos2] = m1;
    medPos2 = (medPos2 + 1) % W2;
    double baseline = rollingMedian(medBuf2);

    double x = deglitched - baseline;

    // 2. NLMS Accelerometer Motion-Artifact Cancellation
    x = runNlmsMotionCancel(x);

    // 3. 60 Hz & 120 Hz Notch Bank
    x = notch120.run(notch60.run(x));

    // 4. 8th-Order Brick-Wall Bandpass Fences (1.0 Hz - 35.0 Hz)
    for (auto &hp : hpSections) x = hp.run(x);
    for (auto &lp : lpSections) x = lp.run(x);

    // 5. Sliding FFT Spectral Equalizer & Adaptive Noise Gate
    double specClean = processSpectralBlock(x);

    // 6. Adaptive Gain Control (AGC) + Non-Linear Soft-Knee Expansion
    double absVal = std::abs(specClean);
    // Fast attack, slow decay (~3.5 seconds at 400 Hz) to track R-wave envelope
    if (absVal > peakEnvelope) {
      peakEnvelope += 0.15 * (absVal - peakEnvelope);
    } else {
      peakEnvelope = std::max(10.0, peakEnvelope * 0.9993);
    }

    double dynamicKnee = std::max(5.0, 0.25 * peakEnvelope);
    double dynamicBoost = std::clamp(targetAmplitude / peakEnvelope, 0.5, 25.0);
    double softGate = (absVal * absVal) / (absVal * absVal + dynamicKnee * dynamicKnee);
    return dynamicBoost * softGate * dynamicKnee * std::asinh(specClean / dynamicKnee);
  }
};
