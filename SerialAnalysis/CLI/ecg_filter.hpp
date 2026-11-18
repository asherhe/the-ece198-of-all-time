#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

class ECGFilter {
private:
    double fs;
    int adcMax;
    int railMargin;
    double maxSlewPerSample;

    // Signal conditioning & baseline tracking state
    double prevCleanRaw = 0.0;
    double dcEstimate = 0.0;
    bool initialized = false;
    bool lastSampleClipped = false;

    // 60 Hz IIR Notch Biquad coefficients and state
    double nb0 = 1.0, nb1 = 0.0, nb2 = 1.0, na1 = 0.0, na2 = 0.0;
    double nx1 = 0.0, nx2 = 0.0, ny1 = 0.0, ny2 = 0.0;

    // 2nd-Order Butterworth High-Pass (0.5 Hz) coefficients and state
    double hpb0 = 1.0, hpb1 = 0.0, hpb2 = 0.0, hpa1 = 0.0, hpa2 = 0.0;
    double hpx1 = 0.0, hpx2 = 0.0, hpy1 = 0.0, hpy2 = 0.0;

    // 2nd-Order Butterworth Low-Pass (40 Hz) coefficients and state
    double lpb0 = 1.0, lpb1 = 0.0, lpb2 = 0.0, lpa1 = 0.0, lpa2 = 0.0;
    double lpx1 = 0.0, lpx2 = 0.0, lpy1 = 0.0, lpy2 = 0.0;

    void computeCoefficients(double notchFreq, double hpFreq, double lpFreq) {
        const double pi = 3.14159265358979323846;

        // 1. Narrow 60 Hz Notch Filter (Q = 30)
        double w0 = 2.0 * pi * notchFreq / fs;
        double alphaN = std::sin(w0) / (2.0 * 30.0);
        double a0N = 1.0 + alphaN;
        nb0 = 1.0 / a0N;
        nb1 = (-2.0 * std::cos(w0)) / a0N;
        nb2 = 1.0 / a0N;
        na1 = (-2.0 * std::cos(w0)) / a0N;
        na2 = (1.0 - alphaN) / a0N;

        // 2. 2nd-Order Butterworth High-Pass (0.5 Hz)
        double wHp = 2.0 * pi * hpFreq / fs;
        double alphaHp = std::sin(wHp) / std::sqrt(2.0);
        double cosHp = std::cos(wHp);
        double a0Hp = 1.0 + alphaHp;
        hpb0 = ((1.0 + cosHp) / 2.0) / a0Hp;
        hpb1 = (-(1.0 + cosHp)) / a0Hp;
        hpb2 = ((1.0 + cosHp) / 2.0) / a0Hp;
        hpa1 = (-2.0 * cosHp) / a0Hp;
        hpa2 = (1.0 - alphaHp) / a0Hp;

        // 3. 2nd-Order Butterworth Low-Pass (40.0 Hz)
        double wLp = 2.0 * pi * lpFreq / fs;
        double alphaLp = std::sin(wLp) / std::sqrt(2.0);
        double cosLp = std::cos(wLp);
        double a0Lp = 1.0 + alphaLp;
        lpb0 = ((1.0 - cosLp) / 2.0) / a0Lp;
        lpb1 = (1.0 - cosLp) / a0Lp;
        lpb2 = ((1.0 - cosLp) / 2.0) / a0Lp;
        lpa1 = (-2.0 * cosLp) / a0Lp;
        lpa2 = (1.0 - alphaLp) / a0Lp;
    }

public:
    // Set maxAdcCounts = 16383 when using analogReadResolution(14) on Uno R4
    ECGFilter(double sampleRate = 200.0, int maxAdcCounts = 16383, double notchHz = 60.0)
        : fs(sampleRate), adcMax(maxAdcCounts) {
        railMargin = std::max(5, adcMax / 100);
        maxSlewPerSample = adcMax * 0.15;
        computeCoefficients(notchHz, 0.5, 40.0);
    }

    void reset() {
        initialized = false;
        lastSampleClipped = false;
        nx1 = nx2 = ny1 = ny2 = 0.0;
        hpx1 = hpx2 = hpy1 = hpy2 = 0.0;
        lpx1 = lpx2 = lpy1 = lpy2 = 0.0;
    }

    double process(double rawSample) {
        if (!initialized) {
            prevCleanRaw = rawSample;
            dcEstimate = rawSample;
            initialized = true;
        }

        // Hold last valid reading if input hits ADC rails to stop square-wave ringing
        if (rawSample <= railMargin || rawSample >= (adcMax - railMargin)) {
            lastSampleClipped = true;
            rawSample = prevCleanRaw;
        } else {
            lastSampleClipped = false;
        }

        // Clamp impossible slew jumps caused by electrode tugs
        double delta = rawSample - prevCleanRaw;
        if (delta > maxSlewPerSample) {
            rawSample = prevCleanRaw + maxSlewPerSample;
        } else if (delta < -maxSlewPerSample) {
            rawSample = prevCleanRaw - maxSlewPerSample;
        }
        prevCleanRaw = rawSample;

        // Track and subtract DC baseline bias
        dcEstimate += (rawSample - dcEstimate) * 0.02;
        double x = rawSample - dcEstimate;

        // Stage 1: 60 Hz Notch
        double yNotch = nb0 * x + nb1 * nx1 + nb2 * nx2 - na1 * ny1 - na2 * ny2;
        nx2 = nx1; nx1 = x;
        ny2 = ny1; ny1 = yNotch;

        // Stage 2: 0.5 Hz High-Pass
        double yHp = hpb0 * yNotch + hpb1 * hpx1 + hpb2 * hpx2 - hpa1 * hpy1 - hpa2 * hpy2;
        hpx2 = hpx1; hpx1 = yNotch;
        hpy2 = hpy1; hpy1 = yHp;

        // Stage 3: 40 Hz Low-Pass
        double yLp = lpb0 * yHp + lpb1 * lpx1 + lpb2 * lpx2 - lpa1 * lpy1 - lpa2 * lpy2;
        lpx2 = lpx1; lpx1 = yHp;
        lpy2 = lpy1; lpy1 = yLp;

        return yLp;
    }

    bool isClipped() const {
        return lastSampleClipped;
    }
};
