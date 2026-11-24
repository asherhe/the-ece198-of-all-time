"""Python ports of the C++ ECG, acceleration, and pulse analysis."""

from collections import deque
from dataclasses import dataclass
from math import cos, pi, sin, sqrt
from statistics import fmean, median

import numpy as np

from .protocol import ECG_RATE_HZ


class Biquad:

    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float):
        self.b0, self.b1, self.b2, self.a1, self.a2 = b0, b1, b2, a1, a2
        self.reset()

    @classmethod
    def notch(cls, f0: float, q: float, fs: float) -> "Biquad":
        w0 = 2.0 * pi * f0 / fs
        alpha = sin(w0) / (2.0 * q)
        a0 = 1.0 + alpha
        return cls(
            1.0 / a0,
            (-2.0 * cos(w0)) / a0,
            1.0 / a0,
            (-2.0 * cos(w0)) / a0,
            (1.0 - alpha) / a0,
        )

    @classmethod
    def high_pass(cls, fc: float, q: float, fs: float) -> "Biquad":
        w0 = 2.0 * pi * fc / fs
        alpha = sin(w0) / (2.0 * q)
        cs = cos(w0)
        a0 = 1.0 + alpha
        return cls(
            ((1.0 + cs) * 0.5) / a0,
            (-(1.0 + cs)) / a0,
            ((1.0 + cs) * 0.5) / a0,
            (-2.0 * cs) / a0,
            (1.0 - alpha) / a0,
        )

    @classmethod
    def low_pass(cls, fc: float, q: float, fs: float) -> "Biquad":
        w0 = 2.0 * pi * fc / fs
        alpha = sin(w0) / (2.0 * q)
        cs = cos(w0)
        a0 = 1.0 + alpha
        return cls(
            ((1.0 - cs) * 0.5) / a0,
            (1.0 - cs) / a0,
            ((1.0 - cs) * 0.5) / a0,
            (-2.0 * cs) / a0,
            (1.0 - alpha) / a0,
        )

    def reset(self) -> None:
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def run(self, x: float) -> float:
        y = (
            self.b0 * x
            + self.b1 * self.x1
            + self.b2 * self.x2
            - self.a1 * self.y1
            - self.a2 * self.y2
        )
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


class EcgFilter:
    """Port of CLI/ecg_filter.hpp (keep the two in sync).

    Stages: spike/step removal by cascaded medians, NLMS accelerometer motion
    cancellation, 60/120 Hz notches, 1 Hz high-pass and 35 Hz low-pass
    biquads, and a sliding-FFT spectral equaliser/noise gate with clean
    linear gain.
    """

    FS = float(ECG_RATE_HZ)
    W1 = 61
    W2 = 201
    NLMS_TAPS = 16
    FFT_N = 128
    FFT_HOP = 32

    def __init__(self, adc_max: float = 16383.0, gain: float = 2.5):
        self.adc_max = adc_max
        self.gain = gain
        self.notch60 = Biquad.notch(60.0, 28.0, self.FS)
        self.notch120 = Biquad.notch(120.0, 28.0, self.FS)
        q8 = (0.50979558, 0.60134489, 0.89997622, 2.56291545)
        self.hp_sections = [Biquad.high_pass(1.0, q, self.FS) for q in q8]
        self.lp_sections = [Biquad.low_pass(35.0, q, self.FS) for q in q8]

        n = self.FFT_N
        self.hann = 0.5 * (1.0 - np.cos(2.0 * pi * np.arange(n) / n))
        freqs = np.arange(n // 2 + 1) * self.FS / n
        self.eq_curve = np.where((freqs < 1.0) | (freqs > 35.0), 0.05,
                                 np.where((freqs >= 5.0) & (freqs <= 20.0), 1.5, 1.0))
        # The noise floor is a long-term estimate; like the C++ filter it
        # survives reset() so a short stream gap does not discard it.
        self.noise_floor = np.full(n // 2 + 1, 5.0)
        self.reset()

    def reset(self) -> None:
        self.ready = False
        self.prev_clean_raw = 0.0
        self.glitch_win = deque(maxlen=5)
        self.med_buf1 = deque(maxlen=self.W1)
        self.med_buf2 = deque(maxlen=self.W2)
        self.motion_hist = deque([0.0] * self.NLMS_TAPS, maxlen=self.NLMS_TAPS)
        self.nlms_weights = [0.0] * self.NLMS_TAPS
        self.latest_dynamic_accel = 0.0
        self.fft_in = np.zeros(self.FFT_N)
        self.fft_out = np.zeros(self.FFT_N)
        self.fft_cursor = 0
        self.fft_hop = 0
        self.notch60.reset()
        self.notch120.reset()
        for hp in self.hp_sections:
            hp.reset()
        for lp in self.lp_sections:
            lp.reset()

    def update_motion(self, dynamic_accel_ms2: float) -> None:
        self.latest_dynamic_accel = float(dynamic_accel_ms2)

    def _run_nlms_motion_cancel(self, ecg_signal: float) -> float:
        self.motion_hist.appendleft(self.latest_dynamic_accel)
        est_artifact = sum(w * m for w, m in zip(self.nlms_weights, self.motion_hist))
        norm_sq = 1e-4 + sum(m * m for m in self.motion_hist)
        err = ecg_signal - est_artifact
        step = (0.015 * err) / norm_sq
        self.nlms_weights = [
            max(-500.0, min(500.0, w + step * m))
            for w, m in zip(self.nlms_weights, self.motion_hist)
        ]
        return err

    def _run_spectral_block(self, sample: float) -> float:
        """128-point sliding FFT (hop 32) equaliser with a gentle noise gate.

        Output is delayed by one FFT length (320 ms at 400 Hz), exactly like
        the C++ implementation.
        """
        n = self.FFT_N
        cursor = self.fft_cursor
        self.fft_in[cursor] = sample
        out = float(self.fft_out[cursor])
        self.fft_out[cursor] = 0.0
        cursor = self.fft_cursor = (cursor + 1) % n
        self.fft_hop += 1
        if self.fft_hop >= self.FFT_HOP:
            self.fft_hop = 0
            spectrum = np.fft.rfft(np.roll(self.fft_in, -cursor) * self.hann)
            mag = np.abs(spectrum)
            floor = self.noise_floor
            floor = np.where(mag < floor * 1.5,
                             0.96 * floor + 0.04 * mag,
                             0.998 * floor + 0.002 * mag)
            self.noise_floor = floor
            snr = mag / np.maximum(floor, 1e-6)
            gate = np.where(snr > 1.1, 1.0 - 0.5 / np.maximum(snr, 1.1) ** 2, 0.35)
            frame = np.fft.irfft(spectrum * (self.eq_curve * gate), n)
            self.fft_out += np.roll(frame, cursor) * (2.0 / 3.0)
        return out

    def run(self, raw: float) -> float:
        raw = float(raw)
        if not self.ready:
            self.prev_clean_raw = raw
            self.glitch_win.extend([raw] * 5)
            self.med_buf1.extend([raw] * self.W1)
            self.med_buf2.extend([raw] * self.W2)
            self.ready = True

        max_slew = self.adc_max * 0.25
        raw = max(
            self.prev_clean_raw - max_slew,
            min(self.prev_clean_raw + max_slew, raw),
        )
        self.prev_clean_raw = raw

        self.glitch_win.append(raw)
        deglitched = median(self.glitch_win)

        self.med_buf1.append(deglitched)
        m1 = median(self.med_buf1)

        self.med_buf2.append(m1)
        baseline = median(self.med_buf2)

        x = deglitched - baseline
        x = self._run_nlms_motion_cancel(x)
        x = self.notch120.run(self.notch60.run(x))
        for hp in self.hp_sections:
            x = hp.run(x)
        for lp in self.lp_sections:
            x = lp.run(x)

        x = self._run_spectral_block(x)
        return x * self.gain


class AccelerationAnalysis:

    def __init__(self) -> None:
        self.smoothed = 0.0

    def run(self, x: int, y: int, z: int) -> tuple[float, float]:
        scale = 9.81 / 16384.0
        magnitude = sqrt((x * scale) ** 2 + (y * scale) ** 2 + (z * scale) ** 2)
        self.smoothed = 0.2 * magnitude + 0.8 * self.smoothed
        return magnitude, self.smoothed

class RepCounter:
    ALPHA_DC = 0.015
    ALPHA_LP = 0.16
    COV_BETA = 0.04
    MIN_NOISE_FLOOR = 0.022
    MAX_JERK_STEP = 0.055
    MIN_PHASE_MS = 220
    MIN_REP_MS = 900
    REP_TIMEOUT_MS = 8000

    def __init__(self) -> None:
        self.dc_x = self.dc_y = self.dc_z = 0.0
        self.prev_ac_x = self.prev_ac_y = self.prev_ac_z = 0.0
        self.lp_x = self.lp_y = self.lp_z = 0.0
        self.cxx = self.cyy = self.czz = 1e-4
        self.cxy = self.cxz = self.cyz = 0.0
        self.u_x, self.u_y, self.u_z = 1.0, 0.0, 0.0
        self.peak_envelope = 0.06
        self.rep_count = 0
        self.rep_state = 0
        self.start_polarity = 1
        self.saw_return_swing = False
        self.phase_start_time = 0
        self.return_start_time = 0
        self.last_rep_time = 0
        self.wave = 0.0
        self._seeded = False

    @staticmethod
    def _clamp_step(target: float, current: float, max_step: float) -> float:
        diff = target - current
        if diff > max_step:
            return current + max_step
        if diff < -max_step:
            return current - max_step
        return target

    def run(self, raw_x: int, raw_y: int, raw_z: int, now_ms: int) -> int:
        scale = 1.0 / 16384.0
        rx, ry, rz = raw_x * scale, raw_y * scale, raw_z * scale

        if not self._seeded:
            self.dc_x, self.dc_y, self.dc_z = rx, ry, rz
            self._seeded = True
            return self.rep_count

        self.dc_x += self.ALPHA_DC * (rx - self.dc_x)
        self.dc_y += self.ALPHA_DC * (ry - self.dc_y)
        self.dc_z += self.ALPHA_DC * (rz - self.dc_z)
        ac_x = rx - self.dc_x
        ac_y = ry - self.dc_y
        ac_z = rz - self.dc_z

        self.prev_ac_x = self._clamp_step(ac_x, self.prev_ac_x, self.MAX_JERK_STEP)
        self.prev_ac_y = self._clamp_step(ac_y, self.prev_ac_y, self.MAX_JERK_STEP)
        self.prev_ac_z = self._clamp_step(ac_z, self.prev_ac_z, self.MAX_JERK_STEP)

        self.lp_x += self.ALPHA_LP * (self.prev_ac_x - self.lp_x)
        self.lp_y += self.ALPHA_LP * (self.prev_ac_y - self.lp_y)
        self.lp_z += self.ALPHA_LP * (self.prev_ac_z - self.lp_z)

        lp_mag = sqrt(self.lp_x**2 + self.lp_y**2 + self.lp_z**2)
        if lp_mag > self.MIN_NOISE_FLOOR * 0.5:
            b = self.COV_BETA
            self.cxx = (1.0 - b) * self.cxx + b * self.lp_x * self.lp_x
            self.cyy = (1.0 - b) * self.cyy + b * self.lp_y * self.lp_y
            self.czz = (1.0 - b) * self.czz + b * self.lp_z * self.lp_z
            self.cxy = (1.0 - b) * self.cxy + b * self.lp_x * self.lp_y
            self.cxz = (1.0 - b) * self.cxz + b * self.lp_x * self.lp_z
            self.cyz = (1.0 - b) * self.cyz + b * self.lp_y * self.lp_z

            nu_x = self.cxx * self.u_x + self.cxy * self.u_y + self.cxz * self.u_z
            nu_y = self.cxy * self.u_x + self.cyy * self.u_y + self.cyz * self.u_z
            nu_z = self.cxz * self.u_x + self.cyz * self.u_y + self.czz * self.u_z
            norm = sqrt(nu_x * nu_x + nu_y * nu_y + nu_z * nu_z)
            if norm > 1e-7:
                nu_x /= norm
                nu_y /= norm
                nu_z /= norm
                if nu_x * self.u_x + nu_y * self.u_y + nu_z * self.u_z < 0.0:
                    nu_x, nu_y, nu_z = -nu_x, -nu_y, -nu_z
                self.u_x, self.u_y, self.u_z = nu_x, nu_y, nu_z

        wave = self.lp_x * self.u_x + self.lp_y * self.u_y + self.lp_z * self.u_z
        self.wave = wave

        abs_wave = abs(wave)
        if abs_wave > self.peak_envelope:
            self.peak_envelope += 0.25 * (abs_wave - self.peak_envelope)
        else:
            self.peak_envelope *= 0.985
        threshold = max(self.MIN_NOISE_FLOOR, self.peak_envelope * 0.38)

        now = now_ms
        if self.rep_state == 0:
            if wave > threshold:
                if self.phase_start_time == 0 or self.start_polarity != 1:
                    self.start_polarity = 1
                    self.phase_start_time = now
                elif now - self.phase_start_time >= self.MIN_PHASE_MS:
                    self.rep_state = 1
            elif wave < -threshold:
                if self.phase_start_time == 0 or self.start_polarity != -1:
                    self.start_polarity = -1
                    self.phase_start_time = now
                elif now - self.phase_start_time >= self.MIN_PHASE_MS:
                    self.rep_state = 1
            elif abs(wave) < threshold * 0.5:
                self.phase_start_time = 0
        elif self.rep_state == 1:
            if wave * self.start_polarity < -threshold:
                self.return_start_time = now
                self.saw_return_swing = False
                self.rep_state = 2
            elif now - self.phase_start_time > self.REP_TIMEOUT_MS:
                self.rep_state = 0
                self.phase_start_time = 0
        elif self.rep_state == 2:
            if (
                now - self.return_start_time >= self.MIN_PHASE_MS
                and wave * self.start_polarity < -threshold * 0.60
            ):
                self.saw_return_swing = True
            if (
                self.saw_return_swing
                and abs(wave) < threshold * 0.35
                and now - self.last_rep_time > self.MIN_REP_MS
            ):
                self.rep_count += 1
                self.last_rep_time = now
                self.rep_state = 0
                self.phase_start_time = 0
                self.saw_return_swing = False
            elif now - self.return_start_time > self.REP_TIMEOUT_MS:
                self.rep_state = 0
                self.phase_start_time = 0
                self.saw_return_swing = False

        return self.rep_count


@dataclass
class Vitals:
    bpm: int = 0
    heart_rate_valid: bool = False
    spo2: int = 0
    spo2_valid: bool = False


class Pulse:

    def __init__(self) -> None:
        self.data = deque()
        self.vitals = Vitals()

    def add(self, time_s: float, red: int, ir: int) -> None:
        self.data.append((time_s, float(red), float(ir)))
        while self.data and time_s - self.data[0][0] > 5.0:
            self.data.popleft()

    def calculate(self) -> Vitals:
        v = self.vitals
        v.heart_rate_valid = v.spo2_valid = False
        data = self.data
        if len(data) < 75 or data[-1][0] - data[0][0] < 3.0:
            return v
        mr = fmean(s[1] for s in data)
        mi = fmean(s[2] for s in data)
        if mr < 1000 or mi < 1000:
            return v
        vr = sqrt(fmean((s[1] - mr) ** 2 for s in data))
        vi = sqrt(fmean((s[2] - mi) ** 2 for s in data))
        if vi < mi * 0.002 or vr < mr * 0.001:
            return v
        saturation = 110 - 25 * (vr / mr) / (vi / mi)
        if 70 <= saturation <= 100:
            v.spo2 = int(saturation + 0.5)
            v.spo2_valid = True
        peaks = []
        for i in range(2, len(data) - 2):
            t, _, x = data[i]
            if (
                x > mi + vi * 0.25
                and x > data[i - 1][2]
                and x >= data[i + 1][2]
                and x > data[i - 2][2]
                and x >= data[i + 2][2]
                and (not peaks or t - peaks[-1] > 0.33)
            ):
                peaks.append(t)
        if len(peaks) >= 3:
            candidate = int(60 * (len(peaks) - 1) / (peaks[-1] - peaks[0]) + 0.5)
            if 40 <= candidate <= 180:
                v.bpm = candidate
                v.heart_rate_valid = True
        return v
