"""Python ports of the C++ ECG, acceleration, and pulse analysis."""

from collections import deque
from dataclasses import dataclass
from math import cos, pi, sin, sqrt
from statistics import fmean


class Biquad:
    def __init__(self, b0: float, b1: float, b2: float, a1: float, a2: float):
        self.b0, self.b1, self.b2, self.a1, self.a2 = b0, b1, b2, a1, a2
        self.reset()

    def reset(self) -> None:
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def run(self, x: float) -> float:
        y = self.b0*x + self.b1*self.x1 + self.b2*self.x2 - self.a1*self.y1 - self.a2*self.y2
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


class EcgFilter:
    def __init__(self, adc_max: float = 16383.0):
        self.notch = Biquad(0.9695312529087462, 0.5992023379391097,
                            0.9695312529087462, 0.5992023379391097,
                            0.9390625058174924)
        scale = 268435456.0
        self.a = Biquad(24511874/scale, 49023748/scale, 24511874/scale,
                        -263375836/scale, 92986877/scale)
        self.b = Biquad(40044810/scale, 80089620/scale, 40044810/scale,
                        -143201402/scale, 34946300/scale)
        wc = 2*pi*0.5/200
        self.taps = [((1 - wc/pi) if m == 0 else -sin(wc*m)/(pi*m))
                     * (0.54 - 0.46*cos(2*pi*k/200))
                     for k in range(201) for m in (k - 100,)]
        self.adc_max = adc_max
        self.rail_margin = max(5.0, adc_max*0.01)
        self.max_slew = adc_max*0.15
        self.reset()

    def reset(self) -> None:
        self.ready = False
        self.cursor = 0

    def run(self, raw: float) -> float:
        if not self.ready:
            self.prev_clean_raw = self.dc_estimate = float(raw)
            self.notch.reset()
            self.a.reset()
            self.b.reset()
            self.history = [0.0]*201
            self.ready = True
        if raw <= self.rail_margin or raw >= self.adc_max - self.rail_margin:
            raw = self.prev_clean_raw
        raw = max(self.prev_clean_raw - self.max_slew,
                  min(self.prev_clean_raw + self.max_slew, raw))
        self.prev_clean_raw = raw
        self.dc_estimate += (raw - self.dc_estimate)*0.02
        lp = self.b.run(self.a.run(self.notch.run(raw - self.dc_estimate)))
        self.history[self.cursor] = lp
        value = sum(tap*self.history[(self.cursor - k) % 201]
                    for k, tap in enumerate(self.taps))
        self.cursor = (self.cursor + 1) % 201
        return value


class AccelerationAnalysis:
    def __init__(self) -> None:
        self.smoothed = 0.0

    def run(self, x: int, y: int, z: int) -> tuple[float, float]:
        scale = 9.81/16384.0
        magnitude = sqrt((x*scale)**2 + (y*scale)**2 + (z*scale)**2)
        self.smoothed = 0.2*magnitude + 0.8*self.smoothed
        return magnitude, self.smoothed


class RepCounter:
    """Port of the Arduino rehab rep counter (BoardDummy/accel sketch).

    Feeds raw MPU-6050 accelerometer samples (in g, 1 g = 16384 counts) through
    a high-pass gravity tracker, a slew-rate jerk limiter, a tremor low-pass
    filter, and an adaptive 3D direction tracker, then counts completed reps
    with a duration-gated hysteresis state machine. Drive it with the board
    timestamp of each sample so the phase/rep timing gates match the sketch.
    """

    # Tuning parameters, unchanged from the sketch.
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
        self.rep_state = 0  # 0 = neutral, 1 = outward confirmed, 2 = return in progress
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
        """Process one raw accel sample; return the current rep count.

        ``raw_x/y/z`` are MPU-6050 counts (1 g = 16384). ``now_ms`` is the
        sample's board time in milliseconds.
        """
        scale = 1.0/16384.0
        rx, ry, rz = raw_x*scale, raw_y*scale, raw_z*scale

        # Seed the gravity vector on the first sample, like setup() does.
        if not self._seeded:
            self.dc_x, self.dc_y, self.dc_z = rx, ry, rz
            self._seeded = True
            return self.rep_count

        # 2. High-pass: subtract slow gravity/orientation baseline.
        self.dc_x += self.ALPHA_DC * (rx - self.dc_x)
        self.dc_y += self.ALPHA_DC * (ry - self.dc_y)
        self.dc_z += self.ALPHA_DC * (rz - self.dc_z)
        ac_x = rx - self.dc_x
        ac_y = ry - self.dc_y
        ac_z = rz - self.dc_z

        # 3. Slew-rate limiter: clip sudden jerks/bumps.
        self.prev_ac_x = self._clamp_step(ac_x, self.prev_ac_x, self.MAX_JERK_STEP)
        self.prev_ac_y = self._clamp_step(ac_y, self.prev_ac_y, self.MAX_JERK_STEP)
        self.prev_ac_z = self._clamp_step(ac_z, self.prev_ac_z, self.MAX_JERK_STEP)

        # 4. Low-pass: smooth out muscle tremors.
        self.lp_x += self.ALPHA_LP * (self.prev_ac_x - self.lp_x)
        self.lp_y += self.ALPHA_LP * (self.prev_ac_y - self.lp_y)
        self.lp_z += self.ALPHA_LP * (self.prev_ac_z - self.lp_z)

        # 5. Update 3x3 motion covariance for 3D orientation independence.
        lp_mag = sqrt(self.lp_x**2 + self.lp_y**2 + self.lp_z**2)
        if lp_mag > self.MIN_NOISE_FLOOR * 0.5:
            b = self.COV_BETA
            self.cxx = (1.0 - b)*self.cxx + b*self.lp_x*self.lp_x
            self.cyy = (1.0 - b)*self.cyy + b*self.lp_y*self.lp_y
            self.czz = (1.0 - b)*self.czz + b*self.lp_z*self.lp_z
            self.cxy = (1.0 - b)*self.cxy + b*self.lp_x*self.lp_y
            self.cxz = (1.0 - b)*self.cxz + b*self.lp_x*self.lp_z
            self.cyz = (1.0 - b)*self.cyz + b*self.lp_y*self.lp_z

            nu_x = self.cxx*self.u_x + self.cxy*self.u_y + self.cxz*self.u_z
            nu_y = self.cxy*self.u_x + self.cyy*self.u_y + self.cyz*self.u_z
            nu_z = self.cxz*self.u_x + self.cyz*self.u_y + self.czz*self.u_z
            norm = sqrt(nu_x*nu_x + nu_y*nu_y + nu_z*nu_z)
            if norm > 1e-7:
                nu_x /= norm
                nu_y /= norm
                nu_z /= norm
                if nu_x*self.u_x + nu_y*self.u_y + nu_z*self.u_z < 0.0:
                    nu_x, nu_y, nu_z = -nu_x, -nu_y, -nu_z
                self.u_x, self.u_y, self.u_z = nu_x, nu_y, nu_z

        # Signed 1D wave along the current 3D motion direction.
        wave = self.lp_x*self.u_x + self.lp_y*self.u_y + self.lp_z*self.u_z
        self.wave = wave

        # 6. Smoothed adaptive amplitude threshold.
        abs_wave = abs(wave)
        if abs_wave > self.peak_envelope:
            self.peak_envelope += 0.25 * (abs_wave - self.peak_envelope)
        else:
            self.peak_envelope *= 0.985
        threshold = max(self.MIN_NOISE_FLOOR, self.peak_envelope * 0.38)

        now = now_ms
        # 7. Duration-gated hysteresis state machine.
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
            if (now - self.return_start_time >= self.MIN_PHASE_MS
                    and wave * self.start_polarity < -threshold * 0.60):
                self.saw_return_swing = True
            if (self.saw_return_swing and abs(wave) < threshold * 0.35
                    and now - self.last_rep_time > self.MIN_REP_MS):
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
        vr = sqrt(fmean((s[1] - mr)**2 for s in data))
        vi = sqrt(fmean((s[2] - mi)**2 for s in data))
        if vi < mi*0.002 or vr < mr*0.001:
            return v
        saturation = 110 - 25*(vr/mr)/(vi/mi)
        if 70 <= saturation <= 100:
            v.spo2 = int(saturation + 0.5)
            v.spo2_valid = True
        peaks = []
        for i in range(2, len(data) - 2):
            t, _, x = data[i]
            if (x > mi + vi*0.25 and x > data[i-1][2] and x >= data[i+1][2]
                    and x > data[i-2][2] and x >= data[i+2][2]
                    and (not peaks or t - peaks[-1] > 0.33)):
                peaks.append(t)
        if len(peaks) >= 3:
            candidate = int(60*(len(peaks)-1)/(peaks[-1]-peaks[0]) + 0.5)
            if 40 <= candidate <= 180:
                v.bpm = candidate
                v.heart_rate_valid = True
        return v
