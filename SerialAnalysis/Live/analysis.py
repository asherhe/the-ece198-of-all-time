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
