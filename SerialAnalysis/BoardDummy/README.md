# Dummy board sketch

Open `BoardDummy.ino` in Arduino IDE and upload it instead of `../Board/Board.ino`.
It uses the same 115200 baud framed protocol, sensor sample formats, timing, and
OLED display updates as the original sketch. The OLED is the only I2C device it
accesses; no ECG, accelerometer, or optical sensor needs to be connected.

The generated signals are deterministic: two 14-bit ECG channels at 400 Hz,
MPU6050-style acceleration counts at 50 Hz, and red/infrared optical counts at
100 Hz. The optical signal represents a 75 BPM pulse and produces approximately
95% SpO2 in the current prototype analyzer. These are test values, not
measurements.
