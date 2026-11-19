# SerialAnalysis

`Board/Board.ino` samples the sensors and drives the SSD1306 OLED. `CLI` is a
C++17 recorder and analyzer for Windows (and POSIX systems). The original
`Prototype-1` sketch is untouched. This is a prototype signal-processing tool,
not a clinically validated monitor.

The CLI keeps serial access in `serial_port.hpp`, framing in `protocol.hpp`,
and session file I/O in `main.cpp`. Signal processing lives in the independent
header-only `ecg_filter.hpp`, `acceleration_analysis.hpp`, and
`pulse_analysis.hpp` files.

## Build and run

Install the Arduino board core used by the existing prototype, the SparkFun
MAX3010x Pulse and Proximity Sensor Library, Adafruit GFX, and Adafruit
SSD1306. Open `Board/Board.ino` in Arduino IDE and upload it. The board uses
the prototype pin assignments: ECG A0/A1, MPU6050 at 0x68, MAX30102 on I2C,
and SSD1306 at 0x3C. Close Arduino Serial Monitor before starting the CLI.

Build with CMake and a C++17 compiler:

```powershell
cmake -S SerialAnalysis/CLI -B SerialAnalysis/CLI/build -G Ninja
cmake --build SerialAnalysis/CLI/build
SerialAnalysis/CLI/build/serial-analysis.exe --self-test
SerialAnalysis/CLI/build/serial-analysis.exe --port COM3 --output sessions --duration-min 30
```

Replace `COM3` with the board's port. The CLI creates a dated session folder
with `ecg.csv`, `acceleration.csv`, `vitals.csv`, and `metadata.json`. Ctrl+C
stops and closes the session. On Linux, pass a device such as `/dev/ttyACM0`.
The serial port is 115200 baud, 8N1, with no flow control. On Windows, the CLI
leaves DTR deasserted so opening a classic Arduino UNO does not trigger its
USB-serial auto-reset. Other serial tools may assert DTR and reset the board.
Recording begins immediately.

## Python live view

`Live/` ports the recorder and analysis to Python. Its Jupyter notebook plots
raw and filtered ECG, acceleration, and optical signals in a five-second
rolling window while saving the same session files. See [Live/README.md](Live/README.md)
for setup and run commands.

## Wire format, version 1

Every frame is `A5 5A | type:u8 | sequence:u16 | first_sample_micros:u32 |
count:u8 | payload_bytes:u16 | payload | CRC16:u16`, little endian. CRC16 is
CCITT-FALSE (initial 0xFFFF, polynomial 0x1021) over `type` through `payload`.
Sequence numbers advance for every attempted board frame, so gaps reveal
transmit drops. A damaged frame is discarded and the decoder searches for the
next marker. Board microsecond timestamps wrap after about 71 minutes; the CLI
unwraps them for recording.

| Type | Payload per sample | Rate |
| --- | --- | --- |
| 1 ECG | `diff_adc:u16, wilson_adc:u16` | 200 Hz, 5 ms between batch samples |
| 2 acceleration | `x:i16, y:i16, z:i16` MPU6050 raw counts | 50 Hz, 20 ms between batch samples |
| 3 optical | `micros:u32, red:u32, infrared:u32` | each available MAX30102 FIFO sample |
| 0x80 display, computer to board | `bpm:u16, remaining_seconds:u32, flags:u8` | once per second; flags bit 0 = valid BPM |

The board writes only complete frames when its USB serial transmit buffer has
room. It does not block sensor sampling to wait for the computer. The OLED
blinks an 8 px square in its upper-right corner when no valid display update
has arrived for three seconds. The CLI reports frame sequence gaps and bad
checksums every ten seconds and saves those counts in metadata.

## Analysis and limits

ECG uses the prototype's two 40 Hz low-pass biquad coefficient sets and
201-tap 0.5 Hz high-pass FIR, with corrected filter state updates. Acceleration
uses the prototype scale and smoothing factor. The pulse/SpO2 analysis is a
portable provisional peak and ratio estimator, **not an exact port of the
SparkFun/MAXIM algorithm**. Results carry validity flags and should be checked
against representative sensor captures before relying on them. Raw optical
and accelerometer samples are transmitted but not recorded, per the selected
recording scope. The board has no offline recording.
