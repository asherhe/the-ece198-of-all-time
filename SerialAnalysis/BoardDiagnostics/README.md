# Board I2C diagnostics

This temporary sketch checks the UNO R4 Minima's shared I2C bus at 115200 baud.
It scans the OLED (`0x3C`), MAX30102 (`0x57`), and MPU6050 (`0x68`), reads the
sensor IDs, and prints raw optical and acceleration data. It also draws an OLED
test message and repeatedly sends the OLED all-pixels-on command. The normal
`../Board/Board.ino` sketch must be uploaded again after testing.

From the repository root, with the board on COM8:

```powershell
arduino-cli compile -b arduino:renesas_uno:minima --libraries C:/Users/asher/OneDrive/Documents/Arduino/libraries SerialAnalysis/BoardDiagnostics
arduino-cli upload -b arduino:renesas_uno:minima -p COM8 SerialAnalysis/BoardDiagnostics
python SerialAnalysis/BoardDiagnostics/read_serial.py COM8
```

An I2C result of `0` means the address or command was acknowledged. The OLED
library's `begin()` result only confirms that its framebuffer was allocated;
it does not confirm the physical panel lit up.

The display and sensors on this board need a startup delay before I2C
initialization. The diagnostic includes the same three-second delay as the
working board firmware. An address ACK alone is insufficient to establish that
the display completed its power-up initialization.
