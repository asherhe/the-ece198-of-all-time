#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>

// Temporary diagnostic sketch for the shared I2C bus. Run the serial monitor
// at 115200 baud; each address is checked again every two seconds.
struct Device {
  const char *name;
  uint8_t address;
};

const Device devices[] = {
    {"SSD1306 OLED", 0x3C},
    {"MAX30102", 0x57},
    {"MPU6050", 0x68},
};

Adafruit_SSD1306 display(128, 64, &Wire, -1);
MAX30105 optical;
bool opticalReady = false;
bool displayBufferReady = false;
int initialPartId = -1;

uint8_t commandOLED(uint8_t command) {
  Wire.beginTransmission(0x3C);
  Wire.write(0x00); // OLED command control byte
  Wire.write(command);
  return Wire.endTransmission();
}

uint8_t wakeMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  return Wire.endTransmission();
}

int readAccel(uint8_t *values) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(0x68, 6, true) != 6) return -2;
  for (uint8_t i = 0; i < 6; ++i) values[i] = Wire.read();
  return 0;
}

int readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(address, uint8_t(1), true) != 1) return -2;
  return Wire.read();
}

void printRegister(const char *label, uint8_t address, uint8_t reg) {
  Serial.print(label);
  Serial.print(" = ");
  int value = readRegister(address, reg);
  if (value < 0) Serial.println(value);
  else {
    Serial.print("0x");
    Serial.println(value, HEX);
  }
}

void setup() {
  Serial.begin(115200);
  // Match the power-up delay required by the working prototype.
  delay(3000);
  Wire.begin();
  Wire.setClock(100000);
  displayBufferReady = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (displayBufferReady) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(0, 8);
    display.println("OLED TEST");
    display.setTextSize(1);
    display.println("I2C diagnostic");
    display.display();
  }
  Wire.setClock(400000);
  initialPartId = readRegister(0x57, 0xFF);
  opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
  if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
}

void loop() {
  Serial.println("I2C scan (0=ACK, 2=address NACK, 4=other bus error)");
  for (const Device &device : devices) {
    Wire.beginTransmission(device.address);
    uint8_t result = Wire.endTransmission();
    Serial.print(device.name);
    Serial.print(" 0x");
    Serial.print(device.address, HEX);
    Serial.print(" result=");
    Serial.println(result);
  }
  Serial.print("OLED buffer initialized = ");
  Serial.println(displayBufferReady);
  Serial.print("OLED all-pixels-on command = ");
  Serial.println(commandOLED(0xA5));
  Serial.print("OLED display-on command = ");
  Serial.println(commandOLED(0xAF));
  Serial.print("MAX30102 begin at 400 kHz = ");
  Serial.println(opticalReady);
  Serial.print("MAX30102 ID before begin = ");
  Serial.println(initialPartId);
  Wire.setClock(100000);
  printRegister("MAX30102 part ID at 100 kHz", 0x57, 0xFF);
  printRegister("MPU6050 WHO_AM_I at 100 kHz", 0x68, 0x75);
  Wire.setClock(400000);
  printRegister("MAX30102 part ID at 400 kHz", 0x57, 0xFF);
  printRegister("MPU6050 WHO_AM_I at 400 kHz", 0x68, 0x75);
  Serial.print("MPU6050 power register before wake = 0x");
  Serial.println(readRegister(0x68, 0x6B), HEX);
  Serial.print("MPU6050 wake write = ");
  Serial.println(wakeMPU());
  delay(20);
  Serial.print("MPU6050 power register after wake = 0x");
  Serial.println(readRegister(0x68, 0x6B), HEX);
  uint8_t acceleration[6] = {};
  Serial.print("MPU6050 accel read status = ");
  Serial.println(readAccel(acceleration));
  Serial.print("MPU6050 accel raw = ");
  for (uint8_t value : acceleration) {
    Serial.print(value, HEX);
    Serial.print(' ');
  }
  Serial.println();
  Serial.print("SparkFun readPartID after begin = 0x");
  Serial.println(optical.readPartID(), HEX);
  if (!opticalReady) {
    opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
    Serial.print("MAX30102 retry begin = ");
    Serial.println(opticalReady);
    if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
  }
  if (opticalReady) {
    optical.check();
    Serial.print("MAX30102 FIFO samples = ");
    Serial.println(optical.available());
    if (optical.available()) {
      Serial.print("red = ");
      Serial.print(optical.getRed());
      Serial.print(", IR = ");
      Serial.println(optical.getIR());
      while (optical.available()) optical.nextSample();
    }
  }
  if (displayBufferReady) display.display();
  Serial.println();
  delay(2000);
}
