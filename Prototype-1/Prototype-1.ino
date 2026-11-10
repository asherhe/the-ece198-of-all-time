#include "Accelerometer.hpp"
#include "OLED.hpp"

OLED oled;
Accelerometer accel(0x68);

void setup() {
  Serial.begin(9600);
  Wire.begin();

  accel.setup();
  oled.setup();

  Serial.println("setup successfully completed");
}

void loop() {
  accel.loop();
  oled.loop();
}
