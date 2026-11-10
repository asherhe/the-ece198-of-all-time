#include "Accelerometer.hpp"
#include "OLED.hpp"
#include "Oximeter.hpp"

OLED oled;
Accelerometer accel(0x68);
Oximeter oximeter;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  accel.begin();
  oled.begin();
  oximeter.begin();

  Serial.println("setup successfully completed");
}

void loop() {
  // accel.loop();
  // oled.loop();
  oximeter.loop();
}
