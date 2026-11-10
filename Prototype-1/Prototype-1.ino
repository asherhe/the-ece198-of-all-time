#include "Accelerometer.hpp"
#include "OLED.hpp"

OLED oled;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  accel_setup();
  oled.setup();

  Serial.println("setup successfully concluded");
}

void loop() {
  accel_loop();
  oled.loop();
}
