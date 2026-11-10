#include "Accelerometer.hpp"
#include "OLED.hpp"

void setup() {
  Serial.begin(9600);
  Wire.begin();

  accel_setup();
  oled_setup();

  Serial.println("setup successfully concluded");
}

void loop() {
  accel_loop();
  oled_loop();
}
