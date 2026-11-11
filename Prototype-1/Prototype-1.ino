#include "Accelerometer.hpp"
#include "Scheduler.hpp"
#include "OLED.hpp"
#include "Oximeter.hpp"

OLED oled;
Oximeter oximeter;
Accelerometer accel(0x68);

Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  Serial.println("initialized I2C");

  accel.begin();
  Serial.println("accelerometer initialization complete");
  oximeter.begin();
  Serial.println("oximeter initialization complete");
  oled.begin();
  Serial.println("OLED initialization complete");

  scheduler.begin(200);
  scheduler.registerDevice(&accel);

  Serial.println("setup successfully completed");
}

void loop() {
  scheduler.loop();

  // accel.loop();

  if (oximeter.sample()) {
    Serial.println("new oximeter sample");
    Serial.print("pulse:");
    Serial.print(oximeter.heartRate);
    Serial.print("SpO2:");
    Serial.println(oximeter.spO2);
  }
 
  oled.loop();
}
