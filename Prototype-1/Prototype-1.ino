#include "Accelerometer.hpp"
#include "ECG.hpp"
#include "Scheduler.hpp"
#include "OLED.hpp"
#include "Oximeter.hpp"

Accelerometer accel(0x68);
ECG ecg(A0, A1);
Oximeter oximeter;
OLED oled;

Scheduler scheduler;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Wire.begin();
  Serial.println("initialized I2C");

  accel.begin();
  Serial.println("accelerometer initialization complete");
  ECG.begin();
  Serial.println("ECG initialization complete");
  oximeter.begin();
  Serial.println("oximeter initialization complete");
  oled.begin();
  Serial.println("OLED initialization complete");

  scheduler.begin(200);
  scheduler.registerDevice(&accel);
  scheduler.registerDevice(&ecg);

  Serial.println("setup successfully completed");
}

void loop() {
  scheduler.loop();

  accel.loop();
  ecg.loop();

  if (oximeter.sample()) {
    oled.heart_rate = oximeter.heartRate;
    // Serial.print("pulse:");
    // Serial.print(oximeter.heartRate);
    // Serial.print("SpO2:");
    // Serial.println(oximeter.spO2);
  }
 
  oled.loop();
}
