#include "Accelerometer.hpp"
#include "OLED.hpp"
#include "Oximeter.hpp"

OLED oled;
Oximeter oximeter;
Accelerometer accel(0x68);

// Low-pass filter variable
float filtered_accel = 0.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  accel.begin();
  oled.begin();
  oximeter.begin();

  Serial.println("setup successfully completed");
}

void loop() {
  Vec3 a = accel.read();

  // Calculate overall 3D acceleration magnitude
  float current_linear_accel = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);

  // Apply a small threshold deadzone to eliminate static sensor noise near zero
  if (abs(current_linear_accel) < 0.15) {
    current_linear_accel = 0.0;
  }

  // Apply Low-Pass Filter: filtered = (alpha * current) + ((1 - alpha) * previous)
  filtered_accel = (ALPHA * current_linear_accel) + ((1.0 - ALPHA) * filtered_accel);

  // Output results to the Serial Monitor
  Serial.print("a:");
  Serial.println(filtered_accel);

  if (oximeter.sample()) {
    Serial.println("new oximeter sample");
    Serial.print("pulse:");
    Serial.print(oximeter.heartRate);
    Serial.print("SpO2:");
    Serial.println(oximeter.spO2);
  }
 
  oled.loop();
}
