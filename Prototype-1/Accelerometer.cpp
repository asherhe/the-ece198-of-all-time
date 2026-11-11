#include "Accelerometer.hpp"

#include <Wire.h>

void Accelerometer::begin() {
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero to wake up
  Wire.endTransmission(true);
}

Vec3 Accelerometer::read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  float accel[3];
  for (int i = 0; i < 3; ++i) {
    int16_t raw = Wire.read() << 8 | Wire.read();
    accel[i] = ((float)raw / ACCEL_SCALE) * GRAVITY;
  }

  return {accel[0], accel[1], accel[2]};
}

void Accelerometer::loop() {
  if (sampleFlag) {
    sampleFlag = false;
    Vec3 a = read();
    // Serial.print(a.x);Serial.print(",");Serial.print(a.y);Serial.print(",");Serial.println(a.z);

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
  }
}
