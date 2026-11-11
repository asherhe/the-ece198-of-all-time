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
    int raw = Wire.read() << 8 | Wire.read();
    accel[i] = ((float)raw / ACCEL_SCALE) * GRAVITY;
  }

  return {accel[0], accel[1], accel[2]};
}
