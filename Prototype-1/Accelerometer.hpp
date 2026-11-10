#pragma once

#include <Arduino.h>

// Constants
#define GRAVITY 9.81        // m/s^2
#define ACCEL_SCALE 16384.0 // LSB/g for +/- 2g range

// Smoothing factor (0.0 to 1.0). Lower = smoother, higher = more responsive.
#define ALPHA 0.2

class Accelerometer {
private:
  // MPU6050 I2C address
  const int MPU_addr;

  // Raw acceleration variables
  int16_t ax_raw, ay_raw, az_raw;

  // Low-pass filter variable
  float filtered_accel = 0.0;

public:
  Accelerometer(int MPU_addr) : MPU_addr(MPU_addr) {}

  void begin();
  void loop();
};
