#pragma once

#include <Arduino.h>

// Constants
#define GRAVITY 9.81        // m/s^2
#define ACCEL_SCALE 16384.0 // LSB/g for +/- 2g range

// Smoothing factor (0.0 to 1.0). Lower = smoother, higher = more responsive.
#define ALPHA 0.2

// accelerometer output register
#define ACCEL_XOUT_H 0x3B

struct Vec3 {
  float x, y, z;
};

class Accelerometer {
private:
  // MPU6050 I2C address
  const int MPU_addr;

public:
  Accelerometer(int MPU_addr) : MPU_addr(MPU_addr) {}

  void begin();
  Vec3 read();  // read 3-axis acceleration in m/s^2
};
