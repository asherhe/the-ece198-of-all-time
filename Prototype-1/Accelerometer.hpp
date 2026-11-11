#pragma once

#include <Arduino.h>
#include "ITickable.hpp"

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

class Accelerometer : public ITickable {
private:
  // MPU6050 I2C address
  const int MPU_addr;

  // Low-pass filter variable
  float filtered_accel = 0.0;

  volatile bool sampleFlag = false;

public:
  Accelerometer(int MPU_addr) : MPU_addr(MPU_addr) {}

  void begin();
  Vec3 read();  // read 3-axis acceleration in m/s^2

  uint16_t getFreq() override { return 10; }
  void onTick() override { sampleFlag = true; }
  void loop();
};
