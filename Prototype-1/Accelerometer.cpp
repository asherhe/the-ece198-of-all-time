#include <Wire.h>

// MPU6050 I2C address
const int MPU_addr = 0x68; 

// Raw acceleration variables
int16_t ax_raw, ay_raw, az_raw;

// Constants
const float GRAVITY = 9.81; // m/s^2
const float ACCEL_SCALE = 16384.0; // LSB/g for +/- 2g range

// Low-pass filter variable
float filtered_accel = 0.0;
const float ALPHA = 0.2; // Smoothing factor (0.0 to 1.0). Lower = smoother, higher = more responsive.

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Wake up the MPU6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero to wake up
  Wire.endTransmission(true);
  
  delay(100);
}

void loop() {
  // Read accelerometer data starting from register 0x3B (ACCEL_XOUT_H)
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  
  ax_raw = Wire.read() << 8 | Wire.read();
  ay_raw = Wire.read() << 8 | Wire.read();
  az_raw = Wire.read() << 8 | Wire.read();
  
  // Convert raw values to m/s^2
  float ax = ((float)ax_raw / ACCEL_SCALE) * GRAVITY;
  float ay = ((float)ay_raw / ACCEL_SCALE) * GRAVITY;
  float az = ((float)az_raw / ACCEL_SCALE) * GRAVITY;
  
  // Calculate overall 3D acceleration magnitude
  float total_accel = sqrt(ax * ax + ay * ay + az * az);
  
  // Subtract gravity so that it starts at 0 m/s^2 when stationary
  float current_linear_accel = total_accel - GRAVITY;
  
  // Apply a small threshold deadzone to eliminate static sensor noise near zero
  if (abs(current_linear_accel) < 0.15) {
    current_linear_accel = 0.0;
  }
  
  // Apply Low-Pass Filter: filtered = (alpha * current) + ((1 - alpha) * previous)
  filtered_accel = (ALPHA * current_linear_accel) + ((1.0 - ALPHA) * filtered_accel);
  
  // Output results to the Serial Monitor
  Serial.print("Filtered Acceleration: ");
  Serial.print(filtered_accel);
  Serial.println(" m/s^2");
  
  delay(50); // Faster loop time works well with filtering
}
