#include "Oximeter.hpp"

void Oximeter::begin() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring.");
    while (1)
      ;
  }

  // ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  bufferIdx = 0;
}

bool Oximeter::sample() {
  while (particleSensor.available()) {
    redBuffer[bufferIdx] = particleSensor.getRed();
    irBuffer[bufferIdx] = particleSensor.getIR();
    ++bufferIdx;
    particleSensor.nextSample();

    if (bufferIdx >= OX_BUFFER_SIZE) {
      maxim_heart_rate_and_oxygen_saturation(irBuffer, OX_BUFFER_SIZE, redBuffer,
                                          &spO2, &validSpO2, &heartRate,
                                          &validHeartRate);
      bufferIdx = 0;
      // stop sampling once we have a result
      return true;
    }
  }

  return false;
}
