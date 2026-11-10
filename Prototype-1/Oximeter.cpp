#include "Oximeter.hpp"

void Oximeter::begin() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring.");
    while (1)
      ;
  }

  // ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
}

void Oximeter::loop() {
  // read samples into data buffer
  // note that this is blocking
  for (byte i = 0; i < OX_BUFFER_SIZE; i++) {
    while (particleSensor.available() == false) // Wait for new data
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); // Move to next sample set
  }

  // calculate SpO2, heart rate from gathered data
  maxim_heart_rate_and_oxygen_saturation(irBuffer, OX_BUFFER_SIZE, redBuffer,
                                         &spO2, &validSpO2, &heartRate,
                                         &validHeartRate);

  Serial.print("heart rate: ");
  Serial.print(heartRate);
  Serial.print(", SpO2: ");
  Serial.println(spO2);
}
