#pragma once

#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <Arduino.h>
#include <Wire.h>

// hold this many past entries for calculation
#define OX_BUFFER_SIZE 100

class Oximeter {
private:
  MAX30105 particleSensor;

  uint32_t irBuffer[OX_BUFFER_SIZE];  // raw infrared data
  uint32_t redBuffer[OX_BUFFER_SIZE]; // raw red led data

  int32_t spO2;          // measured blood SpO2
  int8_t validSpO2;      // indicate whether SpO2 measurement is valid
  int32_t heartRate;     // measured heart rate
  int8_t validHeartRate; // indicate whether heart rate measurement is valid

public:
  void begin();
  void loop();
};
