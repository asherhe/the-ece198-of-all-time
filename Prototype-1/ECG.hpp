#ifndef ECG_HPP
#define ECG_HPP

#include <Arduino.h>
#include <EEPROM.h>
#include "ITickable.hpp"

struct ECGRaw {
  int diff_node;
  int wilson_node;
  bool is_clipped;
};

class ECG : public ITickable {
private:
  uint8_t pinDiff;
  uint8_t pinWils;

  static const int RAM_STORAGE_SIZE = 2048;
  static const int RAM_STORAGE_MASK = RAM_STORAGE_SIZE - 1;
  static const int FLASH_STORAGE_SIZE = 1024;

  static const int32_t ADC_MAX = 16383;
  static const int32_t RAIL_MARGIN = 150;
  static const int32_t MAX_SLEW_PER_SAMPLE = 2500;

  const int64_t b10 = 24511874, b11 = 49023748, b12 = 24511874;
  const int64_t a11 = -263375836, a12 = 92986877;
  const int64_t b20 = 40044810,  b21 = 80089620,  b22 = 40044810;
  const int64_t a21 = -143201402, a22 = 34946300;

  int64_t iir_D1[4] = {0};
  int64_t iir_D2[4] = {0};
  int64_t iir_W1[4] = {0};
  int64_t iir_W2[4] = {0};

  static const int FIR_TAPS = 201;
  int32_t h[FIR_TAPS];

  int32_t firBuffer_diff[FIR_TAPS] = {0};
  int32_t firBuffer_wils[FIR_TAPS] = {0};
  int firIndex = 0;

  int32_t dcEstimateDiff = 8192;
  int32_t dcEstimateWils = 8192;
  int32_t prevCleanRawDiff = 8192;
  int32_t prevCleanRawWils = 8192;

  int32_t storageBuffer_diff[RAM_STORAGE_SIZE];
  int32_t storageBuffer_wils[RAM_STORAGE_SIZE];
  bool clipBuffer[RAM_STORAGE_SIZE];
  int storageIndex = 0;

  bool isSavingToFlash = false;
  uint32_t flashSavedTimestamp = 0;
  int flashCurrentWriteAddress = 0;
  int flashSampleCounter = 0;
  int32_t lastFlashDiffValue = 0;
  int32_t lastFlashWilsonValue = 0;
  uint8_t flatRunCount = 0;

  uint32_t intervalMs = 0;
  uint32_t lastIntervalTriggerMs = 0;
  uint32_t specificTargetTimeMs = 0;
  bool specificTimeTriggered = false;

  volatile bool sampleFlag = false;

  int32_t readIsolatedOversampled(uint8_t pin) {
    analogRead(pin);
    delayMicroseconds(15);

    int32_t sum = 0;
    for (int i = 0; i < 4; i++) {
      sum += analogRead(pin);
    }
    return sum >> 2;
  }

  int32_t conditionRawInput(int32_t rawSample, int32_t &prevSample, int32_t &dcEstimate, bool &clippedFlag) {
    if (rawSample <= RAIL_MARGIN || rawSample >= (ADC_MAX - RAIL_MARGIN)) {
      clippedFlag = true;
      rawSample = prevSample;
    }

    int32_t delta = rawSample - prevSample;
    if (delta > MAX_SLEW_PER_SAMPLE) {
      rawSample = prevSample + MAX_SLEW_PER_SAMPLE;
    } else if (delta < -MAX_SLEW_PER_SAMPLE) {
      rawSample = prevSample - MAX_SLEW_PER_SAMPLE;
    }

    prevSample = rawSample;
    dcEstimate += (rawSample - dcEstimate) >> 6;
    return rawSample - dcEstimate;
  }

  void generateFixedPointFIR() {
    float omega_c = 2.0f * PI * 0.5f / 200.0f;
    int M = FIR_TAPS - 1;
    for (int k = 0; k < FIR_TAPS; k++) {
      float val;
      if (k == M / 2) {
        val = 1.0f - (omega_c / PI);
      } else {
        float arg = k - (M / 2.0f);
        val = -(sin(omega_c * arg) / (PI * arg)) * (0.54f - 0.46f * cos((2.0f * PI * k) / M));
      }
      h[k] = (int32_t)(val * 268435456.0f);
    }
  }

  inline int32_t shiftAndRoundQ28(int64_t value) {
    return (int32_t)((value + 134217728LL) >> 28);
  }

  void tickFlashBackgroundWorker() {
    if (!isSavingToFlash) return;
    if (storageIndex & 1) return;

    int historicalIndex = (storageIndex - (FLASH_STORAGE_SIZE * 2) + (flashSampleCounter * 2)) & RAM_STORAGE_MASK;

    int32_t currentDiff = storageBuffer_diff[historicalIndex];
    int32_t currentWils = storageBuffer_wils[historicalIndex];

    int32_t deltaDiff = currentDiff - lastFlashDiffValue;
    int32_t deltaWils = currentWils - lastFlashWilsonValue;

    const int32_t ACTIVITY_THRESHOLD = 15;
    bool highActivity = (abs(deltaDiff) > ACTIVITY_THRESHOLD) || (abs(deltaWils) > ACTIVITY_THRESHOLD);

    if (!highActivity && flashSampleCounter < (FLASH_STORAGE_SIZE - 1)) {
      flatRunCount++;
      if (flatRunCount == 255) {
        EEPROM.update(flashCurrentWriteAddress++, 0x00);
        EEPROM.update(flashCurrentWriteAddress++, flatRunCount);
        flatRunCount = 0;
      }
      lastFlashDiffValue = currentDiff;
      lastFlashWilsonValue = currentWils;
    } else {
      if (flatRunCount > 0) {
        EEPROM.update(flashCurrentWriteAddress++, 0x00);
        EEPROM.update(flashCurrentWriteAddress++, flatRunCount);
        flatRunCount = 0;
      }
      EEPROM.update(flashCurrentWriteAddress++, 0xFF);

      int8_t compressedDiffByte = (int8_t)constrain(deltaDiff, -128, 127);
      int8_t compressedWilsByte = (int8_t)constrain(deltaWils, -128, 127);

      EEPROM.update(flashCurrentWriteAddress++, compressedDiffByte);
      EEPROM.update(flashCurrentWriteAddress++, compressedWilsByte);

      lastFlashDiffValue = currentDiff;
      lastFlashWilsonValue = currentWils;
    }

    flashSampleCounter++;

    if (flashSampleCounter >= FLASH_STORAGE_SIZE || flashCurrentWriteAddress >= EEPROM.length() - 3) {
      isSavingToFlash = false;
    }
  }

  void checkAutomationSchedules() {
    if (isSavingToFlash) return;
    uint32_t currentMs = millis();

    if (intervalMs > 0 && (currentMs - lastIntervalTriggerMs >= intervalMs)) {
      lastIntervalTriggerMs = currentMs;
      triggerFlashSave(currentMs);
      return;
    }

    if (specificTargetTimeMs > 0 && !specificTimeTriggered && (currentMs >= specificTargetTimeMs)) {
      specificTimeTriggered = true;
      triggerFlashSave(currentMs);
    }
  }

public:
  ECG(uint8_t diff_pin, uint8_t wilson_pin) : pinDiff(diff_pin), pinWils(wilson_pin) {}

  void begin() {
    analogReadResolution(14);
    pinMode(pinDiff, INPUT);
    pinMode(pinWils, INPUT);

    generateFixedPointFIR();

    int32_t initialDiff = readIsolatedOversampled(pinDiff);
    int32_t initialWils = readIsolatedOversampled(pinWils);

    dcEstimateDiff = initialDiff;
    dcEstimateWils = initialWils;
    prevCleanRawDiff = initialDiff;
    prevCleanRawWils = initialWils;

    memset(iir_D1, 0, sizeof(iir_D1));
    memset(iir_D2, 0, sizeof(iir_D2));
    memset(iir_W1, 0, sizeof(iir_W1));
    memset(iir_W2, 0, sizeof(iir_W2));
    memset(firBuffer_diff, 0, sizeof(firBuffer_diff));
    memset(firBuffer_wils, 0, sizeof(firBuffer_wils));
    memset(storageBuffer_diff, 0, sizeof(storageBuffer_diff));
    memset(storageBuffer_wils, 0, sizeof(storageBuffer_wils));
    memset(clipBuffer, 0, sizeof(clipBuffer));

    lastIntervalTriggerMs = millis();
  }

  void setScheduleInterval(uint32_t periodicIntervalMs) {
    intervalMs = periodicIntervalMs;
    lastIntervalTriggerMs = millis();
  }

  void setScheduledTime(uint32_t preprogrammedTimeMs) {
    specificTargetTimeMs = preprogrammedTimeMs;
    specificTimeTriggered = false;
  }

  uint16_t getFreq() override { return 200; }
  void onTick() override { sampleFlag = true; }

  void loop() {
    checkAutomationSchedules();

    if (sampleFlag) {
      sampleFlag = false;

      int32_t rawDiff = readIsolatedOversampled(pinDiff);
      int32_t rawWils = readIsolatedOversampled(pinWils);

      bool sampleClipped = false;
      int32_t acDiff = conditionRawInput(rawDiff, prevCleanRawDiff, dcEstimateDiff, sampleClipped);
      int32_t acWils = conditionRawInput(rawWils, prevCleanRawWils, dcEstimateWils, sampleClipped);

      int32_t out_d1 = shiftAndRoundQ28(b10 * acDiff + b11 * iir_D1[2] + b12 * iir_D1[3] - a11 * iir_D1[0] - a12 * iir_D1[1]);
      iir_D1[3] = iir_D1[2]; iir_D1[2] = acDiff;
      iir_D1[1] = iir_D1[0]; iir_D1[0] = out_d1;

      int32_t out_d2 = shiftAndRoundQ28(b20 * out_d1 + b21 * iir_D2[2] + b22 * iir_D2[3] - a21 * iir_D2[0] - a22 * iir_D2[1]);
      iir_D2[3] = iir_D2[2]; iir_D2[2] = out_d1;
      iir_D2[1] = iir_D2[0]; iir_D2[0] = out_d2;

      int32_t out_w1 = shiftAndRoundQ28(b10 * acWils + b11 * iir_W1[2] + b12 * iir_W1[3] - a11 * iir_W1[0] - a12 * iir_W1[1]);
      iir_W1[3] = iir_W1[2]; iir_W1[2] = acWils;
      iir_W1[1] = iir_W1[0]; iir_W1[0] = out_w1;

      int32_t out_w2 = shiftAndRoundQ28(b20 * out_w1 + b21 * iir_W2[2] + b22 * iir_W2[3] - a21 * iir_W2[0] - a22 * iir_W2[1]);
      iir_W2[3] = iir_W2[2]; iir_W2[2] = out_w1;
      iir_W2[1] = iir_W2[0]; iir_W2[0] = out_w2;

      firBuffer_diff[firIndex] = out_d2;
      firBuffer_wils[firIndex] = out_w2;

      int64_t acc_diff = 0;
      int64_t acc_wils = 0;
      int k = 0;

      for (; k <= firIndex; k++) {
        int64_t hk = h[k]; int idx = firIndex - k;
        acc_diff += hk * firBuffer_diff[idx];
        acc_wils += hk * firBuffer_wils[idx];
      }
      for (; k < FIR_TAPS; k++) {
        int64_t hk = h[k]; int idx = FIR_TAPS + firIndex - k;
        acc_diff += hk * firBuffer_diff[idx];
        acc_wils += hk * firBuffer_wils[idx];
      }

      if (++firIndex >= FIR_TAPS) firIndex = 0;

      int32_t finalDiff = shiftAndRoundQ28(acc_diff);
      int32_t finalWils = shiftAndRoundQ28(acc_wils);

      storageBuffer_diff[storageIndex] = finalDiff;
      storageBuffer_wils[storageIndex] = finalWils;
      clipBuffer[storageIndex] = sampleClipped;
      storageIndex = (storageIndex + 1) & RAM_STORAGE_MASK;

      Serial.print(finalDiff);
      Serial.print(",");
      Serial.print(finalWils);
      Serial.print(",");
      Serial.println(sampleClipped ? 1 : 0);

      tickFlashBackgroundWorker();
    }
  }

  ECGRaw read_raw() {
    int lastIndex = (storageIndex - 1) & RAM_STORAGE_MASK;
    return {
      (int)storageBuffer_diff[lastIndex],
      (int)storageBuffer_wils[lastIndex],
      clipBuffer[lastIndex]
    };
  }

  void triggerFlashSave(uint32_t timestamp) {
    if (isSavingToFlash) return;

    isSavingToFlash = true;
    flashSavedTimestamp = timestamp;
    flashCurrentWriteAddress = 0;
    flashSampleCounter = 0;
    flatRunCount = 0;

    EEPROM.put(flashCurrentWriteAddress, flashSavedTimestamp);
    flashCurrentWriteAddress += sizeof(flashSavedTimestamp);

    int baseHistoryIndex = (storageIndex - FLASH_STORAGE_SIZE * 2) & RAM_STORAGE_MASK;
    lastFlashDiffValue = storageBuffer_diff[baseHistoryIndex];
    lastFlashWilsonValue = storageBuffer_wils[baseHistoryIndex];
  }

  bool isFlashSavingBusy() {
    return isSavingToFlash;
  }
};

#endif
