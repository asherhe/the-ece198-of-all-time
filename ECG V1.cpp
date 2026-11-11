#include <Arduino.h>
#include <EEPROM.h>

struct ECGRaw {
  int diff_node;   
  int wilson_node; 
};

class ECG {
private:
  uint8_t pinDiff;
  uint8_t pinWils;

  static const int RAM_STORAGE_SIZE = 2048;
  static const int RAM_STORAGE_MASK = RAM_STORAGE_SIZE - 1;
  static const int FLASH_STORAGE_SIZE = 1024;

  const int64_t b10 = 24511874, b11 = 49023748, b12 = 24511874;
  const int64_t a11 = -263375836, a12 = 92986877;
  const int64_t b20 = 40044810,  b21 = 80089620,  b22 = 40044810;
  const int64_t a21 = -143201402, a22 = 34946300;

  int64_t iir_D1 = {0}; int64_t iir_D2 = {0};
  int64_t iir_W1 = {0}; int64_t iir_W2 = {0};

  static const int FIR_TAPS = 201; 
  int32_t h[FIR_TAPS];               

  int32_t firBuffer_diff[FIR_TAPS] = {0};
  int32_t firBuffer_wils[FIR_TAPS] = {0};
  int firIndex = 0;

  int32_t storageBuffer_diff[RAM_STORAGE_SIZE];
  int32_t storageBuffer_wils[RAM_STORAGE_SIZE];
  int storageIndex = 0;

  bool isSavingToFlash = false;
  uint32_t flashSavedTimestamp = 0;
  int flashCurrentWriteAddress = 0;
  int flashSampleCounter = 0;
  int32_t lastFlashDiffValue = 0;
  int32_t lastFlashWilsonValue = 0;

  void generateFixedPointFIR() {
    float omega_c = 2.0 * PI * 0.5 / 200.0; 
    int M = FIR_TAPS - 1;               
    for (int k = 0; k < FIR_TAPS; k++) {
      float val;
      if (k == M / 2) {
        val = 1.0 - (omega_c / PI);
      } else {
        float arg = k - (M / 2.0);
        val = -(sin(omega_c * arg) / (PI * arg)) * (0.54 - 0.46 * cos((2.0 * PI * k) / M));
      }
      h[k] = (int32_t)(val * 268435456.0f); 
    }
  }

  inline int32_t shiftAndRoundQ28(int64_t value) {
    return (int32_t)((value + 134217728LL) >> 28);
  }

public:
  ECG(uint8_t diff_pin, uint8_t wilson_pin) : pinDiff(diff_pin), pinWils(wilson_pin) {}

  void begin() {
    generateFixedPointFIR();
    
    int32_t initialDiff = analogRead(pinDiff);
    int32_t initialWils = analogRead(pinWils);

    iir_D1 = initialDiff; iir_D1 = initialDiff; iir_D1 = initialDiff; iir_D1 = initialDiff;
    iir_W1 = initialWils; iir_W1 = initialWils; iir_W1 = initialWils; iir_W1 = initialWils;

    for(int i = 0; i < FIR_TAPS; i++) {
      firBuffer_diff[i] = initialDiff;
      firBuffer_wils[i] = initialWils;
    }

    memset(storageBuffer_diff, 0, sizeof(storageBuffer_diff));
    memset(storageBuffer_wils, 0, sizeof(storageBuffer_wils));
  }

  void processNextSample() {
    int32_t rawDiff = analogRead(pinDiff);
    int32_t rawWils = analogRead(pinWils);

    int32_t out_d1 = shiftAndRoundQ28(b10*rawDiff + b11*iir_D1 + b12*iir_D1 - a11*iir_D1 - a12*iir_D1);
    iir_D1 = iir_D1; iir_D1 = rawDiff; iir_D1 = iir_D1; iir_D1 = out_d1;
    int32_t out_d2 = shiftAndRoundQ28(b20*out_d1 + b21*iir_D2 + b22*iir_D2 - a21*iir_D2 - a22*iir_D2);
    iir_D2 = iir_D2; iir_D2 = out_d1; iir_D2 = iir_D2; iir_D2 = out_d2;

    int32_t out_w1 = shiftAndRoundQ28(b10*rawWils + b11*iir_W1 + b12*iir_W1 - a11*iir_W1 - a12*iir_W1);
    iir_W1 = iir_W1; iir_W1 = rawWils; iir_W1 = iir_W1; iir_W1 = out_w1;
    int32_t out_w2 = shiftAndRoundQ28(b20*out_w1 + b21*iir_W2 + b22*iir_W2 - a21*iir_W2 - a22*iir_W2);
    iir_W2 = iir_W2; iir_W2 = out_w1; iir_W2 = iir_W2; iir_W2 = out_w2;

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

    storageBuffer_diff[storageIndex] = shiftAndRoundQ28(acc_diff);
    storageBuffer_wils[storageIndex] = shiftAndRoundQ28(acc_wils);
    storageIndex = (storageIndex + 1) & RAM_STORAGE_MASK;

    tickFlashBackgroundWorker();
  }

  ECGRaw read_raw() {
    int lastIndex = (storageIndex - 1) & RAM_STORAGE_MASK;
    return { (int)storageBuffer_diff[lastIndex], (int)storageBuffer_wils[lastIndex] };
  }

  void triggerFlashSave(uint32_t timestamp) {
    if (isSavingToFlash) return; 
    
    isSavingToFlash = true;
    flashSavedTimestamp = timestamp;
    flashCurrentWriteAddress = 0;
    flashSampleCounter = 0;

    EEPROM.put(flashCurrentWriteAddress, flashSavedTimestamp);
    flashCurrentWriteAddress += sizeof(flashSavedTimestamp);

    int baseHistoryIndex = (storageIndex - FLASH_STORAGE_SIZE * 2) & RAM_STORAGE_MASK;
    lastFlashDiffValue = storageBuffer_diff[baseHistoryIndex];
    lastFlashWilsonValue = storageBuffer_wils[baseHistoryIndex];
  }

  void tickFlashBackgroundWorker() {
    if (!isSavingToFlash) return;
    if (storageIndex & 1) return; 

    int historicalIndex = (storageIndex - (FLASH_STORAGE_SIZE * 2) + (flashSampleCounter * 2)) & RAM_STORAGE_MASK;
    
    int32_t currentDiff = storageBuffer_diff[historicalIndex];
    int32_t currentWils = storageBuffer_wils[historicalIndex];

    int32_t deltaDiff = currentDiff - lastFlashDiffValue;
    int32_t deltaWils = currentWils - lastFlashWilsonValue;

    int8_t compressedDiffByte = (int8_t)constrain(deltaDiff, -128, 127);
    int8_t compressedWilsByte = (int8_t)constrain(deltaWils, -128, 127);

    EEPROM.update(flashCurrentWriteAddress++, compressedDiffByte);
    EEPROM.update(flashCurrentWriteAddress++, compressedWilsByte);

    lastFlashDiffValue = currentDiff;
    lastFlashWilsonValue = currentWils;

    flashSampleCounter++;

    if (flashSampleCounter >= FLASH_STORAGE_SIZE || flashCurrentWriteAddress >= EEPROM.length() - 2) {
      isSavingToFlash = false; 
    }
  }

  bool isFlashSavingBusy() {
    return isSavingToFlash;
  }
};
