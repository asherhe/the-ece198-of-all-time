#include <Arduino.h>
#include <EEPROM.h>

// disable EEPROM for prototype to save memory lifespan
#define DISABLE_EEPROM_WRITE

struct ECGRaw {
  int diff_node;   
  int wilson_node; 
};

class ECG : public ITickable {
private:
  uint8_t pinDiff;
  uint8_t pinWils;

  // Power-of-2 RAM window sizes for high-speed bitwise masking (& 2047)
  static const int RAM_STORAGE_SIZE = 2048;
  static const int RAM_STORAGE_MASK = RAM_STORAGE_SIZE - 1;
  static const int FLASH_STORAGE_SIZE = 1024;

  // Q28 Fixed-Point Butterworth Filter Coefficients (40 Hz LP @ 200 Hz fs)
  const int64_t b10 = 24511874, b11 = 49023748, b12 = 24511874;
  const int64_t a11 = -263375836, a12 = 92986877;
  const int64_t b20 = 40044810,  b21 = 80089620,  b22 = 40044810;
  const int64_t a21 = -143201402, a22 = 34946300;

  // Direct mathematical filter history structures
  int64_t iir_D1 = {0}; int64_t iir_D2 = {0};
  int64_t iir_W1 = {0}; int64_t iir_W2 = {0};

  // Q28 Linear-Phase FIR High-Pass Configuration (0.5 Hz HP @ 200 Hz fs)
  static const int FIR_TAPS = 201; 
  int32_t h[FIR_TAPS];               

  int32_t firBuffer_diff[FIR_TAPS] = {0};
  int32_t firBuffer_wils[FIR_TAPS] = {0};
  int firIndex = 0;

  // Circular buffers holding the running 10-second history window
  int32_t storageBuffer_diff[RAM_STORAGE_SIZE];
  int32_t storageBuffer_wils[RAM_STORAGE_SIZE];
  int storageIndex = 0;

  // Asynchronous background flash worker state variables
  bool isSavingToFlash = false;
  uint32_t flashSavedTimestamp = 0;
  int flashCurrentWriteAddress = 0;
  int flashSampleCounter = 0;
  int32_t lastFlashDiffValue = 0;
  int32_t lastFlashWilsonValue = 0;

  // --- Automation Scheduling Variables ---
  uint32_t intervalMs = 0;          // 0 means interval logging is disabled
  uint32_t lastIntervalTriggerMs = 0;
  
  uint32_t specificTargetTimeMs = 0;// 0 means specific timestamp target is disabled
  bool specificTimeTriggered = false;

  volatile bool sampleFlag = false;

  // Computes the linear-phase Hamming-windowed Sinc array into Q28 integer space
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

  // Protects the IIR math pipeline against asymmetric negative bit-shift truncation drift
  inline int32_t shiftAndRoundQ28(int64_t value) {
    return (int32_t)((value + 134217728LL) >> 28);
  }

  // Iterates the flash streaming engine on a 100 Hz cadence during raw sample collection
  void tickFlashBackgroundWorker() {
    if (!isSavingToFlash) return;
#ifndef DISABLE_EEPROM_WRITE
    if (storageIndex & 1) return; // Downsample 200 Hz processing to a 100 Hz flash save cadence

    int historicalIndex = (storageIndex - (FLASH_STORAGE_SIZE * 2) + (flashSampleCounter * 2)) & RAM_STORAGE_MASK;
    
    int32_t currentDiff = storageBuffer_diff[historicalIndex];
    int32_t currentWils = storageBuffer_wils[historicalIndex];

    // First-Difference delta computation
    int32_t deltaDiff = currentDiff - lastFlashDiffValue;
    int32_t deltaWils = currentWils - lastFlashWilsonValue;

    // Packs constraints tightly into compressed signed 1-byte blocks
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
#endif
  }

  // Checks time trackers to automatically flag and start flash dumps
  void checkAutomationSchedules() {
    if (isSavingToFlash) return; // Flash engine is currently busy
    
    uint32_t currentMs = millis();

    // 1. Evaluate Periodic Interval Trigger (e.g., every 5 minutes)
    if (intervalMs > 0) {
      if (currentMs - lastIntervalTriggerMs >= intervalMs) {
        lastIntervalTriggerMs = currentMs; // Step clock reference forward
        triggerFlashSave(currentMs);
        return; 
      }
    }

    // 2. Evaluate Preprogrammed Target Timestamp (e.g., exactly 1 minute into operation)
    if (specificTargetTimeMs > 0 && !specificTimeTriggered) {
      if (currentMs >= specificTargetTimeMs) {
        specificTimeTriggered = true; // Burn-once lock flag
        triggerFlashSave(currentMs);
      }
    }
  }

public:
  ECG(uint8_t diff_pin, uint8_t wilson_pin) : pinDiff(diff_pin), pinWils(wilson_pin) {}

  void begin() {
    generateFixedPointFIR();
    
    // Warm-start preloading solution to eliminate the 1-second startup filter spike
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

    lastIntervalTriggerMs = millis();
  }

  // Set an automated recurring save interval in milliseconds
  void setScheduleInterval(uint32_t periodicIntervalMs) {
    intervalMs = periodicIntervalMs;
    lastIntervalTriggerMs = millis();
  }

  // Set a one-time preprogrammed time since boot to capture data
  void setScheduledTime(uint32_t preprogrammedTimeMs) {
    specificTargetTimeMs = preprogrammedTimeMs;
    specificTimeTriggered = false;
  }

  // ITickable Interface compliance definitions
  uint16_t getFreq() override { return 200; }
  void onTick() override { sampleFlag = true; }

  // Main system execution block evaluating paced hardware filter steps
  void loop() {
    // Continually check clocks for automated recording schedules
    checkAutomationSchedules();

    if (sampleFlag) {
      sampleFlag = false;

      int32_t rawDiff = analogRead(pinDiff);
      int32_t rawWils = analogRead(pinWils);

      // Cascaded 4th-Order Butterworth filtering stage
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

      // Branchless unrolled FIR dot-product step calculation 
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

      // Save baseline-corrected calculations directly to RAM storage
      storageBuffer_diff[storageIndex] = shiftAndRoundQ28(acc_diff);
      storageBuffer_wils[storageIndex] = shiftAndRoundQ28(acc_wils);
      storageIndex = (storageIndex + 1) & RAM_STORAGE_MASK;

      // Processes background data flash burning steps asynchronously 
      tickFlashBackgroundWorker();
    }
  }

  // Recovers the most recent processed signal coordinate frame from RAM
  ECGRaw read_raw() {
    int lastIndex = (storageIndex - 1) & RAM_STORAGE_MASK;
    return { (int)storageBuffer_diff[lastIndex], (int)storageBuffer_wils[lastIndex] };
  }

  // Initializes the asynchronous 10-second history recording queue
  void triggerFlashSave(uint32_t timestamp) {
#ifndef DISABLE_EEPROM_WRITE
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
#endif
  }

  bool isFlashSavingBusy() {
    return isSavingToFlash;
  }
};