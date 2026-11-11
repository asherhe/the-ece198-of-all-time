  // Replace the original tickFlashBackgroundWorker() method with this implementation:
  void tickFlashBackgroundWorker() {
    if (!isSavingToFlash) return;

    // Maintain 100 Hz downsampling pace from the 200 Hz loop
    if (storageIndex & 1) return; 

    // 1. Locate the current historical sample frame to process
    int historicalIndex = (storageIndex - (FLASH_STORAGE_SIZE * 2) + (flashSampleCounter * 2)) & RAM_STORAGE_MASK;
    
    int32_t currentDiff = storageBuffer_diff[historicalIndex];
    int32_t currentWils = storageBuffer_wils[historicalIndex];

    // 2. Compute first-derivative slopes to detect signal velocity/bursts
    int32_t deltaDiff = currentDiff - lastFlashDiffValue;
    int32_t deltaWils = currentWils - lastFlashWilsonValue;

    // Define a mathematical threshold for high-velocity heartbeat events (e.g., QRS complex)
    const int32_t ACTIVITY_THRESHOLD = 15; 
    bool highActivity = (abs(deltaDiff) > ACTIVITY_THRESHOLD) || (abs(deltaWils) > ACTIVITY_THRESHOLD);

    if (!highActivity && flashSampleCounter < (FLASH_STORAGE_SIZE - 1)) {
      // --- Baseline Template State: Signal is flat/resting ---
      static uint8_t flatRunCount = 0;
      flatRunCount++;

      // If we hit the max count for a single byte, or the next sample bursts, commit the run
      if (flatRunCount == 255) {
        EEPROM.update(flashCurrentWriteAddress++, 0x00);        // 0x00 token indicates a flat baseline run length follows
        EEPROM.update(flashCurrentWriteAddress++, flatRunCount); // Store the duration of the flat line
        flatRunCount = 0;
      }
      
      // Retain coordinates to keep track of the moving baseline trend
      lastFlashDiffValue = currentDiff;
      lastFlashWilsonValue = currentWils;
    } 
    else {
      // --- Event State: High-velocity heartbeat detected, dump full resolution ---
      
      // Token 0xFF flags the reader that a raw high-resolution heartbeat frame is being logged
      EEPROM.update(flashCurrentWriteAddress++, 0xFF); 

      // Pack differences into 1-byte blocks safely
      int8_t compressedDiffByte = (int8_t)constrain(deltaDiff, -128, 127);
      int8_t compressedWilsByte = (int8_t)constrain(deltaWils, -128, 127);

      EEPROM.update(flashCurrentWriteAddress++, compressedDiffByte);
      EEPROM.update(flashCurrentWriteAddress++, compressedWilsByte);

      lastFlashDiffValue = currentDiff;
      lastFlashWilsonValue = currentWils;
    }

    flashSampleCounter++;

    // Safety check to stop when the full 10-second history has been compressed
    if (flashSampleCounter >= FLASH_STORAGE_SIZE || flashCurrentWriteAddress >= EEPROM.length() - 3) {
      isSavingToFlash = false; 
    }
  }
