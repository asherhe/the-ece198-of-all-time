#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

// Timing diagnostic for Board.ino's real sensor-read path. It performs the
// same ECG ADC reads, MPU6050 I2C read, and MAX30102 FIFO drain as the real
// firmware, but instead of framing data it measures how long each call takes
// and prints, once per second, the worst-case (max) and total time spent in
// each operation during that second. This pinpoints which sensor call stalls
// the single-threaded loop (the suspected cause of ~1 s emission gaps).
//
// Upload with:
//   arduino-cli compile -b arduino:renesas_uno:minima \
//     --libraries C:/Users/asher/OneDrive/Documents/Arduino/libraries \
//     SerialAnalysis/BoardTiming
//   arduino-cli upload -b arduino:renesas_uno:minima -p COM8 SerialAnalysis/BoardTiming
// Then read COM8 at 115200 baud (plain text, newline-terminated).

static const uint32_t ECG_PERIOD_US = 5000;    // 200 Hz
static const uint32_t ACCEL_PERIOD_US = 20000; // 50 Hz

MAX30105 optical;
bool opticalReady = false;
uint32_t nextOpticalRetry = 0;
uint32_t nextEcg = 0, nextAccel = 0;
uint32_t nextAccelWake = 0;

// Per-second accumulators (microseconds).
struct Stat {
  uint32_t maxUs = 0;
  uint32_t totalUs = 0;
  uint32_t calls = 0;
  void add(uint32_t dt) { if (dt > maxUs) maxUs = dt; totalUs += dt; ++calls; }
  void reset() { maxUs = 0; totalUs = 0; calls = 0; }
};
Stat ecgStat, accelStat, opticalStat, loopStat;
uint32_t opticalSamplesThisSec = 0;
uint32_t maxCheckUs = 0, maxDrainUs = 0, maxAvail = 0;
uint32_t ecgSkips = 0, accelSkips = 0;
uint32_t lastReport = 0;

// ----- Replicated sensor reads from Board.ino -----
static void drainAdcCapacitor() {
#if defined(ADMUX)
  uint8_t prevMux = ADMUX;
  ADMUX = (prevMux & 0xF0) | 0x0F;
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  ADMUX = prevMux;
#endif
}

static uint16_t readIsolatedEcg(uint8_t pin) {
  drainAdcCapacitor();
  analogRead(pin);
  delayMicroseconds(250);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; ++i) sum += analogRead(pin);
  return uint16_t(sum >> 2);
}

static void sampleEcg() {
  volatile uint16_t a = readIsolatedEcg(A0);
  volatile uint16_t b = readIsolatedEcg(A1);
  (void)a; (void)b;
}

static void wakeAccel() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0);
  Wire.endTransmission(true);
}

static void sampleAccel() {
  Wire.beginTransmission(0x68); Wire.write(0x3b);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 6, true) != 6) return;
  uint8_t p[6];
  for (uint8_t i = 0; i < 3; ++i) { p[i * 2 + 1] = Wire.read(); p[i * 2] = Wire.read(); }
  if (!(p[0] | p[1] | p[2] | p[3] | p[4] | p[5])) {
    if ((int32_t)(millis() - nextAccelWake) >= 0) {
      wakeAccel();
      nextAccelWake = millis() + 1000;
    }
  }
}

uint32_t checkUs = 0, drainUs = 0, availAtCheck = 0;
static void sampleOptical() {
  if (!opticalReady) return;
  uint32_t c0 = micros();
  optical.check();
  checkUs = micros() - c0;
  availAtCheck = optical.available();
  uint32_t d0 = micros();
  while (optical.available()) {
    optical.getFIFORed(); optical.getFIFOIR(); // non-blocking: read FIFO tail
    optical.nextSample();
    ++opticalSamplesThisSec;
  }
  drainUs = micros() - d0;
}

void setup() {
  Serial.begin(115200);
  delay(3000);
#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(14);
#endif
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Wire.begin();
  wakeAccel();
  opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
  if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
  else nextOpticalRetry = millis() + 1000;

  Serial.println("BoardTiming ready. Columns are microseconds unless noted.");
  const uint32_t startUs = micros();
  nextEcg = startUs;
  nextAccel = startUs;
  lastReport = millis();
}

void loop() {
  const uint32_t loopStart = micros();

  const uint32_t nowMs = millis();
  if (!opticalReady && (int32_t)(nowMs - nextOpticalRetry) >= 0) {
    opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
    if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
    else nextOpticalRetry = nowMs + 1000;
  }

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextEcg) >= 0) {
    uint32_t t0 = micros();
    sampleEcg();
    ecgStat.add(micros() - t0);
    uint32_t after = micros();
    bool skip = (int32_t)(after - (nextEcg + ECG_PERIOD_US)) >= (int32_t)ECG_PERIOD_US;
    if (skip) ecgSkips++;
    nextEcg = skip ? (after + ECG_PERIOD_US) : (nextEcg + ECG_PERIOD_US);
  }

  nowUs = micros();
  if ((int32_t)(nowUs - nextAccel) >= 0) {
    uint32_t t0 = micros();
    sampleAccel();
    accelStat.add(micros() - t0);
    uint32_t after = micros();
    bool skip = (int32_t)(after - (nextAccel + ACCEL_PERIOD_US)) >= (int32_t)ACCEL_PERIOD_US;
    if (skip) accelSkips++;
    nextAccel = skip ? (after + ACCEL_PERIOD_US) : (nextAccel + ACCEL_PERIOD_US);
  }

  // Run optical every loop iteration (as the real firmware does), now using
  // the non-blocking FIFO accessors. If the fix is correct, per-second optical
  // time collapses from ~800 ms to near zero and no sampling slots are skipped.
  {
    uint32_t t0 = micros();
    sampleOptical();
    opticalStat.add(micros() - t0);
    if (checkUs > maxCheckUs) maxCheckUs = checkUs;
    if (drainUs > maxDrainUs) maxDrainUs = drainUs;
    if (availAtCheck > maxAvail) maxAvail = availAtCheck;
  }

  loopStat.add(micros() - loopStart);

  if (millis() - lastReport >= 1000) {
    lastReport += 1000;
    Serial.print("loop: n="); Serial.print(loopStat.calls);
    Serial.print(" max="); Serial.print(loopStat.maxUs);
    Serial.print(" | ECG max="); Serial.print(ecgStat.maxUs);
    Serial.print(" tot="); Serial.print(ecgStat.totalUs);
    Serial.print(" skips="); Serial.print(ecgSkips);
    Serial.print(" | ACCEL max="); Serial.print(accelStat.maxUs);
    Serial.print(" tot="); Serial.print(accelStat.totalUs);
    Serial.print(" skips="); Serial.print(accelSkips);
    Serial.print(" | OPT max="); Serial.print(opticalStat.maxUs);
    Serial.print(" tot="); Serial.print(opticalStat.totalUs);
    Serial.print(" samp="); Serial.print(opticalSamplesThisSec);
    Serial.print(" checkMax="); Serial.print(maxCheckUs);
    Serial.print(" drainMax="); Serial.print(maxDrainUs);
    Serial.print(" availMax="); Serial.print(maxAvail);
    Serial.print(" ready="); Serial.print(opticalReady);
    Serial.println();
    ecgStat.reset(); accelStat.reset(); opticalStat.reset(); loopStat.reset();
    opticalSamplesThisSec = 0; ecgSkips = 0; accelSkips = 0;
    maxCheckUs = 0; maxDrainUs = 0; maxAvail = 0;
  }
}
