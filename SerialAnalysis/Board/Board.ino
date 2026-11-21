#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "OLED.hpp"

// Wire format is documented in ../README.md. All integers are little endian.
static const uint8_t ECG_TYPE = 1, ACCEL_TYPE = 2, OPTICAL_TYPE = 3;
static const uint8_t DISPLAY_TYPE = 0x80;
static const uint32_t HOST_TIMEOUT_MS = 3000;
static const uint32_t ECG_PERIOD_US = 5000;    // 200 Hz
static const uint32_t ACCEL_PERIOD_US = 20000; // 50 Hz
static const uint32_t OPTICAL_PERIOD_US = 10000; // 100 Hz FIFO service rate
const int32_t SAMPLE_TIMEOUT = 3000000; // 3 seconds
MAX30105 optical;
OLED oled;
bool opticalReady = false;
uint32_t nextOpticalRetry = 0;
uint16_t txSequence = 0;
uint32_t dropped[3] = {0, 0, 0};
uint32_t nextEcg = 0, nextAccel = 0, lastHost = 0;
uint32_t nextAccelWake = 0;
uint32_t nextOptical = 0;
// Keep each complete frame below a typical 64-byte USB CDC write buffer.
uint8_t ecgData[32], accelData[30], opticalData[48];
uint8_t ecgCount = 0, accelCount = 0, opticalCount = 0;
uint32_t ecgFirst = 0, accelFirst = 0, opticalFirst = 0;

static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static uint16_t get16(const uint8_t *p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
static uint32_t get32(const uint8_t *p) { return uint32_t(get16(p)) | (uint32_t(get16(p + 2)) << 16); }
static uint16_t crc16(const uint8_t *p, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= uint16_t(p[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
static void sendFrame(uint8_t type, uint32_t time, uint8_t count, const uint8_t *payload, uint16_t len) {
  const size_t size = 14 + len;
  if (Serial.availableForWrite() < (int)size) {
    if (type >= ECG_TYPE && type <= OPTICAL_TYPE) dropped[type - 1] += count;
    return;
  }
  uint8_t frame[134];
  const uint16_t seq = txSequence++;
  frame[0] = 0xa5; frame[1] = 0x5a; frame[2] = type;
  put16(frame + 3, seq);
  put32(frame + 5, time);
  frame[9] = count;
  put16(frame + 10, len);
  memcpy(frame + 12, payload, len);
  put16(frame + 12 + len, crc16(frame + 2, 10 + len));
  Serial.write(frame, size);
}
static void flushEcg() { if (ecgCount) { sendFrame(ECG_TYPE, ecgFirst, ecgCount, ecgData, ecgCount * 4); ecgCount = 0; } }
static void flushAccel() { if (accelCount) { sendFrame(ACCEL_TYPE, accelFirst, accelCount, accelData, accelCount * 6); accelCount = 0; } }
static void flushOptical() { if (opticalCount) { sendFrame(OPTICAL_TYPE, opticalFirst, opticalCount, opticalData, opticalCount * 12); opticalCount = 0; } }

// Discharges internal ADC sample-and-hold capacitor via internal MUX GND (no extra pins used)
static void drainAdcCapacitor() {
#if defined(ADMUX)
  uint8_t prevMux = ADMUX;
  ADMUX = (prevMux & 0xF0) | 0x0F; // Connect internal ADC MUX directly to chip GND (0V)
  ADCSRA |= (1 << ADSC);           // Start dummy conversion to discharge capacitor
  while (ADCSRA & (1 << ADSC));    // Wait for discharge to finish
  ADMUX = prevMux;
#endif
}

// Isolates A0 and A1 without using any extra analog pins
static uint16_t readIsolatedEcg(uint8_t pin) {
  drainAdcCapacitor();      // Discharge leftover voltage from previous channel inside the chip
  analogRead(pin);          // Switch MUX to target pin
  delayMicroseconds(250);   // Allow high-impedance Wilson network to charge ADC capacitor

  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    sum += analogRead(pin);
  }
  return uint16_t(sum >> 2);
}

static void sampleEcg(uint32_t t) {
  if (!ecgCount) ecgFirst = t;
  uint8_t *p = ecgData + ecgCount * 4;
  put16(p, readIsolatedEcg(A0));
  put16(p + 2, readIsolatedEcg(A1));
  if (++ecgCount == 8) flushEcg();
}
static void wakeAccel() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6b);
  Wire.write(0);
  Wire.endTransmission(true);
}
static void sampleAccel(uint32_t t) {
  Wire.beginTransmission(0x68); Wire.write(0x3b);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 6, true) != 6) return;
  uint8_t *p = accelData + accelCount * 6;
  for (uint8_t i = 0; i < 3; ++i) { p[i * 2 + 1] = Wire.read(); p[i * 2] = Wire.read(); }
  if (!(p[0] | p[1] | p[2] | p[3] | p[4] | p[5])) {
    // Six zero bytes are not a resting acceleration; retry a missed wake-up.
    if ((int32_t)(millis() - nextAccelWake) >= 0) {
      wakeAccel();
      nextAccelWake = millis() + 1000;
    }
    return;
  }
  if (!accelCount) accelFirst = t;
  if (++accelCount == 5) flushAccel();
}
static void sampleOptical() {
  if (!opticalReady) return;
  optical.check();
  uint8_t idx = 0;
  while (optical.available()) {
    uint32_t t = (idx == 0) ? micros() : (opticalFirst + uint32_t(idx) * 10000UL);
    if (!opticalCount) opticalFirst = t;
    uint8_t *p = opticalData + opticalCount * 12;
    // Use the non-blocking FIFO-tail accessors. getRed()/getIR() call the
    // library's safeCheck(250), which busy-waits with delay(1) for up to
    // 250 ms per call and stalls the single-threaded loop (~80 ms per optical
    // servicing was observed), interrupting serial emission. getFIFORed()/
    // getFIFOIR() read the already-fetched FIFO sample without any wait.
    put32(p, t); put32(p + 4, optical.getFIFORed()); put32(p + 8, optical.getFIFOIR());
    optical.nextSample();
    ++idx;
    if (++opticalCount == 4) {
      flushOptical();
      idx = 0;
    }
  }
}

static uint8_t rx[32];
static uint8_t rxSize = 0;
static void receiveDisplay() {
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    if (rxSize == 0 && byte != 0xa5) continue;
    if (rxSize == 1 && byte != 0x5a) { rxSize = 0; continue; }
    rx[rxSize++] = byte;
    if (rxSize == 12 && (rx[2] != DISPLAY_TYPE || get16(rx + 10) != 7)) rxSize = 0;
    if (rxSize == 21) {
      if (crc16(rx + 2, 17) == get16(rx + 19)) {
        uint16_t bpm = get16(rx + 12);
        oled.heart_rate = (rx[18] & 1) ? bpm : 0;
        oled.workout_time = millis() + get32(rx + 14) * 1000UL;
        oled.connected = true;
        lastHost = millis();
      }
      rxSize = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // The working Prototype-1 waits before initializing its I2C devices.
  // Let the display and sensor power supplies settle after power-up.
  delay(3000);

#if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(14);
#endif

  // configure analog ports
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // configure accelerometer
  Wire.begin();
  wakeAccel();
  // configure oximeter
  opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
  if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
  else nextOpticalRetry = millis() + 1000;
  // configure OLED
  oled.heart_rate = 0; oled.workout_time = 0; oled.begin();

  // setup timers
  const uint32_t startUs = micros();
  nextEcg = startUs;
  nextAccel = startUs;
  nextOptical = startUs;
}

void loop() {
  receiveDisplay();

  const uint32_t nowMs = millis();
  if (!opticalReady && (int32_t)(nowMs - nextOpticalRetry) >= 0) {
    opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
    if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
    else nextOpticalRetry = nowMs + 1000;
  }

  const uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextEcg) >= 0) {
    sampleEcg(nowUs);
    nextEcg = ((int32_t)(nowUs - (nextEcg + ECG_PERIOD_US)) >= (int32_t)ECG_PERIOD_US)
                ? (nowUs + ECG_PERIOD_US)
                : (nextEcg + ECG_PERIOD_US);
  }
  if ((int32_t)(nowUs - nextAccel) >= 0) {
    sampleAccel(nowUs);
    nextAccel = ((int32_t)(nowUs - (nextAccel + ACCEL_PERIOD_US)) >= (int32_t)ACCEL_PERIOD_US)
                  ? (nowUs + ACCEL_PERIOD_US)
                  : (nextAccel + ACCEL_PERIOD_US);
  }
  if ((int32_t)(nowUs - nextOptical) >= 0) {
    sampleOptical();
    nextOptical = ((int32_t)(nowUs - (nextOptical + OPTICAL_PERIOD_US)) >= (int32_t)OPTICAL_PERIOD_US)
                    ? (nowUs + OPTICAL_PERIOD_US)
                    : (nextOptical + OPTICAL_PERIOD_US);
  }

  if (oled.connected && millis() - lastHost > HOST_TIMEOUT_MS) {
    oled.connected = false; oled.heart_rate = 0; oled.workout_time = millis();
  }
  oled.loop();
}
