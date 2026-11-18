#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "OLED.hpp"

// Wire format is documented in ../README.md. All integers are little endian.
static const uint8_t ECG_TYPE = 1, ACCEL_TYPE = 2, OPTICAL_TYPE = 3;
static const uint8_t DISPLAY_TYPE = 0x80;
static const uint32_t HOST_TIMEOUT_MS = 3000;
MAX30105 optical;
OLED oled;
bool opticalReady = false;
uint16_t txSequence = 0;
uint32_t dropped[3] = {0, 0, 0};
uint32_t nextEcg = 0, nextAccel = 0, lastHost = 0;
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
  uint8_t frame[134];
  frame[0] = 0xa5; frame[1] = 0x5a; frame[2] = type;
  put16(frame + 3, txSequence++);
  put32(frame + 5, time);
  frame[9] = count;
  put16(frame + 10, len);
  memcpy(frame + 12, payload, len);
  put16(frame + 12 + len, crc16(frame + 2, 10 + len));
  const size_t size = 14 + len;
  if (Serial.availableForWrite() >= (int)size) Serial.write(frame, size);
  else if (type >= ECG_TYPE && type <= OPTICAL_TYPE) dropped[type - 1] += count;
}
static void flushEcg() { if (ecgCount) { sendFrame(ECG_TYPE, ecgFirst, ecgCount, ecgData, ecgCount * 4); ecgCount = 0; } }
static void flushAccel() { if (accelCount) { sendFrame(ACCEL_TYPE, accelFirst, accelCount, accelData, accelCount * 6); accelCount = 0; } }
static void flushOptical() { if (opticalCount) { sendFrame(OPTICAL_TYPE, opticalFirst, opticalCount, opticalData, opticalCount * 12); opticalCount = 0; } }

static void sampleEcg(uint32_t t) {
  if (!ecgCount) ecgFirst = t;
  uint8_t *p = ecgData + ecgCount * 4;
  put16(p, analogRead(A0)); put16(p + 2, analogRead(A1));
  if (++ecgCount == 8) flushEcg();
}
static void sampleAccel(uint32_t t) {
  Wire.beginTransmission(0x68); Wire.write(0x3b);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 6, true) != 6) return;
  if (!accelCount) accelFirst = t;
  uint8_t *p = accelData + accelCount * 6;
  for (uint8_t i = 0; i < 3; ++i) { p[i * 2 + 1] = Wire.read(); p[i * 2] = Wire.read(); }
  if (++accelCount == 5) flushAccel();
}
static void sampleOptical() {
  if (!opticalReady) return;
  optical.check();
  while (optical.available()) {
    uint32_t t = micros();
    if (!opticalCount) opticalFirst = t;
    uint8_t *p = opticalData + opticalCount * 12;
    put32(p, t); put32(p + 4, optical.getRed()); put32(p + 8, optical.getIR());
    optical.nextSample();
    if (++opticalCount == 4) flushOptical();
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
  Wire.begin();
  Wire.beginTransmission(0x68); Wire.write(0x6b); Wire.write(0); Wire.endTransmission(true);
  opticalReady = optical.begin(Wire, I2C_SPEED_FAST);
  if (opticalReady) optical.setup(60, 2, 2, 100, 118, 4096);
  oled.begin(); oled.heart_rate = 0; oled.workout_time = 0;
  nextEcg = nextAccel = micros();
}
void loop() {
  receiveDisplay();
  uint32_t now = micros();
  if ((int32_t)(now - nextEcg) >= 0) { sampleEcg(nextEcg); nextEcg += 5000; }
  if ((int32_t)(now - nextAccel) >= 0) { sampleAccel(nextAccel); nextAccel += 20000; }
  sampleOptical();
  if (oled.connected && millis() - lastHost > HOST_TIMEOUT_MS) {
    oled.connected = false; oled.heart_rate = 0; oled.workout_time = millis();
  }
  oled.loop();
}
