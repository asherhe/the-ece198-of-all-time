#include <Arduino.h>
#include <Wire.h>
#include "OLED.hpp"

// Synthetic version of Board/Board.ino. Wire format is documented in ../README.md.
// All integers are little endian. No ECG, accelerometer, or optical sensor is read.
static const uint8_t ECG_TYPE = 1, ACCEL_TYPE = 2, OPTICAL_TYPE = 3;
static const uint8_t DISPLAY_TYPE = 0x80;
static const uint32_t HOST_TIMEOUT_MS = 3000;
OLED oled;
uint16_t txSequence = 0;
uint32_t dropped[3] = {0, 0, 0};
uint32_t nextEcg = 0, nextAccel = 0, nextOptical = 0, lastHost = 0;
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

// Deterministic 75 BPM waveforms make recorded sessions reproducible.
// Pulse positions and widths are in milliseconds; values are raw sensor counts.
static int32_t triangle(uint16_t phase, uint16_t center, uint16_t halfWidth, int32_t height) {
  uint16_t distance = phase > center ? phase - center : center - phase;
  if (distance >= halfWidth) return 0;
  return height * (halfWidth - distance) / halfWidth;
}

static uint16_t dummyEcg(uint32_t t, uint16_t phaseOffset) {
  uint16_t phase = uint16_t((t / 1000UL + phaseOffset) % 800UL);
  int32_t value = 8192;
  value += triangle(phase, 130, 45, 120); // P wave
  value += triangle(phase, 305, 15, -180); // Q wave
  value += triangle(phase, 330, 12, 1100); // R wave
  value += triangle(phase, 355, 18, -260); // S wave
  value += triangle(phase, 510, 90, 220); // T wave
  return uint16_t(value);
}

static void sampleEcg(uint32_t t) {
  if (!ecgCount) ecgFirst = t;
  uint8_t *p = ecgData + ecgCount * 4;
  put16(p, dummyEcg(t, 0));
  put16(p + 2, dummyEcg(t, 8));
  if (++ecgCount == 8) flushEcg();
}
static void sampleAccel(uint32_t t) {
  if (!accelCount) accelFirst = t;
  uint8_t *p = accelData + accelCount * 6;
  uint16_t phase = uint16_t((t / 1000UL) % 800UL);
  // MPU6050-style signed counts, with Z close to one g (16384 counts).
  put16(p, uint16_t(int16_t(triangle(phase, 280, 180, 1400) - 250)));
  put16(p + 2, uint16_t(int16_t(triangle(phase, 510, 220, 900) - 200)));
  put16(p + 4, uint16_t(int16_t(16384 + triangle(phase, 320, 130, 1200))));
  if (++accelCount == 5) flushAccel();
}
static void sampleOptical(uint32_t t) {
  if (!opticalCount) opticalFirst = t;
  uint8_t *p = opticalData + opticalCount * 12;
  uint16_t phase = uint16_t((t / 1000UL) % 800UL);
  int32_t pulse = triangle(phase, 190, 145, 5000);
  put32(p, t);
  put32(p + 4, uint32_t(90000 + pulse * 54 / 100));
  put32(p + 8, uint32_t(100000 + pulse));
  if (++opticalCount == 4) flushOptical();
}

static uint8_t rx[32];
static uint8_t rxSize = 0;
static void receiveDisplay() {
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    if (rxSize == 0 && byte != 0xa5) continue;
    if (rxSize == 1 && byte != 0x5a) { rxSize = 0; continue; }
    rx[rxSize++] = byte;
    if (rxSize == 12 && (rx[2] != DISPLAY_TYPE ||
        (get16(rx + 10) != 7 && get16(rx + 10) != 9))) rxSize = 0;
    if (rxSize >= 12 && rxSize == 14 + get16(rx + 10)) {
      const uint16_t payloadSize = get16(rx + 10);
      if (crc16(rx + 2, 10 + payloadSize) == get16(rx + 12 + payloadSize)) {
        uint16_t bpm = get16(rx + 12);
        oled.heart_rate = (rx[18] & 1) ? bpm : 0;
        oled.workout_time = millis() + get32(rx + 14) * 1000UL;
        oled.reps = payloadSize == 9 ? get16(rx + 19) : 0;
        oled.connected = true;
        lastHost = millis();
      }
      rxSize = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  // The only I2C peripheral used by this sketch is the OLED.
  Wire.begin();
  oled.begin(); oled.heart_rate = 0; oled.workout_time = 0;
  nextEcg = nextAccel = nextOptical = micros();
}

void loop() {
  receiveDisplay();
  uint32_t now = micros();
  if ((int32_t)(now - nextEcg) >= 0) { sampleEcg(nextEcg); nextEcg += 5000; }
  if ((int32_t)(now - nextAccel) >= 0) { sampleAccel(nextAccel); nextAccel += 20000; }
  if ((int32_t)(now - nextOptical) >= 0) { sampleOptical(nextOptical); nextOptical += 10000; }
  if (oled.connected && millis() - lastHost > HOST_TIMEOUT_MS) {
    oled.connected = false; oled.heart_rate = 0; oled.workout_time = millis();
  }
  oled.loop();
}
