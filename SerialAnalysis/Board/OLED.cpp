// @file OLED.cpp
// @author Philip Turturea
// @date 2026-11-09
// @version 1.0
// @since 1.0 Displays user heart rate (in BPM) inside a beating heart
//			  and a countdown timer until the end of the session
// @adapted from ECE 198 Lab 4 code (OLED initialization)
// 			and ADAfruit SSD1306 example code (bitmaps)

// testing observations:
// the OLED can use 3.3V or 5V
// the pixels are monochrome
// after making the pixel art in Krita, I used https://jlamch.net/MXChipWelcome/
// for the bitmaps fitting more information means reduced readability this code
// is suboptimal since the entire display is updated every single time; not just
// the parts that change

#include "OLED.hpp"

// libraries
#include <Adafruit_GFX.h>
#include <Arduino.h>

void OLED::begin() {
  // I2C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  Wire.setClock(400000);
  draw_frame(bigHeart);
}

// copied the following from ChatGPT

// Send one short I2C transaction per loop instead of blocking for a full
// 1024-byte framebuffer. Each chunk sets its own page and column address.
void OLED::sendDisplayChunk() {
  if (!displayPending) return;
  const uint8_t page = displayOffset / SCREEN_WIDTH;
  const uint8_t column = displayOffset % SCREEN_WIDTH;
  Wire.beginTransmission(0x3c);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x21); Wire.write(column); Wire.write(column + 15);
  Wire.write((uint8_t)0x22); Wire.write(page); Wire.write(page);
  Wire.endTransmission();
  Wire.beginTransmission(0x3c); Wire.write((uint8_t)0x40);
  Wire.write(display.getBuffer() + displayOffset, 16);
  Wire.endTransmission();
  displayOffset += 16;
  if (displayOffset == SCREEN_WIDTH * SCREEN_HEIGHT / 8) displayPending = false;
}

void OLED::loop() {
  if (displayPending) { sendDisplayChunk(); return; }
  unsigned long currentMillis = millis();
  bool redraw = false;
  if (currentMillis - previousTimeUpdate >= 1000) {
    previousTimeUpdate = currentMillis;
    redraw = true;
  }
  unsigned long heartInterval = heart_rate == 0 ? 1000 :
      (bigHeart ? (1600UL * 25UL) : (800UL * 25UL)) / heart_rate;
  if (currentMillis - previousHeartUpdate >= heartInterval) {
    previousHeartUpdate = currentMillis;
    bigHeart = !bigHeart;
    redraw = true;
  }
  if (redraw) draw_frame(bigHeart);
}

// copying ends

void OLED::draw_frame(bool big_or_small) {
  display.clearDisplay();

  update_bpm();
  draw_heart(big_or_small);
  time_left();
  draw_reps();

  // Host heartbeat indicator. A blinking 8 px square means disconnected.
  if (!connected && ((millis() / 500) & 1))
    display.fillRect(67, 20, 8, 8, SSD1306_WHITE);

  displayOffset = 0;
  displayPending = true;
}

void OLED::update_bpm() {
  if (heart_rate >= 100) {
    display.setCursor(15, 23);
  } else { // 2-digit heart rate
    display.setCursor(21, 23);
  }

  display.setTextSize(2);
  display.print(heart_rate);
}

void OLED::draw_heart(bool big_or_small) {
  unsigned char const *heart = big_heart;

  if (big_or_small == false) {
    heart = small_heart;
  }

  display.drawBitmap(0, (SCREEN_HEIGHT - HEART_HEIGHT), heart, HEART_WIDTH,
                     HEART_HEIGHT, 1);
}

void OLED::time_left() {
  unsigned long milliseconds = workout_time - millis();

  unsigned long seconds{milliseconds / 1000};

  unsigned long minutes{seconds / 60};
  seconds %= 60;

  // time is up
  if (workout_time <= millis()) {
    seconds = 0;
    minutes = 0;
  }

  display.setTextSize(2);

  display.setCursor(67, 0);
  if (minutes < 10) display.print("0");
  display.print(minutes);
  display.print(":");
  if (seconds < 10) display.print("0");
  display.print(seconds);
}

void OLED::draw_reps() {
  display.setTextSize(2);
  display.setCursor(67, 24);
  display.print("Reps:");
  // Five digits fit in the available width at size 1.
  display.setTextSize(2);
  display.setCursor(67, 40);
  display.print(reps);
}
