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

void OLED::setup() {
  // I2C
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

// copied the following from ChatGPT

void OLED::loop() {
  unsigned long currentMillis = millis();

  // Update clock every second
  if (currentMillis - previousTimeUpdate >= 1000) {
    previousTimeUpdate = currentMillis;
    draw_frame(bigHeart);
  }

  // Animate heart independently and according to heart_rate
  unsigned long heartInterval;
  if (bigHeart) {
    heartInterval = (1600UL * 25UL) / heart_rate;
  } else {
    heartInterval = (800UL * 25UL) / heart_rate;
  }

  if (currentMillis - previousHeartUpdate >= heartInterval) {
    previousHeartUpdate = currentMillis;
    bigHeart = !bigHeart;

    draw_frame(bigHeart);
  }
}

// copying ends

void OLED::draw_frame(bool big_or_small) {
  display.clearDisplay();

  update_bpm();
  draw_heart(big_or_small);
  time_left();

  display.display();
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

  unsigned long hours{minutes / 60};
  minutes %= 60;

  // time is up
  if (workout_time <= millis()) {
    seconds = 0;
    minutes = 0;
    hours = 0;
  }

  display.setTextSize(2);

  // kind of repetitive
  display.setCursor(85, 8);
  if (hours < 10)
    display.print("0");
  display.print(hours);
  display.print(":");

  display.setCursor(85, 25);
  if (minutes < 10)
    display.print("0");
  display.print(minutes);
  display.print(":");

  display.setCursor(85, 42);
  if (seconds < 10)
    display.print("0");
  display.print(seconds);
}
