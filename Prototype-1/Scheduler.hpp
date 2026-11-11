#pragma once

#include <Arduino.h>
#include "ITickable.hpp"
#include "FspTimer.h"

#define SCHED_MAX_DEVICES 4

class Scheduler {
private:
  FspTimer masterTimer;
  uint16_t masterFreqHz;
  volatile bool tickFlag = false;
  
  ITickable *devices[SCHED_MAX_DEVICES];
  uint8_t tickInterval[SCHED_MAX_DEVICES]; // number of master ticks to each device tick
  uint8_t tickCountdown[SCHED_MAX_DEVICES]; // remaining master ticks before next onTick
  uint8_t deviceCount = 0;

  static void timerCallback(timer_callback_args_t *arg) {
    Scheduler* instance = const_cast<Scheduler*>(static_cast<const Scheduler*>(arg->p_context));
    if (instance) {
      instance->tickFlag = true;
    }
  }
public:
  void begin(uint16_t frequencyHz) {
    masterFreqHz = frequencyHz;

    uint8_t timerType = GPT_TIMER;
    int8_t channel = FspTimer::get_available_timer(timerType);
    if (channel < 0) return;

    timer_mode_t mode = TIMER_MODE_PERIODIC;
    masterTimer.begin(mode, timerType, channel, frequencyHz, 50.0f, timerCallback, this);
    masterTimer.setup_overflow_irq();
    masterTimer.open();
    masterTimer.start();
  }

  bool registerDevice(ITickable *device) {
    if (deviceCount < SCHED_MAX_DEVICES) {
      devices[deviceCount] = device;
      tickInterval[deviceCount] = masterFreqHz / device->getFreq();
      tickCountdown[deviceCount] = tickInterval[deviceCount];
      ++deviceCount;
      return true;
    }
    return false;
  }

  void loop() {
    if (tickFlag) {
      tickFlag = false;
      for (uint8_t i = 0; i < deviceCount; ++i) {
        if (--tickCountdown[i] == 0) {
          tickCountdown[i] = tickInterval[i];
          devices[i]->onTick();
        }
      }
    }
  }
};
