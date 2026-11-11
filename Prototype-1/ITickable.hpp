#pragma once

class ITickable {
public:
  virtual ~ITickable() = default;
  virtual uint16_t getFreq() = 0;         // called once by the scheduler
  virtual void onTick() = 0;              // called during clock tick
};