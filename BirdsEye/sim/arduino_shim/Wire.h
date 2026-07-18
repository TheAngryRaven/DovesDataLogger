#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// Wire (I2C) shim — sim build.
// The display/IMU shims don't go through I2C at all; this exists so the
// firmware's bus-recovery code and the SparkFun/display headers compile.
// Every operation succeeds and moves no data.
///////////////////////////////////////////

class TwoWire : public Stream {
 public:
  void begin() {}
  void begin(uint8_t) {}
  void end() {}
  void setClock(uint32_t) {}
  void setPins(uint8_t, uint8_t) {}

  void beginTransmission(uint8_t) {}
  uint8_t endTransmission(bool = true) { return 0; }  // 0 = success
  uint8_t requestFrom(uint8_t, uint8_t, bool = true) { return 0; }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t) override { return 1; }
  using Print::write;
};

extern TwoWire Wire;
