#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// SPI shim — sim build. The SD card is an in-memory VFS and the display
// is I2C, so nothing real flows through here; the SparkFun header just
// needs the types to exist.
///////////////////////////////////////////

#define SPI_MODE0 0

class SPISettings {
 public:
  SPISettings() {}
  SPISettings(uint32_t, uint8_t, uint8_t) {}
};

class SPIClass {
 public:
  void begin() {}
  void end() {}
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return 0xFF; }
  void transfer(void*, size_t) {}
};

extern SPIClass SPI;
