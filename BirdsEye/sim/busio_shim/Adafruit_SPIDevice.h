#pragma once

#include "Arduino.h"
#include "SPI.h"

///////////////////////////////////////////
// Adafruit_BusIO SPI shim — sim build. The display runs on I2C; this
// exists because Adafruit_GrayOLED's SPI constructors reference the
// type. Nothing ever flows through it. (See Adafruit_I2CDevice.h.)
///////////////////////////////////////////

typedef enum {
  SPI_BITORDER_MSBFIRST = 0,
  SPI_BITORDER_LSBFIRST = 1,
} BusIOBitOrder;

class Adafruit_SPIDevice {
 public:
  // Hardware SPI
  Adafruit_SPIDevice(int8_t cspin, uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0, SPIClass* theSPI = &SPI) {
    (void)cspin;
    (void)freq;
    (void)dataOrder;
    (void)dataMode;
    (void)theSPI;
  }
  // Software (bit-bang) SPI
  Adafruit_SPIDevice(int8_t cspin, int8_t sckpin, int8_t misopin,
                     int8_t mosipin, uint32_t freq = 1000000,
                     BusIOBitOrder dataOrder = SPI_BITORDER_MSBFIRST,
                     uint8_t dataMode = SPI_MODE0) {
    (void)cspin;
    (void)sckpin;
    (void)misopin;
    (void)mosipin;
    (void)freq;
    (void)dataOrder;
    (void)dataMode;
  }

  bool begin() { return true; }

  bool write(const uint8_t* buffer, size_t len,
             const uint8_t* prefix_buffer = nullptr, size_t prefix_len = 0) {
    (void)buffer;
    (void)len;
    (void)prefix_buffer;
    (void)prefix_len;
    return true;
  }
};
