#pragma once

#include "Arduino.h"
#include "Wire.h"

///////////////////////////////////////////
// Adafruit_BusIO shim — sim build.
//
// The ENTIRE hardware boundary of the real display stack
// (Adafruit_GFX -> Adafruit_GrayOLED -> Adafruit_SH110X) is the two
// BusIO device pointers inside Adafruit_GrayOLED. Every drawing
// operation lands in the in-RAM framebuffer via the REAL drawPixel();
// only display() pushes bytes through here. So: begin() reports the
// panel present, writes succeed and discard, and the framebuffer stays
// the single source of truth the sim exposes.
///////////////////////////////////////////

class Adafruit_I2CDevice {
 public:
  Adafruit_I2CDevice(uint8_t addr, TwoWire* wire = &Wire)
      : _addr(addr) {
    (void)wire;
  }

  bool begin(bool addr_detect = true) {
    (void)addr_detect;
    return true;
  }
  void end() {}

  uint8_t address() const { return _addr; }

  bool write(const uint8_t* buffer, size_t len, bool stop = true,
             const uint8_t* prefix_buffer = nullptr, size_t prefix_len = 0) {
    (void)buffer;
    (void)len;
    (void)stop;
    (void)prefix_buffer;
    (void)prefix_len;
    return true;
  }

  bool read(uint8_t* buffer, size_t len, bool stop = true) {
    (void)stop;
    if (buffer && len) memset(buffer, 0, len);
    return true;
  }

  bool write_then_read(const uint8_t* wbuf, size_t wlen, uint8_t* rbuf,
                       size_t rlen, bool stop = false) {
    (void)wbuf;
    (void)wlen;
    (void)stop;
    if (rbuf && rlen) memset(rbuf, 0, rlen);
    return true;
  }

  bool setSpeed(uint32_t desiredclk) {
    (void)desiredclk;
    return true;
  }

  // Chunk size for display() transfers. Matches the nRF52 core's I2C
  // buffer headroom; only affects how the (discarded) transfer is split,
  // never the framebuffer contents.
  size_t maxBufferSize() const { return 250; }

 private:
  uint8_t _addr;
};
