#pragma once

#include "Adafruit_GFX.h"
#include "Wire.h"

///////////////////////////////////////////
// Adafruit_SH110X shim — PHASE-1 PLACEHOLDER ONLY (see Adafruit_GFX.h).
// The sketch's SH1106G path compiles against this; Phase 2 swaps in the
// real pinned library for pixel-perfect output.
///////////////////////////////////////////

#define SH110X_WHITE 1
#define SH110X_BLACK 0

#define SH110X_DISPLAYOFF 0xAE
#define SH110X_DISPLAYON 0xAF

class Adafruit_SH110X : public Adafruit_GFX {
 public:
  Adafruit_SH110X(int16_t w, int16_t h, TwoWire* wire, int8_t rst)
      : Adafruit_GFX(w, h) {
    (void)wire;
    (void)rst;
  }

  bool begin(uint8_t addr, bool reset) {
    (void)addr;
    (void)reset;
    return true;
  }

  void display() {}
  void clearDisplay() {}
  void oled_command(uint8_t c) { (void)c; }
};

class Adafruit_SH1106G : public Adafruit_SH110X {
 public:
  using Adafruit_SH110X::Adafruit_SH110X;
};
