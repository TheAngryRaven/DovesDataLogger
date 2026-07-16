#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// Adafruit_GFX shim — PHASE-1 PLACEHOLDER ONLY.
//
// Phase 2 of the sim plan replaces this with the REAL Adafruit_GFX /
// Adafruit_SH110X libraries (pinned sources, BusIO no-op shim) so the
// framebuffer is pixel-perfect. This placeholder just satisfies the
// render calls so the firmware boots; it draws nothing.
///////////////////////////////////////////

class Adafruit_GFX : public Print {
 public:
  Adafruit_GFX(int16_t w, int16_t h) : _width(w), _height(h) {}

  void setCursor(int16_t x, int16_t y) {
    _cx = x;
    _cy = y;
  }
  void setTextSize(uint8_t s) { _ts = s; }
  void setTextColor(uint16_t c) { (void)c; }
  void setTextColor(uint16_t c, uint16_t bg) {
    (void)c;
    (void)bg;
  }
  void setTextWrap(bool w) { (void)w; }
  void invertDisplay(bool i) { (void)i; }

  void fillRect(int16_t, int16_t, int16_t, int16_t, uint16_t) {}
  void drawFastHLine(int16_t, int16_t, int16_t, uint16_t) {}
  void drawBitmap(int16_t, int16_t, const uint8_t*, int16_t, int16_t,
                  uint16_t) {}

  // Classic-font metrics (6x8 per char at size 1) — close enough for the
  // placeholder's layout math; the real GFX takes over in Phase 2.
  void getTextBounds(const char* str, int16_t x, int16_t y, int16_t* x1,
                     int16_t* y1, uint16_t* w, uint16_t* h) {
    size_t len = str ? strlen(str) : 0;
    if (x1) *x1 = x;
    if (y1) *y1 = y;
    if (w) *w = (uint16_t)(len * 6 * _ts);
    if (h) *h = (uint16_t)(8 * _ts);
  }
  void getTextBounds(const __FlashStringHelper* str, int16_t x, int16_t y,
                     int16_t* x1, int16_t* y1, uint16_t* w, uint16_t* h) {
    getTextBounds(reinterpret_cast<const char*>(str), x, y, x1, y1, w, h);
  }

  // Print funnel — placeholder discards glyphs.
  size_t write(uint8_t) override { return 1; }
  using Print::write;

 protected:
  int16_t _width, _height;
  int16_t _cx = 0, _cy = 0;
  uint8_t _ts = 1;
};
