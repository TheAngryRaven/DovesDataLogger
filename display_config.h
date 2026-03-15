#ifndef _DOVES_DISPLAYCONFIG_H
#define _DOVES_DISPLAYCONFIG_H
  #ifndef I2C_DISPLAY_ADDRESS
    #define I2C_DISPLAY_ADDRESS 0x3C
  #endif

  #ifndef SCREEN_WIDTH
    #define SCREEN_WIDTH 128
  #endif

  #ifndef SCREEN_HEIGHT
    #define SCREEN_HEIGHT 64
  #endif

  #include <Adafruit_GFX.h>

  #ifdef USE_1306_DISPLAY
    #include <Adafruit_SSD1306.h>
    #define DISPLAY_TEXT_WHITE SSD1306_WHITE
    #define DISPLAY_TEXT_BLACK SSD1306_BLACK
    #define OLED_RESET -1 // reccomended to use reset pin, not required
    Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #else
    #include <Adafruit_SH110X.h>
    #define DISPLAY_TEXT_WHITE SH110X_WHITE
    #define DISPLAY_TEXT_BLACK SH110X_BLACK
    #define OLED_RESET -1
    Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
  #endif

  // Display sleep/wake macros (~10uA sleep vs 10-30mA active)
  // Framebuffer stays in controller RAM — no re-render needed on wake
  #ifdef USE_1306_DISPLAY
    #define DISPLAY_SLEEP() display.ssd1306_command(SSD1306_DISPLAYOFF)
    #define DISPLAY_WAKE()  display.ssd1306_command(SSD1306_DISPLAYON)
  #else
    #define DISPLAY_SLEEP() display.oled_command(SH110X_DISPLAYOFF)
    #define DISPLAY_WAKE()  display.oled_command(SH110X_DISPLAYON)
  #endif
#endif