#pragma once

///////////////////////////////////////////
// DISPLAY / UI MODULE
// Display bring-up, I2C bus recovery, button handling (multi-sample
// debounce + edge detection), menu dispatch, page routing.
///////////////////////////////////////////

#include "project.h"  // ButtonState

// One-shot bus recovery: bit-bang 9 SCL clocks to free a stuck slave
// after an EMI-induced hang, then re-init Wire and the display.
void i2cBusRecover();

// Wrapper around display.display() that flags I2C recovery if the
// transfer takes >100ms (well above the normal ~25ms at 400 kHz).
void safeDisplayUpdate();

// Bring up the OLED, set 400 kHz I2C clock, init button pins, show
// the boot splash. Call once from setup().
void displaySetup();

// Helpers callable from any displayPage_*() function.
void resetDisplay();
void forceDisplayRefresh();
void switchToDisplayPage(int newDisplayPage);

// Button helpers.
void setupButtons();
void readButtons();
void resetButtons();
void resetButton(ButtonState* button);
void checkButton(ButtonState* button);
bool readButtonMultiSample(int pin);
void updateButtonHoldState();
bool isButtonHeld(int btnNum, unsigned long durationMs);
bool anyButtonPressed();

// Main per-frame dispatch — picks the right displayPage_*() based on
// currentPage, drives menu / non-menu button handling.
void displayLoop();
void handleMenuPageSelection();
void handleRunningPageSelection();
