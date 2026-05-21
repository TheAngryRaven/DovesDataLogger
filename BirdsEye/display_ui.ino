///////////////////////////////////////////
// DISPLAY UI MODULE
// Display setup, button handling, menu navigation, and display loop
///////////////////////////////////////////

#include "display_ui.h"

///////////////////////////////////////////
// I2C BUS RECOVERY
// EMI from ignition can glitch the I2C bus, leaving a slave holding SDA low.
// The Wire library may hang forever waiting. This recovery routine bit-bangs
// 9 SCL clocks to free a stuck slave, then re-initializes Wire.
///////////////////////////////////////////

static bool i2cRecoveryNeeded = false;

void i2cBusRecover() {
  debugln(F("I2C: Bus recovery - bit-banging 9 SCL clocks"));

  Wire.end();

  // Manually toggle SCL 9 times to free stuck slave
  // SDA must be floating (input) so slave can release it
  pinMode(PIN_WIRE_SDA, INPUT);
  pinMode(PIN_WIRE_SCL, OUTPUT);

  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_WIRE_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_WIRE_SCL, HIGH);
    delayMicroseconds(5);
  }

  // Generate STOP condition: SDA low-to-high while SCL is high
  pinMode(PIN_WIRE_SDA, OUTPUT);
  digitalWrite(PIN_WIRE_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_WIRE_SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(PIN_WIRE_SDA, HIGH);
  delayMicroseconds(5);

  // Re-init Wire
  Wire.begin();
  Wire.setClock(400000);  // Must re-set after begin() (resets to 100kHz)

  // Re-init display
  #ifdef USE_1306_DISPLAY
    display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDRESS);
  #else
    display.begin(I2C_DISPLAY_ADDRESS, true);
  #endif

  debugln(F("I2C: Bus recovery complete"));
}

// Safe wrapper around display.display() - detects hung I2C and recovers.
// A normal 1024-byte I2C transfer at 400kHz takes ~25ms.
// If it takes >100ms, something is wrong (EMI glitch or bus hang).
void safeDisplayUpdate() {
  unsigned long start = millis();
  display.display();
  unsigned long elapsed = millis() - start;

  if (elapsed > 100) {
    debugln(F("I2C: display.display() took too long, scheduling recovery"));
    i2cRecoveryNeeded = true;
  }
}

void setupButtons() {
  #ifndef WOKWI
  // greybox
  btn1->pin = 1;
  btn2->pin = 2;
  btn3->pin = 3;

  #else
  btn1->pin = 4;
  btn2->pin = 5;
  btn3->pin = 6;
  #endif

  pinMode(btn1->pin, INPUT_PULLUP);
  pinMode(btn2->pin, INPUT_PULLUP);
  pinMode(btn3->pin, INPUT_PULLUP);
}

void readButtons() {
  checkButton(btn1);
  checkButton(btn2);
  checkButton(btn3);

  //force update when button pressed
  if (
    btn1->pressed ||
    btn2->pressed ||
    btn3->pressed
  ) {
    forceDisplayRefresh();
  }
}

void resetButtons() {
  resetButton(btn1);
  resetButton(btn2);
  resetButton(btn3);
}

void updateButtonHoldState() {
  bool b1 = readButtonMultiSample(btn1->pin);
  bool b2 = readButtonMultiSample(btn2->pin);
  bool b3 = readButtonMultiSample(btn3->pin);

  // Track continuous hold duration per button
  if (b1) { if (!btn1Held) { btn1HoldStart = millis(); btn1Held = true; } }
  else { btn1Held = false; }

  if (b2) { if (!btn2Held) { btn2HoldStart = millis(); btn2Held = true; } }
  else { btn2Held = false; }

  if (b3) { if (!btn3Held) { btn3HoldStart = millis(); btn3Held = true; } }
  else { btn3Held = false; }
}

bool isButtonHeld(int btnNum, unsigned long durationMs) {
  unsigned long start;
  bool held;
  switch(btnNum) {
    case 1: start = btn1HoldStart; held = btn1Held; break;
    case 2: start = btn2HoldStart; held = btn2Held; break;
    case 3: start = btn3HoldStart; held = btn3Held; break;
    default: return false;
  }
  return held && start > 0 && (millis() - start >= durationMs);
}

bool anyButtonPressed() {
  return readButtonMultiSample(btn1->pin) ||
         readButtonMultiSample(btn2->pin) ||
         readButtonMultiSample(btn3->pin);
}

void resetButton(ButtonState* button) {
  button->pressed = false;
}

/**
 * @brief Multi-sample button read with EMI rejection
 *
 * Takes multiple samples with small delays and requires ALL samples
 * to show the button pressed. This rejects transient EMI spikes that
 * might cause a single false LOW reading.
 *
 * @param pin The GPIO pin to read
 * @return true only if ALL samples show button pressed (LOW)
 */
bool readButtonMultiSample(int pin) {
  for (int i = 0; i < BUTTON_SAMPLE_COUNT; i++) {
    if (digitalRead(pin) != LOW) {
      return false;  // Any HIGH reading = not pressed
    }
    if (i < BUTTON_SAMPLE_COUNT - 1) {
      delayMicroseconds(BUTTON_SAMPLE_DELAY_US);
    }
  }
  return true;  // All samples were LOW = definitely pressed
}

/**
 * @brief Check button state with debouncing, edge detection, and multi-sample verification
 *
 * Uses edge detection to only trigger on button PRESS (not while held).
 * Button must be released before it can trigger again.
 */
void checkButton(ButtonState* button) {
  // Multi-sample read: require consistent LOW across all samples
  bool btnCurrentlyPressed = readButtonMultiSample(button->pin);

  if (!btnCurrentlyPressed) {
    // Button is released - mark it as ready for next press
    button->wasReleased = true;
    return;
  }

  // Button is pressed - check if we should register this press
  // Requires: 1) button was released since last press (edge detection)
  //           2) debounce time has passed
  bool btnReady = millis() - button->lastPressed >= antiBounceIntv;

  if (button->wasReleased && btnReady) {
    button->lastPressed = millis();
    button->pressed = true;
    button->wasReleased = false;  // Must release before next press
  }
}

//////////////////////////////////////////
// TODO: make display into own class??
void resetDisplay() {
  if (currentPage != lastPage) {
    lastPage = currentPage;
    recentlyChanged = true;
    menuSelectionIndex = 0;
  } else {
    recentlyChanged = false;
  }
  display.setTextWrap(false);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.setTextColor(DISPLAY_TEXT_WHITE);
}

void forceDisplayRefresh() {
  // why does adding work but not subtracting?
  displayLastUpdate += 5000;
}

void switchToDisplayPage(int newDisplayPage) {
  currentPage = newDisplayPage;
  forceDisplayRefresh();
}

///////////////////////////////////////////

void displaySetup() {
  debugln(F("SETTING UP DISPLAY"));
  delay(250); // wait for the OLED to power up

  // Set I2C timeout to prevent infinite hangs from EMI-induced bus faults
  Wire.setTimeout(100);

#ifdef USE_1306_DISPLAY
  display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDRESS);
#else
  display.begin(I2C_DISPLAY_ADDRESS, true);
#endif

  // 400kHz I2C: reduces display.display() from ~100ms to ~25ms.
  // At 100kHz, the 1024-byte framebuffer transfer blocks long enough
  // for 2-3 GPS PVT messages (40ms each) to arrive, but the SparkFun
  // library's auto-PVT buffer only keeps the latest — losing ~2 samples
  // every display refresh (3Hz). At 400kHz the transfer completes within
  // a single PVT interval, eliminating the loss.
  Wire.setClock(400000);

  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextWrap(false);

  setupButtons();

  displayLastUpdate = millis();

  currentPage = PAGE_BOOT;

  // silly boot splash, maybe anim?
  resetDisplay();
  display.drawBitmap(0, 0, image_data_bird1, 128, 64, 1);
  safeDisplayUpdate();
  delay(750);

  displayPage_boot();
}

void handleMenuPageSelection() {
  if (currentPage == PAGE_MAIN_MENU) {
    if (menuSelectionIndex == 0) {
      // Race selected — go directly to race mode, start logging on GPS fix
      debugln(F("Main Menu: Race selected"));
      raceActive = true;
      enableLogging = true;
      raceSessionStartedAt = millis();
      // Create CourseManager if not already created by track detection
      createLapAnythingCourseManager();
      switchToDisplayPage(GPS_SPEED);
    } else if (menuSelectionIndex == 1) {
      // Replay selected
      debugln(F("Main Menu: Replay selected"));
      resetReplayState();
      if (buildReplayFileList()) {
        switchToDisplayPage(PAGE_REPLAY_FILE_SELECT);
      } else {
        strncpy(internalNotification, "No .dovex files\nfound on SD!", sizeof(internalNotification) - 1);
        internalNotification[sizeof(internalNotification) - 1] = '\0';
        switchToDisplayPage(PAGE_INTERNAL_WARNING);
      }
    } else {
      // Bluetooth selected
      debugln(F("Main Menu: Bluetooth selected"));
      BLE_SETUP();
      switchToDisplayPage(PAGE_BLUETOOTH);
    }
  } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
    if (numReplayFiles == 0) {
      // No files - go back
      switchToDisplayPage(PAGE_MAIN_MENU);
    } else {
      selectedReplayFile = menuSelectionIndex;
      debug(F("Replay: Selected file: "));
      debugln(replayFiles[selectedReplayFile]);

      // DOVEX instant replay: parse header and go straight to results
      if (parseDovexHeader(replayFiles[selectedReplayFile])) {
        replayProcessingComplete = true;
        switchToDisplayPage(PAGE_REPLAY_RESULTS);
      } else {
        strncpy(internalNotification, "Cannot read DOVEX\nheader (incomplete?)", sizeof(internalNotification) - 1);
        internalNotification[sizeof(internalNotification) - 1] = '\0';
        switchToDisplayPage(PAGE_INTERNAL_WARNING);
      }
    }
  } else if (currentPage == PAGE_REPLAY_EXIT) {
    if (menuSelectionIndex == 0) {
      // Back - return to results
      switchToDisplayPage(PAGE_REPLAY_RESULTS);
    } else {
      // Exit - return to main menu
      resetReplayState();
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
  } else if (currentPage == PAGE_BLUETOOTH) {
    // Exit button pressed - go back to main menu and disable bluetooth
    debugln(F("Bluetooth: Exit selected"));
    BLE_STOP();
    switchToDisplayPage(PAGE_MAIN_MENU);
  } else if (currentPage == LOGGING_STOP_CONFIRM) {
    if (menuSelectionIndex == 0) {
      switchToDisplayPage(GPS_SPEED);
    } else {
      // LOGGING STOP
      endRaceSession();
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
    debug(F("Stop Logging?: "));
    debugln(menuSelectionIndex == 0 ? "NO" : "YES");
  }
}

void handleRunningPageSelection() {
  if (currentPage == LOGGING_STOP) {
    switchToDisplayPage(LOGGING_STOP_CONFIRM);
  } else if (currentPage == GPS_LAP_LIST) {
    // Middle button cycles lap list pages (both live and replay mode)
    current_lap_list_page = current_lap_list_page == (lap_list_pages-1) ? 0 : current_lap_list_page + 1;
    forceDisplayRefresh();
  } else if (currentPage == PAGE_REPLAY_RESULTS) {
    // Middle button on results does nothing (use left/right for navigation)
  } else {
    // wokwi doesnt like fancy page switcher
    // inverted eats too much power
    #ifdef WOKWI
      if(displayInverted == true) {
        displayInverted = false;
        display.invertDisplay(false);
      } else {
        displayInverted = true;
        display.invertDisplay(true);
      }
    #else
      if (gps_speed_mph <= 5.0) {
        currentPage = GPS_LAP_BEST;
      } else {
        currentPage = GPS_LAP_PACE;
      }
      switchToDisplayPage(currentPage);
    #endif
  }
}

void displayLoop() {
  // Check if I2C recovery was flagged by a previous slow display update
  if (i2cRecoveryNeeded) {
    i2cRecoveryNeeded = false;
    i2cBusRecover();
  }

  // todo: better page handling
  if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
    displayLastUpdate = millis();

    #ifdef ENDURANCE_MODE
      bool inEndurance = true;
    #else
      bool inEndurance = false;
    #endif

    bool isCrossing = activeTimerCrossing();

    if (
      currentPage != GPS_STATS &&
      currentPage != GPS_DEBUG &&
      currentPage != LOGGING_STOP &&
      currentPage != LOGGING_STOP_CONFIRM &&
      currentPage != PAGE_INTERNAL_FAULT &&
      currentPage != PAGE_INTERNAL_WARNING &&
      isCrossing &&
      inEndurance == false
    ) {
      displayCrossing();
      lastPage = 999;
    } else if (currentPage == PAGE_MAIN_MENU) {
      displayPage_main_menu();
    } else if (currentPage == PAGE_BLUETOOTH) {
      displayPage_bluetooth();
    } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
      displayPage_replay_file_select();
    } else if (currentPage == PAGE_REPLAY_RESULTS) {
      displayPage_replay_results();
    } else if (currentPage == PAGE_REPLAY_EXIT) {
      displayPage_replay_exit();
    } else if (currentPage == GPS_STATS) {
      displayPage_gps_stats();
    } else if (currentPage == GPS_SPEED) {
      displayPage_gps_speed();
    } else if (currentPage == TACHOMETER) {
      displayPage_tachometer();
    } else if (currentPage == GPS_LAP_TIME) {
      displayPage_gps_lap_time();
    } else if (currentPage == GPS_LAP_PACE) {
      displayPage_gps_pace();
    } else if (currentPage == GPS_LAP_BEST) {
      #ifndef ENDURANCE_MODE
        displayPage_gps_best_lap();
      #else
        if (gps_speed_mph <= 10.0) {
          displayPage_gps_best_lap();
        } else {
          if (lastPage == (GPS_LAP_BEST - 1)) {
            currentPage = GPS_SPEED;
          } else {
            currentPage = (GPS_LAP_BEST - 1);
          }
          switchToDisplayPage(currentPage);
        }
      #endif
    } else if (currentPage == OPTIMAL_LAP) {
      displayPage_optimal_lap();
    } else if (currentPage == GPS_LAP_LIST) {
      displayPage_gps_lap_list();
    } else if (currentPage == LOGGING_STOP) {
      // dont let us stop logging while were moving, duh
      if (gps_speed_mph <= 2.0) {
        displayPage_stop_logging();
      } else {
        // todo: not super friendly when expanding pages, assuming "logging stop" is always the "last" page
        if (lastPage == (LOGGING_STOP - 1)) {
          currentPage = runningPageStart;
        } else {
          currentPage = LOGGING_STOP - 1;
        }
        switchToDisplayPage(currentPage);
      }
    } else if (currentPage == LOGGING_STOP_CONFIRM) {
      displayPage_stop_logging_confirm();
    } else if (currentPage == GPS_DEBUG) {
      displayPage_gps_debug();
    } else if (currentPage == PAGE_INTERNAL_FAULT) {
      displayPage_internal_fault();
    } else if (currentPage == PAGE_INTERNAL_WARNING) {
      displayPage_internal_warning();
    }
  }

  // todo: better button handling
  bool insideMenu = false;
  bool buttonsDisabled = false;
  int menuLimit = 0;

  if (
    currentPage == PAGE_MAIN_MENU ||
    currentPage == PAGE_BLUETOOTH ||
    currentPage == LOGGING_STOP_CONFIRM ||
    currentPage == PAGE_REPLAY_FILE_SELECT ||
    currentPage == PAGE_REPLAY_EXIT
  ) {
    insideMenu = true;
    if (currentPage == PAGE_MAIN_MENU) {
      menuLimit = 3; // Race, Replay, Download
    } else if (currentPage == PAGE_BLUETOOTH) {
      menuLimit = 1; // Only "Exit" option
    } else if (
      currentPage == LOGGING_STOP_CONFIRM ||
      currentPage == PAGE_REPLAY_EXIT
    ) {
      menuLimit = 2;
    } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
      menuLimit = numReplayFiles > 0 ? numReplayFiles : 1;
    }
  }

  if (currentPage == PAGE_INTERNAL_FAULT) {
    buttonsDisabled = true;
  }

  // menu operator
  if (insideMenu && !buttonsDisabled) {
    // we are in a menu do weird menu things
    // Main menu has static display (items top-to-bottom = index 0,1,2)
    // so button direction needs to be reversed compared to scrolling menus
    bool reverseDirection = (currentPage == PAGE_MAIN_MENU);

    // BUTTON UP (or DOWN for reversed menus)
    if (btn1->pressed) {
      if (reverseDirection) {
        // Move UP visually = decrease index
        if (menuSelectionIndex == 0) {
          menuSelectionIndex = menuLimit-1;
        } else {
          menuSelectionIndex--;
        }
      } else {
        if (menuSelectionIndex == menuLimit-1) {
          menuSelectionIndex = 0;
        } else {
          menuSelectionIndex++;
        }
      }
      debug(F("menu number: "));
      debugln(menuSelectionIndex);
      forceDisplayRefresh();
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln(F("Button Enter"));
      handleMenuPageSelection();
    }
    // BUTTON DOWN (or UP for reversed menus)
    if (btn3->pressed) {
      if (reverseDirection) {
        // Move DOWN visually = increase index
        if (menuSelectionIndex == menuLimit-1) {
          menuSelectionIndex = 0;
        } else {
          menuSelectionIndex++;
        }
      } else {
        if (menuSelectionIndex == 0) {
          menuSelectionIndex = menuLimit-1;
        } else {
          menuSelectionIndex--;
        }
      }
      debug(F("menu number: "));
      debugln(menuSelectionIndex);
      forceDisplayRefresh();
    }
  } else if (currentPage == PAGE_INTERNAL_WARNING) {
    // Warning page: any button returns to main menu
    if (btn1->pressed || btn2->pressed || btn3->pressed) {
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
  } else if (!buttonsDisabled){
    // page up/down/enter
    // BUTTON LEFT
    if (btn1->pressed) {
      debugln(F("Button Left"));
      // Special handling for replay results - left goes to lap list
      if (currentPage == PAGE_REPLAY_RESULTS) {
        if (lapHistoryCount > 0) {
          current_lap_list_page = 0;
          switchToDisplayPage(GPS_LAP_LIST);
        }
      // Special handling for lap list during replay - left goes back to results
      } else if (currentPage == GPS_LAP_LIST && replayProcessingComplete) {
        switchToDisplayPage(PAGE_REPLAY_RESULTS);
      } else {
        if (currentPage <= runningPageStart) {
          currentPage = runningPageEnd;
        } else {
          currentPage--;
        }
        debug(F("running menu number: "));
        debugln(currentPage);
        switchToDisplayPage(currentPage);
      }
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln(F("Button Middle (running)"));
      handleRunningPageSelection();
    }
    // BUTTON DOWN/RIGHT
    if (btn3->pressed) {
      debugln(F("Button Right"));
      // Special handling for replay results - right goes to exit page
      if (currentPage == PAGE_REPLAY_RESULTS) {
        switchToDisplayPage(PAGE_REPLAY_EXIT);
      // Special handling for lap list during replay - right goes back to results
      } else if (currentPage == GPS_LAP_LIST && replayProcessingComplete) {
        switchToDisplayPage(PAGE_REPLAY_RESULTS);
      } else {
        if (currentPage >= runningPageEnd) {
          currentPage = runningPageStart;
        } else {
          currentPage++;
        }
        debug(F("running menu number: "));
        debugln(currentPage);
        switchToDisplayPage(currentPage);
      }
    }
  }
}
