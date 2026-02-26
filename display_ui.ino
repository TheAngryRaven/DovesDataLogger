///////////////////////////////////////////
// DISPLAY UI MODULE
// Display setup, button handling, menu navigation, and display loop
///////////////////////////////////////////

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

  // Re-init display
  #ifdef USE_1306_DISPLAY
    display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDRESS);
  #else
    display.begin(I2C_DISPLAY_ADDRESS, true);
  #endif

  debugln(F("I2C: Bus recovery complete"));
}

// Safe wrapper around display.display() - detects hung I2C and recovers.
// A normal 1024-byte I2C transfer at 100kHz takes ~100ms.
// If it takes >250ms, something is wrong.
void safeDisplayUpdate() {
  unsigned long start = millis();
  display.display();
  unsigned long elapsed = millis() - start;

  if (elapsed > 250) {
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
      // Race selected
      debugln(F("Main Menu: Race selected"));
      switchToDisplayPage(PAGE_SELECT_LOCATION);
    } else if (menuSelectionIndex == 1) {
      // Replay selected
      debugln(F("Main Menu: Replay selected"));
      resetReplayState();
      if (buildReplayFileList()) {
        switchToDisplayPage(PAGE_REPLAY_FILE_SELECT);
      } else {
        strncpy(internalNotification, "No .dove or .nmea\nfiles found on SD!", sizeof(internalNotification) - 1);
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

      // Show detecting page
      switchToDisplayPage(PAGE_REPLAY_DETECTING);
      forceDisplayRefresh();
      displayPage_replay_detecting();

      // Detect track
      replayDetectedTrackIndex = detectTrackForReplayFile(replayFiles[selectedReplayFile]);

      if (replayDetectedTrackIndex >= 0) {
        // Track found - parse it and go to layout selection
        selectedLocation = replayDetectedTrackIndex;
        char filepath[FILEPATH_MAX];
        makeFullTrackPath(locations[selectedLocation], filepath);
        numOfTracks = 0; // Reset before parsing
        int parseStatus = parseTrackFile(filepath);

        if (parseStatus == PARSE_STATUS_GOOD) {
          switchToDisplayPage(PAGE_REPLAY_SELECT_TRACK);
        } else {
          strncpy(internalNotification, "Failed to parse\ntrack file!", sizeof(internalNotification) - 1);
          internalNotification[sizeof(internalNotification) - 1] = '\0';
          switchToDisplayPage(PAGE_INTERNAL_FAULT);
        }
      } else {
        // No track detected - let user select manually
        strncpy(internalNotification, "Track not detected!\nSelect manually.", sizeof(internalNotification) - 1);
        internalNotification[sizeof(internalNotification) - 1] = '\0';
        // Go to normal track selection
        switchToDisplayPage(PAGE_SELECT_LOCATION);
        replayModeActive = true; // Set flag so we know we're in replay mode
      }
    }
  } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
    selectedTrack = menuSelectionIndex;
    switchToDisplayPage(PAGE_REPLAY_SELECT_DIRECTION);
    debug(F("Replay: Selected layout: "));
    debugln(tracks[selectedTrack]);

    // Configure lap timer with selected track
    crossingPointALat = trackLayouts[selectedTrack].start_a_lat;
    crossingPointALng = trackLayouts[selectedTrack].start_a_lng;
    crossingPointBLat = trackLayouts[selectedTrack].start_b_lat;
    crossingPointBLng = trackLayouts[selectedTrack].start_b_lng;

    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

    if (trackLayouts[selectedTrack].hasSector2) {
      lapTimer.setSector2Line(
        trackLayouts[selectedTrack].sector_2_a_lat,
        trackLayouts[selectedTrack].sector_2_a_lng,
        trackLayouts[selectedTrack].sector_2_b_lat,
        trackLayouts[selectedTrack].sector_2_b_lng
      );
    }

    if (trackLayouts[selectedTrack].hasSector3) {
      lapTimer.setSector3Line(
        trackLayouts[selectedTrack].sector_3_a_lat,
        trackLayouts[selectedTrack].sector_3_a_lng,
        trackLayouts[selectedTrack].sector_3_b_lat,
        trackLayouts[selectedTrack].sector_3_b_lng
      );
    }

    lapTimer.forceLinearInterpolation();
    lapTimer.reset();
  } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
    selectedDirection = menuSelectionIndex;
    debug(F("Replay: Selected direction: "));
    debugln(selectedDirection == RACE_DIRECTION_FORWARD ? "Forward" : "Reverse");

    // Acquire SD access for replay before opening file
    if (!acquireSDAccess(SD_ACCESS_REPLAY)) {
      strncpy(internalNotification, "SD card busy!\nCannot start replay", sizeof(internalNotification) - 1);
      internalNotification[sizeof(internalNotification) - 1] = '\0';
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    } else if (replayFile.open(replayFiles[selectedReplayFile], O_READ)) {
      replayModeActive = true;
      replayProcessingComplete = false;
      switchToDisplayPage(PAGE_REPLAY_PROCESSING);
    } else {
      releaseSDAccess(SD_ACCESS_REPLAY);  // Release on failure
      strncpy(internalNotification, "Failed to open\nreplay file!", sizeof(internalNotification) - 1);
      internalNotification[sizeof(internalNotification) - 1] = '\0';
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
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
  } else if (currentPage == PAGE_SELECT_LOCATION) {
    selectedLocation = menuSelectionIndex;

    char filepath[FILEPATH_MAX];
    makeFullTrackPath(locations[selectedLocation], filepath);
    int parseStatus = parseTrackFile(filepath);

    if (parseStatus == PARSE_STATUS_GOOD) {
      switchToDisplayPage(PAGE_SELECT_TRACK);
    } else if (parseStatus == PARSE_STATUS_LOAD_FAILED) {
      strncpy(internalNotification, "Could not load file!", sizeof(internalNotification) - 1);
      internalNotification[sizeof(internalNotification) - 1] = '\0';
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    } else if (parseStatus == PARSE_STATUS_PARSE_FAILED) {
      strncpy(internalNotification, "JSON Parsing Failed!", sizeof(internalNotification) - 1);
      internalNotification[sizeof(internalNotification) - 1] = '\0';
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    }

    debug(F("Selected Location: "));
    debugln(locations[selectedLocation]);
  } else if (currentPage == PAGE_SELECT_TRACK) {
    selectedTrack = menuSelectionIndex;

    // Check if we're in replay mode (manual track selection after auto-detect failed)
    if (replayModeActive) {
      switchToDisplayPage(PAGE_REPLAY_SELECT_DIRECTION);
    } else {
      switchToDisplayPage(PAGE_SELECT_DIRECTION);
    }

    debug(F("Selected Track: "));
    debugln(tracks[selectedTrack]);
    debug(F("start_a_lat: "));
    debugln(trackLayouts[selectedTrack].start_a_lat, 8);
    debug(F("start_a_lng: "));
    debugln(trackLayouts[selectedTrack].start_a_lng, 8);
    debug(F("start_b_lat: "));
    debugln(trackLayouts[selectedTrack].start_b_lat, 8);
    debug(F("start_b_lng: "));
    debugln(trackLayouts[selectedTrack].start_b_lng, 8);

    crossingPointALat = trackLayouts[selectedTrack].start_a_lat;
    crossingPointALng = trackLayouts[selectedTrack].start_a_lng;
    crossingPointBLat = trackLayouts[selectedTrack].start_b_lat;
    crossingPointBLng = trackLayouts[selectedTrack].start_b_lng;

    // initialize laptimer class
    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

    // Set sector lines if available
    if (trackLayouts[selectedTrack].hasSector2) {
      lapTimer.setSector2Line(
        trackLayouts[selectedTrack].sector_2_a_lat,
        trackLayouts[selectedTrack].sector_2_a_lng,
        trackLayouts[selectedTrack].sector_2_b_lat,
        trackLayouts[selectedTrack].sector_2_b_lng
      );
      #ifdef HAS_DEBUG
      debugln(F("Sector 2 line configured"));
      #endif
    }

    if (trackLayouts[selectedTrack].hasSector3) {
      lapTimer.setSector3Line(
        trackLayouts[selectedTrack].sector_3_a_lat,
        trackLayouts[selectedTrack].sector_3_a_lng,
        trackLayouts[selectedTrack].sector_3_b_lat,
        trackLayouts[selectedTrack].sector_3_b_lng
      );
      #ifdef HAS_DEBUG
      debugln(F("Sector 3 line configured"));
      #endif
    }

    lapTimer.forceLinearInterpolation();
    lapTimer.reset();
  } else if (currentPage == PAGE_SELECT_DIRECTION) {
    selectedDirection = menuSelectionIndex;
    switchToDisplayPage(GPS_SPEED);
    debug(F("Selected Direction: "));
    debugln(selectedDirection == RACE_DIRECTION_FORWARD ? "Forward" : "Reverse");
    enableLogging = true;
    trackSelected = true;
  } else if (currentPage == LOGGING_STOP_CONFIRM) {
    if (menuSelectionIndex == 0) {
      switchToDisplayPage(GPS_SPEED);
    } else {
      // LOGGING STOP
      switchToDisplayPage(PAGE_SELECT_TRACK);
      enableLogging = false;
      sdDataLogInitComplete = false;
      trackSelected = false;
      dataFile.flush();
      dataFile.close();
      releaseSDAccess(SD_ACCESS_LOGGING);  // Release SD access when logging stops
      lapTimer.reset();

      // reset lap history
      lapHistoryCount = 0;
      lastLap = 0;
      memset(lapHistory, 0, sizeof(lapHistory));
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

    if (
      currentPage != GPS_STATS &&
      currentPage != GPS_DEBUG &&
      currentPage != LOGGING_STOP &&
      currentPage != LOGGING_STOP_CONFIRM &&
      currentPage != PAGE_INTERNAL_FAULT &&
      currentPage != PAGE_INTERNAL_WARNING &&
      lapTimer.getCrossing() &&
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
    } else if (currentPage == PAGE_REPLAY_DETECTING) {
      displayPage_replay_detecting();
    } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
      displayPage_replay_select_track();
    } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
      displayPage_replay_select_direction();
    } else if (currentPage == PAGE_REPLAY_PROCESSING) {
      displayPage_replay_processing();
    } else if (currentPage == PAGE_REPLAY_RESULTS) {
      displayPage_replay_results();
    } else if (currentPage == PAGE_REPLAY_EXIT) {
      displayPage_replay_exit();
    } else if (currentPage == PAGE_SELECT_LOCATION) {
      displayPage_select_location();
    } else if (currentPage == PAGE_SELECT_TRACK) {
      displayPage_select_track();
    } else if (currentPage == PAGE_SELECT_DIRECTION) {
      displayPage_select_direction();
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
    currentPage == PAGE_SELECT_LOCATION ||
    currentPage == PAGE_SELECT_TRACK ||
    currentPage == PAGE_SELECT_DIRECTION ||
    currentPage == LOGGING_STOP_CONFIRM ||
    currentPage == PAGE_REPLAY_FILE_SELECT ||
    currentPage == PAGE_REPLAY_SELECT_TRACK ||
    currentPage == PAGE_REPLAY_SELECT_DIRECTION ||
    currentPage == PAGE_REPLAY_EXIT
  ) {
    insideMenu = true;
    if (currentPage == PAGE_MAIN_MENU) {
      menuLimit = 3; // Race, Replay, Download
    } else if (currentPage == PAGE_BLUETOOTH) {
      menuLimit = 1; // Only "Exit" option
    } else if (currentPage == PAGE_SELECT_LOCATION) {
      menuLimit = numOfLocations;
    } else if (currentPage == PAGE_SELECT_TRACK) {
      menuLimit = numOfTracks;
    } else if (
      currentPage == PAGE_SELECT_DIRECTION ||
      currentPage == LOGGING_STOP_CONFIRM ||
      currentPage == PAGE_REPLAY_EXIT
    ) {
      menuLimit = 2;
    } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
      menuLimit = numReplayFiles > 0 ? numReplayFiles : 1;
    } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
      menuLimit = numOfTracks;
    } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
      menuLimit = 2;
    }
  }

  if (
    currentPage == PAGE_INTERNAL_FAULT ||
    currentPage == PAGE_REPLAY_PROCESSING ||
    currentPage == PAGE_REPLAY_DETECTING
  ) {
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
  } else if (!buttonsDisabled){
    // page up/down/enter
    // BUTTON LEFT
    if (btn1->pressed) {
      debugln(F("Button Left"));
      // Special handling for replay results - left goes to lap list
      if (currentPage == PAGE_REPLAY_RESULTS) {
        if (lapTimer.getLaps() > 0) {
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
