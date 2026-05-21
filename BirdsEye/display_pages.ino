///////////////////////////////////////////
// DISPLAY PAGES MODULE
// All displayPage_*() rendering functions for each UI screen
///////////////////////////////////////////

#include "display_pages.h"

void displayPage_boot() {
  resetDisplay();

  display.setTextSize(2);
  display.println(F("   Doves\n MagicBox"));
  display.setTextSize(1);
  display.println(F(""));
  display.println(F(" Timer + Data Logger"));
  display.println(F("\n    Initializing..."));

  safeDisplayUpdate();
}

void displayPage_main_menu() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Doves MagicBox"));
  display.setTextSize(1);
  display.println();
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Race"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Review"));
  display.print(menuSelectionIndex == 2 ? "->" : "  ");
  display.println(F("Transfer"));

  safeDisplayUpdate();
}

void displayPage_bluetooth() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F(" Bluetooth Connection"));
  display.println();

  display.setTextSize(2);
  if (bleConnected) {
    display.println(F(" Connected"));
  } else {
    display.println(F("  Waiting"));
  }

  display.setTextSize(1);
  display.println();

  if (bleTransferInProgress) {
    display.print(F("Transfer: "));
    display.print((bleBytesTransferred * 100) / bleFileSize);
    display.println(F("%"));
  } else {
    display.println();
  }

  display.println();
  display.setTextSize(1);
  display.println(F("->Exit"));

  safeDisplayUpdate();
}

void displayPage_replay_file_select() {
  resetDisplay();

  display.print(F("Select Session: "));
  display.print(menuSelectionIndex + 1);
  display.print(F("/"));
  display.println(numReplayFiles);
  display.println();
  display.setTextSize(1);

  if (numReplayFiles == 0) {
    display.println();
    display.println(F("No .dovex files"));
    display.println(F("found!"));
    display.println();
    display.println(F("Press any key"));
    display.println(F("to go back"));
  } else if (numReplayFiles < 3) {
    // Small menu - show all files
    for (int i = 0; i < numReplayFiles; i++) {
      if (menuSelectionIndex == i) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      // Split long filenames across two lines
      int fileNameLen = strlen(replayFiles[i]);
      char displayName[20];

      // First line: first 19 characters
      strncpy(displayName, replayFiles[i], 19);
      displayName[19] = '\0';
      display.println(displayName);

      // Second line: next 19 characters if filename is longer
      if (fileNameLen > 19) {
        display.print(F("  "));  // Indent to align with first line
        strncpy(displayName, replayFiles[i] + 19, 19);
        displayName[19] = '\0';
        display.println(displayName);
      } else {
        display.println();  // Blank line if no wrap needed
      }
    }
  } else {
    // Scrolling menu
    int indexA = menuSelectionIndex == numReplayFiles - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numReplayFiles - 1 : menuSelectionIndex - 1;

    char displayName[20];
    int fileNameLen;

    // First item
    display.print(F("  "));
    fileNameLen = strlen(replayFiles[indexA]);
    strncpy(displayName, replayFiles[indexA], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexA] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }

    // Second item (selected)
    display.print(F("->"));
    fileNameLen = strlen(replayFiles[indexB]);
    strncpy(displayName, replayFiles[indexB], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexB] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }

    // Third item
    display.print(F("  "));
    fileNameLen = strlen(replayFiles[indexC]);
    strncpy(displayName, replayFiles[indexC], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexC] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }
  }

  safeDisplayUpdate();
}

void displayPage_replay_results() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Replay Results"));

  // DOVEX replay: display from parsed header data
  display.print(F("Laps: "));
  display.println(lapHistoryCount);

  if (lapHistoryCount > 0) {
    // Find best lap from history
    unsigned long bestTime = lapHistory[0];
    int bestNum = 1;
    for (int i = 1; i < lapHistoryCount; i++) {
      if (lapHistory[i] < bestTime) {
        bestTime = lapHistory[i];
        bestNum = i + 1;
      }
    }

    display.print(F("Best: "));
    unsigned long minutes = bestTime / 60000;
    unsigned long seconds = (bestTime % 60000) / 1000;
    unsigned long milliseconds = bestTime % 1000;
    if (minutes > 0) { display.print(minutes); display.print(F(":")); }
    if (seconds < 10 && minutes > 0) display.print(F("0"));
    display.print(seconds);
    display.print(F("."));
    if (milliseconds < 100) display.print(F("0"));
    if (milliseconds < 10) display.print(F("0"));
    display.print(milliseconds);
    display.print(F(" (L"));
    display.print(bestNum);
    display.println(F(")"));

    // Show optimal if available
    if (strcmp(dovexReplayOptimal, "N/A") != 0 && dovexReplayOptimal[0] != '\0') {
      display.print(F("Opt: "));
      unsigned long optMs = strtoul(dovexReplayOptimal, NULL, 10);
      minutes = optMs / 60000;
      seconds = (optMs % 60000) / 1000;
      milliseconds = optMs % 1000;
      if (minutes > 0) { display.print(minutes); display.print(F(":")); }
      if (seconds < 10 && minutes > 0) display.print(F("0"));
      display.print(seconds);
      display.print(F("."));
      if (milliseconds < 100) display.print(F("0"));
      if (milliseconds < 10) display.print(F("0"));
      display.println(milliseconds);
    }
  }

  display.print(F("Driver: "));
  display.println(dovexReplayDriver);
  display.print(F("Course: "));
  display.println(dovexReplayCourseName);

  display.println();
  display.println(F("<- Laps       Exit ->"));

  safeDisplayUpdate();
}

void displayPage_replay_exit() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Exit Replay?"));
  display.println();

  display.setTextSize(2);
  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Back"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Exit"));

  safeDisplayUpdate();
}

void displayPage_gps_stats() {
  resetDisplay();

  // Safety: GPS stats page requires GPS to be initialized
  if (!gpsInitialized) {
    display.println(F("GPS not\ninitialized"));
    safeDisplayUpdate();
    return;
  }

  if (millis() - lastBatteryCheck > batteryUpdateInterval) {
    lastBatteryCheck = millis();
    lastBatteryVoltage = getBatteryVoltage();
  }
  {
    int battPct = getBatteryPercent(lastBatteryVoltage);
    display.print(F("Battery  : "));
    display.print(battPct);
    display.print(F("% "));
    display.print(lastBatteryVoltage, 2);
    display.println(F("V"));
  }


  display.print(F("Sats     : "));
  display.println(gpsData.satellites);

  display.print(F("Rate     : "));
  if (gpsData.fix) {
    display.print(gpsFrameRate, 1);
    display.println(F("Hz"));
  } else {
    display.println(F("NO FIX"));
  }

  display.print(F("HDOP     : "));
  if (gpsData.fix) {
    display.println(gpsData.HDOP, 1);
  } else {
    display.println(F("NO FIX"));
  }

  display.print(F("SDCard   : "));
  if (!sdSetupSuccess) {
    display.println(F("Bad Init"));
  } else if (enableLogging && sdDataLogInitComplete) {
    display.println(F("Logging"));
  } else if (enableLogging && !sdDataLogInitComplete) {
    display.println(F("Waiting GPS"));
  } else {
    display.println(F("Ready"));
  }

  display.println();

  if (courseManager != nullptr) {
    display.print(F("Track: "));
    display.println(courseManager->getShortName());
    display.print(F("Mode : "));
    const char* cn = courseManager->getActiveCourseName();
    display.println(cn ? cn : "Detecting...");
  } else {
    display.print(F("Waiting for GPS..."));
  }

  safeDisplayUpdate();
}

void displayPage_gps_speed() {
  resetDisplay();

  display.println(F("SPEED"));

  {
    int currentLap = activeTimerLaps() + (activeTimerRaceStarted() ? 1 : 0);
    if (currentLap > 0) {
      display.println(F("\nLAP"));
      if (currentLap < 100) {
        display.setTextSize(3);
      } else {
        display.setTextSize(2);
      }
      display.print(currentLap);
    }
  }

  display.setCursor(40, 5);
  display.setTextSize(7);
  // Safety check for GPS access
  if (gpsInitialized && gpsData.fix) {
    display.println(round(gps_speed_mph));
  } else {
    display.println(F("--"));
  }

  safeDisplayUpdate();
}

void displayPage_gps_lap_time() {
  resetDisplay();

  display.println(F("  Current Lap Time"));

  display.print(F("\n\n"));
  display.setTextSize(3);

  bool raceStarted = activeTimerRaceStarted();
  unsigned long currentLapTimeMs = activeTimerCurrentLapTime();

  if (raceStarted) {
    unsigned long minutes = currentLapTimeMs / 60000;
    unsigned long seconds = (currentLapTimeMs % 60000) / 1000;
    unsigned long milliseconds = currentLapTimeMs % 1000;
    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }
  } else {
    display.print("  N/A");
  }

  safeDisplayUpdate();
}

void displayPage_gps_pace() {
  resetDisplay();

  display.println(F("  Current Lap Pace"));

  int paceLaps = activeTimerLaps();
  float paceDiff = activeTimerPaceDifference();
  bool paceRaceStarted = activeTimerRaceStarted();

  // animation
  if (paceLaps >= 1 && paceDiff < (-1)) {
    if (paceFlashStatus) {
      paceFlashStatus = false;
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    } else {
      paceFlashStatus = true;
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    }
  }

  // main page into
  display.setTextColor(DISPLAY_TEXT_WHITE);
  const int lineHeight = 21;
  if (paceRaceStarted && paceLaps >= 1) {
    display.setCursor(0, lineHeight);
    display.setTextSize(4);
    if (paceDiff > 0) {
      display.print(F("+"));
    }
    display.print(paceDiff);
  } else {
    display.setTextSize(2);
    display.println();
    display.setTextSize(3);
    display.print(F("  N/A"));
  }

  // animation
  display.println();
  display.setTextSize(1);

  if (paceLaps >= 1 && paceDiff < (-1)) {
    if (paceFlashStatus) {
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    } else {
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    }
  }


  safeDisplayUpdate();
}

void displayPage_gps_best_lap() {
  resetDisplay();

  display.println(F("      Best Lap"));
  display.print(F("\n"));

  bool bestRaceStarted = activeTimerRaceStarted();
  int bestLaps = activeTimerLaps();
  unsigned long bestLapTimeMs = activeTimerBestLapTime();
  int bestLapNum = activeTimerBestLapNumber();

  if (bestRaceStarted && bestLaps > 0) {
    unsigned long minutes = bestLapTimeMs / 60000;
    unsigned long seconds = (bestLapTimeMs % 60000) / 1000;
    unsigned long milliseconds = bestLapTimeMs % 1000;
    display.setTextSize(3);
    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }

    display.setTextSize(2);
    display.print(F("\n\n"));
    display.print(F("Lap: "));
    display.print(bestLapNum);
  } else {
    display.print(F("\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }

  safeDisplayUpdate();
}

void displayPage_tachometer() {
  resetDisplay();

  if (tachLastReported > 9999) {
    display.println(F("Engine RPM *OVER REV*"));
  } else {
    display.println(F("     Engine RPM"));
  }

  display.setCursor(5, 20);
  display.setTextSize(4);
  if (tachLastReported < 10000) {
    display.print(F(" "));
  }
  if (tachLastReported < 1000) {
    display.print(F(" "));
  }
  if (tachLastReported < 100) {
    display.print(F(" "));
  }
  if (tachLastReported < 10) {
    display.print(F(" "));
  }
  display.println(tachLastReported);


  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(F("     max: "));
  display.print(topTachReported);

  safeDisplayUpdate();
}

void displayPage_optimal_lap() {
  resetDisplay();

  // Hide optimal lap when no sectors configured (Lap Anything mode)
  if (!activeTimerSectorsConfigured()) {
    display.println(F("     Optimal Lap"));
    display.print(F("\n\n"));
    display.setTextSize(2);
    display.println(F("No sectors"));
    safeDisplayUpdate();
    return;
  }

  display.println(F("     Optimal Lap"));

  bool optRaceStarted = activeTimerRaceStarted();
  int optLaps = activeTimerLaps();
  unsigned long optLapTimeMs = activeTimerOptimalLapTime();

  if (optRaceStarted && optLaps > 0) {
    const int lineHeight = 15;
    display.setCursor(0, lineHeight);
    display.setTextSize(2);

    unsigned long minutes = optLapTimeMs / 60000;
    unsigned long seconds = (optLapTimeMs % 60000) / 1000;
    unsigned long milliseconds = optLapTimeMs % 1000;

    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }

    display.setCursor(0, lineHeight+20);
    display.setTextSize(1);
    display.println(F("     Lap Numbers"));
    display.setCursor(0, lineHeight+35);
    display.setTextSize(2);
    display.print(F("  "));
    {
      DovesLapTimer* dlt = getActiveTimerDLT();
      if (dlt) {
        display.print(dlt->getBestSector1LapNumber());
        display.print(F("  "));
        display.print(dlt->getBestSector2LapNumber());
        display.print(F("  "));
        display.print(dlt->getBestSector3LapNumber());
      }
    }
  } else {
    display.print(F("\n\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }

  safeDisplayUpdate();
}

// TODO: this page probably needs some kind of delayed rendering?
void displayPage_gps_lap_list() {
  resetDisplay();
  if (recentlyChanged) {
    current_lap_list_page = 0;
  }
  lap_list_pages = ceil((double)lapHistoryCount / (double)lapsPerPage);

  if (lapHistoryCount >= 1) {
    display.print(F("   Lap History   "));
    display.print(current_lap_list_page + 1);
    display.print(F("/"));
    display.print(lap_list_pages);
    display.println(F("\n"));
    display.setTextSize(2);

    int pageStart = current_lap_list_page * lapsPerPage;
    int pageEnd = pageStart + lapsPerPage;
    for (int lap = pageStart; lap < pageEnd; ++lap) {
      if (lap < lapHistoryCount) {
        unsigned long minutes = lapHistory[lap] / 60000;
        unsigned long seconds = (lapHistory[lap] % 60000) / 1000;
        unsigned long milliseconds = lapHistory[lap] % 1000;

        int actualLap = lap + 1;
        if (actualLap < 10) {
          display.print(F(" "));
        }
        display.print(actualLap);
        display.setTextSize(1);
        display.print(F(" "));
        display.setTextSize(2);
        display.print(minutes);
        display.print(F(":"));
        display.print(seconds);
        display.print(F("."));
        display.print(milliseconds);
        display.println();
      }
    }
  } else {
    display.println(F("     Lap History     "));
    display.setTextSize(2);
    display.println();
    display.setTextSize(3);
    display.print(F("  N/A"));
  }

  safeDisplayUpdate();
}

void displayPage_stop_logging() {
  resetDisplay();

  display.setTextSize(2);
  display.println();
  display.println(F(" END RACE"));
  display.setTextSize(1);
  display.println();
  display.println(F(" press middle button"));

  safeDisplayUpdate();
}

void displayPage_stop_logging_confirm() {
  resetDisplay();

  display.println(F("Stop Logging?"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("BACK"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("END RACE"));

  safeDisplayUpdate();
}

void displayPage_gps_debug() {
  resetDisplay();
  display.println(F("GPS LapTimer Debug"));

  // Safety check for GPS access
  if (!gpsInitialized) {
    display.println(F("\nGPS not available"));
    safeDisplayUpdate();
    return;
  }

  display.print(F("Laps: "));
  display.print(activeTimerLaps());
  display.print(F(" | od:"));
  display.println(activeTimerTotalDistance());
  display.print(F("Strt: "));
  display.print(activeTimerRaceStarted() ? F("T") : F("F"));
  display.print(F(" | Xing: "));
  display.println(activeTimerCrossing() ? F("T") : F("F"));
  display.print(F("Current: "));
  display.println(activeTimerCurrentLapTime());
  display.print(F("Last   : "));
  display.println(activeTimerLastLapTime());
  display.print(F("Best: "));
  display.print(activeTimerBestLapNumber());
  display.print(F(": "));
  display.println(activeTimerBestLapTime());
  display.print(F("Pace   : "));
  display.println(activeTimerPaceDifference());
  if (courseManager) {
    display.print(F("Course: "));
    const char* cn = courseManager->getActiveCourseName();
    display.println(cn ? cn : "...");
  }

  safeDisplayUpdate();
}

void displayPage_internal_fault() {
  resetDisplay();
  display.setCursor(0, 0);
  notificationFlash = notificationFlash == true ? false : true;
  display.setTextSize(2);

  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("   FAULT  "));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  display.println(F(" Please Reboot Device"));
  display.println(F(""));
  display.println(internalNotification);
  safeDisplayUpdate();
}

void displayPage_internal_warning() {
  resetDisplay();
  notificationFlash = notificationFlash == true ? false : true;

  display.setTextSize(2);
  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("  WARNING  "));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  display.println(F("Continue With Caution"));
  display.println(F(""));
  display.println(internalNotification);
  safeDisplayUpdate();
}

void displayPage_sleep_charging() {
  resetDisplay();

  float voltage = getBatteryVoltage();
  int percent = getBatteryPercent(voltage);

  display.setTextSize(1);
  display.setCursor(32, 10);
  display.print(F("Charging"));

  display.setTextSize(3);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", percent);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 28);
  display.print(buf);

  display.setTextSize(1);
  char vbuf[8];
  dtostrf(voltage, 4, 2, vbuf);
  display.setCursor(40, 56);
  display.print(vbuf);
  display.print(F("V"));

  safeDisplayUpdate();
}

///////////////////////////////////////////
void displayCrossing() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  #ifndef ENDURANCE_MODE
    // Draw bitmap on the screen
    calculatingFlip = calculatingFlip == true ? false : true;
    if (calculatingFlip) {
      display.drawBitmap(0, 0, image_data_calculating1, 128, 64, 1);
    } else {
      display.drawBitmap(0, 0, image_data_calculating2, 128, 64, 1);
    }
  #else
  #endif

  safeDisplayUpdate();
}
