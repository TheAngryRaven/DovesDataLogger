///////////////////////////////////////////
// DISPLAY PAGES MODULE
// All displayPage_*() rendering functions for each UI screen
///////////////////////////////////////////

void displayPage_boot() {
  resetDisplay();

  display.setTextSize(2);
  display.println(F("   Doves\n MagicBox"));
  display.setTextSize(1);
  display.println(F(""));
  display.println(F(" Timer + Data Logger"));
  display.println(F("\n    Initializing..."));

  display.display();
}

void displayPage_select_location() {
  resetDisplay();
  if (recentlyChanged && selectedLocation >= 0) {
    menuSelectionIndex = selectedLocation;
  }

  display.print(F("Select Track: "));
  display.print(menuSelectionIndex+1);
  display.print(F("/"));
  display.println(numOfLocations);
  display.println();
  display.setTextSize(2);

  if (numOfLocations < 3) {
    display.println();
    // small menu
    if (numOfLocations >= 1) {
      if (menuSelectionIndex == 0) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(locations[0]);
    }
    if (numOfLocations >= 2) {
      if (menuSelectionIndex == 1) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(locations[1]);
    }
  } else {
    // scrolling menu
    int indexA = menuSelectionIndex == numOfLocations - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfLocations - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(locations[indexA]);
    display.print(F("->"));
    display.println(locations[indexB]);
    display.print(F("  "));
    display.println(locations[indexC]);
  }

  display.display();
}

void displayPage_select_track() {
  resetDisplay();
  if (recentlyChanged && selectedTrack >= 0) {
    menuSelectionIndex = selectedTrack;
  }
  display.print(F("Select Layout: "));
  display.print(menuSelectionIndex+1);
  display.print(F("/"));
  display.println(numOfTracks);
  display.println(locations[selectedLocation]);
  display.setTextSize(2);

  if (numOfTracks < 3) {
    display.println();
    // small menu
    if (numOfTracks >= 1) {
      if (menuSelectionIndex == 0) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(tracks[0]);
    }
    if (numOfTracks >= 2) {
      if (menuSelectionIndex == 1) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(tracks[1]);
    }
  } else {
    // scrolling menu
    int indexA = menuSelectionIndex == numOfTracks - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfTracks - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(tracks[indexA]);
    display.print(F("->"));
    display.println(tracks[indexB]);
    display.print(F("  "));
    display.println(tracks[indexC]);
  }

  display.display();
}

void displayPage_select_direction() {
  resetDisplay();
  if (recentlyChanged && selectedDirection >= 0) {
    menuSelectionIndex = selectedDirection;
  }

  display.print(F("Select Direction"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Forward"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Reverse"));

  display.display();
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

  display.display();
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

  display.display();
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
    display.println(F("No .dove or .nmea"));
    display.println(F("files found!"));
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

  display.display();
}

void displayPage_replay_detecting() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("  Detecting Track"));
  display.println();

  display.setTextSize(2);
  display.println(F(" Please"));
  display.println(F("   Wait..."));

  display.display();
}

void displayPage_replay_select_track() {
  resetDisplay();

  if (recentlyChanged && selectedTrack >= 0) {
    menuSelectionIndex = selectedTrack;
  }

  display.print(F("Replay Layout: "));
  display.print(menuSelectionIndex + 1);
  display.print(F("/"));
  display.println(numOfTracks);
  display.println(locations[selectedLocation]);
  display.setTextSize(2);

  if (numOfTracks < 3) {
    display.println();
    if (numOfTracks >= 1) {
      display.print(menuSelectionIndex == 0 ? "->" : "  ");
      display.println(tracks[0]);
    }
    if (numOfTracks >= 2) {
      display.print(menuSelectionIndex == 1 ? "->" : "  ");
      display.println(tracks[1]);
    }
  } else {
    int indexA = menuSelectionIndex == numOfTracks - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfTracks - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(tracks[indexA]);
    display.print(F("->"));
    display.println(tracks[indexB]);
    display.print(F("  "));
    display.println(tracks[indexC]);
  }

  display.display();
}

void displayPage_replay_select_direction() {
  resetDisplay();

  if (recentlyChanged && selectedDirection >= 0) {
    menuSelectionIndex = selectedDirection;
  }

  display.println(F("Replay Direction"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Forward"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Reverse"));

  display.display();
}

void displayPage_replay_processing() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("  Processing Replay"));
  display.println();

  // Show file name (truncated)
  char displayName[22];
  strncpy(displayName, replayFiles[selectedReplayFile], 21);
  displayName[21] = '\0';
  display.println(displayName);
  display.println();

  // Show progress
  display.setTextSize(2);
  if (replayTotalSamples > 0) {
    display.print(F(" "));
    display.print(replayProcessedSamples);
    display.println();
    display.setTextSize(1);
    display.println(F(" samples processed"));
  } else {
    display.println(F(" Starting..."));
  }

  display.setTextSize(1);
  display.println();
  if (lapTimer.getLaps() > 0) {
    display.print(F("Laps found: "));
    display.println(lapTimer.getLaps());
  }

  display.display();
}

void displayPage_replay_results() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Replay Results"));

  display.print(F("Laps: "));
  display.println(lapTimer.getLaps());

  if (lapTimer.getLaps() > 0) {
    // Best lap with lap number
    display.print(F("Best: "));
    unsigned long bestTime = lapTimer.getBestLapTime();
    unsigned long minutes = bestTime / 60000;
    unsigned long seconds = (bestTime % 60000) / 1000;
    unsigned long milliseconds = bestTime % 1000;
    if (minutes > 0) {
      display.print(minutes);
      display.print(F(":"));
    }
    if (seconds < 10 && minutes > 0) display.print(F("0"));
    display.print(seconds);
    display.print(F("."));
    if (milliseconds < 100) display.print(F("0"));
    if (milliseconds < 10) display.print(F("0"));
    display.print(milliseconds);
    display.print(F(" (L"));
    display.print(lapTimer.getBestLapNumber());
    display.println(F(")"));

    // Optimal lap with sector numbers (only if track has sectors)
    if (trackLayouts[selectedTrack].hasSector2 || trackLayouts[selectedTrack].hasSector3) {
      display.print(F("Opt: "));
      unsigned long optTime = lapTimer.getOptimalLapTime();
      minutes = optTime / 60000;
      seconds = (optTime % 60000) / 1000;
      milliseconds = optTime % 1000;
      if (minutes > 0) {
        display.print(minutes);
        display.print(F(":"));
      }
      if (seconds < 10 && minutes > 0) display.print(F("0"));
      display.print(seconds);
      display.print(F("."));
      if (milliseconds < 100) display.print(F("0"));
      if (milliseconds < 10) display.print(F("0"));
      display.print(milliseconds);
      display.print(F(" ("));
      display.print(lapTimer.getBestSector1LapNumber());
      display.print(F(","));
      display.print(lapTimer.getBestSector2LapNumber());
      display.print(F(","));
      display.print(lapTimer.getBestSector3LapNumber());
      display.println(F(")"));
    }
  }

  display.print(F("Max Spd: "));
  display.print(replayMaxSpeed, 1);
  display.println(F(" mph"));

  if (replayMaxRpm > 0) {
    display.print(F("Max RPM: "));
    display.println(replayMaxRpm);
  } else {
    display.println();
  }

  display.println();
  display.println(F("<- Laps       Exit ->"));

  display.display();
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

  display.display();
}

void displayPage_gps_stats() {
  resetDisplay();

  // Safety: GPS stats page requires GPS to be initialized
  if (!gpsInitialized || gps == NULL) {
    display.println(F("GPS not\ninitialized"));
    display.display();
    return;
  }

  if (millis() - lastBatteryCheck > batteryUpdateInterval) {
    lastBatteryCheck = millis();
    lastBatteryVoltage = getBatteryVoltage();
  }
  display.print(F("Battery  : "));
  display.print("[");
  if (lastBatteryVoltage > 3.7) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 3.8) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 3.9) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 4.0) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 4.1) {
    display.print("#");
  } else {
    display.print(" ");
  }
  display.println("]");


  display.print(F("Sats     : "));
  display.println(gps->satellites);

  display.print(F("Rate     : "));
  if (gps->fix) {
    display.print(gpsFrameRate, 1);
    display.println(F("Hz"));
  } else {
    display.println(F("NO FIX"));
  }

  display.print(F("HDOP     : "));
  if (gps->fix) {
    display.println(gps->HDOP, 1);
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

  if (sdSetupSuccess && sdTrackSuccess) {
    display.print(F("Track : "));
    display.println(locations[selectedLocation]);
    display.print(F("Layout: "));
    display.print(selectedDirection == RACE_DIRECTION_FORWARD ? "->" : "<-");
    display.print(F(" "));
    display.println(tracks[selectedTrack]);
  } else {
    display.print(F("Autologging\nShut off to stop"));
  }

  display.display();
}

void displayPage_gps_speed() {
  resetDisplay();

  display.println(F("SPEED"));

  if (sdSetupSuccess && sdTrackSuccess) {
    int currentLap = lapTimer.getLaps() + (lapTimer.getRaceStarted() ? 1 : 0);
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
  if (gpsInitialized && gps != NULL && gps->fix) {
    display.println(round(gps_speed_mph));
  } else {
    display.println(F("--"));
  }

  display.display();
}

void displayPage_gps_lap_time() {
  resetDisplay();

  display.println(F("  Current Lap Time"));

  display.print(F("\n\n"));
  display.setTextSize(3);
  if (lapTimer.getRaceStarted()) {
    unsigned long minutes = lapTimer.getCurrentLapTime() / 60000;
    unsigned long seconds = (lapTimer.getCurrentLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getCurrentLapTime() % 1000;
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

  display.display();
}

void displayPage_gps_pace() {
  resetDisplay();

  display.println(F("  Current Lap Pace"));

  // animation
  if (lapTimer.getLaps() >= 1 && lapTimer.getPaceDifference() < (-1)) {
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
  if (lapTimer.getRaceStarted() && lapTimer.getLaps() >= 1) {
    display.setCursor(0, lineHeight);
    display.setTextSize(4);
    if (lapTimer.getPaceDifference() > 0) {
      display.print(F("+"));
    }
    display.print(lapTimer.getPaceDifference());
  } else {
    display.setTextSize(2);
    display.println();
    display.setTextSize(3);
    display.print(F("  N/A"));
  }

  // animation
  display.println();
  display.setTextSize(1);

  if (lapTimer.getLaps() >= 1 && lapTimer.getPaceDifference() < (-1)) {
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


  display.display();
}

void displayPage_gps_best_lap() {
  resetDisplay();

  display.println(F("      Best Lap"));
  display.print(F("\n"));
  if (
    lapTimer.getRaceStarted() &&
    lapTimer.getLaps() > 0
  ) {
    unsigned long minutes = lapTimer.getBestLapTime() / 60000;
    unsigned long seconds = (lapTimer.getBestLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getBestLapTime() % 1000;
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
    display.print(lapTimer.getBestLapNumber());
  } else {
    display.print(F("\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }

  display.display();
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

  display.display();
}

void displayPage_optimal_lap() {
  resetDisplay();

  display.println(F("     Optimal Lap"));
  if (
    lapTimer.getRaceStarted() &&
    lapTimer.getLaps() > 0
  ) {
    const int lineHeight = 15;
    display.setCursor(0, lineHeight);
    display.setTextSize(2);

    unsigned long minutes = lapTimer.getOptimalLapTime() / 60000;
    unsigned long seconds = (lapTimer.getOptimalLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getOptimalLapTime() % 1000;

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
    display.print(lapTimer.getBestSector1LapNumber());
    display.print(F("  "));
    display.print(lapTimer.getBestSector2LapNumber());
    display.print(F("  "));
    display.print(lapTimer.getBestSector3LapNumber());
  } else {
    display.print(F("\n\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }

  display.display();
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

  display.display();
}

void displayPage_stop_logging() {
  resetDisplay();

  display.setTextSize(2);
  display.println();
  display.println(F(" END RACE"));
  display.setTextSize(1);
  display.println();
  display.println(F(" press middle button"));

  display.display();
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

  display.display();
}

void displayPage_gps_debug() {
  resetDisplay();
  display.println(F("GPS LapTimer Debug"));

  // Safety check for GPS access
  if (!gpsInitialized || gps == NULL) {
    display.println(F("\nGPS not available"));
    display.display();
    return;
  }

  double dist2Line = lapTimer.pointLineSegmentDistance(gps->latitudeDegrees, gps->longitudeDegrees, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  display.print(F("DistToLine: "));
  display.println(dist2Line, 2);

  display.print(F("Laps: "));
  display.print(lapTimer.getLaps());
  display.print(F(" | od:"));
  display.println(lapTimer.getTotalDistanceTraveled());
  display.print(F("Strt: "));
  display.print(lapTimer.getRaceStarted() ? F("T") : F("F"));
  display.print(F(" | Xing: "));
  display.println(lapTimer.getCrossing() ? F("T") : F("F"));

  display.print(F("Current: "));
  display.println(lapTimer.getCurrentLapTime());
  display.print(F("Last   : "));
  display.println(lapTimer.getLastLapTime());
  display.print(F("Best: "));
  display.print(lapTimer.getBestLapNumber());
  display.print(F(": "));
  display.println(lapTimer.getBestLapTime());
  display.print(F("Pace   : "));
  display.println(lapTimer.getPaceDifference());

  display.display();
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
  display.display();
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
  display.display();
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

  display.display();
}
