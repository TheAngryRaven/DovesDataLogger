///////////////////////////////////////////
// DISPLAY PAGES MODULE
// All displayPage_*() rendering functions for each UI screen
///////////////////////////////////////////

#include "display_pages.h"  // also pulls in project.h's build feature flags
#include "crossing_pattern.h"
#include "gps_status_page.h"
#include "lap_format.h"
#include "sat_bars.h"
#include "sd_format_page.h"
#include "nan_bits.h"
#include "sensoregg_protocol.h"

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

// GPS status boot page (MyChron-style): top half is fix/satellite stats,
// bottom half is one vertical signal bar per satellite (height = CNO).
// Shown on every boot; hold/auto-close logic lives in gpsStatusPageLoop().
void displayPage_gps_status() {
  resetDisplay();

  // GPS never came up — GPS_STATUS_RETRY_LOOP() is re-probing in the
  // background (or has given up). Any button still exits to the menu.
  if (!gpsInitialized) {
    display.println(F("GPS: NOT DETECTED"));
    if (gpsRetriesExhausted()) {
      display.println(F("\nCHECK WIRING"));
    } else {
      display.println(F("\nRetrying..."));
    }
    display.println(F("\nAny button: menu"));
    display.print(F("\nup:"));
    display.print(millis() / 1000);
    display.println(F("s"));
    safeDisplayUpdate();
    return;
  }

  // ---- Top half (4 size-1 lines, 32 px) ----
  // used-in-solution / tracked-with-signal — the second number matches
  // the bar count below (until it exceeds the 16-bar display cap).
  display.print(F("Sats:"));
  display.print(gpsSatUsedCount);
  display.print(F("/"));
  display.print(gpsSatTrackedCount);
  display.print(F("  HDOP:"));
  if (gpsData.fix) {
    display.println(gpsData.HDOP, 1);
  } else {
    display.println(F("--"));
  }

  const uint32_t countdown =
      gps_status_page::countdownSecondsLeft(gpsStatusState, millis());
  if (countdown > 0) {
    display.print(F("LOCKED - ready in "));
    display.println(countdown);
  } else {
    if (gpsData.fix) {
      // "FIX ok" and not "FIX (time sync)": the old wording read as a fix
      // TYPE — a time-only, position-less fix — when it actually meant the
      // opposite (position is good, the clock isn't yet). That misreading
      // cost a bench session, and the natural reaction to it (power-cycle)
      // restarts the very countdown being waited on.
      display.print(F("FIX ok  UTC.. "));
    } else {
      display.print(F("ACQUIRING "));
    }
    // Uptime breadcrumb: if the device ever reboots off this page, the
    // last seconds value on screen identifies which timed path fired
    // (~5 s = PVT watchdog -> baud recovery, ~10 s = GPS re-detect retry).
    display.print(millis() / 1000);
    display.println(F("s"));
  }

  // Line 3 carries whichever is more useful right now: while the clock is
  // still catching up, WHICH milestone is outstanding (the whole point of
  // this line — a bare "no lock" gives the user nothing to wait for or act
  // on); once it is locked, the constellation the module is configured for.
  switch (gps_status_page::timeSyncState(gpsData.timeDateValid, gpsData.timeResolved)) {
    case gps_status_page::TimeSync::kNoDateTime:
      display.println(F("UTC: no date/time"));
      break;
    case gps_status_page::TimeSync::kResolving:
      // Bounded, not estimated: the UTC page repeats every ~12.5 min, so
      // this is the worst case, and seeing it stops the power-cycling.
      display.println(F("UTC: resolving <=12m"));
      break;
    case gps_status_page::TimeSync::kLocked:
      display.println(F("Mode:GPS-only"));
      break;
  }

  if (millis() - lastBatteryCheck > batteryUpdateInterval) {
    lastBatteryCheck = millis();
    lastBatteryVoltage = getBatteryVoltage();
  }
  display.print(F("Batt:"));
  display.print(getBatteryPercent(lastBatteryVoltage));
  display.print(F("% "));
  display.print(lastBatteryVoltage, 2);
  display.println(F("V"));

  // ---- Bottom half: per-satellite CNO bars rising from the baseline ----
  const int kBarAreaH = 28;   // bars live in y [63-kBarAreaH .. 62]
  const int kBaselineY = 63;
  display.drawFastHLine(0, kBaselineY, 128, DISPLAY_TEXT_WHITE);
  sat_bars::Bar bars[sat_bars::kMaxSats];
  const int barCount = sat_bars::layout(gpsSatCnos, gpsSatCnoCount, 128,
                                        kBarAreaH, bars, sat_bars::kMaxSats);
  for (int i = 0; i < barCount; i++) {
    if (bars[i].h <= 0) continue;
    display.fillRect(bars[i].x, kBaselineY - bars[i].h, bars[i].w, bars[i].h,
                     DISPLAY_TEXT_WHITE);
  }

  safeDisplayUpdate();
}

void displayPage_main_menu() {
  resetDisplay();

  // Scrolling menu: 3 size-2 rows (48 px) windowed over the items, plus
  // a size-1 scroll-hint line. Four full size-2 rows fill the panel's
  // nominal 64 px exactly, but the last row is cut off on real hardware
  // — so the window follows the selection instead.
  static const char* const kMenuItems[] = {"Race", "Review", "Transfer", "Create", "Camera"};
  const int itemCount = (int)(sizeof(kMenuItems) / sizeof(kMenuItems[0]));
  const int visibleRows = 3;

  // Keep the selection inside the window (max window start = count - rows).
  int first = menuSelectionIndex - 1;
  if (first < 0) first = 0;
  if (first > itemCount - visibleRows) first = itemCount - visibleRows;

  display.setTextSize(2);
  for (int i = first; i < first + visibleRows; i++) {
    display.print(menuSelectionIndex == i ? "->" : "  ");
    display.println(kMenuItems[i]);
  }

  // Scroll hints on the spare bottom line.
  display.setTextSize(1);
  if (first > 0) {
    display.print(F("^"));
  } else {
    display.print(F(" "));
  }
  if (first + visibleRows < itemCount) {
    display.print(F(" v more"));
  }

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

void displayPage_transfer_menu() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Transfer Mode"));
  display.println();
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Bluetooth"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("USB"));

  safeDisplayUpdate();
}

void displayPage_usb_storage() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   USB Storage"));
  display.println();

  display.setTextSize(2);
  display.println(F(" Drive On"));

  display.setTextSize(1);
  display.println();
  display.println(F("Connected to PC."));
  display.println(F("Drag & drop files."));
  display.println();
  display.println(F("->Exit (reboots)"));

  safeDisplayUpdate();
}

void displayPage_pair_camera() {
  resetDisplay();

  if (cameraIsPaired()) {
    // Paired: show the stored serial + Back/Test/Unpair menu. This branch
    // also takes over the frame after pairing captures a serial (FSM ->
    // kIdle). The three size-2 rows below start at y=16 and fill to y=64,
    // so the header stays tight (no blank lines) to keep "Unpair" on-panel.
    display.setTextSize(1);
    display.println(F("      CAMERA"));

    char serial[7];
    cameraPairedSerial(serial, sizeof(serial));
    display.print(F("Paired: "));
    display.println(serial);

    // Back first (index 0): the page flips from the pairing screen to
    // this menu the frame a serial is captured, and a "Cancel" press
    // landing one frame late must not hit Unpair and erase the
    // just-captured serial.
    display.setTextSize(2);
    display.print(menuSelectionIndex == 0 ? "->" : "  ");
    display.println(F("Back"));
    display.print(menuSelectionIndex == 1 ? "->" : "  ");
    display.println(F("Test"));
    display.print(menuSelectionIndex == 2 ? "->" : "  ");
    display.println(F("Unpair"));
  } else {
    // Unpaired: live pairing status from the camera FSM.
    display.setTextSize(1);
    display.println(F("     PAIR CAMERA"));
    display.println();

    if (cameraFsmState() == camera_fsm::State::kPairing) {
      if (cameraRemoteLinkUp()) {
        display.println(F("Connected -"));
        display.println(F("reading serial..."));
      } else {
        display.println(F("Power on camera"));
        display.println(F("nearby..."));
      }
    } else {
      // Pairing ended without a capture (e.g. 2-min timeout).
      display.println(F("Pairing stopped"));
      display.println();
    }

    display.println();
    display.println();
    display.println();
    display.println(F("B1:Manual B2:Cancel"));
  }

  safeDisplayUpdate();
}

void displayPage_camera_test() {
  resetDisplay();

  display.setTextSize(1);
  // Title, with the camera's OWN reported record state (its 0x10 timer) on
  // the right as an explicit rec:yes/no — proves a Record press actually
  // started the camera, not just that we sent a frame. (`rec:--` when there's
  // no fresh observation at all: no R-link, or the camera hasn't pushed a
  // 0x10 frame yet.)
  display.print(F("CAMERA TEST rec:"));
  if (!cameraRecordObservationFresh()) {
    display.println(F("--"));   // no fresh 0x10 (no link, or camera hasn't reported yet)
  } else {
    display.println(cameraObservedRecording() ? F("yes") : F("no"));
  }

  // Live link status so the tester can see what's actually connected:
  // R = remote (peripheral) link — the camera connects to us and must be
  // paired from its own Bluetooth-remote menu for this to come up.
  // R:UP+ = camera connected AND subscribed to ce82 (buttons deliverable);
  // R:UP without the + = connected but our button frames go nowhere.
  display.print(F("R:"));
  if (cameraRemoteLinkUp()) {
    display.print(cameraCe82Subscribed() ? F("UP+") : F("UP"));
  } else {
    display.print(F("--"));
  }
  // Adv: our advert actually on air — a silently-rejected wake/connect
  // advert shows Adv:-- (the "no blue LED" symptom).
  display.print(F(" Adv:"));
  display.print(cameraAdvertisingUp() ? F("UP") : F("--"));
  // G: the 10 Hz GPS/RMC feed to the camera. SYNC = streaming with a fix,
  // V = streaming but no lock (voided RMC — still a valid heartbeat), -- =
  // not streaming. Confirms the GPS link end-to-end. ("R:UP+ Adv:UP G:SYNC"
  // is 19 chars — fits the 21-char panel width.)
  display.print(F(" G:"));
  if (cameraGpsStreaming()) {
    display.println(gpsData.fix ? F("SYNC") : F("V"));
  } else {
    display.println(F("--"));
  }

  // Four size-1 rows follow — no blank line, so "Back" stays on-panel.
  // Wake burst only wakes a standby camera (see camera_ble.ino).
  static const char* const kTestItems[] = {
    "Wake", "Record", "Power Off", "Back"};
  const int itemCount = (int)(sizeof(kTestItems) / sizeof(kTestItems[0]));
  for (int i = 0; i < itemCount; i++) {
    display.print(menuSelectionIndex == i ? F("->") : F("  "));
    display.println(kTestItems[i]);
  }

#if BIRDSEYE_ENABLE_SENSOREGG
  // SensorEgg readout (bottom line): live Temp1 or NA when the egg is
  // silent (>1 s) / faulted. Makes this page the coexistence soak-test
  // harness: camera linked above + egg streaming here, and the page never
  // idle-sleeps (the idle-shutdown and USB-charging entries are
  // main-menu-only), so it can sit on a desk indefinitely.
  display.print(F("egg: "));
  const float soakEgtF = sensoregg_protocol::celsiusToFahrenheit(sensoreggEgtC());
  if (isNanF(soakEgtF)) {   // isNanF: plain isnan() folds to false under -Ofast
    display.println(F("NA"));
  } else {
    display.print(soakEgtF, 1);
    display.println(F("F"));
  }
#endif

  safeDisplayUpdate();
}

void displayPage_camera_serial_entry() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("    CAMERA SERIAL"));

  // Six entry characters, size 2 (12 px per column), left margin 16 px.
  display.setTextSize(2);
  display.setCursor(16, 16);
  for (int i = 0; i < 6; i++) {
    display.print(cameraSerialEntryBuf[i]);
  }

  // Caret under the character being edited (cursor 6/7 = OK/CANCEL row).
  if (cameraSerialEntryCursor < 6) {
    display.setCursor(16 + cameraSerialEntryCursor * 12, 34);
    display.print(F("^"));
  }

  // OK / CANCEL on the bottom line; the cursor target renders inverted.
  display.setTextSize(1);
  display.setCursor(28, 56);
  if (cameraSerialEntryCursor == 6) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.print(F(" OK "));
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.print(F("  "));
  if (cameraSerialEntryCursor == 7) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.print(F(" CANCEL "));
  display.setTextColor(DISPLAY_TEXT_WHITE);

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
    char lapStr[lap_format::kLapTimeStrLen];
    lap_format::formatLapTime(bestTime, lap_format::kOmit, lapStr, sizeof(lapStr));
    display.print(lapStr);
    display.print(F(" (L"));
    display.print(bestNum);
    display.println(F(")"));

    // Show optimal if available
    if (strcmp(dovexReplayOptimal, "N/A") != 0 && dovexReplayOptimal[0] != '\0') {
      display.print(F("Opt: "));
      unsigned long optMs = strtoul(dovexReplayOptimal, NULL, 10);
      lap_format::formatLapTime(optMs, lap_format::kOmit, lapStr, sizeof(lapStr));
      display.println(lapStr);
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

  // Pipeline-health summary: missing PVT frames and overflow events
  // (core RX ring / 4 KB ring). Full attribution on the debug page.
  display.print(F("Drops    : "));
  display.print(gpsStatsDroppedPvt());
  display.print(F(" Ovf:"));
  display.print(gpsStatsCoreSatEvents());
  display.print(F("/"));
  display.println(gpsStatsRingFullEvents());

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

  if (sprintModeIsActive() && !activeTimerRunActive()) {
    // Sprint mode, between runs: the session stays live (all pages work),
    // but there is no lap ticking — say so instead of a dead 0:00.
    display.setTextSize(2);
    display.print(F(" *waiting*"));
  } else if (raceStarted) {
    char lapStr[lap_format::kLapTimeStrLen];
    lap_format::formatLapTime(currentLapTimeMs, lap_format::kSpace, lapStr, sizeof(lapStr));
    display.print(lapStr);
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
  if (sprintModeIsActive() && !activeTimerRunActive()) {
    // Sprint mode, between runs — no live pace to compare (see lap page).
    display.setCursor(0, lineHeight);
    display.setTextSize(2);
    display.print(F(" *waiting*"));
  } else if (paceRaceStarted && paceLaps >= 1) {
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
    display.setTextSize(3);
    char lapStr[lap_format::kLapTimeStrLen];
    lap_format::formatLapTime(bestLapTimeMs, lap_format::kSpace, lapStr, sizeof(lapStr));
    display.print(lapStr);

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
  if (gpsLockHoldActive) {
    // The GPS-lock hold pins the user here with navigation disabled (see
    // displayLoop). Say so — a silent pin reads as a crash in the field.
    display.print(F("  WAITING GPS LOCK.."));
  } else {
    display.print(F("     max: "));
    display.print(topTachReported);
  }

  safeDisplayUpdate();
}

#if BIRDSEYE_ENABLE_SENSOREGG
// SensorEgg wireless EGT page — mirrors the tachometer layout: big value,
// small status subtext. NaN (stale link OR egg-reported invalid probe)
// renders '---'; the reading is NEVER held across a dropout.
// Rendered in Fahrenheit (DOVEX logging stays Celsius); a C/F display
// setting comes later.
void displayPage_sensorTemp() {
  resetDisplay();

  if (sensoreggTcFault()) {
    display.println(F("Temp1 F   *TC FAULT*"));
  } else {
    display.println(F("      Temp1 F"));
  }

  const float egt = sensoregg_protocol::celsiusToFahrenheit(sensoreggEgtC());

  display.setCursor(5, 20);
  display.setTextSize(4);
  // isNanF, not isnan: -Ofast folds isnan() to false, and this branch
  // then feeds lroundf(NaN) into %5d - the page showed "-214748" on a
  // stale link instead of '---'.
  if (isNanF(egt)) {
    display.println(F("  ---"));
  } else {
    char egtStr[8];
    snprintf(egtStr, sizeof(egtStr), "%5d", (int)lroundf(egt));
    display.println(egtStr);
  }

  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(F(" junc: "));
  const float junc = sensoregg_protocol::celsiusToFahrenheit(sensoreggJunctionC());
  if (isNanF(junc)) {
    display.print(F("---"));
  } else {
    display.print(junc, 1);
  }
  display.print(F("   rf: "));
  if (sensoreggAppHung()) {
    // Packets arriving but the egg's app is frozen (sequence not moving) —
    // its radio beacons the stale payload forever. Power-cycle the egg.
    display.print(F("HUNG"));
  } else {
    display.print(sensoreggLinkUp() ? F("OK") : F("--"));
  }

  safeDisplayUpdate();
}

// SensorEgg aux intake-air temp (Temp2, v2 eggs) — same layout and
// staleness rules as the Temp1 page. NaN ('---') also covers a v1 egg,
// which has no aux field at all. The subtext shows the egg's battery
// (real on v2 eggs; '--' = unknown/stale/v1) instead of a junction —
// the thermistor has no cold junction.
void displayPage_sensorTemp2() {
  resetDisplay();

  display.println(F("      Temp2 F"));

  const float aux = sensoregg_protocol::celsiusToFahrenheit(sensoreggAuxC());

  display.setCursor(5, 20);
  display.setTextSize(4);
  if (isNanF(aux)) {   // isNanF: isnan() folds to false under -Ofast
    display.println(F("  ---"));
  } else {
    char auxStr[8];
    snprintf(auxStr, sizeof(auxStr), "%5d", (int)lroundf(aux));
    display.println(auxStr);
  }

  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(F(" batt: "));
  const uint8_t pct = sensoreggBatteryPct();
  if (pct > 100) {   // 0xFF = unknown (stale link, v1 egg, no pack)
    display.print(F("--"));
  } else {
    display.print(pct);
    display.print(F("%"));
  }
  display.print(F("   rf: "));
  if (sensoreggAppHung()) {
    display.print(F("HUNG"));
  } else {
    display.print(sensoreggLinkUp() ? F("OK") : F("--"));
  }

  safeDisplayUpdate();
}
#endif  // BIRDSEYE_ENABLE_SENSOREGG

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

    char lapStr[lap_format::kLapTimeStrLen];
    lap_format::formatLapTime(optLapTimeMs, lap_format::kSpace, lapStr, sizeof(lapStr));
    display.print(lapStr);

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
        int actualLap = lap + 1;
        if (actualLap < 10) {
          display.print(F(" "));
        }
        display.print(actualLap);
        display.setTextSize(1);
        display.print(F(" "));
        display.setTextSize(2);
        char lapStr[lap_format::kLapTimeStrLen];
        lap_format::formatLapTime(lapHistory[lap], lap_format::kShow, lapStr, sizeof(lapStr));
        display.println(lapStr);
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
  display.println(F("GPS/RF DEBUG"));

  // Safety check for GPS access
  if (!gpsInitialized) {
    display.println(F("\nGPS not available"));
    safeDisplayUpdate();
    return;
  }

  // Serial-pipeline health (gps_stats + ISR counters): missing PVT
  // frames + live rate, worst TIMER3 deferral by radio ISRs, drain
  // burst high-water vs the core RX capacity, and overflow events
  // (core-ring saturations / 4 KB-ring fulls).
  display.print(F("Drops:"));
  display.print(gpsStatsDroppedPvt());
  display.print(F(" R:"));
  display.print(gpsFrameRate, 1);
  display.println(F("Hz"));
  display.print(F("ISRmax:"));
  display.print(gpsStatsIsrLatencyMaxUs());
  display.println(F("us"));
  display.print(F("Drain:"));
  display.print(gpsStatsDrainMaxBytes());
  display.print(F("/"));
  display.print(SERIAL_BUFFER_SIZE);
  display.print(F(" Ovf:"));
  display.print(gpsStatsCoreSatEvents());
  display.print(F("/"));
  display.println(gpsStatsRingFullEvents());

  // Lap-timer debug (trimmed to fit the 8-row page with the stats).
  display.print(F("Laps:"));
  display.print(activeTimerLaps());
  display.print(F(" Strt:"));
  display.print(activeTimerRaceStarted() ? F("T") : F("F"));
  display.print(F(" X:"));
  display.println(activeTimerCrossing() ? F("T") : F("F"));
  display.print(F("Cur : "));
  display.println(activeTimerCurrentLapTime());
  display.print(F("Best: "));
  display.print(activeTimerBestLapNumber());
  display.print(F(": "));
  display.println(activeTimerBestLapTime());
  display.print(F("Pace: "));
  display.println(activeTimerPaceDifference());

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

// Boot format-confirm page (PAGE_SD_FORMAT): the SD card answers but has
// no mountable FAT volume. Renders the hold-Select instructions + live
// countdown from the sd_format_page unit. The in-progress/done screens
// are painted by sdPerformFormat() via displayPage_sd_format_progress()
// (the format blocks the main loop, so displayLoop() never runs then).
void displayPage_sd_format() {
  resetDisplay();
  display.setCursor(0, 0);

  // Flashing header, same idiom as the fault/warning pages.
  notificationFlash = notificationFlash == true ? false : true;
  display.setTextSize(2);
  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("SD FORMAT"));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  if (sdFormatLastFailed) {
    display.println(F("Format FAILED - retry"));
  } else {
    display.println(F("Card is not formatted"));
  }

  uint32_t secondsLeft = sd_format_page::holdSecondsLeft(sdFormatState, millis());
  if (secondsLeft > 0) {
    display.println(F(""));
    display.print(F("Formatting in "));
    display.print(secondsLeft);
    display.println(F("s..."));
    display.println(F("Keep holding SELECT"));
  } else {
    display.println(F("Hold SELECT 3s to"));
    display.println(F("format the card"));
    display.println(F("(ERASES EVERYTHING)"));
  }
  safeDisplayUpdate();
}

// Static two-line status screen used by sdPerformFormat() for its
// "formatting" and "format OK" frames — painted directly because the
// format blocks the main loop and displayLoop() cannot run.
void displayPage_sd_format_progress(const __FlashStringHelper* line1,
                                    const __FlashStringHelper* line2) {
  resetDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println(F("SD FORMAT"));
  display.setTextSize(1);
  display.println(F(""));
  display.println(line1);
  display.println(line2);
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
    // Two-frame block animation, generated rather than stored: the frames
    // were 2 KB of PROGMEM describing eight 16x16 cells. The host-tested
    // crossing_pattern unit emits those cells and its golden test asserts
    // the raster is byte-identical to the bitmaps this replaced.
    calculatingFlip = calculatingFlip == true ? false : true;
    crossing_pattern::Rect cells[crossing_pattern::kMaxRects];
    const int cellCount =
        crossing_pattern::frameRects(calculatingFlip, cells, crossing_pattern::kMaxRects);
    for (int i = 0; i < cellCount; i++) {
      display.fillRect(cells[i].x, cells[i].y, cells[i].w, cells[i].h, DISPLAY_TEXT_WHITE);
    }
  #else
  #endif

  safeDisplayUpdate();
}

///////////////////////////////////////////
// ON-DEVICE COURSE CREATOR PAGES (plan 0002 §5)
//
// Five screens over the host-tested course_creator model. Every row shown
// here comes from course_creator::rowAt() rather than a local list, so a
// row can never render in one order and act in another.
///////////////////////////////////////////

// Shared header: what is being built and where it lands, so the user
// always knows whether they are walking a circuit or a sprint course.
//
// Budget is the panel's 21 size-1 characters and a track name can use 13
// of them (MAX_LOCATION_LENGTH), so the type is abbreviated to keep the
// name whole — the name is the part that answers "am I adding this to the
// right track?".
static void courseCreatorHeader() {
  display.setTextSize(1);
  display.print(courseCreator.kind == course_creator::CourseKind::kSprint
                    ? F("SPRINT @") : F("CIRC @"));
  display.println(courseCreator.newTrack ? "NEW" : courseCreatorTrackName);
}

void displayPage_course_track() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("  CREATE COURSE"));
  display.println(F("Are you at:"));
  display.setTextSize(2);
  display.println(courseCreatorTrackName);

  display.setTextSize(1);
  display.println();
  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Yes - add course"));
  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.println(F("No - new track"));

  safeDisplayUpdate();
}

void displayPage_course_type() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   COURSE TYPE"));
  display.println();
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Circuit"));
  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.println(F("Sprint"));

  // The difference that matters when you are about to walk it.
  display.setTextSize(1);
  display.println();
  display.println(menuSelectionIndex == 0 ? F("one start/finish line")
                                          : F("start + finish lines"));

  safeDisplayUpdate();
}

// Short reason Save is refused, for the Save row. Kept to the panel width.
static const __FlashStringHelper* courseSaveBlockText() {
  switch (course_creator::saveBlocked(courseCreator)) {
    case course_creator::SaveBlock::kStartMissing:  return F("need start");
    case course_creator::SaveBlock::kFinishMissing: return F("need finish");
    case course_creator::SaveBlock::kSectorPair:    return F("need S2+S3");
    case course_creator::SaveBlock::kSplitOrder:    return F("S2 before S3");
    case course_creator::SaveBlock::kNone:          return F("");
  }
  return F("");
}

// Why the last save attempt failed, or nullptr when nothing has failed.
static const __FlashStringHelper* courseSaveErrorText() {
  switch (courseCreatorLastError) {
    case SD_COURSE_WRITE_BUSY:     return F("SD busy - retry");
    case SD_COURSE_WRITE_NO_TRACK: return F("track file bad");
    case SD_COURSE_WRITE_TOO_BIG:  return F("track file full");
    case SD_COURSE_WRITE_IO:       return F("SD write failed");
    case SD_COURSE_WRITE_EXISTS:   return F("name taken");
    case SD_COURSE_WRITE_OK:       return nullptr;
  }
  return nullptr;
}

void displayPage_course_lines() {
  resetDisplay();
  courseCreatorHeader();

  const uint8_t rows = course_creator::rowCount(courseCreator);
  for (uint8_t i = 0; i < rows; i++) {
    const course_creator::RowRef ref = course_creator::rowAt(courseCreator, i);
    display.print(menuSelectionIndex == (int)i ? F("->") : F("  "));

    if (ref.row == course_creator::Row::kLine) {
      display.print(course_creator::lineLabel(ref.line, courseCreator.kind));
      if (course_creator::lineRequired(ref.line, courseCreator.kind)) {
        display.print(F("*"));
      }
      if (course_creator::lineDone(course_creator::lineOf(courseCreator, ref.line))) {
        display.print(F(" DONE"));
      }
      display.println();
    } else if (ref.row == course_creator::Row::kSave) {
      display.print(F("Save"));
      // Saying WHY beats a row that silently does nothing when pressed.
      if (!course_creator::canSave(courseCreator)) {
        display.print(F(" - "));
        display.print(courseSaveBlockText());
      }
      display.println();
    } else {
      display.println(F("Cancel"));
    }
  }

  const __FlashStringHelper* err = courseSaveErrorText();
  if (err != nullptr) display.print(err);

  safeDisplayUpdate();
}

void displayPage_course_line() {
  resetDisplay();

  display.setTextSize(1);
  display.print(F("LINE: "));
  display.println(course_creator::lineLabel(courseCreator.editing, courseCreator.kind));
  display.println();

  // Point rows read from the SCRATCH copy — what Save would commit, not
  // what is already stored. That is what makes Back a real undo.
  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.print(F("Point A"));
  display.println(courseCreator.scratch.hasA ? F(" DONE") : F(" *"));

  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.print(F("Point B"));
  display.println(courseCreator.scratch.hasB ? F(" DONE") : F(" *"));

  display.println();
  display.print(menuSelectionIndex == 2 ? F("->") : F("  "));
  display.println(F("Save line"));
  display.print(menuSelectionIndex == 3 ? F("->") : F("  "));
  display.println(F("Back (discard)"));

  safeDisplayUpdate();
}

void displayPage_course_point() {
  resetDisplay();

  display.setTextSize(1);
  display.print(course_creator::lineLabel(courseCreator.editing, courseCreator.kind));
  display.print(F(" : "));
  display.println(courseCreator.editingPointB ? F("B") : F("A"));

  const uint32_t now = millis();
  const course_creator::CaptureResult result =
      course_creator::capturePoll(courseCreator, now);

  if (result == course_creator::CaptureResult::kRunning) {
    // Hold-still feedback: the average is only as good as the user standing
    // still for it, so show both the countdown and the fix count.
    display.setTextSize(2);
    display.print(course_creator::capturePercent(courseCreator, now));
    display.println(F("%"));
    display.setTextSize(1);
    display.println(F("hold still..."));
    display.print(F("fixes: "));
    display.println(courseCreator.capture.fixes);
    if (courseCreator.capture.rejected > 0) {
      display.print(F("dropped: "));
      display.println(courseCreator.capture.rejected);
    }
    safeDisplayUpdate();
    return;
  }

  // Live accuracy, so the user can wait for the fix to settle before
  // starting a hold instead of discovering it afterwards.
  display.print(F("acc: "));
  if (gpsData.fix) {
    display.print(gpsData.horizontalAccuracy, 1);
    display.print(F("m"));
    if (gpsData.horizontalAccuracy > course_creator::kCaptureMaxHAccM) {
      display.println(F(" TOO POOR"));
    } else if (gpsData.horizontalAccuracy > course_creator::kCaptureWarnHAccM) {
      display.println(F(" weak"));
    } else {
      display.println();
    }
  } else {
    display.println(F("NO FIX"));
  }

  if (courseCreator.captureFailed) {
    display.println(F("too few fixes -"));
    display.println(F("try again"));
  } else {
    display.println();
    display.println();
  }

  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Save current pos"));
  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.println(F("Back"));

  safeDisplayUpdate();
}
