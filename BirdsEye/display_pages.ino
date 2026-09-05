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
#include "sd_functions.h"
#include "wake_cause.h"
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
  static const char* const kMenuItems[] = {"Race", "Drag", "Review", "Transfer", "Create", "Camera"
#if BIRDSEYE_ENABLE_SENSOREGG
    , "Egg"  // pairing + bench test for the wireless EGT pod (plan 0017)
#endif
  };
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

void displayPage_drag_distance() {
  resetDisplay();

  // Same scrolling 3-row window as the main menu, plus a title line
  // (title 8 px + 3 size-2 rows 48 px + hint line 8 px = the panel).
  display.setTextSize(1);
  display.println(F("    Drag Distance"));

  const int itemCount = drag_timer::kDistanceCount + 1;  // + Back
  const int visibleRows = 3;
  int first = menuSelectionIndex - 1;
  if (first < 0) first = 0;
  if (first > itemCount - visibleRows) first = itemCount - visibleRows;

  display.setTextSize(2);
  for (int i = first; i < first + visibleRows; i++) {
    display.print(menuSelectionIndex == i ? "->" : "  ");
    display.println(i < drag_timer::kDistanceCount ? drag_timer::label(i)
                                                   : "Back");
  }

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

void displayPage_drag_mode() {
  resetDisplay();

  // Title carries the distance picked one page back, so the choice being
  // confirmed is visible while picking how to run it.
  display.setTextSize(1);
  display.print(F("  Drag "));
  display.println(drag_timer::label(dragPendingDistanceIdx));
  display.println();

  // Three static size-2 rows (title 8 + blank 8 + 3x16 = the panel).
  static const char* const kRows[] = {"Auto", "Manual", "Back"};
  display.setTextSize(2);
  for (int i = 0; i < 3; i++) {
    display.print(menuSelectionIndex == i ? "->" : "  ");
    display.println(kRows[i]);
  }

  safeDisplayUpdate();
}

/**
 * @brief The pinned manual-staging screen (plan 0016): tree countdown,
 * live run, results, fouls — every glyph and flash decision comes from
 * the drag_tree unit (same state, same flash clock as the LED strip),
 * this function only lays it out.
 */
void displayPage_drag_staging() {
  resetDisplay();

  const drag_tree::Stage st = dragTreeStage();
  const uint32_t nowMs = millis();

  switch (st) {
    case drag_tree::Stage::kAwaitArm:
      display.setTextSize(1);
      display.print(F("  MANUAL "));
      display.println(dragDistanceLabel());
      display.println();
      display.setTextSize(2);
      display.println(F("  READY"));
      display.setTextSize(1);
      display.println();
      display.println(F("  press any button"));
      display.println(F("  hold SEL 2s: exit"));
      break;

    case drag_tree::Stage::kWaitStop:
      display.setTextSize(1);
      display.print(F("  MANUAL "));
      display.println(dragDistanceLabel());
      display.println();
      display.setTextSize(2);
      if (!gpsData.fix || !gpsData.timeValid) {
        display.println(F(" WAITING"));
        display.println(F(" FOR GPS"));
      } else if (gps_speed_mph >= drag_timer::kLaunchMinMph) {
        display.println(F(" STOP TO"));
        display.println(F("  STAGE"));
      } else {
        display.println(F("STAGING.."));
      }
      display.setTextSize(1);
      display.println(F("  hold SEL 2s: exit"));
      break;

    case drag_tree::Stage::kPreStage:
      display.setTextSize(1);
      display.println(F("  MANUAL STAGING"));
      display.println();
      display.setTextSize(2);
      display.println(F("  STAGED"));
      display.setTextSize(1);
      display.println();
      display.println(F("  hold still..."));
      break;

    case drag_tree::Stage::kYellow1:
    case drag_tree::Stage::kYellow2:
    case drag_tree::Stage::kYellow3:
      // One huge digit, the speed page's single-glyph placement.
      display.setCursor(43, 5);
      display.setTextSize(7);
      display.print(drag_tree::countdownDigit(st));
      break;

    case drag_tree::Stage::kGreen:
      // Flashes on the SAME phase as the strip (drag_tree::flashPhase),
      // so screen and LEDs agree; blank half-phases stay blank.
      if (drag_tree::flashPhase(nowMs)) {
        display.setCursor(22, 5);
        display.setTextSize(7);
        display.print(F("GO"));
      }
      break;

    case drag_tree::Stage::kRunning: {
      display.setTextSize(1);
      display.print(F("  Drag "));
      display.println(dragDistanceLabel());
      display.print(F("\n"));
      display.setTextSize(3);
      char lapStr[lap_format::kLapTimeStrLen];
      lap_format::formatLapTime(activeTimerCurrentLapTime(),
                                lap_format::kSpace, lapStr, sizeof(lapStr));
      display.print(lapStr);
      // Size-3 newline clears the ET's glyph row (see the lap page).
      display.print(F("\n"));
      display.setTextSize(1);
      display.print(F(" "));
      {
        unsigned long split60 = dragCurrent0to60Ms();
        if (split60 > 0) {
          display.print(F("0-60 "));
          displayPrintSplitSeconds(split60);
        }
      }
      break;
    }

    case drag_tree::Stage::kResults: {
      display.setTextSize(1);
      display.print(F("  RUN "));
      display.println(activeTimerLaps());
      display.setTextSize(3);
      char lapStr[lap_format::kLapTimeStrLen];
      lap_format::formatLapTime(activeTimerLastLapTime(),
                                lap_format::kSpace, lapStr, sizeof(lapStr));
      display.print(lapStr);
      // Size-3 newline clears the ET's glyph row (see the lap page).
      display.print(F("\n"));
      display.setTextSize(1);
      display.print(F(" "));
      displayPrintDragStats(dragLastTrapMph(), dragLast0to60Ms());
      if (dragLastReactionMs() > 0) {
        display.print(F("\n RT "));
        displayPrintSplitSeconds(dragLastReactionMs());
      }
      display.println();
      display.println(F(" press any button"));
      break;
    }

    case drag_tree::Stage::kRedLight:
    case drag_tree::Stage::kFailedLaunch:
    case drag_tree::Stage::kAborted:
      display.setTextSize(1);
      display.println();
      display.setTextSize(2);
      if (drag_tree::flashPhase(nowMs)) {
        if (st == drag_tree::Stage::kRedLight) {
          display.println(F("RED LIGHT"));
        } else if (st == drag_tree::Stage::kFailedLaunch) {
          display.println(F("FAILED TO"));
          display.println(F("  LAUNCH"));
        } else {
          display.println(F("   RUN"));
          display.println(F(" ABORTED"));
        }
      } else {
        display.println();
        display.println();
      }
      display.setTextSize(1);
      display.println();
      display.println(F("  press any button"));
      break;
  }

  safeDisplayUpdate();
}

// The ONE seconds.hundredths renderer for drag splits — both the results
// subtext and the pace page's live 0-60 readout go through it, so the
// format can't diverge between the two.
void displayPrintSplitSeconds(unsigned long ms) {
  display.print(ms / 1000);
  display.print(F("."));
  unsigned long hundredths = (ms % 1000) / 10;
  if (hundredths < 10) display.print(F("0"));
  display.print(hundredths);
}

// Trap-speed + 0-60 subtext shared by the drag results renderings below.
// The ET itself always goes through lap_format like every other time; the
// split is seconds to two decimals, "0-60" omitted if never reached.
void displayPrintDragStats(float trapMph, unsigned long split60Ms) {
  display.print(F("trap "));
  display.print(trapMph, 1);
  if (split60Ms > 0) {
    display.print(F("  0-60 "));
    displayPrintSplitSeconds(split60Ms);
  }
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

  if (bleTransferInProgress) {
    // The spacer row the else-branch keeps is spent on the second
    // diagnostic line below — with it the page is exactly 8 rows.
    display.print(F("Transfer: "));
    // bleFileSize is 0 for a zero-byte file — print 100% rather than divide
    // by zero.
    display.print(bleFileSize ? ((bleBytesTransferred * 100) / bleFileSize) : 100);
    display.println(F("%"));

    // The diagnostic line. A download that has gone slow used to show only a
    // creeping percentage, which says nothing about WHY — so this names the
    // three things that decide the rate: the SD clock actually in force (the
    // 8 MHz transfer bump falls back to 2 MHz silently), the negotiated
    // link-layer PDU (27 = Data Length Extension never happened, the biggest
    // tax there is), and the ATT payload per notification.
    display.print(bleTransferRateBps() / 1024);
    display.print(F("KB/s "));
    display.print(sdActiveSpiHz() / 1000000UL);
    display.print(F("M "));
    display.print(bleLinkDataLength());
    display.print(F(" "));
    display.println(bleLinkChunkSize());

    // Second diagnostic line (plan 0012): the two levers the line above
    // cannot show. The connection interval is the CENTRAL's choice — the
    // device only requests, and 30 ms instead of 15 ms is a silent 2x on
    // every download. The PHY doubles per-packet airtime if the 2M request
    // was ignored. A slow transfer with a healthy first line is one of
    // these two, or radio contention.
    display.print(bleLinkIntervalUnits() * 1.25, 1);
    display.print(F("ms "));
    const uint8_t phy = bleLinkPhy();
    display.println(phy == 2 ? F("2M")
                    : phy == 1 ? F("1M")
                    : phy == 4 ? F("Coded")
                               : F("?"));
  } else {
    display.println();
    display.println();
    display.println();
  }

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
  // Back matters more here than on most menus: both transfer modes leave by
  // rebooting, and this page is not the main menu so the idle-shutdown timer
  // never runs on it. Without this row, opening Transfer by mistake could
  // only be undone with the (unlabelled) reboot combo.
  display.print(menuSelectionIndex == 2 ? "->" : "  ");
  display.println(F("Back"));

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
  // silent (>1 s) / faulted. Entering this page latches the egg bench
  // mode (sensoreggTestEnterMode, plan 0017) so the race-gated scanner
  // actually runs here — the plan-0012 gate had silently broken the
  // desk-soak behavior this line was built for (it read NA off-track).
  // With the latch, this page is again the camera+egg coexistence soak
  // harness: camera linked above + egg streaming here, and the page
  // never idle-sleeps (idle-shutdown and USB-charging entries are
  // main-menu-only), so it can sit on a desk indefinitely. The EGG TEST
  // page (plan 0017) shows the full egg picture.
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

#if BIRDSEYE_ENABLE_SENSOREGG

void displayPage_pair_egg() {
  resetDisplay();

  if (sensoreggIsPaired()) {
    // Paired: stored MAC + Back/Test/Unpair. This branch also takes over
    // the frame after a capture (insideMenu is derived per frame). Back
    // first (index 0): a "Cancel" press landing one frame late must not
    // hit Unpair and erase the just-captured MAC (camera precedent).
    display.setTextSize(1);
    display.println(F("        EGG"));

    char mac[sensoregg_protocol::kMacStrLen];
    sensoreggPairedMac(mac, sizeof(mac));
    display.println(mac);  // 17 chars — a "Paired: " prefix would not fit

    display.setTextSize(2);
    display.print(menuSelectionIndex == 0 ? "->" : "  ");
    display.println(F("Back"));
    display.print(menuSelectionIndex == 1 ? "->" : "  ");
    display.println(F("Test"));
    display.print(menuSelectionIndex == 2 ? "->" : "  ");
    display.println(F("Unpair"));
  } else {
    // Unpaired: window-gated capture status. The scanner is forced on
    // while the window is open and observes EVERY egg in range; capture
    // waits for one advertising ITS OWN pairing window (the egg-side
    // long-press) — physical possession is the authorization.
    display.setTextSize(1);
    display.println(F("      PAIR EGG"));
    display.println();

    if (sensoreggPairingInProgress()) {
      display.println(F("Hold button on egg"));
      display.println(F("to start pairing..."));
      display.print(F("Egg: "));
      if (sensoreggLinkUp()) {
        display.print(F("heard v"));
        display.println(sensoreggProtoVersion());
      } else {
        display.println(F("---"));
      }
      display.print(F("Window: "));
      display.println(sensoreggPairingFlag() ? F("OPEN") : F("--"));
    } else {
      // Window closed without a capture (2-min timeout or a cancel that
      // landed while the page was still up).
      display.println(F("Pairing stopped"));
      display.println();
      display.println();
      display.println();
    }

    display.println();
    display.println(F("B2:Cancel"));
  }

  safeDisplayUpdate();
}

void displayPage_egg_test() {
  resetDisplay();

  // Eight size-1 rows, 21 chars each. Entering this page latched the
  // bench scan (sensoreggTestEnterMode), so everything below is live on
  // a desk — this is the egg's soak/diagnostic harness, and like the
  // camera test page it never idle-sleeps.
  display.setTextSize(1);

  // Row 0: title, link tri-state (HUNG outranks OK — packets arriving
  // but the sequence frozen means the egg needs a power cycle), and the
  // protocol version of the latest frame.
  display.print(F("EGG TEST rf:"));
  if (sensoreggAppHung()) {
    display.print(F("HUNG"));
  } else if (sensoreggLinkUp()) {
    display.print(F("OK"));
  } else {
    display.print(F("--"));
  }
  display.print(F(" v"));
  const uint8_t eggVer = sensoreggProtoVersion();
  if (eggVer == 0) {
    display.println(F("-"));
  } else {
    display.println(eggVer);
  }

  // Rows 1-2: temperatures (Fahrenheit at render — house rule; logging
  // stays Celsius) and battery. Every NaN check is isNanF: plain
  // isnan() folds to false under -Ofast.
  const float egtF = sensoregg_protocol::celsiusToFahrenheit(sensoreggEgtC());
  const float cjF =
      sensoregg_protocol::celsiusToFahrenheit(sensoreggJunctionC());
  const float auxF = sensoregg_protocol::celsiusToFahrenheit(sensoreggAuxC());
  display.print(F("EGT "));
  if (isNanF(egtF)) {
    display.print(F("---"));
  } else {
    display.print(egtF, 1);
    display.print(F("F"));
  }
  display.print(F(" CJ "));
  if (isNanF(cjF)) {
    display.println(F("---"));
  } else {
    display.print((int)lroundf(cjF));  // rounded: keeps the row <= 21 chars
    display.println(F("F"));
  }

  display.print(F("AUX "));
  if (isNanF(auxF)) {
    display.print(F("---"));  // v1 egg, stale link, or divider sentinel
  } else {
    display.print(auxF, 1);
    display.print(F("F"));
  }
  display.print(F(" BAT "));
  const uint8_t eggBatt = sensoreggBatteryPct();
  if (eggBatt == 0xFF) {
    display.println(F("--%"));
  } else {
    display.print(eggBatt);
    display.println(F("%"));
  }

  // Row 3: raw sequence counter (first real consumer of
  // sensoreggSequence()) + measured packet rate (~9-10 Hz healthy).
  display.print(F("SEQ "));
  display.print(sensoreggSequence());
  display.print(F("  "));
  display.print(sensoreggPacketHz(), 1);
  display.println(F("Hz"));

  // Row 4: live flags from the latest frame.
  display.print(F("FLG"));
  if (sensoreggPairingFlag()) display.print(F(" PAIR"));
  if (sensoreggTcFault()) display.print(F(" FAULT"));
  if (!sensoreggPairingFlag() && !sensoreggTcFault()) display.print(F(" -"));
  display.println();

  // Row 5: which egg the filter accepts.
  char eggMac[sensoregg_protocol::kMacStrLen];
  if (sensoreggPairedMac(eggMac, sizeof(eggMac))) {
    display.print(F("MAC "));  // 4 + 17 = 21 chars exactly
    display.println(eggMac);
  } else {
    display.println(F("MAC any (unpaired)"));
  }

  // Rows 6-7: spacer + the single menu row.
  display.println();
  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Back"));

  safeDisplayUpdate();
}

#endif  // BIRDSEYE_ENABLE_SENSOREGG

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

/**
 * @brief Draw one row of the session browser.
 *
 * Every row occupies exactly two panel lines — a filename wrapped onto a
 * second line when it exceeds the panel width, or a blank filler — so the
 * window arithmetic below can count rows instead of lines.
 *
 * `index == numReplayFiles` is the trailing Back row (see
 * replayItemCount()): browsing the sessions was otherwise a one-way door,
 * with no way out but opening a session and walking its exit page.
 */
static void replayDrawEntry(int index, bool selected) {
  display.print(selected ? F("->") : F("  "));

  if (index >= numReplayFiles) {
    display.println(F("Back"));
    display.println();
    return;
  }

  const int fileNameLen = strlen(replayFiles[index]);
  char displayName[20];

  strncpy(displayName, replayFiles[index], 19);
  displayName[19] = '\0';
  display.println(displayName);

  if (fileNameLen > 19) {
    display.print(F("  "));  // Indent to align with the first line
    strncpy(displayName, replayFiles[index] + 19, 19);
    displayName[19] = '\0';
    display.println(displayName);
  } else {
    display.println();  // Blank line if no wrap needed
  }
}

void displayPage_replay_file_select() {
  resetDisplay();

  const int itemCount = replayItemCount();

  display.print(F("Select Session: "));
  // The Back row is not a session, so it gets no "n/total" — counting it
  // would report one more session than the card holds.
  if (menuSelectionIndex < numReplayFiles) {
    display.print(menuSelectionIndex + 1);
    display.print(F("/"));
    display.println(numReplayFiles);
  } else {
    display.println();
  }
  display.println();
  display.setTextSize(1);

  if (numReplayFiles == 0) {
    display.println();
    display.println(F("No .dovex files"));
    display.println(F("found!"));
    display.println();
    display.println(F("Press any key"));
    display.println(F("to go back"));
  } else if (itemCount <= 3) {
    // Small menu — show everything, Back row included.
    for (int i = 0; i < itemCount; i++) {
      replayDrawEntry(i, menuSelectionIndex == i);
    }
  } else {
    // Scrolling menu: next / selected / previous, wrapping over the whole
    // item list so the Back row scrolls into view like any other row.
    const int next = menuSelectionIndex == itemCount - 1 ? 0 : menuSelectionIndex + 1;
    const int prev = menuSelectionIndex == 0 ? itemCount - 1 : menuSelectionIndex - 1;

    replayDrawEntry(next, false);
    replayDrawEntry(menuSelectionIndex, true);
    replayDrawEntry(prev, false);
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
  // Logging died mid-session (an SD write failure stops logging for the
  // session while the race deliberately continues). With the stats page
  // hidden by default (debug_pages), this corner flag is the rotation's
  // only signal that laps are no longer reaching the card.
  if (raceActive && !enableLogging) {
    display.setCursor(92, 0);
    display.print(F("NO LOG"));
    display.setCursor(0, 8);
  }

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

  if (dragModeIsActive()) {
    display.print(F("  Drag "));
    display.println(dragDistanceLabel());
  } else {
    display.println(F("  Current Lap Time"));
  }

  display.print(F("\n\n"));
  display.setTextSize(3);

  bool raceStarted = activeTimerRaceStarted();
  unsigned long currentLapTimeMs = activeTimerCurrentLapTime();

  if (dragModeIsActive() && !activeTimerRunActive()) {
    if (activeTimerLaps() > 0) {
      // Between runs with a result: show the last ET big, trap + 0-60
      // under it — the "what did I just run" glance at the top end.
      char lapStr[lap_format::kLapTimeStrLen];
      lap_format::formatLapTime(activeTimerLastLapTime(), lap_format::kSpace,
                                lapStr, sizeof(lapStr));
      display.print(lapStr);
      // First newline at size 3 so the advance clears the 24 px glyph
      // row — size-1 newlines (8 px) would put the subtext ON the ET.
      display.print(F("\n"));
      display.setTextSize(1);
      display.print(F(" "));
      displayPrintDragStats(dragLastTrapMph(), dragLast0to60Ms());
    } else {
      // No run yet: staged = clock armed, launch when ready.
      display.setTextSize(2);
      display.print(dragIsStaged() ? F(" *staged*") : F(" *waiting*"));
    }
  } else if (sprintModeIsActive() && !activeTimerRunActive()) {
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

  // Drag mode has no reference lap to pace against — this page becomes
  // the live 0-60 readout during a run instead.
  if (dragModeIsActive()) {
    display.println(F("     0-60 Split"));
  } else {
    display.println(F("  Current Lap Pace"));
  }

  int paceLaps = activeTimerLaps();
  float paceDiff = activeTimerPaceDifference();
  bool paceRaceStarted = activeTimerRaceStarted();
  // Engine died mid-session (plan 0007): the timer keeps running, but a
  // live-counting pace next to a dead engine is a lie — say STOPPED.
  // Also gates the notably-faster flash animation below.
  bool engineStopped = raceEngineStopped();

  // animation
  if (!engineStopped && paceLaps >= 1 && paceDiff < (-1)) {
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
  if (engineStopped) {
    // 7 chars, NOT 8 with a leading space: size 3 is an 18 px advance, so
    // " STOPPED" needs 144 px on a 128 px panel and the trailing D was
    // clipped on every render. 7 x 18 = 126 fits, and x=1 centres it.
    display.setCursor(1, lineHeight);
    display.setTextSize(3);
    display.print(F("STOPPED"));
  } else if ((sprintModeIsActive() || dragModeIsActive()) &&
             !activeTimerRunActive()) {
    // Sprint/drag, between runs — no live pace to compare (see lap page).
    display.setCursor(0, lineHeight);
    display.setTextSize(2);
    display.print(dragModeIsActive() && dragIsStaged() ? F(" *staged*")
                                                       : F(" *waiting*"));
  } else if (dragModeIsActive()) {
    // Run in progress: the split once 60 is crossed, dashes until then.
    display.setCursor(0, lineHeight);
    display.setTextSize(3);
    unsigned long split60 = dragCurrent0to60Ms();
    if (split60 > 0) {
      display.print(F(" "));
      displayPrintSplitSeconds(split60);
      display.print(F("s"));
    } else {
      display.print(F(" -.--"));
    }
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

  if (!engineStopped && paceLaps >= 1 && paceDiff < (-1)) {
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

    if (dragModeIsActive()) {
      // The best run's own trap + 0-60 (snapshotted with the best ET).
      display.setTextSize(1);
      display.print(F("\n\n "));
      displayPrintDragStats(dragBestTrapMph(), dragBest0to60Ms());
    }
  } else {
    display.print(F("\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }

  safeDisplayUpdate();
}

void displayPage_tachometer() {
  resetDisplay();

  // OVER REV header means ACTUAL overrev (plan 0007): it trips only at
  // the PROBLEM limit, and only when that limit is enabled — the
  // target_rpm warning flag is a status LED's job, not the header's.
  // (Was hardcoded >9999, which a 7600-limiter engine could never reach.)
  if (settingOverrevLimit > 0 && tachLastReported >= settingOverrevLimit) {
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
  } else if (raceActive && !enableLogging) {
    // Same "logging died" flag as the speed page — this is the tach
    // session's landing page, so the signal has to exist here too.
    display.print(F("  ** NOT LOGGING **"));
  } else if (runningPageStart <= GPS_DEBUG) {
    // Debug pages on: spend the subtext line on the tach diagnostic
    // instead of centring the max. Which estimator is running (S/L/R —
    // the `tach_filter` setting) and how many inter-pulse periods the
    // outlier gate has thrown away. A reject count that climbs with RPM
    // is the pickup, not the filter, and that distinction is the whole
    // reason this line exists. 19 of the 21 columns at text size 1.
    display.print(F("max:"));
    display.print(topTachReported);
    display.print(F(" "));
    display.print(tach_filter::modeTag(tachFilterMode));
    display.print(F(" rj:"));
    display.print(tachRejectedPeriods());
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

#if BIRDSEYE_ENABLE_PROFILING
/**
 * @brief LOOP PROFILE — where a main-loop iteration's time goes.
 *
 * First page of the race rotation on a profiling build (plan 0011).
 * Eight rows, all of them data:
 *
 *   14481Hz 61us mx42       loop rate, mean iteration, worst iteration
 *                           (mean in us below 1 ms, else ms; mx in ms)
 *   GPS 41.2 TCH  0.4       every section's share of the last second,
 *   ...                     as a percentage, two per row
 *   DSP 14.9 OTH  4.1       OTH = loop() time no section bracketed
 *   SLP 11.3                SLP = wall time not inside loop() at all
 *
 * All fourteen slots are shares of the same wall-clock second, so they
 * sum to ~100 (integer truncation loses a few tenths). If they do not,
 * something is wrong with the measurement, not with the firmware.
 *
 * SLP is the headroom number: scheduler dispatch, other FreeRTOS tasks,
 * and any time the CPU spent asleep. A large SLP means the loop rate is
 * NOT the thing limiting this firmware.
 *
 * Two markers can lead the first row. '*' means the DWT cycle counter
 * would not run and the numbers came from micros() instead — at 1 us
 * resolution most of these sections quantise to zero, so treat the small
 * ones as noise until that is fixed. '!' means the profiling pin was
 * refused (NFC pads never converted), so the scope is dark even though
 * these numbers are good. There is no title row: the rate line plus the
 * two-column grid is unmistakable, and the eighth row is worth more than
 * a caption.
 *
 * Shares are of WALL TIME, not of the iteration, so they are directly
 * comparable with what a scope reads off the profiling pin.
 */
void displayPage_profile() {
  resetDisplay();

  const loop_profile::Report& r = profilingReport();
  if (!r.valid) {
    display.println(F("LOOP PROFILE"));
    display.println();
    display.println(F("sampling..."));
    safeDisplayUpdate();
    return;
  }

  // The mean is rendered in WHOLE MICROSECONDS below a millisecond and
  // in milliseconds above it. The first hardware run came back reading
  // `999Hz av0.0` — a loop rate pinned at what was then a 999 clamp and
  // a mean quantised to nothing, because both fields had been sized
  // from the "~250 Hz / 4 ms iteration" figure this project had carried
  // in its docs for years. The real loop turned out to be an order of
  // magnitude faster than that, which is precisely the sort of thing
  // this page exists to find — so the clamps must not be the thing that
  // hides it.
  //
  // The row still cannot outgrow the 21-character line, because rate
  // and mean are reciprocal: a five-digit rate forces a sub-millisecond
  // (<=4-char) mean, and a mean big enough to need "99.9ms" forces a
  // three-digit rate. Text wrap is off, so an overflow would silently
  // truncate rather than wrap — worth the coupling argument.
  const uint32_t rate = (r.loopRateHz > 99999) ? 99999 : r.loopRateHz;
  const uint32_t maxMs = (r.loopMaxUs > 9999000) ? 9999 : (r.loopMaxUs / 1000);

  char mean[12];
  if (r.loopMeanUs < 1000) {
    snprintf(mean, sizeof(mean), "%luus", (unsigned long)r.loopMeanUs);
  } else {
    const uint32_t tenthMs =
        (r.loopMeanUs > 99900) ? 999 : (r.loopMeanUs / 100);
    snprintf(mean, sizeof(mean), "%lu.%lums", (unsigned long)(tenthMs / 10),
             (unsigned long)(tenthMs % 10));
  }

  // Sized well past the 21-column line so the compiler can prove no
  // truncation: 1 marker + 5 rate + "Hz " + 6 mean + " mx" + 4 max + NUL.
  char line[32];
  snprintf(line, sizeof(line), "%s%luHz %s mx%lu",
           profilingPinLive() ? "" : "!", (unsigned long)rate, mean,
           (unsigned long)maxMs);
  // '*' prefix = degraded timebase (see the doc comment); '!' = the
  // profiling pin was refused, so the scope is dark even though these
  // numbers are good. Both can apply; the timebase one wins the column
  // because it is the one that makes the numbers untrustworthy.
  if (profilingTimebaseTag()[0] != 'D') {
    display.print(F("*"));
  }
  display.println(line);

  // Seven rows of two slots covers all twelve sections plus OTH and SLP
  // with nothing left over — adding a section means finding it a row.
  char slot[12];
  for (uint8_t row = 0; row < 7; row++) {
    for (uint8_t col = 0; col < 2; col++) {
      const uint8_t idx = (uint8_t)(row * 2 + col);
      if (idx >= loop_profile::kReportSlots) break;
      if (col == 1) display.print(F(" "));
      const uint16_t pm = r.permille[idx];
      snprintf(slot, sizeof(slot), "%s %2lu.%lu", loop_profile::sectionTag(idx),
               (unsigned long)(pm / 10), (unsigned long)(pm % 10));
      display.print(slot);
    }
    display.println();
  }

  safeDisplayUpdate();
}
#endif  // BIRDSEYE_ENABLE_PROFILING

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
  // Line 2 (+3): what the last attempt said. Six size-1 rows fit under
  // the size-2 title, so the MOUNT case spends its spare row on the one
  // action that actually helps — re-formatting a card that formatted fine
  // but will not mount only loops.
  switch (sdFormatFailure) {
    case SD_FORMAT_FAIL_ERASE:
      display.println(F("Format FAILED - retry"));
      break;
    case SD_FORMAT_FAIL_MOUNT:
      display.println(F("Formatted: no mount"));
      display.println(F("Power-cycle the unit"));
      break;
    default:
      display.println(F("Card is not formatted"));
      break;
  }

  uint32_t secondsLeft = sd_format_page::holdSecondsLeft(sdFormatState, millis());
  if (secondsLeft > 0) {
    if (sdFormatFailure != SD_FORMAT_FAIL_MOUNT) display.println(F(""));
    display.print(F("Formatting in "));
    display.print(secondsLeft);
    display.println(F("s..."));
    display.println(F("Keep holding SELECT"));
  } else {
    display.println(F("Hold SELECT 3s to"));
    display.println(F("format the card"));
    display.println(F("(ERASES EVERYTHING)"));
  }
  // Diagnostic line: why this boot happened + SdFat's last card error.
  // "boot:WDT" here means the watchdog reset the device mid-boot (it
  // survives a soft reset) — the card is probably fine and mid-command,
  // and a power cycle, not an erase, is the fix.
  if (secondsLeft == 0) {
    display.print(F("boot:"));
    display.print(wake_cause::shortName(bootWakeCause));
    display.print(F(" err:"));
    if (sdLastErrorCode < 0x10) display.print('0');
    display.println(sdLastErrorCode, HEX);
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
  display.print(menuSelectionIndex == 2 ? F("->") : F("  "));
  display.println(F("Cancel"));

  safeDisplayUpdate();
}

void displayPage_course_type() {
  resetDisplay();

  // No blank line under the title: three size-2 rows (16 px each) plus the
  // header and the hint line fill the panel exactly, and "Cancel" has to
  // stay on-screen to be worth having.
  display.setTextSize(1);
  display.println(F("   COURSE TYPE"));
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Circuit"));
  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.println(F("Sprint"));
  display.print(menuSelectionIndex == 2 ? F("->") : F("  "));
  display.println(F("Cancel"));

  // The difference that matters when you are about to walk it. Blank on the
  // Cancel row — describing a course type the cursor is not on reads as a
  // description OF Cancel.
  display.setTextSize(1);
  if (menuSelectionIndex == 0) {
    display.println(F("one start/finish line"));
  } else if (menuSelectionIndex == 1) {
    display.println(F("start + finish lines"));
  }

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

/**
 * @brief "The track is full — drop the oldest runs?" (plan 0005).
 *
 * Only reached when at least one course being dropped still carries the name
 * the DEVICE gave it, so it has never been through the webapp and this card
 * may be the only place it exists. A drop of courses renamed in the app is
 * done quietly — there is nothing at stake to interrupt anyone for.
 */
void displayPage_course_prune() {
  resetDisplay();

  display.println(F("TRACK FULL"));
  display.println();
  display.print(F("Drop "));
  display.print(coursePruneDropCount);
  display.println(coursePruneDropCount == 1 ? F(" old run?") : F(" old runs?"));
  display.println(F("Not saved in app."));
  display.println();

  display.print(menuSelectionIndex == 0 ? F("->") : F("  "));
  display.println(F("Keep them"));
  display.print(menuSelectionIndex == 1 ? F("->") : F("  "));
  display.println(F("Drop + save"));

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
