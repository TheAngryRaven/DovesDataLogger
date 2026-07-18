#pragma once

#include <stdint.h>

///////////////////////////////////////////
// GPS STATUS BOOT PAGE STATE MACHINE
// Every boot lands on the GPS status page (MyChron-style satellite
// view). This unit owns the page's temporal behavior: hold until a
// stable lock, auto-close 3 s after fix + fully-resolved time, exit
// immediately on any button, pick the exit destination (main menu vs
// straight into race mode when the tach woke us / the engine is
// running), and give up to a full shutdown if nothing happens for the
// idle timeout. The rendering and the actual page switching stay in
// the sketch; this unit answers "should we leave, and to where?".
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace gps_status_page {

// Fix + timeValid must hold continuously this long before the page
// auto-closes. A flickering marginal lock restarts the full countdown.
constexpr uint32_t kAutoCloseMs = 3000;

// No lock armed and no engine activity for this long -> shut down.
// Keyed on *current* engine state (not the boot cause), so a spurious
// EMI tach wake with no follow-up pulses cannot strand the device
// awake and drain the pack. Matches SLEEP_IDLE_TIMEOUT_MS.
constexpr uint32_t kIdleTimeoutMs = 300000;

enum class Exit : uint8_t {
  kStay,        // keep holding the page
  kToMenu,      // leave to the main menu
  kToRace,      // leave straight into race mode (tach wake / engine on)
  kToShutdown,  // idle timeout — power the device back off
};

// Snapshot built fresh by the sketch each loop iteration.
struct Inputs {
  bool fix;            // gpsData.fix
  bool timeValid;      // gpsData.timeValid (fullyResolved date/time)
  bool buttonPressed;  // any button pressed this frame
  bool tachWakeBoot;   // boot wake cause was the tach pin
  bool engineRunning;  // live RPM above the auto-race threshold
  uint32_t nowMs;
};

struct State {
  uint32_t enteredAtMs = 0;
  uint32_t lockSinceMs = 0;      // when fix+timeValid last became true
  uint32_t lastActivityMs = 0;   // refreshed by engine activity
  bool lockArmed = false;        // countdown running
};

// Countdown progress for the renderer: seconds remaining (1..3) while
// the auto-close countdown is armed, 0 when it is not.
uint32_t countdownSecondsLeft(const State& s, uint32_t nowMs);

void begin(State& s, uint32_t nowMs);

// Advance the state machine one step. kToMenu/kToRace/kToShutdown are
// terminal for this page visit; the caller switches pages and stops
// stepping.
Exit step(State& s, const Inputs& in);

}  // namespace gps_status_page
