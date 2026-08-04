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

///////////////////////////////////////////
// TIME-SYNC PROGRESS (renderer hint)
//
// A position fix and a usable clock are separate milestones, and the
// second is much slower: `fullyResolved` needs the UTC/leap-second
// parameters, which the receiver decodes from the GPS navigation
// message — subframe 4 page 18, repeating every ~12.5 minutes. A clean
// 3D fix in under a minute followed by several more minutes of no
// timeValid is normal on a cold start, and weak signal makes it worse
// (tracking a satellite is a far lower bar than decoding its data bits
// without errors).
//
// The page used to collapse all of that into "FIX (time sync)", which
// reads as a fix *type* rather than "fix acquired, clock pending" — so
// a perfectly healthy device looked broken and got power-cycled, which
// restarts the 12.5-minute clock and makes it strictly worse. This
// classifier exists so the page can say which milestone is outstanding.
///////////////////////////////////////////
enum class TimeSync : uint8_t {
  kNoDateTime,  // module hasn't asserted validDate+validTime yet
  kResolving,   // date/time present, waiting on fullyResolved (the slow one)
  kLocked,      // all three — gpsData.timeValid is true
};

// Which time milestone is outstanding. `dateTimeValid` is validDate AND
// validTime; `fullyResolved` is the UTC-resolution bit.
TimeSync timeSyncState(bool dateTimeValid, bool fullyResolved);

// Countdown progress for the renderer: seconds remaining (1..3) while
// the auto-close countdown is armed, 0 when it is not.
uint32_t countdownSecondsLeft(const State& s, uint32_t nowMs);

void begin(State& s, uint32_t nowMs);

// Advance the state machine one step. kToMenu/kToRace/kToShutdown are
// terminal for this page visit; the caller switches pages and stops
// stepping.
Exit step(State& s, const Inputs& in);

}  // namespace gps_status_page
