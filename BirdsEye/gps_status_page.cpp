#include "gps_status_page.h"

namespace gps_status_page {

void begin(State& s, uint32_t nowMs) {
  s.enteredAtMs = nowMs;
  s.lockSinceMs = 0;
  s.lastActivityMs = nowMs;
  s.lockArmed = false;
}

uint32_t countdownSecondsLeft(const State& s, uint32_t nowMs) {
  if (!s.lockArmed) return 0;
  const uint32_t elapsed = nowMs - s.lockSinceMs;
  if (elapsed >= kAutoCloseMs) return 0;
  return (kAutoCloseMs - elapsed + 999) / 1000;
}

Exit step(State& s, const Inputs& in) {
  const Exit raceOrMenu =
      (in.tachWakeBoot || in.engineRunning) ? Exit::kToRace : Exit::kToMenu;

  // Any button skips the hold immediately — the page holds, it never locks.
  if (in.buttonPressed) return raceOrMenu;

  // Engine activity counts as "someone is using this thing".
  if (in.engineRunning) s.lastActivityMs = in.nowMs;

  // Stable-lock countdown: fix + fully-resolved time, held continuously.
  if (in.fix && in.timeValid) {
    if (!s.lockArmed) {
      s.lockArmed = true;
      s.lockSinceMs = in.nowMs;
    }
    if (in.nowMs - s.lockSinceMs >= kAutoCloseMs) return raceOrMenu;
  } else {
    s.lockArmed = false;  // dropped mid-countdown — restart from scratch
  }

  // Idle give-up: nothing locked, engine silent. Buttons exit above, so
  // the only way to sit here this long is a spurious wake or a device
  // left on a shelf without GPS view.
  if (!s.lockArmed && in.nowMs - s.lastActivityMs >= kIdleTimeoutMs) {
    return Exit::kToShutdown;
  }

  return Exit::kStay;
}

}  // namespace gps_status_page
