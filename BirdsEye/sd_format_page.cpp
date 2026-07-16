#include "sd_format_page.h"

namespace sd_format_page {

void begin(State& s, uint32_t nowMs) {
  s.holdSinceMs = 0;
  s.lastActivityMs = nowMs;
  s.holdArmed = false;
  s.selectSeenReleased = false;
}

uint32_t holdSecondsLeft(const State& s, uint32_t nowMs) {
  if (!s.holdArmed) return 0;
  const uint32_t elapsed = nowMs - s.holdSinceMs;
  if (elapsed >= kHoldToConfirmMs) return 0;
  return (kHoldToConfirmMs - elapsed + 999) / 1000;
}

Exit step(State& s, const Inputs& in) {
  // Any button activity counts as "someone is standing at the device",
  // and a running engine counts too — shutting down mid-session would
  // just tach-re-wake into this page in a 5-minute power cycle.
  if (in.selectHeld || in.otherButtonHeld || in.otherButtonPressed ||
      in.engineRunning) {
    s.lastActivityMs = in.nowMs;
  }

  // The wake press itself must never count toward an erase: a Select held
  // through the dark boot only becomes eligible after it is seen up once.
  if (!in.selectHeld) s.selectSeenReleased = true;

  // Confirm hold: Select held ALONE continuously for the full window. A
  // release disarms and the next hold restarts from scratch — a bump or
  // EMI flicker never accumulates toward an erase. A side button held
  // alongside Select also disarms: that's the user going for the global
  // Select+side reboot combo (5 s), which must never lose the race to a
  // 3 s erase.
  if (in.selectHeld && !in.otherButtonHeld && s.selectSeenReleased) {
    if (!s.holdArmed) {
      s.holdArmed = true;
      s.holdSinceMs = in.nowMs;
    }
    if (in.nowMs - s.holdSinceMs >= kHoldToConfirmMs) return Exit::kFormat;
  } else {
    s.holdArmed = false;  // released mid-hold — restart from scratch
  }

  // Idle give-up: an armed hold refreshes lastActivityMs every step, so
  // it can never be interrupted by the idle deadline.
  if (in.nowMs - s.lastActivityMs >= kIdleTimeoutMs) {
    return Exit::kToShutdown;
  }

  return Exit::kStay;
}

}  // namespace sd_format_page
