#include "sd_format_page.h"

namespace sd_format_page {

void begin(State& s, uint32_t nowMs) {
  s.enteredAtMs = nowMs;
  s.holdSinceMs = 0;
  s.lastActivityMs = nowMs;
  s.holdArmed = false;
}

uint32_t holdSecondsLeft(const State& s, uint32_t nowMs) {
  if (!s.holdArmed) return 0;
  const uint32_t elapsed = nowMs - s.holdSinceMs;
  if (elapsed >= kHoldToConfirmMs) return 0;
  return (kHoldToConfirmMs - elapsed + 999) / 1000;
}

Exit step(State& s, const Inputs& in) {
  // Any button activity counts as "someone is standing at the device".
  if (in.selectHeld || in.otherButtonPressed) s.lastActivityMs = in.nowMs;

  // Confirm hold: Select held continuously for the full window. A release
  // disarms and the next hold restarts from scratch — a bump or EMI
  // flicker never accumulates toward an erase.
  if (in.selectHeld) {
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
