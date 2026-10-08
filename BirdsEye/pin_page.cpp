#include "pin_page.h"

namespace pin_page {

void begin(State& s) { s = State(); }

Action step(State& s, const Inputs& in) {
  if (!in.selectHeld) {
    s.selectSeenReleased = true;
    s.holdArmed = false;
    return Action::kNone;
  }
  const bool actionRow = in.row == kRowShow || in.row == kRowNewPin;
  if (!s.selectSeenReleased || in.otherButtonHeld || !actionRow) {
    s.holdArmed = false;
    return Action::kNone;
  }
  if (!s.holdArmed || s.holdRow != in.row) {
    s.holdArmed = true;
    s.holdRow = in.row;
    s.holdSinceMs = in.nowMs;
    return Action::kNone;
  }
  if (uint32_t(in.nowMs - s.holdSinceMs) < kHoldMs) return Action::kNone;

  // Fire once; a fresh hold needs a release first.
  s.holdArmed = false;
  s.selectSeenReleased = false;
  return in.row == kRowNewPin ? Action::kNewPin : Action::kReveal;
}

void reveal(State& s, uint32_t nowMs) {
  s.revealed = true;
  s.revealSinceMs = nowMs;
}

bool isRevealed(State& s, uint32_t nowMs) {
  if (s.revealed && uint32_t(nowMs - s.revealSinceMs) >= kRevealMs) s.revealed = false;
  return s.revealed;
}

uint32_t revealSecondsLeft(State& s, uint32_t nowMs) {
  if (!isRevealed(s, nowMs)) return 0;
  return (kRevealMs - uint32_t(nowMs - s.revealSinceMs) + 999u) / 1000u;
}

uint32_t holdSecondsLeft(const State& s, uint32_t nowMs) {
  if (!s.holdArmed) return 0;
  const uint32_t held = uint32_t(nowMs - s.holdSinceMs);
  if (held >= kHoldMs) return 1;
  return (kHoldMs - held + 999u) / 1000u;
}

}  // namespace pin_page
