#pragma once

#include <stdint.h>

///////////////////////////////////////////
// TRANSFER PIN PAGE STATE MACHINE (plan 0019)
// Transfer → PIN: the one place the remote-transfer PIN is shown on the
// device, and where a lost one is replaced. With the SD card soldered in,
// this page (plus the USB drive and a local Bluetooth start) is the whole
// recovery story — but the PIN must not sit on screen for anyone walking
// past, so both actions are deliberate holds:
//   - Show:    hold Select 3 s → PIN visible for 15 s, then hidden again.
//   - New PIN: hold Select 3 s → a fresh PIN is generated and shown.
//   - Back:    an ordinary press (handled by the menu code, not here).
// A release, a change of row, or a side button held alongside Select (the
// Select+side reboot combo) cancels a hold with no partial credit. A
// Select still down from the press that opened the page never counts —
// it must be seen released first, exactly like the SD format confirm.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace pin_page {

constexpr uint32_t kHoldMs = 3000;
constexpr uint32_t kRevealMs = 15000;

// Row order on the page. Back is last, as on every menu.
constexpr int kRowShow = 0;
constexpr int kRowNewPin = 1;
constexpr int kRowBack = 2;
constexpr int kRowCount = 3;

enum class Action : uint8_t {
  kNone,
  kReveal,  // show the stored PIN
  kNewPin,  // generate + store a new PIN, then show it
};

struct Inputs {
  bool selectHeld = false;
  bool otherButtonHeld = false;
  int row = kRowShow;
  uint32_t nowMs = 0;
};

struct State {
  bool selectSeenReleased = false;
  bool holdArmed = false;
  int holdRow = kRowShow;
  uint32_t holdSinceMs = 0;
  bool revealed = false;
  uint32_t revealSinceMs = 0;
};

// Page entry: everything hidden, nothing armed.
void begin(State& s);

// Advance one loop iteration.
Action step(State& s, const Inputs& in);

// Start (or restart) the 15 s reveal window — the caller does this once it
// has the digits to show, after kReveal or a successful kNewPin.
void reveal(State& s, uint32_t nowMs);

// True while the PIN should be on screen. Expires the window as a side
// effect so the caller can drop its copy of the digits.
bool isRevealed(State& s, uint32_t nowMs);

// Seconds left in the reveal (1..15), 0 when hidden.
uint32_t revealSecondsLeft(State& s, uint32_t nowMs);

// Seconds left in a running hold (1..3), 0 when none.
uint32_t holdSecondsLeft(const State& s, uint32_t nowMs);

}  // namespace pin_page
