#pragma once

#include <stdint.h>

///////////////////////////////////////////
// SD FORMAT CONFIRM PAGE STATE MACHINE
// Shown at boot when the SD card answers at the SPI level but its FAT
// volume will not mount (factory-blank or corrupted card). With the
// module soldered in there is no pulling the card to format it on a
// PC, so the device must offer to format it itself — but never without
// a deliberate confirmation: hold Select ALONE continuously for the
// full confirm window. A release restarts the window from scratch; the
// other buttons only count as activity (deferring the idle shutdown),
// they can never confirm — and holding one alongside Select DISARMS
// the confirm, so the global Select+side reboot combo (5 s, any page)
// can never be beaten to the punch by a 3 s format. A Select that was
// already down at page entry (the wake press itself, still held through
// a dark boot) never counts — the hold can only arm after Select has
// been seen released once. Sitting idle powers the device back off;
// a running engine counts as activity so a tach-wake with a bad card
// doesn't shutdown/re-wake in a 5-minute power cycle all session.
//
// The rendering, the button sampling, and the actual format stay in
// the sketch; this unit answers "format now, keep waiting, or give up?".
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace sd_format_page {

// Select must be held continuously this long to confirm the format.
// A release before the threshold disarms and a re-hold restarts the
// full window — no credit for partial holds.
constexpr uint32_t kHoldToConfirmMs = 3000;

// No button activity for this long -> shut down. Matches the GPS
// status page / SLEEP_IDLE_TIMEOUT_MS so a spurious wake with an
// unformatted card cannot strand the device awake and drain the pack.
constexpr uint32_t kIdleTimeoutMs = 300000;

enum class Exit : uint8_t {
  kStay,        // keep holding the page
  kFormat,      // confirm hold completed — format the card
  kToShutdown,  // idle timeout — power the device back off
};

// Snapshot built fresh by the sketch each loop iteration.
struct Inputs {
  bool selectHeld;          // live debounced Select (B2) level
  bool otherButtonHeld;     // live B1/B3 level — disarms the confirm hold
  bool otherButtonPressed;  // B1/B3 pressed edge this frame (activity only)
  bool engineRunning;       // live RPM above the auto-race threshold
  uint32_t nowMs;
};

struct State {
  uint32_t holdSinceMs = 0;     // when the confirm hold last armed
  uint32_t lastActivityMs = 0;  // refreshed by button/engine activity
  bool holdArmed = false;       // confirm hold running
  bool selectSeenReleased = false;  // Select observed up since page entry
};

// Countdown progress for the renderer: seconds remaining (1..3) while
// the confirm hold is armed, 0 when it is not.
uint32_t holdSecondsLeft(const State& s, uint32_t nowMs);

void begin(State& s, uint32_t nowMs);

// Advance the state machine one step. kFormat/kToShutdown are terminal
// for this page visit; the caller acts and stops stepping.
Exit step(State& s, const Inputs& in);

}  // namespace sd_format_page
