#pragma once

#include <stdint.h>

#include "led_frame.h"

///////////////////////////////////////////
// LED STRIP MODES + STATUS LED ACTIONS
// The decision math behind what the 9-px strip and the two status LEDs
// show. Three pieces:
//
//  - Pace pip: a single pixel that walks left/right of the centerline by
//    the live pace delta (activeTimerPaceDifference(), ms per METER,
//    positive = slower than best). Slower = LEFT of center in red,
//    faster = RIGHT in green, on-pace = dim white centerline only.
//  - Scale: a generic left-fill bar (ScaleSpec) — RPM against the rev
//    limit today, temperatures later. Lit pixels past a fill fraction
//    render the high color ("red past the halfway mark").
//  - Status actions: each status LED is driven by a StatusAction POD —
//    source + threshold + hysteresis + flash rate. This table IS the
//    phase-2 assignability hook: user settings will parse into the same
//    PODs; evalStatus() never changes.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace led_modes {

// ---- Pace pip ----------------------------------------------------------

// Pace delta (ms/m) that pins the pip to the end pixel. 4 steps per
// side, so one pixel step = 0.25 ms/m. On a ~1.2 km kart lap 1.0 ms/m
// is ~1.2 s/lap — a pin-the-needle delta. The OLED pace page treats
// -1.0 as "notably faster", consistent with this.
constexpr float kPaceFullScaleMsPerM = 1.0f;

// Half a pixel step: inside this the driver is "on pace" and only the
// centerline shows.
constexpr float kPaceDeadbandMsPerM = 0.125f;

// Centerline brightness (pre-cap): visible as a reference mark without
// competing with the pip.
constexpr uint8_t kPaceCenterLevel = 64;

struct PacePip {
  int stripIndex;  // 0..8 strip-relative; kStripCenter = on pace
  led_frame::Rgb color;
};

// Map a pace delta to pip position + color. Positive (slower) walks LEFT
// from center in red; negative (faster) walks RIGHT in green; |pace| <=
// deadband parks on the center in white. Clamps at the end pixels.
PacePip pacePip(float paceMsPerM);

// Render the full 9-px strip: dim white centerline + the pip.
void renderPace(float paceMsPerM, led_frame::Rgb out[led_frame::kStripCount]);

// ---- Generic scale (RPM now, temps later) ------------------------------

struct ScaleSpec {
  float min;  // fill starts here (0 lit)
  float max;  // full bar
  // Lit pixels at/past this FILL FRACTION of the bar render highColor
  // instead of lowColor.
  float redFrac;
  led_frame::Rgb lowColor;
  led_frame::Rgb highColor;
};

// User spec for the RPM bar: "starts green, towards a defined revlimit
// starts turning red past the halfway mark".
constexpr float kRpmRedFrac = 0.5f;

// Left-fill: lit count = round(fraction * 9), value clamped into
// [min, max]. Unlit pixels are off.
void renderScale(float value, const ScaleSpec& spec,
                 led_frame::Rgb out[led_frame::kStripCount]);

// ---- Status LED actions ------------------------------------------------

// What feeds an action. Phase 2 grows this enum as sources appear; the
// sketch glue owns mapping Source -> live value + validity.
enum class Source : uint8_t {
  kNone = 0,
  kRpm,
  kEgtC,
};

// One status LED's assignment. POD on purpose: the phase-2 settings
// parser fills these from SETTINGS.json; nothing else changes.
struct StatusAction {
  Source source;
  float threshold;   // fires at/above
  float clearBelow;  // releases below (hysteresis; must be < threshold)
  led_frame::Rgb color;
  uint16_t flashHalfPeriodMs;  // half-period of the on/off flash
  // Shown (solid) while the source is invalid/NaN — kOff for a source
  // whose absence means nothing (RPM reads 0, never NaN), a color for
  // one whose absence IS information (temp probe dropout -> blue).
  led_frame::Rgb invalidColor;
};

// Per-LED latch state, owned by the caller.
struct StatusState {
  bool active = false;
};

// EGT alert default: ~1200 F, a typical 2T kart EGT ceiling — the
// compiled-in default behind the `temp1_alert_c` setting. The clear
// point sits kEgtClearDeltaC below whatever threshold is configured;
// hysteresis keeps the flasher from chattering on sensor noise.
constexpr float kEgtAlertC = 650.0f;
constexpr float kEgtClearDeltaC = 20.0f;

// Rev flasher clears at this fraction of the threshold — deep enough
// that filter jitter at the limiter doesn't strobe the latch.
constexpr float kRevClearFrac = 0.97f;

constexpr uint16_t kRevFlashHalfPeriodMs = 100;  // urgent
constexpr uint16_t kEgtFlashHalfPeriodMs = 250;  // noticeable, calmer

// Evaluate one action. valid=false (stale/NaN input — the glue decides,
// isNanF-guarded) releases the latch and shows the action's
// invalidColor (solid): a stale source must never keep an alert
// flashing (house rule: never latch stale data), but its absence can
// still be shown. Flash phase derives from nowMs so all timing is
// host-testable.
led_frame::Rgb evalStatus(const StatusAction& a, StatusState& s, float value,
                          bool valid, uint32_t nowMs);

// ---- GPS-search pip ------------------------------------------------------

// Full round-trip time of the bouncing pip (end -> end -> back).
constexpr uint32_t kSearchBouncePeriodMs = 1600;

// One green pixel bouncing 0<->8 as a triangle wave of elapsed time —
// the race strip's "GPS not locked yet" state. Pure function of tMs.
void renderSearchPip(uint32_t tMs, led_frame::Rgb out[led_frame::kStripCount]);

}  // namespace led_modes
