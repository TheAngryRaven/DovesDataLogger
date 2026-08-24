#pragma once

#include <stdint.h>

#include "led_frame.h"
#include "led_modes.h"
#include "sector_purple.h"

///////////////////////////////////////////
// ASSIGNABLE STATUS-LED MODES
// The two status pixels (led_frame::kStatusLeft / kStatusRight) each
// render whatever the user assigned them from the companion app. This
// is the "phase 2" led_modes.h has always pointed at: the evaluation
// machinery does not change, only where the assignment comes from.
//
// Before plan 0013 both pixels were hardcoded — left the rev flasher,
// right the SensorEgg Temp1 tri-state. The right one was the bug: on a
// build without BIRDSEYE_ENABLE_SENSOREGG the accessor is permanently
// NaN, so the pixel showed a solid blue "no probe signal" for every
// second of every race on every stock logger. Hence Inputs.eggSupported
// below, and test case "egt on an unsupported build is OFF, never blue".
//
// THRESHOLD MODES DELEGATE. kRpm / kSpeed / kEgt build a
// led_modes::StatusAction on the stack and hand it to
// led_modes::evalStatus() — hysteresis, latch-release-on-invalid and
// flash phase stay in one place, with the regression tests they already
// have (including the inverted-hysteresis strobe guard). This unit owns
// the MODE table; led_modes keeps owning threshold evaluation.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
// All timing arrives as nowMs; nothing here calls millis().
///////////////////////////////////////////

namespace led_status {

// What a status LED is assigned to show. Persisted as the short
// lowercase token in modeName() — these strings are stored in
// /SETTINGS.json, which has a hard 1023-byte ceiling, so keep them short.
enum class Mode : uint8_t {
  kOff = 0,   // "off"
  kRpm,       // "rpm"     flash at/above target_rpm (the old rev flasher)
  kSpeed,     // "speed"   flash at/above target_speed_mph
  kGps,       // "gps"     sats / fix / time-lock quality
  kCamera,    // "camera"  paired / linked / subscribed / recording
  kLap,       // "lap"     last lap vs the lap before it
  kSector,    // "sector"  last closed sector vs the same sector last lap
  kEgt,       // "egt"     Temp1 tri-state (SensorEgg builds only)
  kCount,
};

// STRICT parse — deliberately unlike tach_filter::modeFromSetting(),
// which degrades every unknown value to a safe default. Here a typo
// must not silently pick a different mode or dark a status LED, so
// false means "keep the compiled-in default", the same contract
// setting_parse::parseIntSetting() has. Case-insensitive exact match;
// rejects nullptr, "", " ", "rp", "rpmm", "Off ".
bool parseMode(const char* s, Mode* out);

// Canonical token for a mode. Never null; out-of-range answers "off".
// modeName(m) always re-parses to m — the setting round-trips through
// the companion app's SGET/SLIST unchanged, even for a mode this build
// cannot render.
const char* modeName(Mode m);

// True for modes that mean something with no session running, so the
// glue can keep them lit on the main menu. GPS lock and camera
// readiness are exactly what you want to see from the paddock; a pace
// or rev indicator on a parked device is just glow.
bool activeOutsideRace(Mode m);

// "Cannot command the camera" nag: slow enough to read as deliberate,
// fast enough to distinguish from the steady blue next to it.
constexpr uint16_t kCameraFlashHalfPeriodMs = 400;

// GPS quality ladder, worst to best.
//
// NOTE ON kSats: gpsData.satellites is NAV-PVT's numSV, which counts
// satellites USED IN THE NAV SOLUTION, not satellites in view. NAV-SAT
// (the in-view figure) is switched off in race mode, so numSV is all
// there is. The practical effect is that the yellow band is briefer
// than "sats but no fix" sounds — numSV climbs off zero as satellites
// join the solution, shortly before the fix lands.
enum class GpsLevel : uint8_t { kNoSats = 0, kSats, kFix, kLocked };

// Camera readiness ladder. kLinkPartial is connected-but-not-subscribed
// to ce82: the camera is there but our button frames go nowhere, so it
// flashes rather than sitting steady.
enum class CamLevel : uint8_t {
  kUnpaired = 0, kIdle, kLinkPartial, kLinkReady, kRecording,
};

using Verdict = sector_purple::Verdict;

// One frame's snapshot of every source any mode can read. The glue
// builds this ONCE per frame and evaluates it twice, one per pixel.
struct Inputs {
  // -- threshold sources --
  float rpm = 0.0f;             // tachLastReported; 0 when stopped, never NaN
  float targetRpm = 0.0f;       // settingTargetRpm
  float speedMph = 0.0f;        // gps_speed_mph (mph, NOT gpsData.speed)
  bool speedValid = false;      // gpsInitialized && gpsData.fix
  float targetSpeedMph = 0.0f;  // settingTargetSpeedMph
  float egtC = 0.0f;            // sensoreggEgtC()
  bool egtValid = false;        // !isNanF(egtC) — the glue does the isNanF
  float egtAlertC = 0.0f;       // settingTemp1AlertC
  // THE build gate. False on a BIRDSEYE_ENABLE_SENSOREGG=0 image, where
  // kEgt renders OFF rather than the solid-blue "no probe signal" — on
  // such a build there is no probe to lose, so blue would be a lie.
  bool eggSupported = false;

  // -- gps --
  int gpsSats = 0;
  bool gpsFix = false;
  bool gpsTimeValid = false;

  // -- camera --
  bool cameraPaired = false;
  bool sessionActive = false;  // raceActive
  bool cameraLinkUp = false;
  bool cameraSubscribed = false;
  bool cameraRecording = false;

  // -- held verdicts from the sector_purple monitor --
  Verdict lapVerdict = Verdict::kNone;
  Verdict sectorVerdict = Verdict::kNone;
};

// Per-LED latch. ONLY the threshold modes latch; gps/camera/lap/sector
// are pure functions of Inputs (their "hold" lives in the timing
// monitor, not here). A struct rather than a bare StatusState so a
// future mode can add state without touching every call site.
struct State {
  led_modes::StatusState threshold;
};

// Release the latch — called when a session ends, so a stale alert
// can't flash the instant the next one starts.
void reset(State& s);

// Classify the raw GPS/camera booleans into their ladders. Free
// functions so the glue stays a straight translation and the ladder
// rules are host-tested independently of the colour mapping.
GpsLevel gpsLevel(int sats, bool fix, bool timeValid);
CamLevel cameraLevel(bool paired, bool sessionActive, bool linkUp,
                     bool subscribed, bool recording);

// Evaluate one status LED. Authors colours in full 0-255;
// led_frame::applyCap() still owns brightness at push time.
led_frame::Rgb evalMode(Mode m, const Inputs& in, State& s, uint32_t nowMs);

}  // namespace led_status
