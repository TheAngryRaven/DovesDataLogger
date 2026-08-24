#include "led_status.h"

#include <stddef.h>

namespace led_status {

namespace {

// Indexed by Mode. Kept in enum order so modeName() is a lookup and
// parseMode() is a walk — one table, no chance of the two disagreeing.
const char* const kModeNames[] = {
    "off", "rpm", "speed", "gps", "camera", "lap", "sector", "egt",
};
static_assert(sizeof(kModeNames) / sizeof(kModeNames[0]) ==
                  static_cast<size_t>(Mode::kCount),
              "kModeNames must have one entry per Mode");

// ASCII lower — no locale, no <cctype> (which would drag a locale table
// into a firmware image for eight lowercase tokens).
char lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

bool equalsIgnoreCase(const char* a, const char* b) {
  while (*a != '\0' && *b != '\0') {
    if (lower(*a) != lower(*b)) {
      return false;
    }
    a++;
    b++;
  }
  return *a == '\0' && *b == '\0';  // both ended: no prefix matches
}

// The two threshold modes and the temp mode share this shape.
led_frame::Rgb evalThreshold(led_modes::Source source, float threshold,
                             float clearBelow, led_frame::Rgb color,
                             uint16_t halfPeriodMs,
                             led_frame::Rgb invalidColor, float value,
                             bool valid, led_modes::StatusState& latch,
                             uint32_t nowMs) {
  const led_modes::StatusAction action = {source,           threshold,
                                          clearBelow,       color,
                                          halfPeriodMs,     invalidColor};
  return led_modes::evalStatus(action, latch, value, valid, nowMs);
}

led_frame::Rgb verdictColor(Verdict v) {
  switch (v) {
    case Verdict::kBetter:
      return led_frame::kGreen;
    case Verdict::kWorse:
      return led_frame::kRed;
    case Verdict::kBest:
      return led_frame::kPurple;
    case Verdict::kNone:
    default:
      // Nothing to compare against yet. Dark is the honest answer — a
      // colour here would be a guess the driver would act on.
      return led_frame::kOff;
  }
}

}  // namespace

bool parseMode(const char* s, Mode* out) {
  if (s == nullptr || out == nullptr || s[0] == '\0') {
    return false;
  }
  for (size_t i = 0; i < static_cast<size_t>(Mode::kCount); i++) {
    if (equalsIgnoreCase(s, kModeNames[i])) {
      *out = static_cast<Mode>(i);
      return true;
    }
  }
  return false;
}

const char* modeName(Mode m) {
  const size_t i = static_cast<size_t>(m);
  if (i >= static_cast<size_t>(Mode::kCount)) {
    return kModeNames[0];
  }
  return kModeNames[i];
}

bool activeOutsideRace(Mode m) {
  return m == Mode::kGps || m == Mode::kCamera;
}

void reset(State& s) {
  s.threshold.active = false;
}

GpsLevel gpsLevel(int sats, bool fix, bool timeValid) {
  if (fix) {
    return timeValid ? GpsLevel::kLocked : GpsLevel::kFix;
  }
  return sats > 0 ? GpsLevel::kSats : GpsLevel::kNoSats;
}

CamLevel cameraLevel(bool paired, bool sessionActive, bool linkUp,
                     bool subscribed, bool recording) {
  if (!paired) {
    return CamLevel::kUnpaired;
  }
  // Recording is checked before the link ladder: if the camera is
  // rolling, that is the fact worth showing regardless of whether our
  // subscription happens to be up at this instant.
  if (recording) {
    return CamLevel::kRecording;
  }
  if (linkUp) {
    return subscribed ? CamLevel::kLinkReady : CamLevel::kLinkPartial;
  }
  // Paired but no link. Only worth flagging once a session is running —
  // a parked, powered-off camera is not a problem.
  return sessionActive ? CamLevel::kIdle : CamLevel::kUnpaired;
}

led_frame::Rgb evalMode(Mode m, const Inputs& in, State& s, uint32_t nowMs) {
  switch (m) {
    case Mode::kRpm:
      // Byte-for-byte the pre-0012 left status LED.
      return evalThreshold(led_modes::Source::kRpm, in.targetRpm,
                           in.targetRpm * led_modes::kRevClearFrac,
                           led_frame::kRed, led_modes::kRevFlashHalfPeriodMs,
                           led_frame::kOff, in.rpm, true, s.threshold, nowMs);

    case Mode::kSpeed:
      // No speed fix -> off (and the latch released). The strip's search
      // pip already says "no GPS"; a second signal for the same fact is
      // noise, and invalidColor would make it a solid lit pixel.
      return evalThreshold(led_modes::Source::kSpeedMph, in.targetSpeedMph,
                           in.targetSpeedMph - led_modes::kSpeedClearDeltaMph,
                           led_frame::kRed,
                           led_modes::kSpeedFlashHalfPeriodMs, led_frame::kOff,
                           in.speedMph, in.speedValid, s.threshold, nowMs);

    case Mode::kEgt:
      if (!in.eggSupported) {
        // No egg support compiled in: there is no probe, so there is no
        // dropout to report. Dark, not blue. (Belt and braces — the glue
        // also leaves egtValid false — so the host test can prove the
        // gate independently of the NaN path.)
        s.threshold.active = false;
        return led_frame::kOff;
      }
      return evalThreshold(led_modes::Source::kEgtC, in.egtAlertC,
                           in.egtAlertC - led_modes::kEgtClearDeltaC,
                           led_frame::kRed, led_modes::kEgtFlashHalfPeriodMs,
                           led_frame::kBlue, in.egtC, in.egtValid, s.threshold,
                           nowMs);

    case Mode::kGps:
      s.threshold.active = false;
      switch (gpsLevel(in.gpsSats, in.gpsFix, in.gpsTimeValid)) {
        case GpsLevel::kNoSats:
          return led_frame::kRed;
        case GpsLevel::kSats:
          return led_frame::kYellow;
        case GpsLevel::kFix:
          return led_frame::kBlue;
        case GpsLevel::kLocked:
          return led_frame::kGreen;
      }
      return led_frame::kOff;

    case Mode::kCamera:
      s.threshold.active = false;
      switch (cameraLevel(in.cameraPaired, in.sessionActive, in.cameraLinkUp,
                          in.cameraSubscribed, in.cameraRecording)) {
        case CamLevel::kUnpaired:
          return led_frame::kOff;
        case CamLevel::kIdle:
          return led_frame::kYellow;
        case CamLevel::kLinkPartial:
          // Connected but not subscribed to ce82: our button frames go
          // nowhere, so we cannot start or stop it. Flashing says "not
          // ready" without claiming the camera is missing.
          return led_modes::flashOn(nowMs, kCameraFlashHalfPeriodMs)
                     ? led_frame::kBlue
                     : led_frame::kOff;
        case CamLevel::kLinkReady:
          return led_frame::kBlue;
        case CamLevel::kRecording:
          return led_frame::kRed;
      }
      return led_frame::kOff;

    case Mode::kLap:
      s.threshold.active = false;
      return verdictColor(in.lapVerdict);

    case Mode::kSector:
      s.threshold.active = false;
      return verdictColor(in.sectorVerdict);

    case Mode::kOff:
    case Mode::kCount:
    default:
      s.threshold.active = false;
      return led_frame::kOff;
  }
}

}  // namespace led_status
