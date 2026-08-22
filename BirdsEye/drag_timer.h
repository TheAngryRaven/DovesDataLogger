#pragma once

#include <stdint.h>

///////////////////////////////////////////
// DRAG MODE RUN TIMER (plan 0015)
// The whole drag-run state machine: stage at a standstill, launch
// rollout-style (clock starts when the car has moved 11.25 in from its
// staged position), accumulate GPS distance to the selected target, and
// record ET + trap speed + a 0-60 mph split per run. Re-arms itself
// after every run — come back to a standstill and it stages again.
//
// The sketch keeps only the glue (sprint-timer precedent): every
// transition here edges on a GPS fix passed to onFix(), the glue gates
// on gpsData.fix, and a dropout reaches this unit purely as a timestamp
// gap. Units at the boundary: mph, feet, GPS epoch milliseconds — no
// millis(), no Arduino headers, host-tested.
//
// The load-bearing detail is the STAGED anchor: it is a running mean of
// standstill fixes that keeps re-latching while the car is stopped, so
// GPS drift over a multi-minute staging-lane wait cannot walk the fix
// past the rollout radius and fake a launch. The only false-launch
// source left is real motion (queue creep), and that self-cancels
// through the mid-run standstill abort without recording anything.
///////////////////////////////////////////

namespace drag_timer {

///////////////////////////////////////////
// Distance table — the single source for the picker labels, the run
// targets, and the DOVEX course name. Index order is picker order.
///////////////////////////////////////////
constexpr int kDistanceCount = 5;
float       targetFeet(int idx);  // 660 / 1000 / 1320 / 2640 / 5280
const char* label(int idx);       // "1/8 Mile", "1000 ft", ...
const char* dovexName(int idx);   // "DRAG 1/8 MILE", "DRAG 1000 FT", ...

///////////////////////////////////////////
// Tunables
///////////////////////////////////////////

// Drag-strip standard rollout: 11.25 in between pre-stage and the ET
// clock starting. GPS h_acc is larger than this, but the start is
// interpolated on the displacement curve between two 25 Hz fixes during
// a hard launch, so the error is bounded by fix-to-fix noise plus the
// anchor mean's residual, not absolute accuracy (plan 0015).
constexpr float kRolloutFt = 0.9375f;

// At/below this the car counts as stopped (staging + anchor folding).
constexpr float kStagedMaxMph = 1.0f;

// Standstill must hold this long before ARMED promotes to STAGED.
constexpr uint32_t kStageHoldMs = 1000;

// The rollout displacement only launches with real speed behind it —
// position jitter and dead-slow creep never start a clock.
constexpr float kLaunchMinMph = 2.0f;

// The bonus split. Interpolated on speed between the straddling fixes;
// 0 when the run never reaches it (short cars on the 1/8).
constexpr float kSplitTargetMph = 60.0f;

// Mid-run standstill this long -> the run is abandoned silently
// (driver lifted, wrong distance selected, or a phantom creep launch).
constexpr float    kAbortBelowMph = 2.0f;
constexpr uint32_t kAbortHoldMs   = 3000;

// A fix gap this long mid-run aborts the run: the chord across the gap
// is untrustworthy distance at speed. Gaps below it are tolerated (the
// chord ~ the path; drag runs are straight).
constexpr uint32_t kFixGapAbortMs = 2000;

// STAGED car that drifts this far from its anchor WITHOUT launching
// (speed stayed under kLaunchMinMph) has moved to a new spot — drop
// back to ARMED and re-stage there. Deliberately much larger than the
// rollout so standstill jitter can never un-stage a car that is about
// to launch; only a real slow reposition trips it.
constexpr float kRestageFt = 10.0f;

// Anchor running-mean window (fix count). Averaging shrinks the anchor
// noise well below a single fix's jitter; the cap keeps it responsive
// to the re-latch (the mean follows the parked car's drifting fix).
constexpr int kAnchorMeanWindow = 32;

enum class Phase : uint8_t {
  kArmed,     // waiting for a standstill (also post-run / post-abort)
  kStaged,    // stopped, anchor latched, watching for the rollout
  kLaunched,  // clock running, accumulating distance
};

class DragTimer {
 public:
  // idx into the distance table above. Resets all run state.
  void setTarget(int idx);

  // One valid GPS fix (glue gates on gpsData.fix). speedMph is ground
  // speed in mph, gpsTimeMs is GPS epoch milliseconds. Returns true
  // exactly when this fix COMPLETED a run (the glue's run-count edge
  // fires off runs() anyway; the bool is a convenience).
  bool onFix(double lat, double lng, float speedMph, uint64_t gpsTimeMs);

  Phase phase() const { return phase_; }
  bool  runActive() const { return phase_ == Phase::kLaunched; }
  bool  staged() const { return phase_ == Phase::kStaged; }
  int   targetIdx() const { return targetIdx_; }
  float targetFt() const { return targetFt_; }

  int runs() const { return runs_; }

  // Live ET while a run is on, 0 otherwise. nowGpsMs lets the display
  // tick between fixes.
  unsigned long currentEtMs(uint64_t nowGpsMs) const;

  // Cumulative distance covered in the current run, feet.
  float distanceFt() const { return runDistFt_; }

  // Live 0-60 of the run IN PROGRESS: 0 until the crossing (and outside
  // a run); the split the moment it happens. The pace page shows this.
  unsigned long current0to60Ms() const {
    return phase_ == Phase::kLaunched ? run0to60Ms_ : 0;
  }

  // Last completed run. 0 / 0.0f until one exists; last0to60Ms() is 0
  // when that run never reached 60 mph.
  unsigned long lastEtMs() const { return lastEtMs_; }
  float         lastTrapMph() const { return lastTrapMph_; }
  unsigned long last0to60Ms() const { return last0to60Ms_; }

  // Best run = lowest ET. Trap/split are snapshots OF that run.
  unsigned long bestEtMs() const { return bestEtMs_; }
  int           bestRunNumber() const { return bestRunNumber_; }  // 1-based
  float         bestTrapMph() const { return bestTrapMph_; }
  unsigned long best0to60Ms() const { return best0to60Ms_; }

 private:
  void resetToArmed();
  void foldAnchor(double lat, double lng);

  int   targetIdx_ = 0;
  float targetFt_ = 0.0f;

  Phase phase_ = Phase::kArmed;

  // Previous fix (valid once havePrev_).
  bool     havePrev_ = false;
  double   prevLat_ = 0.0, prevLng_ = 0.0;
  float    prevSpeedMph_ = 0.0f;
  uint64_t prevTimeMs_ = 0;

  // Standstill anchor (running mean) + the stage-hold clock.
  bool     anchorValid_ = false;
  double   anchorLat_ = 0.0, anchorLng_ = 0.0;
  int      anchorCount_ = 0;
  bool     stillTracking_ = false;
  uint64_t stillSinceMs_ = 0;

  // Live run.
  double   runStartMs_ = 0.0;   // interpolated -> fractional ms internally
  float    runDistFt_ = 0.0f;
  bool     sixtyCrossed_ = false;
  unsigned long run0to60Ms_ = 0;
  bool     slowTracking_ = false;
  uint64_t slowSinceMs_ = 0;

  // Records.
  int           runs_ = 0;
  unsigned long lastEtMs_ = 0;
  float         lastTrapMph_ = 0.0f;
  unsigned long last0to60Ms_ = 0;
  unsigned long bestEtMs_ = 0;
  int           bestRunNumber_ = 0;
  float         bestTrapMph_ = 0.0f;
  unsigned long best0to60Ms_ = 0;
};

// Great-circle distance in FEET (spherical Earth, R matching
// haversine.cpp's 3958.8 mi). Exposed for the tests' synthetic tracks.
double distanceFeet(double lat1, double lng1, double lat2, double lng2);

}  // namespace drag_timer
