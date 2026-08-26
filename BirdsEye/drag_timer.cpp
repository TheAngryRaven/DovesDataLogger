#include "drag_timer.h"

#include "haversine.h"

namespace drag_timer {

namespace {
constexpr double kFeetPerMile = 5280.0;

constexpr float kTargetsFt[kDistanceCount] = {
    660.0f,   // 1/8 mile
    1000.0f,  // 1000 ft
    1320.0f,  // 1/4 mile
    2640.0f,  // 1/2 mile
    5280.0f,  // 1 mile
};
const char* const kLabels[kDistanceCount] = {
    "1/8 Mile", "1000 ft", "1/4 Mile", "1/2 Mile", "1 Mile",
};
// DOVEX course names (must fit dovex_header::kCourseLen).
const char* const kDovexNames[kDistanceCount] = {
    "DRAG 1/8 MILE", "DRAG 1000 FT", "DRAG 1/4 MILE",
    "DRAG 1/2 MILE", "DRAG 1 MILE",
};

double lerp(double a, double b, double f) { return a + (b - a) * f; }
}  // namespace

float targetFeet(int idx) {
  if (idx < 0 || idx >= kDistanceCount) return kTargetsFt[0];
  return kTargetsFt[idx];
}

const char* label(int idx) {
  if (idx < 0 || idx >= kDistanceCount) return kLabels[0];
  return kLabels[idx];
}

const char* dovexName(int idx) {
  if (idx < 0 || idx >= kDistanceCount) return kDovexNames[0];
  return kDovexNames[idx];
}

double distanceFeet(double lat1, double lng1, double lat2, double lng2) {
  // The ONE haversine in the codebase (track detection uses it too) —
  // two copies of the formula/radius would be free to drift apart.
  return haversineDistanceMiles(lat1, lng1, lat2, lng2) * kFeetPerMile;
}

void DragTimer::setTarget(int idx) {
  if (idx < 0 || idx >= kDistanceCount) idx = 0;
  targetIdx_ = idx;
  targetFt_ = kTargetsFt[idx];
  havePrev_ = false;
  runs_ = 0;
  lastEtMs_ = 0;
  lastTrapMph_ = 0.0f;
  last0to60Ms_ = 0;
  bestEtMs_ = 0;
  bestRunNumber_ = 0;
  bestTrapMph_ = 0.0f;
  best0to60Ms_ = 0;
  resetToArmed();
}

void DragTimer::resetToArmed() {
  phase_ = Phase::kArmed;
  anchorValid_ = false;
  anchorCount_ = 0;
  stillTracking_ = false;
  runDistFt_ = 0.0f;
  provenOut_ = false;
  sixtyCrossed_ = false;
  run0to60Ms_ = 0;
  slowTracking_ = false;
}

void DragTimer::foldAnchor(double lat, double lng) {
  if (!anchorValid_) {
    anchorValid_ = true;
    anchorCount_ = 1;
    anchorLat_ = lat;
    anchorLng_ = lng;
    return;
  }
  if (anchorCount_ < kAnchorMeanWindow) anchorCount_++;
  anchorLat_ += (lat - anchorLat_) / anchorCount_;
  anchorLng_ += (lng - anchorLng_) / anchorCount_;
}

bool DragTimer::onFix(double lat, double lng, float speedMph,
                      uint64_t gpsTimeMs) {
  // Non-monotonic GPS time. A duplicate timestamp is dropped whole; a
  // BACKWARDS step (receiver clock correction mid-lock) means the
  // timebase under any in-flight measurement is gone — abandon it and
  // RESYNC by treating this as the first fix of a fresh stream. The
  // pre-fix guard only returned, which never updated prevTimeMs_, so
  // one backwards step rejected every later fix and wedged the timer
  // for the rest of the session.
  if (havePrev_ && gpsTimeMs <= prevTimeMs_) {
    if (gpsTimeMs == prevTimeMs_) return false;
    resetToArmed();
    havePrev_ = false;
  }

  bool completed = false;

  switch (phase_) {
    case Phase::kArmed: {
      if (speedMph <= kStagedMaxMph) {
        if (!stillTracking_) {
          stillTracking_ = true;
          stillSinceMs_ = gpsTimeMs;
          anchorValid_ = false;  // fresh stop, fresh anchor
          anchorCount_ = 0;
        }
        foldAnchor(lat, lng);
        if (gpsTimeMs - stillSinceMs_ >= kStageHoldMs) {
          phase_ = Phase::kStaged;
        }
      } else {
        stillTracking_ = false;
      }
      break;
    }

    case Phase::kStaged: {
      // A fix gap while staged: the launch may have happened INSIDE the
      // gap, and interpolating the ET start across it would anchor the
      // clock to a parked-car fix from before the dropout. Re-stage
      // instead — a car that is in fact still parked is staged again
      // one second later.
      if (havePrev_ && gpsTimeMs - prevTimeMs_ >= kFixGapAbortMs) {
        resetToArmed();
        break;
      }

      // Re-latching anchor: the mean follows the parked car's drifting
      // fix, so only real motion can open the rollout gap (plan 0015).
      if (speedMph <= kStagedMaxMph) foldAnchor(lat, lng);

      const double d = distanceFeet(anchorLat_, anchorLng_, lat, lng);
      if (d >= kRolloutFt && speedMph >= kLaunchMinMph) {
        // Launch. ET starts at the interpolated rollout crossing on the
        // displacement curve between the previous fix and this one.
        double f = 1.0;
        if (havePrev_) {
          const double dPrev =
              distanceFeet(anchorLat_, anchorLng_, prevLat_, prevLng_);
          if (d > dPrev) f = (kRolloutFt - dPrev) / (d - dPrev);
          if (f < 0.0) f = 0.0;
          if (f > 1.0) f = 1.0;
          runStartMs_ = lerp((double)prevTimeMs_, (double)gpsTimeMs, f);
        } else {
          runStartMs_ = (double)gpsTimeMs;
        }
        phase_ = Phase::kLaunched;
        runDistFt_ = (float)(d - kRolloutFt);  // overshoot past rollout
        provenOut_ = speedMph >= kProveOutMph;
        sixtyCrossed_ = false;
        run0to60Ms_ = 0;
        slowTracking_ = false;
      } else if (d >= kRestageFt) {
        // Moved to a new spot without ever reaching launch speed — a
        // slow reposition (staging-lane creep). Re-stage from scratch.
        resetToArmed();
      }
      break;
    }

    case Phase::kLaunched: {
      if (!havePrev_) {  // cannot happen (launch requires a prev), guard
        resetToArmed();
        break;
      }
      const uint64_t dt = gpsTimeMs - prevTimeMs_;
      if (dt >= kFixGapAbortMs) {
        // Fix gap at speed — the chord across it is not distance we can
        // stand behind. Abandon silently; nothing recorded.
        resetToArmed();
        break;
      }

      // Prove-out: a real pass reaches kProveOutMph within seconds of
      // the ET start. A wave-off driven to the pits at 4 mph does not,
      // and it never holds the sub-2 mph standstill the abort below
      // needs — without this gate it would record 660 ft of pit road
      // as a run.
      if (!provenOut_) {
        if (speedMph >= kProveOutMph) {
          provenOut_ = true;
        } else if ((double)gpsTimeMs - runStartMs_ >= (double)kProveOutMs) {
          resetToArmed();
          break;
        }
      }

      const double stepFt = distanceFeet(prevLat_, prevLng_, lat, lng);
      const float distBefore = runDistFt_;
      const float distAfter = runDistFt_ + (float)stepFt;

      // 0-60 split: first fix pair straddling the target speed.
      if (!sixtyCrossed_ && prevSpeedMph_ < kSplitTargetMph &&
          speedMph >= kSplitTargetMph) {
        double f = 1.0;
        if (speedMph > prevSpeedMph_) {
          f = (kSplitTargetMph - prevSpeedMph_) / (speedMph - prevSpeedMph_);
        }
        const double t60 = lerp((double)prevTimeMs_, (double)gpsTimeMs, f);
        sixtyCrossed_ = true;
        run0to60Ms_ = (unsigned long)(t60 - runStartMs_ + 0.5);
      }

      if (distAfter >= targetFt_ && stepFt > 0.0) {
        // Finish — ET and trap interpolated at the target distance.
        const double f = (targetFt_ - distBefore) / stepFt;
        const double tFin = lerp((double)prevTimeMs_, (double)gpsTimeMs, f);
        const float trap = (float)lerp(prevSpeedMph_, speedMph, f);

        runs_++;
        lastEtMs_ = (unsigned long)(tFin - runStartMs_ + 0.5);
        lastTrapMph_ = trap;
        last0to60Ms_ = run0to60Ms_;
        if (bestEtMs_ == 0 || lastEtMs_ < bestEtMs_) {
          bestEtMs_ = lastEtMs_;
          bestRunNumber_ = runs_;
          bestTrapMph_ = trap;
          best0to60Ms_ = run0to60Ms_;
        }
        completed = true;
        resetToArmed();
        break;
      }
      runDistFt_ = distAfter;

      // Mid-run standstill: driver lifted / wrong distance / phantom
      // creep launch. Abandon after the hold, silently.
      if (speedMph <= kAbortBelowMph) {
        if (!slowTracking_) {
          slowTracking_ = true;
          slowSinceMs_ = gpsTimeMs;
        } else if (gpsTimeMs - slowSinceMs_ >= kAbortHoldMs) {
          resetToArmed();
        }
      } else {
        slowTracking_ = false;
      }
      break;
    }
  }

  havePrev_ = true;
  prevLat_ = lat;
  prevLng_ = lng;
  prevSpeedMph_ = speedMph;
  prevTimeMs_ = gpsTimeMs;
  return completed;
}

unsigned long DragTimer::currentEtMs(uint64_t nowGpsMs) const {
  if (phase_ != Phase::kLaunched) return 0;
  const double et = (double)nowGpsMs - runStartMs_;
  if (et <= 0.0) return 0;
  return (unsigned long)(et + 0.5);
}

}  // namespace drag_timer
