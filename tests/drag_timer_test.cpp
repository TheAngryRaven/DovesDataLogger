#include <math.h>
#include <string.h>

#include "doctest.h"
#include "dovex_header.h"
#include "drag_timer.h"

using drag_timer::DragTimer;
using drag_timer::Phase;

// ---------------------------------------------------------------------------
// Synthetic drag strip: fixes along a meridian at 25 Hz. Position is a
// 1-D "feet down the strip" coordinate converted to latitude with the
// unit's own Earth radius, so distanceFeet() round-trips exactly.
// ---------------------------------------------------------------------------
namespace {

constexpr uint64_t kT0 = 1700000000000ULL;  // arbitrary GPS epoch ms
constexpr uint64_t kDtMs = 40;              // 25 Hz
constexpr double kFtPerSecToMph = 0.6818181818;

double degPerFoot() {
  static const double d = 1.0 / drag_timer::distanceFeet(0.0, 0.0, 1.0, 0.0);
  return d;
}

struct Strip {
  DragTimer t;
  uint64_t now = kT0;

  explicit Strip(int idx = 0) { t.setTarget(idx); }

  // One fix at `xFt` down the strip moving at `mph`, then advance time.
  bool fix(double xFt, double mph) {
    const bool done = t.onFix(xFt * degPerFoot(), 0.0, (float)mph, now);
    now += kDtMs;
    return done;
  }

  // Hold a standstill at `xFt` for `ms`.
  void standstill(double xFt, uint64_t ms) {
    const uint64_t until = now + ms;
    while (now < until) fix(xFt, 0.0);
  }

  // Constant acceleration from rest at `x0`: x = x0 + a/2 t², v = a t.
  // Runs until a run completes or `maxMs` elapses; returns completion.
  bool launchConstAccel(double x0, double aFtPerS2, uint64_t maxMs) {
    const uint64_t start = now;
    while (now - start < maxMs) {
      const double tS = (double)(now - start) / 1000.0;
      const double x = x0 + 0.5 * aFtPerS2 * tS * tS;
      const double v = aFtPerS2 * tS * kFtPerSecToMph;
      if (fix(x, v)) return true;
    }
    return false;
  }

  // Constant speed from `x0`; same contract.
  bool cruise(double x0, double ftPerS, uint64_t maxMs) {
    const uint64_t start = now;
    while (now - start < maxMs) {
      const double tS = (double)(now - start) / 1000.0;
      if (fix(x0 + ftPerS * tS, ftPerS * kFtPerSecToMph)) return true;
    }
    return false;
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// Staging
// ---------------------------------------------------------------------------

TEST_CASE("arms to staged after a standstill hold, not before") {
  Strip s;
  s.standstill(0.0, drag_timer::kStageHoldMs - 200);
  CHECK(s.t.phase() == Phase::kArmed);
  s.standstill(0.0, 400);
  CHECK(s.t.phase() == Phase::kStaged);
  CHECK(s.t.staged());
  CHECK_FALSE(s.t.runActive());
}

TEST_CASE("standstill position jitter never launches") {
  Strip s;
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  // Minutes of ±2 ft jitter at 0 mph — well past the rollout radius but
  // with no speed behind it.
  for (int i = 0; i < 25 * 120; i++) {
    s.fix((i % 2) ? 2.0 : -2.0, 0.0);
  }
  CHECK(s.t.phase() == Phase::kStaged);
  CHECK(s.t.runs() == 0);
}

TEST_CASE("slow anchor drift at standstill stays staged (anchor re-latch)") {
  Strip s;
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  // GPS drift: the fix walks 6 ft over 3 minutes at 0 mph. Without the
  // re-latching anchor this crosses the rollout radius and fakes a
  // launch; with it the anchor follows.
  const int n = 25 * 180;
  for (int i = 0; i < n; i++) {
    s.fix(6.0 * i / n, 0.0);
  }
  CHECK(s.t.phase() == Phase::kStaged);
  CHECK(s.t.runs() == 0);
}

TEST_CASE("slow reposition re-stages at the new spot") {
  Strip s;
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  // Creep 40 ft at ~1.4 mph (above standstill, below launch speed).
  const double v = 2.0;  // ft/s = 1.36 mph
  for (double x = 0.0; x < 40.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  CHECK(s.t.phase() == Phase::kArmed);
  s.standstill(40.0, 2000);
  CHECK(s.t.phase() == Phase::kStaged);
  CHECK(s.t.runs() == 0);
}

TEST_CASE("creep through the rollout below launch speed re-stages") {
  // Review D8: creeping at 1-2 mph past the rollout, then pushing past
  // 2 mph, used to "launch" with the rollout already behind the car —
  // the interpolation clamped to the previous fix, starting the clock
  // late and crediting crept ground to the run.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  const double v = 2.2;  // ft/s = 1.5 mph: above staged, below launch
  double x = 0.0;
  while (x < drag_timer::kRolloutFt - 0.1) {
    s.fix(x, v * kFtPerSecToMph);
    x += v * 0.04;
  }
  CHECK(s.t.phase() == Phase::kStaged);  // inside the rollout: still staged
  for (int i = 0; i < 10; i++) {
    s.fix(x, v * kFtPerSecToMph);
    x += v * 0.04;
  }
  CHECK(s.t.phase() == Phase::kArmed);   // crept through it: re-stage

  // Rolling straight into a launch from the creep: no standing start,
  // nothing timed.
  CHECK_FALSE(s.launchConstAccel(x, 20.0, 15000));
  CHECK(s.t.runs() == 0);
}

TEST_CASE("creep, stop, then launch times from the new spot exactly") {
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 2.2;  // 1.5 mph creep for 3 ft
  for (double x = 0.0; x < 3.0; x += v * 0.04) s.fix(x, v * kFtPerSecToMph);
  s.standstill(3.0, 1500);
  REQUIRE(s.t.phase() == Phase::kStaged);

  const double a = 14.7;
  REQUIRE(s.launchConstAccel(3.0, a, 60000));
  const double tRoll = sqrt(2.0 * drag_timer::kRolloutFt / a);
  const double tFin = sqrt(2.0 * (660.0 + drag_timer::kRolloutFt) / a);
  CHECK(fabs((double)s.t.lastEtMs() - (tFin - tRoll) * 1000.0) <= kDtMs);
}

// ---------------------------------------------------------------------------
// The run: rollout, ET, trap, 0-60
// ---------------------------------------------------------------------------

TEST_CASE("rollout crossing starts the clock, interpolated") {
  Strip s(0);  // 1/8 mile = 660 ft
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);

  const double a = 14.7;  // ft/s² (~0-60 in 6 s)
  REQUIRE(s.launchConstAccel(0.0, a, 60000));
  CHECK(s.t.runs() == 1);

  // Analytic: the clock starts at the rollout crossing and the timed
  // distance counts from there, so the finish is at rollout + 660 ft of
  // absolute travel.
  const double tRoll = sqrt(2.0 * drag_timer::kRolloutFt / a);
  const double tFin = sqrt(2.0 * (660.0 + drag_timer::kRolloutFt) / a);
  const double etMs = (tFin - tRoll) * 1000.0;
  CHECK(fabs((double)s.t.lastEtMs() - etMs) <= kDtMs);
}

TEST_CASE("ET and trap interpolated at target distance (constant speed)") {
  Strip s(0);  // 660 ft
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);

  const double v = 60.0;  // ft/s = 40.9 mph
  REQUIRE(s.cruise(0.0, v, 60000));
  CHECK(s.t.runs() == 1);

  // Timed distance counts from the rollout point, so at constant speed
  // the ET is exactly target / v.
  const double etMs = 660.0 / v * 1000.0;
  CHECK(fabs((double)s.t.lastEtMs() - etMs) <= kDtMs);
  CHECK(s.t.lastTrapMph() == doctest::Approx(v * kFtPerSecToMph).epsilon(0.01));
}

TEST_CASE("0-60 split interpolated between straddling fixes") {
  Strip s(2);  // 1/4 mile so 60 mph is reached well before the finish
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);

  const double a = 14.6667;  // ft/s²: 88 ft/s (60 mph) at exactly t=6 s
  REQUIRE(s.launchConstAccel(0.0, a, 60000));
  CHECK(s.t.runs() == 1);

  const double tRoll = sqrt(2.0 * drag_timer::kRolloutFt / a);
  const double splitMs = (6.0 - tRoll) * 1000.0;
  REQUIRE(s.t.last0to60Ms() != 0);
  // Speed is linear in t, so the speed interpolation is exact; the only
  // error is the rollout start (bounded by one fix interval).
  CHECK(fabs((double)s.t.last0to60Ms() - splitMs) <= kDtMs);
}

TEST_CASE("split is 0 when 60 mph is never reached") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.cruise(0.0, 44.0, 60000));  // 30 mph the whole way
  CHECK(s.t.runs() == 1);
  CHECK(s.t.last0to60Ms() == 0);
}

// ---------------------------------------------------------------------------
// Run bookkeeping + re-arm
// ---------------------------------------------------------------------------

TEST_CASE("faster second run takes best with its trap/split snapshot") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.cruise(0.0, 50.0, 60000));  // run 1, slower
  const unsigned long et1 = s.t.lastEtMs();

  s.standstill(660.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);  // re-armed and re-staged
  REQUIRE(s.launchConstAccel(660.0, 25.0, 60000));  // run 2, faster + hits 60

  CHECK(s.t.runs() == 2);
  CHECK(s.t.lastEtMs() < et1);
  CHECK(s.t.bestEtMs() == s.t.lastEtMs());
  CHECK(s.t.bestRunNumber() == 2);
  CHECK(s.t.bestTrapMph() == doctest::Approx(s.t.lastTrapMph()));
  CHECK(s.t.best0to60Ms() == s.t.last0to60Ms());
}

TEST_CASE("slower second run leaves best untouched") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.cruise(0.0, 60.0, 60000));
  const unsigned long best = s.t.bestEtMs();
  const float bestTrap = s.t.bestTrapMph();

  s.standstill(660.0, 2000);
  REQUIRE(s.cruise(660.0, 40.0, 60000));

  CHECK(s.t.runs() == 2);
  CHECK(s.t.bestEtMs() == best);
  CHECK(s.t.bestRunNumber() == 1);
  CHECK(s.t.bestTrapMph() == doctest::Approx(bestTrap));
}

// ---------------------------------------------------------------------------
// Aborts
// ---------------------------------------------------------------------------

TEST_CASE("creep launch abandons silently on the mid-run standstill") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  // Queue creep: 30 ft at 5 mph trips a phantom launch...
  const double v = 7.33;  // ft/s = 5 mph
  for (double x = 0.0; x < 30.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  CHECK(s.t.runActive());
  // ...then the car stops and the phantom run evaporates.
  s.standstill(30.0, drag_timer::kAbortHoldMs + 500);
  CHECK(s.t.runs() == 0);
  CHECK_FALSE(s.t.runActive());
  // And it re-stages at the new spot for the real launch.
  s.standstill(30.0, 2000);
  CHECK(s.t.phase() == Phase::kStaged);
}

TEST_CASE("fix gap over threshold aborts the run") {
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  for (double x = 0.0; x < 200.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());
  s.now += drag_timer::kFixGapAbortMs;  // GPS dropout at speed
  s.fix(500.0, v * kFtPerSecToMph);
  CHECK_FALSE(s.t.runActive());
  CHECK(s.t.runs() == 0);
}

TEST_CASE("fix lost mid-run with no fix ever returning aborts on the watchdog") {
  // Review D1: every rule edges on onFix(), so a fix that drops at speed
  // and never returns used to leave the run LAUNCHED forever (on the
  // manual tree's pinned screen, a wedge with no exit).
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  for (double x = 0.0; x < 200.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());

  const uint32_t lastFixMillis = 50000;  // host millis of the last fix fed
  CHECK_FALSE(s.t.checkFixLoss(lastFixMillis + drag_timer::kFixLossAbortMs - 1,
                               lastFixMillis));
  CHECK(s.t.runActive());
  CHECK(s.t.checkFixLoss(lastFixMillis + drag_timer::kFixLossAbortMs,
                         lastFixMillis));
  CHECK_FALSE(s.t.runActive());
  CHECK(s.t.phase() == Phase::kArmed);
  CHECK(s.t.runs() == 0);
  // Idempotent once armed.
  CHECK_FALSE(s.t.checkFixLoss(lastFixMillis + 60000, lastFixMillis));
}

TEST_CASE("fix-loss watchdog is millis-wrap safe") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  const uint32_t last = 0xFFFFFF00u;
  CHECK_FALSE(s.t.checkFixLoss(last + 100u, last));  // wrapped, 100 ms
  CHECK(s.t.checkFixLoss(last + drag_timer::kFixLossAbortMs, last));
  CHECK(s.t.phase() == Phase::kArmed);
}

TEST_CASE("a fix returning after the watchdog starts a fresh stream, not a stale run") {
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  for (double x = 0.0; x < 200.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());
  REQUIRE(s.t.checkFixLoss(10000 + drag_timer::kFixLossAbortMs, 10000));
  // The fix comes back well down the strip still at speed: nothing may
  // resume, and crossing the target must not complete a run.
  s.now += 500;
  CHECK_FALSE(s.t.runActive());
  for (double x = 400.0; x < 900.0; x += v * 0.04) {
    CHECK_FALSE(s.fix(x, v * kFtPerSecToMph));
  }
  CHECK(s.t.runs() == 0);
  // And the timer is healthy: stop, stage, run.
  s.standstill(900.0, 2000);
  CHECK(s.t.phase() == Phase::kStaged);
  CHECK(s.launchConstAccel(900.0, 30.0, 15000));
  CHECK(s.t.runs() == 1);
}

TEST_CASE("fix gap under threshold accumulates the chord and continues") {
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  bool done = false;
  double x = 0.0;
  int i = 0;
  while (!done && x < 1000.0) {
    x += v * 0.04;
    if (i++ == 100) {  // one 500 ms dropout mid-run
      s.now += 460;    // + the regular 40 ms step = 500 ms gap
      x += v * 0.46;
    }
    done = s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(done);
  const double etMs = 660.0 / v * 1000.0;
  CHECK(fabs((double)s.t.lastEtMs() - etMs) <= kDtMs);
}

TEST_CASE("duplicate timestamp is dropped without disturbing the run") {
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  double x = 0.0;
  for (; x < 200.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());
  const float distBefore = s.t.distanceFt();
  // Same timestamp as the last accepted fix: dropped whole.
  CHECK_FALSE(s.t.onFix((x + 50.0) * degPerFoot(), 0.0,
                        (float)(v * kFtPerSecToMph), s.now - kDtMs));
  CHECK(s.t.runActive());
  CHECK(s.t.distanceFt() == doctest::Approx(distBefore));
}

TEST_CASE("backwards time step aborts the run and the timer recovers") {
  // Regression for the session-wedge bug: the old guard rejected a
  // backwards fix without resyncing its reference, so ONE backwards
  // step (e.g. a wrapping time-of-day clock at UTC midnight, or a
  // receiver clock correction) rejected every later fix for the rest
  // of the session. A backwards step must abandon the in-flight run
  // AND leave the timer fully functional on the new timebase.
  Strip s(0);
  s.standstill(0.0, 2000);
  const double v = 60.0;
  for (double x = 0.0; x < 200.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());

  s.now = 1000;  // the clock "wrapped" — far in the past
  s.fix(200.0, 0.0);
  CHECK_FALSE(s.t.runActive());
  CHECK(s.t.runs() == 0);

  // The stream continues on the new timebase — a full stage + run works.
  s.standstill(200.0, 2000);
  CHECK(s.t.phase() == Phase::kStaged);
  REQUIRE(s.cruise(200.0, v, 60000));
  CHECK(s.t.runs() == 1);
}

TEST_CASE("fix gap while staged re-stages instead of timing across it") {
  // A launch that happens INSIDE a GPS dropout must not be timed by
  // interpolating across the gap — that anchors the ET start to a
  // parked-car fix from before the dropout and inflates the ET by the
  // gap length.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);

  s.now += 5000;  // dropout; the car launched somewhere inside it
  const double v = 60.0;
  for (double x = 100.0; x < 400.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  CHECK(s.t.runs() == 0);  // nothing recorded off the gap

  // Parked again after the pass -> stages normally.
  s.standstill(400.0, 2000);
  CHECK(s.t.phase() == Phase::kStaged);
}

TEST_CASE("wave-off drive to the pits never records a run") {
  // Waved off and driven away at a steady 4-5 mph: above the launch
  // threshold, never below the abort threshold — without the prove-out
  // gate this recorded 660 ft of pit road as a ~90 s "run".
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  const double v = 7.33;  // ft/s = 5 mph
  for (double x = 0.0; x < 700.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  CHECK(s.t.runs() == 0);
  CHECK_FALSE(s.t.runActive());  // abandoned by the prove-out gate
}

TEST_CASE("a slow but real pass proves out and records") {
  // Gentle launch (~6 ft/s^2): 15 mph arrives well inside the prove-out
  // window, so the gate never fires and the run completes normally.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.launchConstAccel(0.0, 6.0, 60000));
  CHECK(s.t.runs() == 1);
}

// ---------------------------------------------------------------------------
// Launch gate (manual staging tree, plan 0016)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Return-road gate (review D7)
// ---------------------------------------------------------------------------

namespace {

// Stage at `x0`, then drive BACK down the strip (decreasing x): brisk
// 3 s acceleration to `cruiseMph`, then hold it. Returns completion.
bool returnRoadDrive(Strip& s, double x0, double cruiseMph, uint64_t maxMs) {
  const double vMax = cruiseMph / kFtPerSecToMph;  // ft/s
  const double a = vMax / 3.0;
  const uint64_t start = s.now;
  while (s.now - start < maxMs) {
    const double tS = (double)(s.now - start) / 1000.0;
    double x, v;
    if (tS < 3.0) {
      x = 0.5 * a * tS * tS;
      v = a * tS;
    } else {
      x = 0.5 * a * 9.0 + vMax * (tS - 3.0);
      v = vMax;
    }
    if (s.fix(x0 - x, v * kFtPerSecToMph)) return true;
  }
  return false;
}

}  // namespace

TEST_CASE("return-road drive after a run is not recorded") {
  // The car finishes a real pass, stops in the shutdown area, stages
  // there, then drives the return road back at 25 mph. It clears the
  // rollout, the 15 mph prove-out and the 660 ft target — a bogus ~20 s
  // "run" before the gate.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.launchConstAccel(0.0, 20.0, 15000));
  REQUIRE(s.t.runs() == 1);
  const unsigned long realEt = s.t.lastEtMs();

  // Coast on past the stripe and park in the shutdown area.
  s.standstill(900.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  CHECK_FALSE(returnRoadDrive(s, 900.0, 25.0, 40000));
  CHECK(s.t.runs() == 1);
  CHECK(s.t.rejectedRuns() == 1);
  CHECK(s.t.lastEtMs() == realEt);
  CHECK(s.t.bestEtMs() == realEt);
  CHECK_FALSE(s.t.runActive());

  // Back in the lanes, the next real pass still records.
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  CHECK(s.launchConstAccel(0.0, 20.0, 15000));
  CHECK(s.t.runs() == 2);
}

TEST_CASE("a slow cruise-profile pass in the strip's direction still records") {
  // Conservative: the heading test alone never discards a run going the
  // same way as the last one, however gently it was driven.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.launchConstAccel(0.0, 20.0, 15000));
  s.standstill(-900.0, 2000);  // back behind the start line, staged
  REQUIRE(s.t.phase() == Phase::kStaged);
  // Same direction as run 1 (increasing x), cruise profile.
  const double vMax = 25.0 / kFtPerSecToMph;
  const double a = vMax / 3.0;
  const uint64_t start = s.now;
  bool done = false;
  while (!done && s.now - start < 40000) {
    const double tS = (double)(s.now - start) / 1000.0;
    const double x = tS < 3.0 ? 0.5 * a * tS * tS
                              : 0.5 * a * 9.0 + vMax * (tS - 3.0);
    const double v = tS < 3.0 ? a * tS : vMax;
    done = s.fix(-900.0 + x, v * kFtPerSecToMph);
  }
  CHECK(done);
  CHECK(s.t.runs() == 2);
  CHECK(s.t.rejectedRuns() == 0);
}

TEST_CASE("a full-effort pass in the opposite direction still records") {
  // Two-way passes (e.g. wind-averaged top-speed runs) accelerate the
  // whole way — trap well above average — so heading alone never drops
  // them.
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.launchConstAccel(0.0, 20.0, 15000));
  s.standstill(1500.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  const uint64_t start = s.now;
  bool done = false;
  while (!done && s.now - start < 15000) {
    const double tS = (double)(s.now - start) / 1000.0;
    done = s.fix(1500.0 - 0.5 * 20.0 * tS * tS, 20.0 * tS * kFtPerSecToMph);
  }
  CHECK(done);
  CHECK(s.t.runs() == 2);
  CHECK(s.t.rejectedRuns() == 0);
}

TEST_CASE("the first run of a session is never discarded by the gate") {
  // No previous heading to compare against: a cruise-profile first run
  // records (it can only be judged against a later one's direction).
  Strip s(0);
  s.standstill(900.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  CHECK(returnRoadDrive(s, 900.0, 25.0, 40000));
  CHECK(s.t.runs() == 1);
  CHECK(s.t.rejectedRuns() == 0);
}

TEST_CASE("launch disabled: rollout at speed re-arms instead of running") {
  Strip s(0);
  s.t.setLaunchEnabled(false);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  // A full pass worth of motion while the tree holds the clock — the
  // pre-green move is the tree's foul, never a run.
  const double v = 60.0;
  for (double x = 0.0; x < 700.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  CHECK(s.t.runs() == 0);
  CHECK_FALSE(s.t.runActive());
  CHECK(s.t.phase() == Phase::kArmed);
}

TEST_CASE("re-enabling the launch gate restores a normal run") {
  Strip s(0);
  s.t.setLaunchEnabled(false);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  s.t.setLaunchEnabled(true);  // the tree went green
  REQUIRE(s.cruise(0.0, 60.0, 60000));
  CHECK(s.t.runs() == 1);
}

TEST_CASE("runStartEpochMs is the interpolated rollout crossing") {
  Strip s(0);
  s.standstill(0.0, 2000);
  REQUIRE(s.t.phase() == Phase::kStaged);
  const uint64_t t0 = s.now;  // timestamp of the cruise's first fix
  const double v = 60.0;      // ft/s
  REQUIRE(s.cruise(0.0, v, 60000));
  // Constant speed from the anchor: the rollout crossing is at exactly
  // kRolloutFt / v seconds after the first moving fix.
  const double expected = (double)t0 + drag_timer::kRolloutFt / v * 1000.0;
  CHECK(fabs((double)s.t.runStartEpochMs() - expected) <= 1.0);
}

// ---------------------------------------------------------------------------
// Live ET + distance table
// ---------------------------------------------------------------------------

TEST_CASE("currentEtMs is live between fixes and zero outside a run") {
  Strip s(0);
  CHECK(s.t.currentEtMs(s.now) == 0);
  s.standstill(0.0, 2000);
  CHECK(s.t.currentEtMs(s.now) == 0);
  const double v = 60.0;
  for (double x = 0.0; x < 100.0; x += v * 0.04) {
    s.fix(x, v * kFtPerSecToMph);
  }
  REQUIRE(s.t.runActive());
  const unsigned long a = s.t.currentEtMs(s.now);
  const unsigned long b = s.t.currentEtMs(s.now + 20);
  CHECK(a > 0);
  CHECK(b == a + 20);
}

TEST_CASE("distance table goldens") {
  CHECK(drag_timer::targetFeet(0) == 660.0f);
  CHECK(drag_timer::targetFeet(1) == 1000.0f);
  CHECK(drag_timer::targetFeet(2) == 1320.0f);
  CHECK(drag_timer::targetFeet(3) == 2640.0f);
  CHECK(drag_timer::targetFeet(4) == 5280.0f);

  CHECK(strcmp(drag_timer::label(0), "1/8 Mile") == 0);
  CHECK(strcmp(drag_timer::label(1), "1000 ft") == 0);
  CHECK(strcmp(drag_timer::label(2), "1/4 Mile") == 0);
  CHECK(strcmp(drag_timer::label(3), "1/2 Mile") == 0);
  CHECK(strcmp(drag_timer::label(4), "1 Mile") == 0);

  CHECK(strcmp(drag_timer::dovexName(2), "DRAG 1/4 MILE") == 0);
  // Every DOVEX name must fit the header's course field, and the DRAG
  // race_mode token must fit its field.
  for (int i = 0; i < drag_timer::kDistanceCount; i++) {
    CHECK(strlen(drag_timer::dovexName(i)) < dovex_header::kCourseLen);
  }
  CHECK(strlen("DRAG") < dovex_header::kRaceModeLen);

  // Out-of-range indexes clamp instead of reading wild.
  CHECK(drag_timer::targetFeet(-1) == 660.0f);
  CHECK(drag_timer::targetFeet(99) == 660.0f);
}
