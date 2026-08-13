#include "doctest.h"
#include "sector_purple.h"

using sector_purple::Sample;
using sector_purple::State;

// A live circuit race sample with everything zeroed; tests mutate what
// they need.
static Sample base() {
  Sample s{};
  s.sectorsConfigured = true;
  s.raceStarted = true;
  s.currentSector = 1;
  s.laps = 0;
  s.lastLapTime = 0;
  for (int i = 0; i < 3; i++) {
    s.lapSectorTime[i] = 0;
    s.bestSectorTime[i] = 0;
  }
  return s;
}

TEST_CASE("no sectors configured never fires and stays reset") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  in.sectorsConfigured = false;
  in.bestSectorTime[0] = 30000;
  in.lapSectorTime[0] = 1;  // absurdly fast — still must not fire
  for (int i = 0; i < 5; i++) {
    in.currentSector = (i % 3) + 1;
    CHECK(sector_purple::update(st, in) == 0);
  }
}

TEST_CASE("lap 1: no purple anywhere (no prior bests)") {
  State st;
  sector_purple::reset(st);
  Sample in = base();

  CHECK(sector_purple::update(st, in) == 0);  // adopt S1 open

  // S1 closes fast — but bestAtOpen was 0.
  in.currentSector = 2;
  in.lapSectorTime[0] = 25000;
  CHECK(sector_purple::update(st, in) == 0);

  in.currentSector = 3;
  in.lapSectorTime[1] = 26000;
  CHECK(sector_purple::update(st, in) == 0);

  // Lap line: S3 derived, but bestAtOpen[2] was 0.
  in.currentSector = 1;
  in.laps = 1;
  in.lastLapTime = 78000;
  in.bestSectorTime[0] = 25000;
  in.bestSectorTime[1] = 26000;
  in.bestSectorTime[2] = 27000;
  CHECK(sector_purple::update(st, in) == 0);
}

// Drive one full lap: closes S1/S2 at the given times, then the lap
// line with the given total. Returns the OR of fired values per edge in
// a small struct for assertions.
struct LapResult {
  int s1Fired, s2Fired, s3Fired;
};
static LapResult runLap(State& st, Sample& in, uint32_t s1, uint32_t s2,
                        uint32_t lapTotal, int lapNum) {
  LapResult r{0, 0, 0};
  in.currentSector = 2;
  in.lapSectorTime[0] = s1;
  r.s1Fired = sector_purple::update(st, in);
  in.currentSector = 3;
  in.lapSectorTime[1] = s2;
  r.s2Fired = sector_purple::update(st, in);
  // Lap line. The library may have already updated bests — simulate the
  // worst case by folding this lap's times in BEFORE the edge is seen.
  in.currentSector = 1;
  in.laps = lapNum;
  in.lastLapTime = lapTotal;
  uint32_t s3 = lapTotal - s1 - s2;
  if (in.bestSectorTime[0] == 0 || s1 < in.bestSectorTime[0])
    in.bestSectorTime[0] = s1;
  if (in.bestSectorTime[1] == 0 || s2 < in.bestSectorTime[1])
    in.bestSectorTime[1] = s2;
  if (in.bestSectorTime[2] == 0 || s3 < in.bestSectorTime[2])
    in.bestSectorTime[2] = s3;
  // Current-lap times roll over to the new lap (0 = not yet completed).
  in.lapSectorTime[0] = 0;
  in.lapSectorTime[1] = 0;
  in.lapSectorTime[2] = 0;
  r.s3Fired = sector_purple::update(st, in);
  return r;
}

TEST_CASE("S1/S2 improvements fire once at the transition; equal does not") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  // Lap 1 establishes bests: 25/26/27 (total 78).
  LapResult l1 = runLap(st, in, 25000, 26000, 78000, 1);
  CHECK(l1.s1Fired == 0);
  CHECK(l1.s2Fired == 0);
  CHECK(l1.s3Fired == 0);

  // Lap 2: faster S1, equal S2, slower S3.
  LapResult l2 = runLap(st, in, 24000, 26000, 79000, 2);
  CHECK(l2.s1Fired == 1);
  CHECK(l2.s2Fired == 0);  // equal time is not an improvement
  CHECK(l2.s3Fired == 0);

  // Lap 3: S2 improvement fires as 2.
  LapResult l3 = runLap(st, in, 24500, 25000, 80000, 3);
  CHECK(l3.s1Fired == 0);
  CHECK(l3.s2Fired == 2);
  CHECK(l3.s3Fired == 0);
}

TEST_CASE("S3 purple survives the library updating bests at the lap line") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  // Lap 1: bests 25/26/27.
  runLap(st, in, 25000, 26000, 78000, 1);

  // Lap 2: S3 = 80000 - 25500 - 26500 = 28000 (slower) — no fire.
  LapResult l2 = runLap(st, in, 25500, 26500, 80000, 2);
  CHECK(l2.s3Fired == 0);

  // Lap 3: S3 = 76000 - 25000 - 26000 = 25000, beats the 27000 best.
  // runLap folds the new best into bestSectorTime[2] BEFORE the edge —
  // the open-snapshot must still see the old 27000 and fire.
  LapResult l3 = runLap(st, in, 25000, 26000, 76000, 3);
  CHECK(l3.s3Fired == 3);

  // Lap 4: same S3 again (25000) — best is now 25000, no fire.
  LapResult l4 = runLap(st, in, 25000, 26000, 76000, 4);
  CHECK(l4.s3Fired == 0);
}

TEST_CASE("degenerate lap (missing S1/S2 closes) fires nothing at the line") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  // Lap 2 jumps straight from S1 to the lap line (missed transitions —
  // e.g. GPS dropout): no s2 recorded, so S3 cannot be derived.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  CHECK(sector_purple::update(st, in) == 1);  // S1 close still fires
  in.currentSector = 1;
  in.laps = 2;
  in.lastLapTime = 70000;  // would be a monster S3 if mis-derived
  CHECK(sector_purple::update(st, in) == 0);
}

TEST_CASE("lastLapTime smaller than s1+s2 cannot underflow") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  in.currentSector = 2;
  in.lapSectorTime[0] = 26000;
  sector_purple::update(st, in);
  in.currentSector = 3;
  in.lapSectorTime[1] = 27000;
  sector_purple::update(st, in);
  in.currentSector = 1;
  in.laps = 2;
  in.lastLapTime = 40000;  // < s1+s2 (inconsistent library state)
  CHECK(sector_purple::update(st, in) == 0);
}

TEST_CASE("sprint between-runs (sector 0) resets cleanly") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  // Run ends: currentSector drops to 0 between sprint runs.
  in.currentSector = 0;
  CHECK(sector_purple::update(st, in) == 0);

  // New run opens S1; a fast S1 against the lap-1 best still fires.
  in.currentSector = 1;
  CHECK(sector_purple::update(st, in) == 0);
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  CHECK(sector_purple::update(st, in) == 1);
}

TEST_CASE("race not started resets; mid-race attach adopts without firing") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  in.raceStarted = false;
  CHECK(sector_purple::update(st, in) == 0);

  // Attach mid-lap in sector 2 with bests already on the board.
  in.raceStarted = true;
  in.currentSector = 2;
  in.laps = 3;
  in.bestSectorTime[0] = 25000;
  in.bestSectorTime[1] = 26000;
  in.bestSectorTime[2] = 27000;
  CHECK(sector_purple::update(st, in) == 0);  // adopt, no fire

  // S2 closes faster than best: fires (snapshot taken at attach).
  in.currentSector = 3;
  in.lapSectorTime[1] = 25500;
  CHECK(sector_purple::update(st, in) == 2);
}
