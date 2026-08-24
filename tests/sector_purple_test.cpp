#include "doctest.h"
#include "sector_purple.h"

using sector_purple::Sample;
using sector_purple::State;
using sector_purple::Verdict;

// update() returns an Event since plan 0013. The purple-sector contract
// these cases pin is unchanged, so they read it through this shim
// rather than being rewritten around the new struct.
static int purpleOf(State& st, const Sample& in) {
  return sector_purple::update(st, in).purpleSector;
}

// A live circuit race sample with everything zeroed; tests mutate what
// they need.
static Sample base() {
  Sample s{};
  s.sectorsConfigured = true;
  s.raceStarted = true;
  s.currentSector = 1;
  s.laps = 0;
  s.lastLapTime = 0;
  s.bestLapTime = 0;
  for (int i = 0; i < 3; i++) {
    s.lapSectorTime[i] = 0;
    s.bestSectorTime[i] = 0;
  }
  return s;
}

TEST_CASE("no sectors configured: no sector events, but laps still track") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  in.sectorsConfigured = false;
  in.bestSectorTime[0] = 30000;
  in.lapSectorTime[0] = 1;  // absurdly fast — still must not fire
  for (int i = 0; i < 5; i++) {
    in.currentSector = (i % 3) + 1;
    sector_purple::Event ev = sector_purple::update(st, in);
    CHECK(ev.purpleSector == 0);
    CHECK(ev.closedSector == 0);
  }

  // Lap Anything has no sector lines but absolutely has laps, and the
  // `lap` status mode has to work there. Before plan 0013 the whole
  // monitor reset every frame on this input.
  in.currentSector = 0;
  in.laps = 1;
  in.lastLapTime = 78000;
  in.bestLapTime = 78000;  // the library folds it in AT the crossing
  sector_purple::Event l1 = sector_purple::update(st, in);
  CHECK(l1.lapClosed);
  CHECK(l1.lapVerdict == Verdict::kNone);  // nothing to compare against

  in.laps = 2;
  in.lastLapTime = 77000;
  sector_purple::Event l2 = sector_purple::update(st, in);
  CHECK(l2.lapClosed);
  CHECK(l2.lapVerdict == Verdict::kBest);
  CHECK(l2.purpleLap);
  CHECK(l2.closedSector == 0);  // still no sectors
}

TEST_CASE("lap 1: no purple anywhere (no prior bests)") {
  State st;
  sector_purple::reset(st);
  Sample in = base();

  CHECK(purpleOf(st, in) == 0);  // adopt S1 open

  // S1 closes fast — but bestAtOpen was 0.
  in.currentSector = 2;
  in.lapSectorTime[0] = 25000;
  CHECK(purpleOf(st, in) == 0);

  in.currentSector = 3;
  in.lapSectorTime[1] = 26000;
  CHECK(purpleOf(st, in) == 0);

  // Lap line: S3 derived, but bestAtOpen[2] was 0.
  in.currentSector = 1;
  in.laps = 1;
  in.lastLapTime = 78000;
  in.bestSectorTime[0] = 25000;
  in.bestSectorTime[1] = 26000;
  in.bestSectorTime[2] = 27000;
  CHECK(purpleOf(st, in) == 0);
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
  r.s1Fired = purpleOf(st, in);
  in.currentSector = 3;
  in.lapSectorTime[1] = s2;
  r.s2Fired = purpleOf(st, in);
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
  // Same worst case one level up: the library folds the finished lap
  // into getBestLapTime() at the crossing, BEFORE our poll sees the edge.
  if (in.bestLapTime == 0 || lapTotal < in.bestLapTime)
    in.bestLapTime = lapTotal;
  // Current-lap times roll over to the new lap (0 = not yet completed).
  in.lapSectorTime[0] = 0;
  in.lapSectorTime[1] = 0;
  in.lapSectorTime[2] = 0;
  r.s3Fired = purpleOf(st, in);
  return r;
}

TEST_CASE("S1/S2 improvements fire once at the transition; equal does not") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  purpleOf(st, in);

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
  purpleOf(st, in);

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
  purpleOf(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  // Lap 2 jumps straight from S1 to the lap line (missed transitions —
  // e.g. GPS dropout): no s2 recorded, so S3 cannot be derived.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  CHECK(purpleOf(st, in) == 1);  // S1 close still fires
  in.currentSector = 1;
  in.laps = 2;
  in.lastLapTime = 70000;  // would be a monster S3 if mis-derived
  CHECK(purpleOf(st, in) == 0);
}

TEST_CASE("lastLapTime smaller than s1+s2 cannot underflow") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  purpleOf(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  in.currentSector = 2;
  in.lapSectorTime[0] = 26000;
  purpleOf(st, in);
  in.currentSector = 3;
  in.lapSectorTime[1] = 27000;
  purpleOf(st, in);
  in.currentSector = 1;
  in.laps = 2;
  in.lastLapTime = 40000;  // < s1+s2 (inconsistent library state)
  CHECK(purpleOf(st, in) == 0);
}

TEST_CASE("sprint between-runs (sector 0) resets cleanly") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  purpleOf(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  // Run ends: currentSector drops to 0 between sprint runs.
  in.currentSector = 0;
  CHECK(purpleOf(st, in) == 0);

  // New run opens S1; a fast S1 against the lap-1 best still fires.
  in.currentSector = 1;
  CHECK(purpleOf(st, in) == 0);
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  CHECK(purpleOf(st, in) == 1);
}

TEST_CASE("race not started resets; mid-race attach adopts without firing") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  in.raceStarted = false;
  CHECK(purpleOf(st, in) == 0);

  // Attach mid-lap in sector 2 with bests already on the board.
  in.raceStarted = true;
  in.currentSector = 2;
  in.laps = 3;
  in.bestSectorTime[0] = 25000;
  in.bestSectorTime[1] = 26000;
  in.bestSectorTime[2] = 27000;
  CHECK(purpleOf(st, in) == 0);  // adopt, no fire

  // S2 closes faster than best: fires (snapshot taken at attach).
  in.currentSector = 3;
  in.lapSectorTime[1] = 25500;
  CHECK(purpleOf(st, in) == 2);
}

///////////////////////////////////////////
// Plan 0013: lap + sector VERDICTS (better/worse vs the LAST recorded
// one, not vs the best), the purple LAP, and the hold semantics the
// status LEDs depend on.
///////////////////////////////////////////

TEST_CASE("lap verdict: none on lap 1 — nothing to compare, nothing to beat") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  in.currentSector = 2;
  in.lapSectorTime[0] = 25000;
  sector_purple::update(st, in);
  in.currentSector = 3;
  in.lapSectorTime[1] = 26000;
  sector_purple::update(st, in);

  in.currentSector = 1;
  in.laps = 1;
  in.lastLapTime = 78000;
  in.bestLapTime = 78000;  // the library folds it in at the crossing
  in.lapSectorTime[0] = 0;
  in.lapSectorTime[1] = 0;
  sector_purple::Event l1 = sector_purple::update(st, in);
  CHECK(l1.lapClosed);
  CHECK(l1.lapVerdict == Verdict::kNone);
  CHECK(l1.purpleLap == false);  // being the only lap is not an achievement
}

TEST_CASE("purple lap survives the library updating bestLapTime at the line") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  runLap(st, in, 25000, 26000, 78000, 1);   // best lap = 78000

  // Lap 2 slower: worse, not purple.
  in.currentSector = 2; in.lapSectorTime[0] = 26000;
  sector_purple::update(st, in);
  in.currentSector = 3; in.lapSectorTime[1] = 27000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 2; in.lastLapTime = 80000;
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::Event l2 = sector_purple::update(st, in);
  CHECK(l2.lapClosed);
  CHECK(l2.lapVerdict == Verdict::kWorse);
  CHECK(l2.purpleLap == false);

  // Lap 3 beats 78000. runLap's fold puts the new best into
  // bestLapTime BEFORE the edge is seen — the open snapshot must still
  // see the old 78000 and fire.
  in.currentSector = 2; in.lapSectorTime[0] = 24000;
  sector_purple::update(st, in);
  in.currentSector = 3; in.lapSectorTime[1] = 25000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 3; in.lastLapTime = 76000;
  in.bestLapTime = 76000;  // library already folded it in
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::Event l3 = sector_purple::update(st, in);
  CHECK(l3.lapVerdict == Verdict::kBest);
  CHECK(l3.purpleLap);

  // Lap 4 identical to lap 3: faster than nothing, equal to the best.
  // Strict improvement — no purple, and equal is not "better".
  in.currentSector = 2; in.lapSectorTime[0] = 24000;
  sector_purple::update(st, in);
  in.currentSector = 3; in.lapSectorTime[1] = 25000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 4; in.lastLapTime = 76000;
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::Event l4 = sector_purple::update(st, in);
  CHECK(l4.lapVerdict == Verdict::kWorse);
  CHECK(l4.purpleLap == false);
}

TEST_CASE("sector verdict compares the SAME sector last lap, not the best") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  // Lap 1: S1 = 25000 (becomes both the last and the best).
  runLap(st, in, 25000, 26000, 78000, 1);

  // Lap 2: S1 = 24000 — beats the 25000 best, so kBest.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  sector_purple::Event a = sector_purple::update(st, in);
  CHECK(a.closedSector == 1);
  CHECK(a.sectorVerdict == Verdict::kBest);
  CHECK(a.purpleSector == 1);

  // Re-polling an unchanged sample must not re-fire the edge — the LED
  // holds the verdict, the monitor does not repeat it.
  sector_purple::Event again = sector_purple::update(st, in);
  CHECK(again.closedSector == 0);
  CHECK(again.purpleSector == 0);

  // Finish lap 2, folding 24000 into the S1 best.
  in.currentSector = 3; in.lapSectorTime[1] = 26000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 2; in.lastLapTime = 78000;
  in.bestSectorTime[0] = 24000;
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::update(st, in);

  // Lap 3: S1 = 24500. SLOWER than last lap's 24000 -> kWorse. This is
  // the whole point of the verdict: against the best it would also be
  // "not purple", but against the LAST lap it is specifically "you lost
  // half a second here".
  in.currentSector = 2;
  in.lapSectorTime[0] = 24500;
  sector_purple::Event b = sector_purple::update(st, in);
  CHECK(b.closedSector == 1);
  CHECK(b.sectorVerdict == Verdict::kWorse);
  CHECK(b.purpleSector == 0);

  // Finish lap 3 (S1 best stays 24000, last recorded S1 becomes 24500).
  in.currentSector = 3; in.lapSectorTime[1] = 26000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 3; in.lastLapTime = 79000;
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::update(st, in);

  // Lap 4: S1 = 24200. Still off the 24000 best, but BETTER than last
  // lap's 24500 -> kBetter (green), which best-based logic could never
  // report.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24200;
  sector_purple::Event c = sector_purple::update(st, in);
  CHECK(c.closedSector == 1);
  CHECK(c.sectorVerdict == Verdict::kBetter);
  CHECK(c.purpleSector == 0);
}

TEST_CASE("first pass through a sector reports kNone, not a guess") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);

  in.currentSector = 2;
  in.lapSectorTime[0] = 25000;
  sector_purple::Event e = sector_purple::update(st, in);
  CHECK(e.closedSector == 1);
  CHECK(e.sectorVerdict == Verdict::kNone);  // no prior S1, no prior best
}

TEST_CASE("a sector that never closed emits no event and poisons nothing") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  // Lap 2: the S1->S2 transition arrives with a zero current-lap S1
  // (dropout across the line). No event, and the recorded S1 must stay
  // at lap 1's 25000 so the NEXT lap still has something to compare to.
  in.currentSector = 2;
  in.lapSectorTime[0] = 0;
  sector_purple::Event bad = sector_purple::update(st, in);
  CHECK(bad.closedSector == 0);
  CHECK(bad.sectorVerdict == Verdict::kNone);

  in.currentSector = 3; in.lapSectorTime[1] = 26000;
  sector_purple::update(st, in);
  in.currentSector = 1; in.laps = 2; in.lastLapTime = 78000;
  in.lapSectorTime[0] = 0; in.lapSectorTime[1] = 0;
  sector_purple::update(st, in);

  // Lap 3's S1 of 24000 compares against lap 1's 25000, not against 0.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  sector_purple::Event ok = sector_purple::update(st, in);
  CHECK(ok.closedSector == 1);
  CHECK(ok.sectorVerdict == Verdict::kBest);  // also beats the 25000 best
}

TEST_CASE("sprint between-runs holds history; race-not-started clears it") {
  State st;
  sector_purple::reset(st);
  Sample in = base();
  sector_purple::update(st, in);
  runLap(st, in, 25000, 26000, 78000, 1);

  in.currentSector = 0;  // between runs
  sector_purple::update(st, in);
  in.currentSector = 1;
  sector_purple::update(st, in);

  // Run 2's S1 is compared against run 1's — the queue wait between
  // them is not a reason to forget.
  in.currentSector = 2;
  in.lapSectorTime[0] = 24000;
  CHECK(sector_purple::update(st, in).sectorVerdict == Verdict::kBest);

  // Race over: everything goes, so the next session starts blank.
  in.raceStarted = false;
  sector_purple::update(st, in);
  CHECK(st.lastSector == -1);
  CHECK(st.prevLapTime == 0);
  CHECK(st.lastSectorTime[0] == 0);
  CHECK(st.bestLapAtOpen == 0);
}
