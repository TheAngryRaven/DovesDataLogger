#pragma once

#include <stdint.h>

///////////////////////////////////////////
// LAP / SECTOR CLOSE-EDGE MONITOR
// Watches the active lap timer and reports what just closed: which
// sector or lap ended this step, how it compared to the last recorded
// one, and whether it went session-purple. The LED subsystem turns
// those verdicts into a status colour (led_status) and the purple
// events into the celebration animation.
//
// NOTE ON THE NAME: this unit started life as purple-sector detection
// only (plan 0006) and kept its filename through plan 0012, which added
// the lap edge and the better/worse verdicts. Purple is now one of its
// outputs, not its whole job. A rename is a clean standalone follow-up;
// it was not worth burying the behaviour diff under file churn.
//
// WHY THE EDGE MACHINERY IS SHAPED LIKE THIS: the lap timer library
// exposes getCurrentSector() / getCurrentLapSectorNTime() /
// getBestSectorNTime() but no events, and updateBestSectors() runs AT
// THE START/FINISH CROSSING — not at each sector line — so at the exact
// moment sector 3 closes, the library may already have folded this lap
// into the "best" values and rolled the current-lap times over. Same
// for getBestLapTime(). This unit therefore never compares against live
// library state at a close edge:
//
//  - Best times are snapshotted when each sector OPENS (bestAtOpen[])
//    and when each lap opens (bestLapAtOpen).
//  - S1/S2 close on getCurrentSector() transitions (1->2, 2->3) and
//    compare the just-closed current-lap time against the snapshot.
//  - S3 closes on the LAP-COUNT edge, and its time is DERIVED:
//    lastLapTime - s1 - s2 — both inputs immune to the lap-line update.
//  - Purple fires only when the beaten best was nonzero: no purple
//    anywhere on lap 1 (celebrating every first-ever sector is noise).
//
// THE BETTER/WORSE VERDICT IS AGAINST THE LAST RECORDED TIME, NOT THE
// BEST. Comparing against the session best only ever answers "purple or
// red", which tells the driver nothing about whether they are improving
// lap over lap. So kBetter/kWorse compare against lastSectorTime[] /
// prevLapTime, and kBest overrides when the session best also fell.
//
// SECTORS ARE OPTIONAL. Lap tracking runs whenever the race has
// started, including a Lap Anything (WaypointLapTimer) session with no
// sector lines at all — the lap verdict has to work there too. Only the
// sector half is gated on sectorsConfigured.
//
// Poll-and-diff, the house pattern (sprintLastRunCount). The sketch
// builds a Sample from the activeTimer*() wrappers each LED frame.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace sector_purple {

// How a just-closed lap or sector compared. kNone = nothing to compare
// against (first lap, first time through a sector, or a time we never
// recorded), which renders as an unlit status LED rather than a guess.
enum class Verdict : uint8_t {
  kNone = 0,
  kBetter,  // strictly faster than the last recorded one
  kWorse,   // slower than, or identical to, the last recorded one
  kBest,    // beat the session best — overrides kBetter
};

struct Sample {
  // activeTimerSectorsConfigured(): Lap Anything / WaypointLapTimer has
  // no sectors — false suppresses the sector half; laps still track.
  bool sectorsConfigured;
  bool raceStarted;
  // getCurrentSector(): 0 = race not started / between sprint runs,
  // 1..3 = active sector.
  int currentSector;
  int laps;                  // completed laps — the S3 / lap close edge
  uint32_t lastLapTime;      // ms
  uint32_t bestLapTime;      // getBestLapTime(), ms, 0 = none
  uint32_t lapSectorTime[3]; // getCurrentLapSector{1,2,3}Time(), ms
  uint32_t bestSectorTime[3];// getBestSector{1,2,3}Time(), ms, 0 = none
};

// What closed on this step. Default-constructed = nothing happened.
struct Event {
  int purpleSector = 0;   // 1..3 when that sector went session-purple
  bool purpleLap = false; // the lap just completed is the session best
  int closedSector = 0;   // 1..3 sector that closed this step, else 0
  Verdict sectorVerdict = Verdict::kNone;  // for closedSector
  bool lapClosed = false;
  Verdict lapVerdict = Verdict::kNone;     // for the lap just completed
};

struct State {
  int lastSector;          // -1 = uninitialized
  int lastLaps;
  uint32_t bestAtOpen[3];  // best time snapshotted when sector N opened
  uint32_t s1;             // this lap's closed sector times (0 = not yet)
  uint32_t s2;
  // The most recently COMPLETED time for each sector, and for a lap.
  // Not strictly "last lap's": if a sector never closed (a GPS dropout
  // skipped the transition) its entry keeps the older value rather than
  // going blank, so one glitch doesn't dark the indicator for a lap.
  uint32_t lastSectorTime[3];
  uint32_t prevLapTime;
  uint32_t bestLapAtOpen;  // session-best lap as of this lap's start
};

void reset(State& s);

// Feed one sample and get back whatever closed on this step. Equal
// times never count as an improvement (strict <), so a repeated
// identical time reads kWorse and never fires purple.
Event update(State& s, const Sample& in);

}  // namespace sector_purple
