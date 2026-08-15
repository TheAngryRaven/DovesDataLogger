#pragma once

#include <stdint.h>

///////////////////////////////////////////
// PURPLE (SESSION-BEST) SECTOR DETECTION
// Decides when the driver just set a session-best sector so the LED
// strip can fire its purple celebration. The lap timer library exposes
// getCurrentSector() / getCurrentLapSectorNTime() / getBestSectorNTime()
// but no events, and updateBestSectors() runs AT THE START/FINISH
// CROSSING — not at each sector line — so at the exact moment sector 3
// closes, the library may already have folded this lap into the "best"
// values and rolled the current-lap times over. This unit never
// compares against live library state at a close edge:
//
//  - Best times are snapshotted when each sector OPENS (bestAtOpen[]).
//  - S1/S2 close on getCurrentSector() transitions (1->2, 2->3) and
//    compare the just-closed current-lap time against the snapshot.
//  - S3 closes on the LAP-COUNT edge, and its time is DERIVED:
//    lastLapTime - s1 - s2 — both inputs immune to the lap-line update.
//  - Purple fires only when the beaten best was nonzero: no purple
//    anywhere on lap 1 (celebrating every first-ever sector is noise).
//
// Poll-and-diff, the house pattern (sprintLastRunCount). The sketch
// builds a Sample from the activeTimer*() wrappers each LED frame.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace sector_purple {

struct Sample {
  // activeTimerSectorsConfigured(): Lap Anything / WaypointLapTimer has
  // no sectors — false resets the monitor and nothing ever fires.
  bool sectorsConfigured;
  bool raceStarted;
  // getCurrentSector(): 0 = race not started / between sprint runs,
  // 1..3 = active sector.
  int currentSector;
  int laps;                  // completed laps — the S3 close edge
  uint32_t lastLapTime;      // ms
  uint32_t lapSectorTime[3]; // getCurrentLapSector{1,2,3}Time(), ms
  uint32_t bestSectorTime[3];// getBestSector{1,2,3}Time(), ms, 0 = none
};

struct State {
  int lastSector;          // -1 = uninitialized
  int lastLaps;
  uint32_t bestAtOpen[3];  // best time snapshotted when sector N opened
  uint32_t s1;             // this lap's closed sector times (0 = not yet)
  uint32_t s2;
};

void reset(State& s);

// Feed one sample; returns 1..3 when that sector just went
// session-purple, else 0. Equal times do not fire (strict improvement).
int update(State& s, const Sample& in);

}  // namespace sector_purple
