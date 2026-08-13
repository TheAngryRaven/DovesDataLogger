#include "sector_purple.h"

namespace sector_purple {

void reset(State& s) {
  s.lastSector = -1;
  s.lastLaps = 0;
  for (int i = 0; i < 3; i++) {
    s.bestAtOpen[i] = 0;
  }
  s.s1 = 0;
  s.s2 = 0;
}

int update(State& s, const Sample& in) {
  if (!in.sectorsConfigured || !in.raceStarted) {
    reset(s);
    return 0;
  }

  if (s.lastSector < 0) {
    // First sample of a live race: adopt the current position without
    // firing. Snapshot the open sector's best now — mid-lap attach is
    // safe because bests only change at the lap line, which we handle
    // as an edge below from here on.
    s.lastSector = in.currentSector;
    s.lastLaps = in.laps;
    if (in.currentSector >= 1 && in.currentSector <= 3) {
      s.bestAtOpen[in.currentSector - 1] =
          in.bestSectorTime[in.currentSector - 1];
    }
    return 0;
  }

  int fired = 0;

  // Lap edge = sector 3 closed at the start/finish line. The library may
  // already have updated bests and rolled the current-lap times, so S3
  // is derived from the lap total minus our own recorded S1/S2 and
  // compared against the snapshot taken when S3 opened.
  if (in.laps > s.lastLaps) {
    if (s.s1 > 0 && s.s2 > 0 && in.lastLapTime > s.s1 + s.s2) {
      uint32_t s3 = in.lastLapTime - s.s1 - s.s2;
      if (s.bestAtOpen[2] != 0 && s3 < s.bestAtOpen[2]) {
        fired = 3;
      }
    }
    // Re-arm for the new lap: S1 is open now.
    s.s1 = 0;
    s.s2 = 0;
    s.bestAtOpen[0] = in.bestSectorTime[0];
    s.lastLaps = in.laps;
    s.lastSector = in.currentSector;
    return fired;
  }

  if (in.currentSector != s.lastSector) {
    if (s.lastSector == 1 && in.currentSector == 2) {
      // S1 closed: its current-lap time is now populated.
      s.s1 = in.lapSectorTime[0];
      if (s.bestAtOpen[0] != 0 && s.s1 > 0 && s.s1 < s.bestAtOpen[0]) {
        fired = 1;
      }
      s.bestAtOpen[1] = in.bestSectorTime[1];
    } else if (s.lastSector == 2 && in.currentSector == 3) {
      s.s2 = in.lapSectorTime[1];
      if (s.bestAtOpen[1] != 0 && s.s2 > 0 && s.s2 < s.bestAtOpen[1]) {
        fired = 2;
      }
      s.bestAtOpen[2] = in.bestSectorTime[2];
    } else if (in.currentSector == 1) {
      // Race start / new sprint run: S1 just opened.
      s.s1 = 0;
      s.s2 = 0;
      s.bestAtOpen[0] = in.bestSectorTime[0];
    } else if (in.currentSector == 0) {
      // Between sprint runs: nothing in progress.
      s.s1 = 0;
      s.s2 = 0;
    }
    s.lastSector = in.currentSector;
  }

  return fired;
}

}  // namespace sector_purple
