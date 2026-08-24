#include "sector_purple.h"

namespace sector_purple {

namespace {

// One rule for both laps and sectors. kBest is checked FIRST: beating
// the previous time when that previous time was also the session best
// is a session best, and purple outranks green.
Verdict verdictFor(uint32_t t, uint32_t prev, uint32_t bestAtOpen) {
  if (t == 0) {
    return Verdict::kNone;  // never closed / not recorded
  }
  if (bestAtOpen != 0 && t < bestAtOpen) {
    return Verdict::kBest;
  }
  if (prev == 0) {
    return Verdict::kNone;  // nothing to compare against yet
  }
  return t < prev ? Verdict::kBetter : Verdict::kWorse;
}

}  // namespace

void reset(State& s) {
  s.lastSector = -1;
  s.lastLaps = 0;
  for (int i = 0; i < 3; i++) {
    s.bestAtOpen[i] = 0;
    s.lastSectorTime[i] = 0;
  }
  s.s1 = 0;
  s.s2 = 0;
  s.prevLapTime = 0;
  s.bestLapAtOpen = 0;
}

Event update(State& s, const Sample& in) {
  Event ev;

  if (!in.raceStarted) {
    reset(s);
    return ev;
  }

  if (s.lastSector < 0) {
    // First sample of a live race: adopt the current position without
    // firing. Snapshot the open sector's best and the session-best lap
    // now — mid-lap attach is safe because bests only change at the lap
    // line, which we handle as an edge below from here on.
    s.lastSector = in.sectorsConfigured ? in.currentSector : 0;
    s.lastLaps = in.laps;
    s.bestLapAtOpen = in.bestLapTime;
    if (in.sectorsConfigured && in.currentSector >= 1 &&
        in.currentSector <= 3) {
      s.bestAtOpen[in.currentSector - 1] =
          in.bestSectorTime[in.currentSector - 1];
    }
    return ev;
  }

  // Lap edge = sector 3 closed at the start/finish line, and the lap
  // itself closed. The library may already have updated bests and
  // rolled the current-lap times, so S3 is derived from the lap total
  // minus our own recorded S1/S2 and compared against the snapshot
  // taken when S3 opened.
  if (in.laps > s.lastLaps) {
    if (in.sectorsConfigured && s.s1 > 0 && s.s2 > 0 &&
        in.lastLapTime > s.s1 + s.s2) {
      uint32_t const s3 = in.lastLapTime - s.s1 - s.s2;
      ev.closedSector = 3;
      ev.sectorVerdict = verdictFor(s3, s.lastSectorTime[2], s.bestAtOpen[2]);
      if (ev.sectorVerdict == Verdict::kBest) {
        ev.purpleSector = 3;
      }
      s.lastSectorTime[2] = s3;
    }

    // The lap half runs with or without sector lines.
    if (in.lastLapTime > 0) {
      ev.lapClosed = true;
      ev.lapVerdict =
          verdictFor(in.lastLapTime, s.prevLapTime, s.bestLapAtOpen);
      ev.purpleLap = (ev.lapVerdict == Verdict::kBest);
      s.prevLapTime = in.lastLapTime;
    }

    // Re-arm for the new lap: S1 is open now, and the library has by
    // now folded the finished lap into its best — which is exactly the
    // baseline the NEXT lap must beat.
    s.s1 = 0;
    s.s2 = 0;
    s.bestAtOpen[0] = in.bestSectorTime[0];
    s.bestLapAtOpen = in.bestLapTime;
    s.lastLaps = in.laps;
    s.lastSector = in.sectorsConfigured ? in.currentSector : 0;
    return ev;
  }

  if (in.sectorsConfigured && in.currentSector != s.lastSector) {
    if (s.lastSector == 1 && in.currentSector == 2) {
      // S1 closed: its current-lap time is now populated. A zero here
      // means it never really closed (GPS dropout across the line), so
      // report nothing rather than a bogus verdict.
      s.s1 = in.lapSectorTime[0];
      if (s.s1 > 0) {
        ev.closedSector = 1;
        ev.sectorVerdict =
            verdictFor(s.s1, s.lastSectorTime[0], s.bestAtOpen[0]);
        if (ev.sectorVerdict == Verdict::kBest) {
          ev.purpleSector = 1;
        }
        s.lastSectorTime[0] = s.s1;
      }
      s.bestAtOpen[1] = in.bestSectorTime[1];
    } else if (s.lastSector == 2 && in.currentSector == 3) {
      s.s2 = in.lapSectorTime[1];
      if (s.s2 > 0) {
        ev.closedSector = 2;
        ev.sectorVerdict =
            verdictFor(s.s2, s.lastSectorTime[1], s.bestAtOpen[1]);
        if (ev.sectorVerdict == Verdict::kBest) {
          ev.purpleSector = 2;
        }
        s.lastSectorTime[1] = s.s2;
      }
      s.bestAtOpen[2] = in.bestSectorTime[2];
    } else if (in.currentSector == 1) {
      // Race start / new sprint run: S1 just opened.
      s.s1 = 0;
      s.s2 = 0;
      s.bestAtOpen[0] = in.bestSectorTime[0];
    } else if (in.currentSector == 0) {
      // Between sprint runs: nothing in progress. lastSectorTime[] and
      // prevLapTime deliberately survive — run 2's sector 1 wants to be
      // compared against run 1's.
      s.s1 = 0;
      s.s2 = 0;
    }
    s.lastSector = in.currentSector;
  }

  return ev;
}

}  // namespace sector_purple
