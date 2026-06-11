#pragma once

///////////////////////////////////////////
// SD ACCESS ARBITRATION POLICY
// The decision table for the SD card access "mutex" in sd_functions.ino:
// which subsystem may take ownership given who currently holds it.
//
// The policy itself is pure logic (no Arduino headers) so it is exercised
// by host tests. ATOMICITY IS NOT PROVIDED HERE — acquireSDAccess() /
// releaseSDAccess() must evaluate these predicates and commit the state
// change inside a critical section, because the Bluefruit callback task
// and the main loop share the owner variable.
///////////////////////////////////////////

namespace sd_access_policy {

// SD owner modes. The SD_ACCESS_* macros in sd_functions.h alias these —
// these constants are the single source of truth for the values.
constexpr int kNone = 0;         // card is free
constexpr int kLogging = 1;      // DOVEX session logging (held all session)
constexpr int kReplay = 2;       // DOVEX header replay
constexpr int kBleTransfer = 3;  // BLE file transfer / track write / OTA staging
constexpr int kTrackParse = 4;   // brief track-JSON / settings reads

// May `requested` take ownership when `current` holds the card?
//   - Re-acquiring the mode you already hold is allowed (idempotent), so
//     retry loops (e.g. the 1 Hz log-file open retry) don't deadlock on
//     themselves. Callers must therefore never use a same-mode acquire to
//     enter a *second* concurrent operation — see bleDeleteFile()'s
//     explicit bleTransferInProgress guard.
//   - kTrackParse is preemptible: every TRACK_PARSE section is brief and
//     synchronous (acquire and release inside one call), so a holder seen
//     across calls can only be a leaked lock from a forgotten error-path
//     release — letting others claim the card keeps a leak from bricking
//     logging mid-race.
bool canAcquire(int current, int requested);

// Does releasing `releasing` free the card when `current` holds it?
// Only the current holder's release clears ownership; a stale release
// from an error path that lost the lock must not free someone else's.
bool releaseClears(int current, int releasing);

}  // namespace sd_access_policy
