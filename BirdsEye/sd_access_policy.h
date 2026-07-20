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
constexpr int kUsbMsc = 5;       // USB mass-storage active (host owns the card)
constexpr int kFormat = 6;       // on-device FAT format (blank soldered-in card)

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
//   - kTrackParse may NEST under kLogging: logging holds the card for the
//     whole race session, but track detection (parseTrackFile) and settings
//     reads run on the same main-loop task with their own File objects, so
//     they are safe alongside an open log file. Before this rule, any boot
//     where the log file was created before the 1 Hz track-detect parse
//     (tach-wake with a warm GPS) had the parse denied and silently fell
//     back to Lap Anything for the whole session (2026-07-19 field
//     incident). The nested grant does NOT transfer ownership — see
//     ownerAfterAcquire() — so the logging hold survives the parse's
//     release untouched.
bool canAcquire(int current, int requested);

// The owner value to record after a granted acquire. Normally `requested`
// takes ownership; the one exception is kTrackParse nesting under kLogging,
// where kLogging must stay the recorded owner (the parse is a same-task
// guest, and releaseClears(kLogging, kTrackParse) is false, so the guest's
// release leaves the logging hold in place).
int ownerAfterAcquire(int current, int requested);

// Does releasing `releasing` free the card when `current` holds it?
// Only the current holder's release clears ownership; a stale release
// from an error path that lost the lock must not free someone else's.
bool releaseClears(int current, int releasing);

}  // namespace sd_access_policy
