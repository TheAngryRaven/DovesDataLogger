#include "doctest.h"
#include "sd_access_policy.h"

using namespace sd_access_policy;

static const int kAllModes[] = {kNone, kLogging, kReplay, kBleTransfer, kTrackParse, kUsbMsc, kFormat};
static const int kHolderModes[] = {kLogging, kReplay, kBleTransfer, kTrackParse, kUsbMsc, kFormat};

// ---------------------------------------------------------------------------
// canAcquire — the arbitration decision table
// ---------------------------------------------------------------------------

TEST_CASE("canAcquire - a free card grants any mode") {
    for (int requested : kHolderModes) {
        CHECK(canAcquire(kNone, requested));
    }
}

TEST_CASE("canAcquire - re-acquiring the held mode is idempotent") {
    for (int mode : kHolderModes) {
        CHECK(canAcquire(mode, mode));
    }
}

TEST_CASE("canAcquire - a non-preemptible holder denies every other mode") {
    const int exclusive[] = {kLogging, kReplay, kBleTransfer, kUsbMsc, kFormat};
    for (int current : exclusive) {
        for (int requested : kHolderModes) {
            if (requested == current) continue;
            // The one sanctioned exception: track parse nests under logging.
            if (current == kLogging && requested == kTrackParse) continue;
            CHECK_FALSE(canAcquire(current, requested));
        }
    }
}

TEST_CASE("canAcquire - track-parse holder is preemptible by everyone") {
    // TRACK_PARSE sections are brief and synchronous; a holder observed
    // from another call site can only be a leaked lock, which must not
    // brick logging mid-race.
    for (int requested : kHolderModes) {
        CHECK(canAcquire(kTrackParse, requested));
    }
}

TEST_CASE("canAcquire - concrete cross-subsystem cases") {
    // BLE transfer cannot steal the card from an active logging session.
    CHECK_FALSE(canAcquire(kLogging, kBleTransfer));
    // Logging cannot start while a BLE transfer is streaming.
    CHECK_FALSE(canAcquire(kBleTransfer, kLogging));
    // Replay and BLE transfer exclude each other.
    CHECK_FALSE(canAcquire(kReplay, kBleTransfer));
    CHECK_FALSE(canAcquire(kBleTransfer, kReplay));
    // Settings/track reads (TRACK_PARSE) nest under a logging hold: they
    // run on the same main-loop task with their own File objects, and a
    // denial here silently degraded track detection to Lap Anything on any
    // boot where logging started first (2026-07-19 field incident).
    CHECK(canAcquire(kLogging, kTrackParse));
    // USB mass-storage and the other exclusive holders lock each other out.
    CHECK_FALSE(canAcquire(kLogging, kUsbMsc));
    CHECK_FALSE(canAcquire(kUsbMsc, kBleTransfer));
    CHECK_FALSE(canAcquire(kUsbMsc, kLogging));
    // USB mass-storage can still preempt a leaked track-parse lock.
    CHECK(canAcquire(kTrackParse, kUsbMsc));
    // On-device format is exclusive (full matrix covered by the parametric
    // cases above) but still preempts a leaked track-parse lock.
    CHECK_FALSE(canAcquire(kFormat, kLogging));
    CHECK(canAcquire(kTrackParse, kFormat));
}

// ---------------------------------------------------------------------------
// ownerAfterAcquire — who is recorded as holder after a granted acquire
// ---------------------------------------------------------------------------

TEST_CASE("ownerAfterAcquire - the requester normally takes ownership") {
    for (int current : kAllModes) {
        for (int requested : kHolderModes) {
            if (current == kLogging && requested == kTrackParse) continue;
            CHECK(ownerAfterAcquire(current, requested) == requested);
        }
    }
}

TEST_CASE("ownerAfterAcquire - nested track parse leaves logging as owner") {
    // The guest parse must not take ownership: its release is ignored by
    // releaseClears (stale-release rule), so the logging hold survives the
    // whole nested acquire/release cycle untouched.
    CHECK(ownerAfterAcquire(kLogging, kTrackParse) == kLogging);
    CHECK_FALSE(releaseClears(kLogging, kTrackParse));
    CHECK(releaseClears(kLogging, kLogging));
}

// ---------------------------------------------------------------------------
// releaseClears — only the current holder's release frees the card
// ---------------------------------------------------------------------------

TEST_CASE("releaseClears - matching holder release frees the card") {
    for (int mode : kHolderModes) {
        CHECK(releaseClears(mode, mode));
    }
}

TEST_CASE("releaseClears - a stale release must not free another holder") {
    for (int current : kHolderModes) {
        for (int releasing : kAllModes) {
            if (releasing == current) continue;
            CHECK_FALSE(releaseClears(current, releasing));
        }
    }
}

TEST_CASE("releaseClears - releasing an already-free card is a no-op") {
    for (int releasing : kAllModes) {
        CHECK_FALSE(releaseClears(kNone, releasing));
    }
}
