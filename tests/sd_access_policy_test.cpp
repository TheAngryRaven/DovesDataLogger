#include "doctest.h"
#include "sd_access_policy.h"

using namespace sd_access_policy;

static const int kAllModes[] = {kNone, kLogging, kReplay, kBleTransfer, kTrackParse};
static const int kHolderModes[] = {kLogging, kReplay, kBleTransfer, kTrackParse};

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
    const int exclusive[] = {kLogging, kReplay, kBleTransfer};
    for (int current : exclusive) {
        for (int requested : kHolderModes) {
            if (requested == current) continue;
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
    // Settings/track reads (TRACK_PARSE) are denied while logging holds it.
    CHECK_FALSE(canAcquire(kLogging, kTrackParse));
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
