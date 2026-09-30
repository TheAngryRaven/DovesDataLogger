#include "doctest.h"
#include "sd_probe.h"

using namespace sd_probe;

namespace {

Attempt probe(bool cardOk, bool hasSectors, bool volumeOk) {
    return {cardOk, hasSectors, volumeOk};
}

const Attempt kMountedProbe = probe(true, true, true);
const Attempt kNoVolume = probe(true, true, false);

}  // namespace

// ---------------------------------------------------------------------------
// Happy path
// ---------------------------------------------------------------------------

TEST_CASE("classify - a mounting volume is mounted on the first probe") {
    CHECK(classify(kMountedProbe, 0) == Verdict::kMounted);
}

TEST_CASE("classify - a volume that mounts on the retry is mounted, not unformatted") {
    // The whole point of the retry: a card that was mid-command from an
    // interrupted boot answers properly once it has settled.
    CHECK(classify(kNoVolume, 0) == Verdict::kRetryVolume);
    CHECK(classify(kMountedProbe, 1) == Verdict::kMounted);
}

// ---------------------------------------------------------------------------
// Unformatted needs consecutive evidence
// ---------------------------------------------------------------------------

TEST_CASE("classify - one no-volume probe is a retry, never an erase offer") {
    CHECK(classify(kNoVolume, 0) == Verdict::kRetryVolume);
}

TEST_CASE("classify - consecutive no-volume probes declare the card unformatted") {
    CHECK(classify(kNoVolume, kVolumeFailuresToDeclare - 1) == Verdict::kUnformatted);
    // And anything past the threshold too (a caller looping further).
    CHECK(classify(kNoVolume, kVolumeFailuresToDeclare) == Verdict::kUnformatted);
}

TEST_CASE("classify - the retry budget is exactly the declared constant") {
    int failures = 0;
    Verdict v = Verdict::kRetryVolume;
    int probes = 0;
    while (v == Verdict::kRetryVolume) {
        v = classify(kNoVolume, failures);
        failures++;
        probes++;
    }
    CHECK(v == Verdict::kUnformatted);
    CHECK(probes == kVolumeFailuresToDeclare);
}

// ---------------------------------------------------------------------------
// The card layer decides "dead" outright
// ---------------------------------------------------------------------------

TEST_CASE("classify - no SPI-level answer is dead regardless of history") {
    CHECK(classify(probe(false, false, false), 0) == Verdict::kDead);
    CHECK(classify(probe(false, false, false), 1) == Verdict::kDead);
    // volumeOk cannot be true without cardOk in practice, but the card
    // layer must still win if the sketch ever hands in an odd tuple.
    CHECK(classify(probe(false, true, true), 0) == Verdict::kDead);
}

TEST_CASE("classify - a card with no size is dead, not blank") {
    // sectorCount() == 0 means the CSD never read back: the card layer
    // is not healthy, so an erase must not be offered.
    CHECK(classify(probe(true, false, false), 0) == Verdict::kDead);
    CHECK(classify(probe(true, false, false), 1) == Verdict::kDead);
}

TEST_CASE("classify - a flapping card is dead, not unformatted") {
    // Probe 1: answers, no volume. Probe 2: stops answering. That is a
    // marginal card/wiring, and the safe page is FAULT (no erase offer).
    CHECK(classify(kNoVolume, 0) == Verdict::kRetryVolume);
    CHECK(classify(probe(false, false, false), 1) == Verdict::kDead);
}

// ---------------------------------------------------------------------------
// Constants the sketch's timing depends on
// ---------------------------------------------------------------------------

TEST_CASE("constants - the settle is short enough to fit the WDT budget") {
    // The boot after a soft reset may be running under a 4 s watchdog
    // carried over from the previous session; the settle plus one SdFat
    // init attempt must stay well inside that with a pet between them.
    CHECK(kVolumeRetryDelayMs >= 100);
    CHECK(kVolumeRetryDelayMs <= 1000);
    CHECK(kVolumeFailuresToDeclare >= 2);
}
