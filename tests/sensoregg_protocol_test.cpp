#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>

#include "sensoregg_protocol.h"

using namespace sensoregg_protocol;

// Golden frames: the byte layouts are the wire contract with the
// DovesSensorEgg firmware — the v2 bytes are IDENTICAL to the egg
// repo's pw_adv_encode golden fixture. If either layout changes, both
// repos' vectors must change with it — deliberately.
//
// v1 (14 bytes):
//   FF FF        company ID (SIG test/internal, inside the array)
//   50 57        magic 'P' 'W'
//   01           protocol version
//   00           flags
//   64 19        EGT 0x1964 = 6500 deci-degC = 650.0 C
//   4D 01        cold junction 0x014D = 333 deci-degC = 33.3 C
//   00           MCP9600 STATUS
//   FF           battery stub
//   2A 01        sequence 0x012A = 298
static const uint8_t kGoldenFrame[kPayloadLen] = {
    0xFF, 0xFF, 0x50, 0x57, 0x01, 0x00, 0x64, 0x19,
    0x4D, 0x01, 0x00, 0xFF, 0x2A, 0x01};

// v2 (16 bytes): version 02, battery real (0x57 = 87%), and the aux
// thermistor appended: 0x00EA = 234 deci-degC = 23.4 C.
static const uint8_t kGoldenFrameV2[kPayloadLenV2] = {
    0xFF, 0xFF, 0x50, 0x57, 0x02, 0x00, 0x64, 0x19,
    0x4D, 0x01, 0x00, 0x57, 0x2A, 0x01, 0xEA, 0x00};

// ---------------------------------------------------------------------------
// Magic filter
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - magic matches golden frame") {
    CHECK(matchesMagic(kGoldenFrame, sizeof(kGoldenFrame)));
    // The cheap callback filter only needs the 4-byte prefix.
    CHECK(matchesMagic(kGoldenFrame, 4));
}

TEST_CASE("sensoregg_protocol - magic rejects wrong prefix") {
    uint8_t frame[kPayloadLen];
    for (size_t i = 0; i < kPayloadLen; i++) frame[i] = kGoldenFrame[i];

    frame[0] = 0x4C;  // some other company ID
    CHECK_FALSE(matchesMagic(frame, sizeof(frame)));

    frame[0] = 0xFF;
    frame[2] = 'X';  // 0xFFFF squatter without our magic
    CHECK_FALSE(matchesMagic(frame, sizeof(frame)));
}

TEST_CASE("sensoregg_protocol - magic rejects null and short input") {
    CHECK_FALSE(matchesMagic(nullptr, kPayloadLen));
    CHECK_FALSE(matchesMagic(kGoldenFrame, 3));
    CHECK_FALSE(matchesMagic(kGoldenFrame, 0));
}

// ---------------------------------------------------------------------------
// Payload parse
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - golden v1 frame parses (aux is NaN)") {
    Reading r;
    REQUIRE(parsePayload(kGoldenFrame, sizeof(kGoldenFrame), r));

    CHECK(r.egtC == doctest::Approx(650.0f));
    CHECK(r.junctionC == doctest::Approx(33.3f));
    CHECK(r.flags == 0);
    CHECK_FALSE(r.pairingActive);
    CHECK_FALSE(r.tcFault);
    CHECK(r.status == 0);
    CHECK(r.battery == 0xFF);
    CHECK(r.sequence == 298);
    // v1 has no aux field: it must parse as NaN, never 0.0 (a fake
    // freezing-point reading on the intake page/log).
    CHECK(std::isnan(r.auxC));
    CHECK(r.protoVersion == kProtocolVersion);
}

TEST_CASE("sensoregg_protocol - golden v2 frame parses (egg-repo fixture bytes)") {
    Reading r;
    REQUIRE(parsePayload(kGoldenFrameV2, sizeof(kGoldenFrameV2), r));

    CHECK(r.egtC == doctest::Approx(650.0f));
    CHECK(r.junctionC == doctest::Approx(33.3f));
    CHECK(r.auxC == doctest::Approx(23.4f));
    CHECK(r.battery == 87);
    CHECK(r.sequence == 298);
    CHECK(r.protoVersion == kProtocolVersionV2);
}

TEST_CASE("sensoregg_protocol - v2 aux sentinel becomes NaN") {
    uint8_t frame[kPayloadLenV2];
    for (size_t i = 0; i < kPayloadLenV2; i++) frame[i] = kGoldenFrameV2[i];
    frame[14] = 0x00;  // 0x8000 LE: divider open/shorted on the egg
    frame[15] = 0x80;

    Reading r;
    REQUIRE(parsePayload(frame, sizeof(frame), r));
    CHECK(std::isnan(r.auxC));
    CHECK(r.egtC == doctest::Approx(650.0f));  // other fields unaffected
}

TEST_CASE("sensoregg_protocol - negative temperatures decode") {
    uint8_t frame[kPayloadLen];
    for (size_t i = 0; i < kPayloadLen; i++) frame[i] = kGoldenFrame[i];
    // EGT -12.5 C = -125 deci-degC = 0xFF83 LE
    frame[6] = 0x83;
    frame[7] = 0xFF;
    // Junction -0.1 C = -1 deci-degC = 0xFFFF LE
    frame[8] = 0xFF;
    frame[9] = 0xFF;

    Reading r;
    REQUIRE(parsePayload(frame, sizeof(frame), r));
    CHECK(r.egtC == doctest::Approx(-12.5f));
    CHECK(r.junctionC == doctest::Approx(-0.1f));
}

TEST_CASE("sensoregg_protocol - invalid sentinel becomes NaN") {
    uint8_t frame[kPayloadLen];
    for (size_t i = 0; i < kPayloadLen; i++) frame[i] = kGoldenFrame[i];
    // 0x8000 LE in both temperature fields.
    frame[6] = 0x00;
    frame[7] = 0x80;
    frame[8] = 0x00;
    frame[9] = 0x80;

    Reading r;
    REQUIRE(parsePayload(frame, sizeof(frame), r));
    CHECK(std::isnan(r.egtC));
    CHECK(std::isnan(r.junctionC));
}

TEST_CASE("sensoregg_protocol - flag bits decode") {
    uint8_t frame[kPayloadLen];
    for (size_t i = 0; i < kPayloadLen; i++) frame[i] = kGoldenFrame[i];
    frame[5] = 0x03;  // pairing + TC fault
    frame[10] = 0x10; // STATUS input-range bit (open probe)

    Reading r;
    REQUIRE(parsePayload(frame, sizeof(frame), r));
    CHECK(r.pairingActive);
    CHECK(r.tcFault);
    CHECK(r.status == 0x10);
}

TEST_CASE("sensoregg_protocol - parse rejects short / null / bad version") {
    Reading r;
    CHECK_FALSE(parsePayload(nullptr, kPayloadLen, r));
    CHECK_FALSE(parsePayload(kGoldenFrame, kPayloadLen - 1, r));

    uint8_t frame[kPayloadLen];
    for (size_t i = 0; i < kPayloadLen; i++) frame[i] = kGoldenFrame[i];
    frame[4] = 0x03;  // future protocol version — layout unknown, reject
    CHECK_FALSE(parsePayload(frame, sizeof(frame), r));
}

TEST_CASE("sensoregg_protocol - truncated v2 is corrupt, not v1") {
    // A frame CLAIMING v2 but shorter than 16 bytes must be rejected —
    // parsing its first 14 bytes as v1 would be accepting corruption.
    uint8_t frame[kPayloadLenV2];
    for (size_t i = 0; i < kPayloadLenV2; i++) frame[i] = kGoldenFrameV2[i];

    Reading r;
    CHECK_FALSE(parsePayload(frame, kPayloadLen, r));       // 14 bytes
    CHECK_FALSE(parsePayload(frame, kPayloadLenV2 - 1, r)); // 15 bytes
    REQUIRE(parsePayload(frame, kPayloadLenV2, r));         // 16 is whole
}

TEST_CASE("sensoregg_protocol - parse accepts longer-than-spec payloads") {
    // Forward compatibility: a future egg may append fields to either
    // version's layout.
    uint8_t f1[kPayloadLen + 4] = {0};
    for (size_t i = 0; i < kPayloadLen; i++) f1[i] = kGoldenFrame[i];
    uint8_t f2[kPayloadLenV2 + 4] = {0};
    for (size_t i = 0; i < kPayloadLenV2; i++) f2[i] = kGoldenFrameV2[i];

    Reading r;
    REQUIRE(parsePayload(f1, sizeof(f1), r));
    CHECK(r.egtC == doctest::Approx(650.0f));
    REQUIRE(parsePayload(f2, sizeof(f2), r));
    CHECK(r.auxC == doctest::Approx(23.4f));
}

// ---------------------------------------------------------------------------
// Staleness
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - staleness threshold") {
    CHECK(isFresh(1000, 1000));
    CHECK(isFresh(1000, 1000 + kStalenessMs - 1));
    CHECK_FALSE(isFresh(1000, 1000 + kStalenessMs));
    CHECK_FALSE(isFresh(1000, 1000 + kStalenessMs + 5000));
}

TEST_CASE("sensoregg_protocol - staleness is wrap-safe across millis rollover") {
    const uint32_t nearWrap = 0xFFFFFF38u;  // 200 ms before rollover
    CHECK(isFresh(nearWrap, nearWrap + 999));  // wraps past 0, still fresh
    CHECK_FALSE(isFresh(nearWrap, nearWrap + kStalenessMs));
}

// ---------------------------------------------------------------------------
// Display-unit conversion (Temp1 page renders Fahrenheit; logging stays C)
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - celsiusToFahrenheit known points") {
    CHECK(celsiusToFahrenheit(0.0f) == doctest::Approx(32.0f));
    CHECK(celsiusToFahrenheit(100.0f) == doctest::Approx(212.0f));
    CHECK(celsiusToFahrenheit(-40.0f) == doctest::Approx(-40.0f));
    // Golden-frame EGT: 650.0 C — a realistic two-stroke EGT reading.
    CHECK(celsiusToFahrenheit(650.0f) == doctest::Approx(1202.0f));
}

TEST_CASE("sensoregg_protocol - celsiusToFahrenheit propagates NaN") {
    // NaN is the no-reading sentinel end to end: a stale/invalid probe must
    // still render '---' after conversion, never a numeric artifact.
    CHECK(std::isnan(celsiusToFahrenheit(std::numeric_limits<float>::quiet_NaN())));
}

// ---------------------------------------------------------------------------
// Zombie-egg detection (frozen sequence counter, 2026-07-19 field incident)
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - seq monitor: advancing sequence stays live") {
    SeqMonitor m;
    CHECK_FALSE(seqMonitorLive(m, 0));  // nothing fed yet
    uint32_t t = 1000;
    for (uint16_t s = 100; s < 130; s++) {
        seqMonitorFeed(m, s, t);
        CHECK(seqMonitorLive(m, t));
        t += 100;  // 10 Hz packets
        CHECK(seqMonitorLive(m, t));  // still inside kStalenessMs
    }
}

TEST_CASE("sensoregg_protocol - seq monitor: frozen sequence goes dead at 1 s") {
    // The hung-egg case: packets keep arriving (radio alive) but the app
    // stopped updating the payload, so every packet carries the same seq.
    SeqMonitor m;
    seqMonitorFeed(m, 42, 1000);
    CHECK(seqMonitorLive(m, 1000));
    uint32_t t = 1000;
    for (int i = 0; i < 30; i++) {  // 3 s of identical packets at 10 Hz
        t += 100;
        seqMonitorFeed(m, 42, t);
    }
    CHECK_FALSE(seqMonitorLive(m, t));                       // dead
    CHECK(seqMonitorLive(m, 1000 + kStalenessMs - 1));       // was live until 1 s
    CHECK_FALSE(seqMonitorLive(m, 1000 + kStalenessMs));     // exact boundary
}

TEST_CASE("sensoregg_protocol - seq monitor: recovery after the egg reboots") {
    SeqMonitor m;
    seqMonitorFeed(m, 500, 1000);
    for (uint32_t t = 1100; t <= 5000; t += 100) seqMonitorFeed(m, 500, t);
    CHECK_FALSE(seqMonitorLive(m, 5000));   // hung
    seqMonitorFeed(m, 0, 5100);             // egg power-cycled: seq restarts
    CHECK(seqMonitorLive(m, 5100));
}

TEST_CASE("sensoregg_protocol - seq monitor: uint16 wrap counts as a change") {
    // At 10 Hz the sequence wraps every ~109 min; 65535 -> 0 must read as
    // life, not a freeze.
    SeqMonitor m;
    seqMonitorFeed(m, 65535, 1000);
    seqMonitorFeed(m, 0, 1100);
    CHECK(m.lastSeq == 0);
    CHECK(seqMonitorLive(m, 1100));
}

TEST_CASE("sensoregg_protocol - seq monitor: duplicate reception is not a freeze") {
    // The scanner can catch the same advertising event twice; a repeated
    // seq within the staleness window must not kill a live egg.
    SeqMonitor m;
    seqMonitorFeed(m, 7, 1000);
    seqMonitorFeed(m, 7, 1050);   // duplicate 50 ms later
    seqMonitorFeed(m, 8, 1200);   // next real update
    CHECK(seqMonitorLive(m, 1200));
    CHECK(seqMonitorLive(m, 1200 + kStalenessMs - 1));
}

// ---------------------------------------------------------------------------
// Scan tuning invariants
// ---------------------------------------------------------------------------

TEST_CASE("sensoregg_protocol - scan tuning: pinned values") {
    // 0.625 ms units: 90 ms interval / 40 ms window. Deliberate values —
    // change them only with the anti-phase-lock + GPS-drop rationale in
    // sensoregg_protocol.h updated to match.
    CHECK(kScanIntervalUnits == 144);
    CHECK(kScanWindowUnits == 64);
}

TEST_CASE("sensoregg_protocol - scan tuning: invariants") {
    // Window must fit inside the interval (BLE spec), and radio duty must
    // stay under 45% — the 60% duty of the original 100/60 tuning deferred
    // the TIMER3 GPS drain enough to drop 25 Hz PVT frames.
    CHECK(kScanWindowUnits < kScanIntervalUnits);
    CHECK(kScanWindowUnits * 100 <= kScanIntervalUnits * 45);

    // The interval must sit off the egg's adv interval so the phases
    // sweep instead of locking; at least a 5 ms offset keeps a deaf-zone
    // park escaping within ~1 s (kStalenessMs). The egg de-aliased its
    // interval to 179 units (111.875 ms) — this pin was stale at 160
    // until 2026-07-27.
    constexpr uint16_t kEggAdvUnits = 179;
    uint16_t diff = kScanIntervalUnits > kEggAdvUnits
                        ? kScanIntervalUnits - kEggAdvUnits
                        : kEggAdvUnits - kScanIntervalUnits;
    CHECK(diff >= 8);  // >= 5 ms of per-cycle phase sweep
}
