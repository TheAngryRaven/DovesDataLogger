#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "sensoregg_gatt.h"

using namespace sensoregg_gatt;

// ---------------------------------------------------------------------------
// Golden fixtures — BYTE-IDENTICAL to the DovesSensorEgg repo's
// pw_gatt_encode golden tests (its encoder produces these exact buffers).
// If either side changes a layout, both repos' vectors change with it —
// deliberately. Same cross-repo discipline as sensoregg_protocol <->
// pw_adv_encode for the beacon.
// ---------------------------------------------------------------------------

// The EGT pod's 104-byte Descriptor: schema 1, device type 0x01, fw 1.1,
// 4 channels x 24-byte records (EGT/CJ @250ms scale 0.1, IAT @1000ms
// scale 0.1, BATT @30000ms scale 1.0).
static const uint8_t kGoldenDescriptor[104] = {
    // header
    0x01, 0x01, 0x01, 0x01, 0x04, 0x18, 0x00, 0x00,
    // ch0 EGT
    0x00, 0x01, 0xFA, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x45, 0x47, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch1 CJ
    0x01, 0x01, 0xFA, 0x00, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x43, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch2 IAT
    0x02, 0x01, 0xE8, 0x03, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00, 0x00,
    0x49, 0x41, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // ch3 BATT
    0x03, 0x08, 0x30, 0x75, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
    0x42, 0x41, 0x54, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// buildClock(0xA5, 0x01234567) from the egg side.
static const uint8_t kGoldenClock[6] = {0xA5, 0x00, 0x67, 0x45, 0x23, 0x01};

// buildSampleFrame(ch 0, boot 0xA5, seq 0x2A, base 0xDEADBEEF, interval
// 250, samples {1234, -40, sentinel}).
static const uint8_t kGoldenFrame[16] = {0x00, 0xA5, 0x2A, 0xEF, 0xBE, 0xAD,
                                         0xDE, 0xFA, 0x00, 0x03, 0xD2, 0x04,
                                         0xD8, 0xFF, 0x00, 0x80};

TEST_CASE("sensoregg_gatt - golden descriptor parses field-for-field") {
    PodDescriptor pd;
    REQUIRE(parseDescriptor(kGoldenDescriptor, sizeof(kGoldenDescriptor), pd));
    CHECK(pd.schemaVersion == 1);
    CHECK(pd.deviceType == 0x01);
    CHECK(pd.fwMajor == 1);
    CHECK(pd.fwMinor == 1);
    CHECK(pd.channelCount == 4);
    CHECK(pd.recordLen == 24);
    CHECK(pd.ch[0].id == 0);
    CHECK(pd.ch[0].quantity == 0x01);
    CHECK(pd.ch[0].periodMs == 250);
    CHECK(pd.ch[0].scale == doctest::Approx(0.1f));
    CHECK(pd.ch[0].offset == doctest::Approx(0.0f));
    CHECK(strcmp(pd.ch[0].name, "EGT") == 0);
    CHECK(strcmp(pd.ch[1].name, "CJ") == 0);
    CHECK(pd.ch[2].periodMs == 1000);
    CHECK(strcmp(pd.ch[2].name, "IAT") == 0);
    CHECK(pd.ch[3].quantity == 0x08);
    CHECK(pd.ch[3].periodMs == 30000);
    CHECK(pd.ch[3].scale == doctest::Approx(1.0f));
    CHECK(strcmp(pd.ch[3].name, "BATT") == 0);
}

TEST_CASE("sensoregg_gatt - descriptor rejects malformed values") {
    PodDescriptor pd;
    CHECK(!parseDescriptor(nullptr, 104, pd));
    CHECK(!parseDescriptor(kGoldenDescriptor, 7, pd));    // short header
    CHECK(!parseDescriptor(kGoldenDescriptor, 103, pd));  // truncated records
    uint8_t bad[104];
    memcpy(bad, kGoldenDescriptor, sizeof(bad));
    bad[0] = 2;  // unknown schema version
    CHECK(!parseDescriptor(bad, sizeof(bad), pd));
    memcpy(bad, kGoldenDescriptor, sizeof(bad));
    bad[5] = 23;  // record_len below the schema-v1 minimum
    CHECK(!parseDescriptor(bad, sizeof(bad), pd));
    memcpy(bad, kGoldenDescriptor, sizeof(bad));
    bad[4] = 0;  // no channels
    CHECK(!parseDescriptor(bad, sizeof(bad), pd));
    memcpy(bad, kGoldenDescriptor, sizeof(bad));
    bad[4] = 22;  // over the 21-channel / 512-byte ceiling
    CHECK(!parseDescriptor(bad, sizeof(bad), pd));
}

TEST_CASE("sensoregg_gatt - descriptor strides by a larger declared record_len") {
    // A future schema may append record fields: same header shape,
    // record_len 28, one channel; the first 24 bytes keep their layout
    // and the tail must be skipped.
    uint8_t d[8 + 28] = {0};
    d[0] = 1;   // schema
    d[1] = 1;   // device type
    d[4] = 1;   // one channel
    d[5] = 28;  // fatter record
    memcpy(&d[8], &kGoldenDescriptor[8], 24);  // EGT record + 4 junk bytes
    d[8 + 24] = 0xEE;  // the appended bytes a v1 parser must ignore
    PodDescriptor pd;
    REQUIRE(parseDescriptor(d, sizeof(d), pd));
    CHECK(pd.channelCount == 1);
    CHECK(pd.recordLen == 28);
    CHECK(strcmp(pd.ch[0].name, "EGT") == 0);
    CHECK(pd.ch[0].periodMs == 250);
}

TEST_CASE("sensoregg_gatt - golden sample frame parses field-for-field") {
    SampleFrame f;
    REQUIRE(parseSampleFrame(kGoldenFrame, sizeof(kGoldenFrame), f));
    CHECK(f.channelId == 0);
    CHECK(f.bootId == 0xA5);
    CHECK(f.seq == 0x2A);
    CHECK(f.baseMs == 0xDEADBEEFUL);
    CHECK(f.intervalMs == 250);
    CHECK(f.n == 3);
    CHECK(f.raw[0] == 1234);
    CHECK(f.raw[1] == -40);
    CHECK(f.raw[2] == INT16_MIN);
}

TEST_CASE("sensoregg_gatt - sample frame rejects length mismatches") {
    SampleFrame f;
    CHECK(!parseSampleFrame(nullptr, 16, f));
    CHECK(!parseSampleFrame(kGoldenFrame, 15, f));  // truncated sample
    CHECK(!parseSampleFrame(kGoldenFrame, 11, f));  // header + half a sample
    uint8_t bad[16];
    memcpy(bad, kGoldenFrame, sizeof(bad));
    bad[9] = 0;  // n = 0
    CHECK(!parseSampleFrame(bad, sizeof(bad), f));
    bad[9] = 2;  // n disagrees with len
    CHECK(!parseSampleFrame(bad, sizeof(bad), f));
}

TEST_CASE("sensoregg_gatt - clock parses and tolerates a longer value") {
    uint8_t bootId = 0;
    uint32_t podMs = 0;
    REQUIRE(parseClock(kGoldenClock, sizeof(kGoldenClock), bootId, podMs));
    CHECK(bootId == 0xA5);
    CHECK(podMs == 0x01234567UL);
    CHECK(!parseClock(kGoldenClock, 5, bootId, podMs));
    uint8_t longer[8] = {0};
    memcpy(longer, kGoldenClock, 6);  // future revision appends bytes
    REQUIRE(parseClock(longer, sizeof(longer), bootId, podMs));
    CHECK(bootId == 0xA5);
}

TEST_CASE("sensoregg_gatt - sampleToReal applies scale after the sentinel") {
    ChannelInfo c;
    c.scale = 0.1f;
    c.offset = 0.0f;
    CHECK(sampleToReal(1234, c) == doctest::Approx(123.4f));
    CHECK(sampleToReal(-40, c) == doctest::Approx(-4.0f));
    CHECK(std::isnan(sampleToReal(INT16_MIN, c)));  // host code: isnan OK
    c.scale = 1.0f;
    c.offset = 10.0f;
    CHECK(sampleToReal(87, c) == doctest::Approx(97.0f));
}

TEST_CASE("sensoregg_gatt - role mapping is descriptor-driven") {
    PodDescriptor pd;
    REQUIRE(parseDescriptor(kGoldenDescriptor, sizeof(kGoldenDescriptor), pd));
    int8_t idx[ROLE_COUNT];
    mapChannels(pd, idx);
    CHECK(idx[ROLE_EGT] == 0);
    CHECK(idx[ROLE_CJ] == 1);
    CHECK(idx[ROLE_AUX] == 2);
    CHECK(idx[ROLE_BATT] == 3);
    CHECK(fastestChannel(pd) == 0);  // 250 ms EGT is the heartbeat

    // A pod with no battery and no IAT still maps what it has.
    PodDescriptor small = pd;
    small.channelCount = 2;
    mapChannels(small, idx);
    CHECK(idx[ROLE_EGT] == 0);
    CHECK(idx[ROLE_CJ] == 1);
    CHECK(idx[ROLE_AUX] == -1);
    CHECK(idx[ROLE_BATT] == -1);
}

TEST_CASE("sensoregg_gatt - clock fit anchors on the round-trip midpoint") {
    ClockFit f;
    CHECK(!f.valid);
    // Request at logger 10000, response at 10060, pod said 555000.
    clockFitAnchor(f, 0xA5, 555000UL, 10000UL, 10060UL);
    CHECK(f.valid);
    CHECK(f.loggerMs0 == 10030UL);
    CHECK(f.halfRttMs == 30UL);
    CHECK(clockFitSameEpoch(f, 0xA5));
    CHECK(!clockFitSameEpoch(f, 0xA6));  // pod rebooted
    // Forward and backward mapping around the anchor.
    CHECK(clockFitPodToLogger(f, 555000UL) == 10030UL);
    CHECK(clockFitPodToLogger(f, 556000UL) == 11030UL);
    CHECK(clockFitPodToLogger(f, 554000UL) == 9030UL);
}

TEST_CASE("sensoregg_gatt - clock fit survives the u32 millis wrap") {
    ClockFit f;
    // Pod anchored just before its millis wrap; a frame lands just after.
    clockFitAnchor(f, 1, 0xFFFFFF00UL, 500000UL, 500020UL);
    const uint32_t mapped = clockFitPodToLogger(f, 0x00000100UL);
    // Pod advanced 0x200 = 512 ms across the wrap.
    CHECK(mapped == 500010UL + 512UL);
    // And a frame slightly BEFORE the anchor maps backward, not 4 Gms off.
    CHECK(clockFitPodToLogger(f, 0xFFFFFE00UL) == 500010UL - 256UL);
}
