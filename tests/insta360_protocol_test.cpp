#include "doctest.h"

#include <cstring>

#include "insta360_protocol.h"

using namespace insta360_protocol;

// The byte tables here pin the wire format to the X4-verified reference
// implementations. If a builder changes shape, a golden vector below
// must change with it — deliberately.

namespace {

// Copy a double's IEEE-754 binary64 little-endian bytes into `out`.
// The goldens compute expected double bytes this way rather than
// hand-transcribing hex.
void doubleBytes(double v, uint8_t out[8]) { std::memcpy(out, &v, 8); }

}  // namespace

// ---------------------------------------------------------------------------
// Serial parse / format
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - serial round-trip") {
    uint8_t serial[kSerialLen];
    REQUIRE(parseSerial("A1B2C3", serial));

    char str[kSerialLen + 1];
    serialToString(serial, str);
    CHECK(std::strcmp(str, "A1B2C3") == 0);
}

TEST_CASE("insta360_protocol - serial lowercase input is uppercased") {
    uint8_t serial[kSerialLen];
    REQUIRE(parseSerial("a1b2c3", serial));

    char str[kSerialLen + 1];
    serialToString(serial, str);
    CHECK(std::strcmp(str, "A1B2C3") == 0);
}

TEST_CASE("insta360_protocol - serial rejects wrong length") {
    uint8_t serial[kSerialLen];
    CHECK_FALSE(parseSerial("A1B2C", serial));    // too short
    CHECK_FALSE(parseSerial("A1B2C34", serial));  // too long
    CHECK_FALSE(parseSerial("", serial));         // empty
    CHECK_FALSE(parseSerial(nullptr, serial));    // null
}

TEST_CASE("insta360_protocol - serial rejects non-alphanumeric") {
    uint8_t serial[kSerialLen];
    CHECK_FALSE(parseSerial("A1B2C!", serial));
    CHECK_FALSE(parseSerial("A1 2C3", serial));
    CHECK_FALSE(parseSerial("A1B-C3", serial));
    CHECK_FALSE(parseSerial("A1B2C\t", serial));
}

TEST_CASE("insta360_protocol - serialToString NUL-terminates") {
    const uint8_t serial[kSerialLen] = {'0', '1', '2', '3', '4', '5'};
    char str[kSerialLen + 1];
    std::memset(str, 0x7F, sizeof(str));
    serialToString(serial, str);
    CHECK(str[kSerialLen] == '\0');
    CHECK(std::strcmp(str, "012345") == 0);
}

// ---------------------------------------------------------------------------
// Wake advertisement
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - wake advert golden vector for serial 012345") {
    const uint8_t serial[kSerialLen] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
    uint8_t advert[kWakeAdvertLen];
    CHECK(buildWakeAdvert(advert, serial) == kWakeAdvertLen);

    const uint8_t expected[kWakeAdvertLen] = {
        0x02, 0x01, 0x1A,                    // Flags AD
        0x1B, 0xFF,                          // mfg AD header
        0x4C, 0x00,                          // Apple company ID (LE)
        0x02, 0x15,                          // iBeacon type + len
        0x09, 0x4F, 0x52, 0x42, 0x49, 0x54,  // 0x09 "ORBIT"
        0x09, 0xFF, 0x0F, 0x00,
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35,  // serial "012345"
        0x00, 0x00, 0x00, 0x00,
        0xE4, 0x01,
    };
    CHECK(std::memcmp(advert, expected, kWakeAdvertLen) == 0);
}

TEST_CASE("insta360_protocol - wake advert serial lands at offsets 19-24") {
    const uint8_t serial[kSerialLen] = {'X', '9', 'Z', '8', 'W', '7'};
    uint8_t advert[kWakeAdvertLen];
    CHECK(buildWakeAdvert(advert, serial) == kWakeAdvertLen);
    CHECK(std::memcmp(advert + 19, serial, kSerialLen) == 0);

    // The Flags AD comes first regardless of serial.
    CHECK(advert[0] == 0x02);
    CHECK(advert[1] == 0x01);
    CHECK(advert[2] == 0x1A);
}

// ---------------------------------------------------------------------------
// ce82 button frames
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - shutter toggle golden bytes") {
    uint8_t frame[9];
    CHECK(buildShutterToggle(frame, sizeof(frame)) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00,
                                 0x03, 0x01, 0x02, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildShutterToggle(frame, 8) == 0);
}

TEST_CASE("insta360_protocol - mode cycle golden bytes") {
    uint8_t frame[9];
    CHECK(buildModeCycle(frame, sizeof(frame)) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00,
                                 0x03, 0x01, 0x01, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildModeCycle(frame, 8) == 0);
}

TEST_CASE("insta360_protocol - screen toggle golden bytes") {
    uint8_t frame[9];
    CHECK(buildScreenToggle(frame, sizeof(frame)) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00,
                                 0x03, 0x01, 0x00, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildScreenToggle(frame, 8) == 0);
}

TEST_CASE("insta360_protocol - power off golden bytes") {
    uint8_t frame[9];
    CHECK(buildPowerOff(frame, sizeof(frame)) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00,
                                 0x03, 0x01, 0x00, 0x03};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildPowerOff(frame, 8) == 0);
}

// ---------------------------------------------------------------------------
// be81 control frames
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - start video golden vector, counter 0x0200") {
    uint8_t frame[18];
    CHECK(buildStartVideo(frame, sizeof(frame), 0x0200) == 18);
    const uint8_t expected[18] = {0x12, 0x00, 0x00, 0x00, 0x04, 0x00,
                                  0x00, 0x04, 0x00, 0x02, 0x00, 0x02,
                                  0x00, 0x80, 0x00, 0x00, 0x08, 0x01};
    CHECK(std::memcmp(frame, expected, 18) == 0);
    CHECK(buildStartVideo(frame, 17, 0x0200) == 0);
}

TEST_CASE("insta360_protocol - stop video golden vector, counter 0x0200") {
    uint8_t frame[18];
    CHECK(buildStopVideo(frame, sizeof(frame), 0x0200) == 18);
    const uint8_t expected[18] = {0x12, 0x00, 0x00, 0x00, 0x04, 0x00,
                                  0x00, 0x05, 0x00, 0x02, 0x00, 0x02,
                                  0x00, 0x80, 0x00, 0x00, 0x10, 0x01};
    CHECK(std::memcmp(frame, expected, 18) == 0);
    CHECK(buildStopVideo(frame, 17, 0x0200) == 0);
}

TEST_CASE("insta360_protocol - counter changes only bytes 10-11") {
    uint8_t a[18];
    uint8_t b[18];
    REQUIRE(buildStartVideo(a, sizeof(a), 0x0200) == 18);
    REQUIRE(buildStartVideo(b, sizeof(b), 0x0201) == 18);

    for (size_t i = 0; i < 18; ++i) {
        if (i == 10 || i == 11) continue;
        CHECK(a[i] == b[i]);
    }
    // Counter is little-endian u16.
    CHECK(b[10] == 0x01);
    CHECK(b[11] == 0x02);
    CHECK((a[10] != b[10] || a[11] != b[11]));
}

TEST_CASE("insta360_protocol - be81 length byte matches returned size") {
    uint8_t frame[71];

    CHECK(buildStartVideo(frame, sizeof(frame), kInitialMsgCounter) == 18);
    CHECK(frame[0] == 18);

    CHECK(buildStopVideo(frame, sizeof(frame), kInitialMsgCounter) == 18);
    CHECK(frame[0] == 18);

    CHECK(buildKeepAlive(frame, sizeof(frame)) == 7);
    CHECK(frame[0] == 7);

    const GpsSample s;
    CHECK(buildGpsFrame(frame, sizeof(frame), kInitialMsgCounter, s) == 71);
    CHECK(frame[0] == 71);
}

TEST_CASE("insta360_protocol - keep-alive golden bytes") {
    uint8_t frame[7];
    CHECK(buildKeepAlive(frame, sizeof(frame)) == 7);
    const uint8_t expected[7] = {0x07, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00};
    CHECK(std::memcmp(frame, expected, 7) == 0);
    CHECK(buildKeepAlive(frame, 6) == 0);
}

// ---------------------------------------------------------------------------
// GPS frame
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - GPS frame golden vector") {
    GpsSample s;
    s.unixTimeSec = 1710512400;
    s.latitudeDeg = 28.4127081705638;
    s.longitudeDeg = -81.4622;
    s.speedMps = 17.88;
    s.headingDeg = 123.4;
    s.altitudeM = 25.0;

    uint8_t frame[71];
    CHECK(buildGpsFrame(frame, sizeof(frame), 0x0200, s) == 71);

    uint8_t expected[71];
    std::memset(expected, 0x00, sizeof(expected));
    const uint8_t header[18] = {0x47, 0x00, 0x00, 0x00, 0x04, 0x00,
                                0x00, 0x35, 0x00, 0x02, 0x00, 0x02,
                                0x00, 0x80, 0x00, 0x00, 0x0A, 0x35};
    std::memcpy(expected, header, sizeof(header));
    // u32 LE unix epoch seconds.
    expected[18] = static_cast<uint8_t>(s.unixTimeSec & 0xFFu);
    expected[19] = static_cast<uint8_t>((s.unixTimeSec >> 8) & 0xFFu);
    expected[20] = static_cast<uint8_t>((s.unixTimeSec >> 16) & 0xFFu);
    expected[21] = static_cast<uint8_t>((s.unixTimeSec >> 24) & 0xFFu);
    // Bytes 22-27 stay zero.
    expected[28] = 0x41;
    doubleBytes(28.4127081705638, expected + 29);  // abs(lat)
    expected[37] = 0x4E;                           // 'N'
    doubleBytes(81.4622, expected + 38);           // abs(lon)
    expected[46] = 0x57;                           // 'W'
    doubleBytes(17.88, expected + 47);
    doubleBytes(123.4, expected + 55);
    doubleBytes(25.0, expected + 63);

    CHECK(std::memcmp(frame, expected, 71) == 0);
    CHECK(frame[0] == 71);  // length field = total frame length
}

TEST_CASE("insta360_protocol - GPS frame hemisphere quadrants") {
    struct Quadrant {
        double lat;
        double lon;
        uint8_t latChar;
        uint8_t lonChar;
    };
    const Quadrant quadrants[] = {
        {28.4127, -81.4622, 'N', 'W'},   // Orlando
        {-37.8497, 144.9680, 'S', 'E'},  // Melbourne
        {51.0577, -1.0170, 'N', 'W'},    // Thruxton
        {-33.0472, -71.6127, 'S', 'W'},  // Valparaiso
    };

    for (const Quadrant& q : quadrants) {
        GpsSample s;
        s.unixTimeSec = 1710512400;
        s.latitudeDeg = q.lat;
        s.longitudeDeg = q.lon;

        uint8_t frame[71];
        REQUIRE(buildGpsFrame(frame, sizeof(frame), 0x0200, s) == 71);
        CHECK(frame[37] == q.latChar);
        CHECK(frame[46] == q.lonChar);

        // Stored doubles are absolute values.
        uint8_t absLat[8];
        uint8_t absLon[8];
        doubleBytes(q.lat < 0.0 ? -q.lat : q.lat, absLat);
        doubleBytes(q.lon < 0.0 ? -q.lon : q.lon, absLon);
        CHECK(std::memcmp(frame + 29, absLat, 8) == 0);
        CHECK(std::memcmp(frame + 38, absLon, 8) == 0);
    }

    // Positive-lon quadrant explicit 'E' check (both positive).
    GpsSample s;
    s.latitudeDeg = 35.3606;
    s.longitudeDeg = 138.7274;  // Fuji
    uint8_t frame[71];
    REQUIRE(buildGpsFrame(frame, sizeof(frame), 0x0200, s) == 71);
    CHECK(frame[37] == 'N');
    CHECK(frame[46] == 'E');
}

TEST_CASE("insta360_protocol - GPS frame cap 70 returns 0") {
    const GpsSample s;
    uint8_t frame[71];
    std::memset(frame, 0xAA, sizeof(frame));
    CHECK(buildGpsFrame(frame, 70, 0x0200, s) == 0);
    // Nothing written.
    for (uint8_t byte : frame) CHECK(byte == 0xAA);
}

// ---------------------------------------------------------------------------
// parseCe81Frame
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - ce81 serial frame extracts the serial") {
    // Realistic serial frame: FE EF FE 07 00 00 + serial as the last 6
    // bytes of a 12-byte frame.
    const uint8_t frame[12] = {0xFE, 0xEF, 0xFE, 0x07, 0x00, 0x00,
                               'A',  '1',  'B',  '2',  'C',  '3'};
    uint8_t serial[kSerialLen];
    std::memset(serial, 0, sizeof(serial));
    CHECK(parseCe81Frame(frame, sizeof(frame), serial) == Ce81Frame::kSerial);
    CHECK(std::memcmp(serial, "A1B2C3", kSerialLen) == 0);
}

TEST_CASE("insta360_protocol - ce81 serial frame takes the LAST 6 bytes") {
    // Longer frame: the serial still comes from the tail.
    const uint8_t frame[14] = {0xFE, 0xEF, 0xFE, 0x07, 0x00, 0x00, 0x01,
                               0x02, 'X',  '9',  'Z',  '8',  'W',  '7'};
    uint8_t serial[kSerialLen];
    CHECK(parseCe81Frame(frame, sizeof(frame), serial) == Ce81Frame::kSerial);
    CHECK(std::memcmp(serial, "X9Z8W7", kSerialLen) == 0);
}

TEST_CASE("insta360_protocol - ce81 null serialOut does not crash") {
    const uint8_t frame[12] = {0xFE, 0xEF, 0xFE, 0x07, 0x00, 0x00,
                               'A',  '1',  'B',  '2',  'C',  '3'};
    CHECK(parseCe81Frame(frame, sizeof(frame), nullptr) ==
          Ce81Frame::kSerial);
}

TEST_CASE("insta360_protocol - ce81 status frames") {
    const uint8_t status[8] = {0xFE, 0xEF, 0xFE, 0x10, 0x81,
                               0x00, 0x00, 0x00};
    CHECK(parseCe81Frame(status, sizeof(status), nullptr) ==
          Ce81Frame::kStatus);

    // FE EF FE prefix with a non-serial type byte, minimum length.
    const uint8_t other[5] = {0xFE, 0xEF, 0xFE, 0x01, 0x00};
    CHECK(parseCe81Frame(other, sizeof(other), nullptr) ==
          Ce81Frame::kStatus);
}

TEST_CASE("insta360_protocol - ce81 short or garbage frames are unknown") {
    const uint8_t shortFrame[4] = {0xFE, 0xEF, 0xFE, 0x07};
    CHECK(parseCe81Frame(shortFrame, sizeof(shortFrame), nullptr) ==
          Ce81Frame::kUnknown);

    const uint8_t garbage[12] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                                 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C};
    CHECK(parseCe81Frame(garbage, sizeof(garbage), nullptr) ==
          Ce81Frame::kUnknown);

    CHECK(parseCe81Frame(nullptr, 12, nullptr) == Ce81Frame::kUnknown);
    CHECK(parseCe81Frame(garbage, 0, nullptr) == Ce81Frame::kUnknown);
}

// ---------------------------------------------------------------------------
// parseBe82RecordState
// ---------------------------------------------------------------------------

namespace {

// Build a minimal well-formed be82 record-state frame.
void makeBe82Frame(uint8_t out[18], uint8_t code, uint8_t state) {
    std::memset(out, 0x00, 18);
    out[0] = 0x12;  // length-ish leader; parser ignores bytes 0-1
    out[2] = 0x00;
    out[3] = 0x00;
    out[4] = 0x04;
    out[5] = 0x00;
    out[6] = 0x00;
    out[7] = code;
    out[17] = state;
}

}  // namespace

TEST_CASE("insta360_protocol - be82 recording frame returns 1") {
    uint8_t frame[18];
    makeBe82Frame(frame, 0x10, 0x01);
    CHECK(parseBe82RecordState(frame, sizeof(frame)) == 1);

    // Any non-zero state byte counts as recording.
    makeBe82Frame(frame, 0x10, 0x03);
    CHECK(parseBe82RecordState(frame, sizeof(frame)) == 1);
}

TEST_CASE("insta360_protocol - be82 stopped frame returns 0") {
    uint8_t frame[18];
    makeBe82Frame(frame, 0x10, 0x00);
    CHECK(parseBe82RecordState(frame, sizeof(frame)) == 0);
}

TEST_CASE("insta360_protocol - be82 wrong code returns -1") {
    uint8_t frame[18];
    makeBe82Frame(frame, 0x11, 0x01);
    CHECK(parseBe82RecordState(frame, sizeof(frame)) == -1);
}

TEST_CASE("insta360_protocol - be82 short frame returns -1") {
    uint8_t frame[18];
    makeBe82Frame(frame, 0x10, 0x01);
    CHECK(parseBe82RecordState(frame, 17) == -1);
    CHECK(parseBe82RecordState(frame, 0) == -1);
    CHECK(parseBe82RecordState(nullptr, 18) == -1);
}

TEST_CASE("insta360_protocol - be82 wrong signature returns -1") {
    uint8_t frame[18];
    makeBe82Frame(frame, 0x10, 0x01);
    frame[4] = 0x05;  // corrupt the 00 00 04 00 00 signature
    CHECK(parseBe82RecordState(frame, sizeof(frame)) == -1);
}

// ---------------------------------------------------------------------------
// Chunker
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - chunker empty input yields nothing") {
    Chunker c;
    uint8_t out[kChunkSize];
    CHECK(nextChunk(c, out) == 0);

    uint8_t buf[4] = {1, 2, 3, 4};
    Chunker zero{buf, 0, 0};
    CHECK(nextChunk(zero, out) == 0);
}

TEST_CASE("insta360_protocol - chunker exact one chunk") {
    uint8_t buf[kChunkSize];
    for (size_t i = 0; i < kChunkSize; ++i) buf[i] = static_cast<uint8_t>(i);

    Chunker c{buf, sizeof(buf), 0};
    uint8_t out[kChunkSize];
    CHECK(nextChunk(c, out) == kChunkSize);
    CHECK(std::memcmp(out, buf, kChunkSize) == 0);
    CHECK(nextChunk(c, out) == 0);
}

TEST_CASE("insta360_protocol - chunker 21 bytes splits 20 + 1") {
    uint8_t buf[21];
    for (size_t i = 0; i < sizeof(buf); ++i) buf[i] = static_cast<uint8_t>(i);

    Chunker c{buf, sizeof(buf), 0};
    uint8_t out[kChunkSize];
    CHECK(nextChunk(c, out) == 20);
    CHECK(std::memcmp(out, buf, 20) == 0);
    CHECK(nextChunk(c, out) == 1);
    CHECK(out[0] == buf[20]);
    CHECK(nextChunk(c, out) == 0);
}

TEST_CASE("insta360_protocol - chunker 71-byte GPS frame splits 20/20/20/11") {
    uint8_t buf[71];
    for (size_t i = 0; i < sizeof(buf); ++i) {
        buf[i] = static_cast<uint8_t>(i * 3);
    }

    Chunker c{buf, sizeof(buf), 0};
    uint8_t out[kChunkSize];
    const size_t expectedSizes[4] = {20, 20, 20, 11};
    size_t offset = 0;
    for (size_t want : expectedSizes) {
        CHECK(nextChunk(c, out) == want);
        CHECK(std::memcmp(out, buf + offset, want) == 0);
        offset += want;
    }
    CHECK(offset == sizeof(buf));

    // Once done, it keeps returning 0.
    CHECK(nextChunk(c, out) == 0);
    CHECK(nextChunk(c, out) == 0);
}
