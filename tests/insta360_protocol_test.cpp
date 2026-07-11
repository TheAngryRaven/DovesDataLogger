#include "doctest.h"

#include <cstring>

#include "insta360_protocol.h"

using namespace insta360_protocol;

// The byte tables here pin the wire format to the X4-verified reference
// implementations. If a builder changes shape, a golden vector below
// must change with it — deliberately.

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

    // Byte-for-byte the ADV_IND captured from a genuine GPS Remote
    // (X4-confirmed 2026-07-10: replaying it from a phone woke the X4).
    // This golden pins the exact on-air layout.
    const uint8_t expected[kWakeAdvertLen] = {
        0x02, 0x01, 0x05,                    // Flags AD (0x05, as captured)
        0x1B, 0xFF,                          // mfg AD header (len 27)
        0x4C, 0x00,                          // Apple company ID (LE)
        0x02, 0x15,                          // iBeacon-style subtype + len
        0x09,                                // mfg[4]
        0x4F, 0x52, 0x42, 0x49, 0x54,        // mfg[5..9] "ORBIT"
        0x09, 0xFF, 0x0F, 0x00,              // mfg[10..13]
        0x30, 0x31, 0x32, 0x33, 0x34, 0x35,  // mfg[14..19] serial "012345"
        0x00, 0x00, 0x00, 0x00,              // mfg[20..23] all-zero
        0xE4, 0x01,                          // mfg[24..25] tail
    };
    CHECK(std::memcmp(advert, expected, kWakeAdvertLen) == 0);
}

TEST_CASE("insta360_protocol - wake advert serial lands at offsets 19-24") {
    const uint8_t serial[kSerialLen] = {'X', '9', 'Z', '8', 'W', '7'};
    uint8_t advert[kWakeAdvertLen];
    CHECK(buildWakeAdvert(advert, serial) == kWakeAdvertLen);
    // mfg[14..19] = absolute offsets 19-24 of the full PDU.
    CHECK(std::memcmp(advert + 19, serial, kSerialLen) == 0);

    // The Flags AD comes first regardless of serial.
    CHECK(advert[0] == 0x02);
    CHECK(advert[1] == 0x01);
    CHECK(advert[2] == 0x05);
}

TEST_CASE("insta360_protocol - remote scan response golden bytes") {
    uint8_t rsp[kRemoteScanRspLen];
    CHECK(buildRemoteScanResponse(rsp) == kRemoteScanRspLen);
    // Captured from the genuine remote: Appearance 0x0180 then
    // Complete Local Name "Insta360 GPS Remote".
    const uint8_t expected[kRemoteScanRspLen] = {
        0x03, 0x19, 0x80, 0x01,
        0x14, 0x09,
        'I', 'n', 's', 't', 'a', '3', '6', '0', ' ',
        'G', 'P', 'S', ' ', 'R', 'e', 'm', 'o', 't', 'e',
    };
    CHECK(std::memcmp(rsp, expected, kRemoteScanRspLen) == 0);
}

// ---------------------------------------------------------------------------
// ce82 button frames
// ---------------------------------------------------------------------------

// Golden bytes are the exact frames captured from a genuine remote
// (2026-07-10 nRF Connect log): shutter seq 0x00, mode seq 0x02,
// power-tap seq 0x04, power-hold stream seq 0x06+.
TEST_CASE("insta360_protocol - shutter toggle golden bytes") {
    uint8_t frame[9];
    CHECK(buildShutterToggle(frame, sizeof(frame), 0x00) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x00,
                                 0x03, 0x01, 0x02, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildShutterToggle(frame, 8, 0x00) == 0);
}

TEST_CASE("insta360_protocol - mode cycle golden bytes") {
    uint8_t frame[9];
    CHECK(buildModeCycle(frame, sizeof(frame), 0x02) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x02,
                                 0x03, 0x01, 0x01, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildModeCycle(frame, 8, 0x02) == 0);
}

TEST_CASE("insta360_protocol - screen toggle (power tap) golden bytes") {
    uint8_t frame[9];
    CHECK(buildScreenToggle(frame, sizeof(frame), 0x04) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x04,
                                 0x03, 0x01, 0x00, 0x00};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildScreenToggle(frame, 8, 0x04) == 0);
}

TEST_CASE("insta360_protocol - power off (hold) golden bytes") {
    uint8_t frame[9];
    // First hold frame in the captured stream: seq 0x06.
    CHECK(buildPowerOff(frame, sizeof(frame), 0x06) == 9);
    const uint8_t expected[9] = {0xFC, 0xEF, 0xFE, 0x86, 0x06,
                                 0x03, 0x01, 0x00, 0x03};
    CHECK(std::memcmp(frame, expected, 9) == 0);
    CHECK(buildPowerOff(frame, 8, 0x06) == 0);
}

TEST_CASE("insta360_protocol - button seq lands at byte 4 and only there") {
    uint8_t a[9], b[9];
    CHECK(buildPowerOff(a, sizeof(a), 0x06) == 9);
    CHECK(buildPowerOff(b, sizeof(b), 0x08) == 9);  // next hold frame
    CHECK(a[4] == 0x06);
    CHECK(b[4] == 0x08);
    // Everything but byte 4 is identical between consecutive hold frames.
    for (int i = 0; i < 9; i++) {
        if (i == 4) continue;
        CHECK(a[i] == b[i]);
    }
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
