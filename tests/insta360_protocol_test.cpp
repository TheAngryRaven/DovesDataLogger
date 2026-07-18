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
// buildGpsRmcFrame — golden vector from the 2026-07-10 Wireshark capture
// ---------------------------------------------------------------------------

TEST_CASE("insta360_protocol - GPS RMC frame matches the captured bytes") {
    // The exact ce82 notification captured from the genuine remote:
    //   ,26.7,\x07,$GNRMC,033113.000,A,2845.0938,N,-8156.0570,E,
    //   0.06,0.00,110726,0.0,W,A,V*69
    const uint8_t expected[] = {
        0xFC, 0xEF, 0xFE, 0x83, 0x00, 0x52,
        0x2C, 0x32, 0x36, 0x2E, 0x37, 0x2C, 0x07, 0x2C, 0x24, 0x47,
        0x4E, 0x52, 0x4D, 0x43, 0x2C, 0x30, 0x33, 0x33, 0x31, 0x31,
        0x33, 0x2E, 0x30, 0x30, 0x30, 0x2C, 0x41, 0x2C, 0x32, 0x38,
        0x34, 0x35, 0x2E, 0x30, 0x39, 0x33, 0x38, 0x2C, 0x4E, 0x2C,
        0x2D, 0x38, 0x31, 0x35, 0x36, 0x2E, 0x30, 0x35, 0x37, 0x30,
        0x2C, 0x45, 0x2C, 0x30, 0x2E, 0x30, 0x36, 0x2C, 0x30, 0x2E,
        0x30, 0x30, 0x2C, 0x31, 0x31, 0x30, 0x37, 0x32, 0x36, 0x2C,
        0x30, 0x2E, 0x30, 0x2C, 0x57, 0x2C, 0x41, 0x2C, 0x56, 0x2A,
        0x36, 0x39,
    };

    GpsRmc s;
    s.valid = true;
    s.hour = 3; s.minute = 31; s.second = 13; s.milli = 0;
    s.day = 11; s.month = 7; s.year = 26;
    // 2845.0938 N = 28 deg 45.0938 min; -8156.0570 E = 81 deg 56.0570 min W.
    s.latitudeDeg = 28.0 + 45.0938 / 60.0;
    s.longitudeDeg = -(81.0 + 56.0570 / 60.0);
    s.speedKnots = 0.06;
    s.courseDeg = 0.00;

    uint8_t out[kMaxGpsFrameLen];
    const size_t n = buildGpsRmcFrame(out, sizeof(out), s);
    CHECK(n == sizeof(expected));
    CHECK(std::memcmp(out, expected, sizeof(expected)) == 0);

    // Length byte (offset 5) equals the ASCII payload length.
    CHECK(out[5] == n - 6);
    // SN slot stays 0 for GPS (buttons own the counter).
    CHECK(out[4] == 0x00);
    // Too-small buffer writes nothing.
    CHECK(buildGpsRmcFrame(out, 10, s) == 0);
}

TEST_CASE("insta360_protocol - GPS RMC void status and hemispheres") {
    GpsRmc s;
    s.valid = false;             // no fix -> 'V'
    s.latitudeDeg = -12.5;       // south
    s.longitudeDeg = 7.25;       // east (positive)
    uint8_t out[kMaxGpsFrameLen];
    const size_t n = buildGpsRmcFrame(out, sizeof(out), s);
    REQUIRE(n > 6);
    // Copy the ASCII payload (after the 6-byte header, which contains a NUL
    // at the SN slot) into a NUL-terminated buffer for substring checks.
    char payload[kMaxGpsFrameLen];
    const size_t plen = out[5];
    std::memcpy(payload, out + 6, plen);
    payload[plen] = '\0';
    // Status field 'V' present; south latitude -> 'S'; longitude letter is
    // ALWAYS 'E' even for a positive (eastern) value.
    CHECK(std::strstr(payload, ",V,") != nullptr);
    CHECK(std::strstr(payload, ",S,") != nullptr);
    CHECK(std::strstr(payload, ",E,") != nullptr);
    // The extra 'V' + checksum tail is present.
    CHECK(std::strstr(payload, ",A,V*") != nullptr);
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

// -------------------------------------------------------------------------
// parseRecordingState — the 0x10 display-string record signal (spec §6.1)
// -------------------------------------------------------------------------

TEST_CASE("insta360_protocol - 0x10 recording timer is detected") {
    // Full §6.1 frame: FE EF FE 10 80 0D 01 0E 46 01 <".00:00:05"> — payLen
    // 0x0D = 13 (4 control + 9 ASCII), so the complete frame is 19 bytes.
    const uint8_t recFull[19] = {0xFE, 0xEF, 0xFE, 0x10, 0x80, 0x0D,
                                 0x01, 0x0E, 0x46, 0x01,
                                 0x2E, 0x30, 0x30, 0x3A, 0x30, 0x30, 0x3A,
                                 0x30, 0x35};  // . 0 0 : 0 0 : 0 5
    CHECK(parseRecordingState(recFull, sizeof(recFull)) ==
          RecordObs::kRecording);

    // A read shorter than the header-claimed payload must never over-report:
    // the same frame truncated to 16 bytes (< 6 + payLen 13) is kUnknown.
    CHECK(parseRecordingState(recFull, 16) == RecordObs::kUnknown);
}

TEST_CASE("insta360_protocol - 0x10 idle mode/battery strings are not recording") {
    // "4K|30|UW" — idle mode string.
    const uint8_t mode[18] = {0xFE, 0xEF, 0xFE, 0x10, 0x81, 0x0C,
                              0x01, 0x1C, 0x5E, 0x00,
                              0x34, 0x4B, 0x7C, 0x33, 0x30, 0x7C, 0x55, 0x57};
    CHECK(parseRecordingState(mode, sizeof(mode)) == RecordObs::kIdle);

    // " 13h09m" — idle battery/runtime string.
    const uint8_t batt[17] = {0xFE, 0xEF, 0xFE, 0x10, 0x80, 0x0B,
                              0x01, 0x12, 0x46, 0x01,
                              0x20, 0x31, 0x33, 0x68, 0x30, 0x39, 0x6D};
    CHECK(parseRecordingState(batt, sizeof(batt)) == RecordObs::kIdle);
}

TEST_CASE("insta360_protocol - non-0x10 / malformed frames are unknown") {
    // A serial handshake frame is not a display-string frame.
    const uint8_t serial[12] = {0xFE, 0xEF, 0xFE, 0x07, 0x00, 0x06,
                                '3', '4', 'U', 'Q', 'G', '5'};
    CHECK(parseRecordingState(serial, sizeof(serial)) == RecordObs::kUnknown);

    // Wrong magic.
    const uint8_t badMagic[8] = {0xFC, 0xEF, 0xFE, 0x10, 0x80,
                                 0x02, 0x00, 0x00};
    CHECK(parseRecordingState(badMagic, sizeof(badMagic)) ==
          RecordObs::kUnknown);

    // 0x10 header claiming a longer payload than provided -> truncated.
    const uint8_t truncated[8] = {0xFE, 0xEF, 0xFE, 0x10, 0x80, 0x40,
                                  0x01, 0x0E};
    CHECK(parseRecordingState(truncated, sizeof(truncated)) ==
          RecordObs::kUnknown);

    CHECK(parseRecordingState(nullptr, 12) == RecordObs::kUnknown);
    CHECK(parseRecordingState(serial, 0) == RecordObs::kUnknown);
}
