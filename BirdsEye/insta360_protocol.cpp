#include "insta360_protocol.h"

#include <math.h>
#include <string.h>

// The GPS frame stores doubles as raw IEEE-754 binary64 little-endian
// bytes. Both the nRF52840 (Cortex-M4F) and the CI test hosts are
// little-endian machines with 8-byte IEEE-754 doubles, so a memcpy of
// the in-memory representation IS the wire format.
static_assert(sizeof(double) == 8,
              "insta360_protocol requires 8-byte IEEE-754 doubles");

namespace insta360_protocol {

namespace {

// ---------------------------------------------------------------------
// ce82 button frames: FC EF FE 86 00 03 01 <button> <press>
// ---------------------------------------------------------------------

constexpr size_t kButtonFrameLen = 9;

// X4-VERIFY(sniff): ce82 button-frame preamble.
constexpr uint8_t kButtonPreamble[7] = {0xFC, 0xEF, 0xFE, 0x86,
                                        0x00, 0x03, 0x01};

size_t buildButtonFrame(uint8_t* out, size_t cap, uint8_t button,
                        uint8_t press) {
  if (out == nullptr || cap < kButtonFrameLen) return 0;
  memcpy(out, kButtonPreamble, sizeof(kButtonPreamble));
  out[7] = button;
  out[8] = press;
  return kButtonFrameLen;
}

// ---------------------------------------------------------------------
// be81 control frames. 16-byte header:
//   bytes 0-3   total frame length, u32 LE (includes the length field)
//   bytes 4-6   04 00 00
//   byte  7     command
//   bytes 8-9   00 02
//   bytes 10-11 message counter, u16 LE
//   bytes 12-15 00 80 00 00
// Payload follows from byte 16.
// ---------------------------------------------------------------------

constexpr size_t kBe81HeaderLen = 16;
constexpr size_t kVideoFrameLen = 18;  // header + 2-byte payload
constexpr size_t kKeepAliveLen = 7;
constexpr size_t kGpsFrameLen = 71;

// X4-VERIFY(sniff): be81 command bytes.
constexpr uint8_t kCmdStartVideo = 0x04;
constexpr uint8_t kCmdStopVideo = 0x05;
constexpr uint8_t kCmdGps = 0x35;

void writeBe81Header(uint8_t* out, uint32_t totalLen, uint8_t command,
                     uint16_t counter) {
  out[0] = static_cast<uint8_t>(totalLen & 0xFFu);
  out[1] = static_cast<uint8_t>((totalLen >> 8) & 0xFFu);
  out[2] = static_cast<uint8_t>((totalLen >> 16) & 0xFFu);
  out[3] = static_cast<uint8_t>((totalLen >> 24) & 0xFFu);
  out[4] = 0x04;  // X4-VERIFY(sniff): fixed 04 00 00 at bytes 4-6
  out[5] = 0x00;
  out[6] = 0x00;
  out[7] = command;
  out[8] = 0x00;  // X4-VERIFY(sniff): fixed 00 02 at bytes 8-9
  out[9] = 0x02;
  out[10] = static_cast<uint8_t>(counter & 0xFFu);
  out[11] = static_cast<uint8_t>((counter >> 8) & 0xFFu);
  out[12] = 0x00;  // X4-VERIFY(sniff): fixed 00 80 00 00 at bytes 12-15
  out[13] = 0x80;
  out[14] = 0x00;
  out[15] = 0x00;
}

size_t buildVideoFrame(uint8_t* out, size_t cap, uint8_t command,
                       uint16_t counter, uint8_t payload0,
                       uint8_t payload1) {
  if (out == nullptr || cap < kVideoFrameLen) return 0;
  writeBe81Header(out, kVideoFrameLen, command, counter);
  out[16] = payload0;
  out[17] = payload1;
  return kVideoFrameLen;
}

void writeU32LE(uint8_t* out, uint32_t v) {
  out[0] = static_cast<uint8_t>(v & 0xFFu);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
  out[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
  out[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

void writeDoubleLE(uint8_t* out, double v) {
  memcpy(out, &v, sizeof(v));  // host is little-endian IEEE-754 (see
                               // static_assert above)
}

}  // namespace

// -----------------------------------------------------------------------
// Serial handling
// -----------------------------------------------------------------------

bool parseSerial(const char* s, uint8_t out[kSerialLen]) {
  if (s == nullptr) return false;
  uint8_t parsed[kSerialLen];
  for (size_t i = 0; i < kSerialLen; ++i) {
    char c = s[i];
    if (c == '\0') return false;  // too short
    if (c >= 'a' && c <= 'z') c = static_cast<char>(c - 'a' + 'A');
    const bool valid = (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
    if (!valid) return false;
    parsed[i] = static_cast<uint8_t>(c);
  }
  if (s[kSerialLen] != '\0') return false;  // too long
  memcpy(out, parsed, kSerialLen);
  return true;
}

void serialToString(const uint8_t serial[kSerialLen],
                    char out[kSerialLen + 1]) {
  memcpy(out, serial, kSerialLen);
  out[kSerialLen] = '\0';
}

// -----------------------------------------------------------------------
// Wake advertisement
// -----------------------------------------------------------------------

size_t buildWakeAdvert(uint8_t out[kWakeAdvertLen],
                       const uint8_t serial[kSerialLen]) {
  // X4-VERIFY(sniff): iBeacon-masquerade wake advertisement. The camera
  // wakes when it sees this manufacturer-specific payload carrying its
  // own serial at mfg[14..19]. The 6 serial bytes land at absolute
  // offsets 19-24; they are filled in below.
  static const uint8_t kTemplate[kWakeAdvertLen] = {
      0x02, 0x01, 0x1A,                    // AD1: Flags = 0x1A
      0x1B, 0xFF,                          // AD2: len 27, type 0xFF (mfg)
      0x4C, 0x00,                          // company ID 0x004C (Apple, LE)
      0x02, 0x15,                          // iBeacon type + len
      0x09, 0x4F, 0x52, 0x42, 0x49, 0x54,  // mfg[4..9] = 0x09 "ORBIT"
      0x09, 0xFF, 0x0F, 0x00,              // mfg[10..13]
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // mfg[14..19] = serial
      0x00, 0x00, 0x00, 0x00,              // mfg[20..23]
      0xE4, 0x01,                          // mfg[24..25]
  };
  constexpr size_t kSerialOffset = 19;  // absolute offset of mfg[14]

  memcpy(out, kTemplate, kWakeAdvertLen);
  memcpy(out + kSerialOffset, serial, kSerialLen);
  return kWakeAdvertLen;
}

// -----------------------------------------------------------------------
// ce82 button frames
// -----------------------------------------------------------------------

size_t buildShutterToggle(uint8_t* out, size_t cap) {
  // X4-VERIFY(sniff): button 0x02, press 0x00.
  return buildButtonFrame(out, cap, 0x02, 0x00);
}

size_t buildModeCycle(uint8_t* out, size_t cap) {
  // X4-VERIFY(sniff): button 0x01, press 0x00.
  return buildButtonFrame(out, cap, 0x01, 0x00);
}

size_t buildScreenToggle(uint8_t* out, size_t cap) {
  // X4-VERIFY(sniff): button 0x00, press 0x00.
  return buildButtonFrame(out, cap, 0x00, 0x00);
}

size_t buildPowerOff(uint8_t* out, size_t cap) {
  // X4-VERIFY(sniff): button 0x00, press 0x03 (3-second hold).
  return buildButtonFrame(out, cap, 0x00, 0x03);
}

// -----------------------------------------------------------------------
// be81 control frames
// -----------------------------------------------------------------------

size_t buildStartVideo(uint8_t* out, size_t cap, uint16_t counter) {
  // X4-VERIFY(sniff): start-video payload 08 01.
  return buildVideoFrame(out, cap, kCmdStartVideo, counter, 0x08, 0x01);
}

size_t buildStopVideo(uint8_t* out, size_t cap, uint16_t counter) {
  // X4-VERIFY(sniff): stop-video payload 10 01.
  return buildVideoFrame(out, cap, kCmdStopVideo, counter, 0x10, 0x01);
}

size_t buildKeepAlive(uint8_t* out, size_t cap) {
  // X4-VERIFY(sniff): keep-alive is 07 00 00 00 05 00 00 — length,
  // command 0x05 variant with no counter or payload.
  static const uint8_t kKeepAlive[kKeepAliveLen] = {0x07, 0x00, 0x00, 0x00,
                                                    0x05, 0x00, 0x00};
  if (out == nullptr || cap < kKeepAliveLen) return 0;
  memcpy(out, kKeepAlive, kKeepAliveLen);
  return kKeepAliveLen;
}

size_t buildGpsFrame(uint8_t* out, size_t cap, uint16_t counter,
                     const GpsSample& s) {
  if (out == nullptr || cap < kGpsFrameLen) return 0;

  writeBe81Header(out, kGpsFrameLen, kCmdGps, counter);

  // X4-VERIFY(sniff): protobuf field-1 length-delimited wrapper, 0x35 =
  // 53 payload bytes (bytes 18-70).
  out[16] = 0x0A;
  out[17] = 0x35;

  writeU32LE(out + 18, s.unixTimeSec);
  memset(out + 22, 0x00, 6);  // X4-VERIFY(sniff): bytes 22-27 zero
  out[28] = 0x41;             // X4-VERIFY(sniff): fixed 0x41 at byte 28

  writeDoubleLE(out + 29, fabs(s.latitudeDeg));
  out[37] = (s.latitudeDeg >= 0.0) ? 0x4E : 0x53;  // 'N' / 'S'
  writeDoubleLE(out + 38, fabs(s.longitudeDeg));
  out[46] = (s.longitudeDeg >= 0.0) ? 0x45 : 0x57;  // 'E' / 'W'
  writeDoubleLE(out + 47, fabs(s.speedMps));
  writeDoubleLE(out + 55, s.headingDeg);
  writeDoubleLE(out + 63, s.altitudeM);
  return kGpsFrameLen;
}

// -----------------------------------------------------------------------
// Parsers for camera -> us traffic
// -----------------------------------------------------------------------

Ce81Frame parseCe81Frame(const uint8_t* data, size_t len,
                         uint8_t serialOut[kSerialLen]) {
  if (data == nullptr || len < 5) return Ce81Frame::kUnknown;

  // X4-VERIFY(sniff): ce81 frames start FE EF FE; type bytes follow.
  if (data[0] != 0xFE || data[1] != 0xEF || data[2] != 0xFE) {
    return Ce81Frame::kUnknown;
  }

  // X4-VERIFY(sniff): serial frame starts FE EF FE 07 00 and carries the
  // 6 ASCII serial bytes at the END of the frame.
  if (data[3] == 0x07 && data[4] == 0x00 && len >= 11) {
    if (serialOut != nullptr) {
      memcpy(serialOut, data + len - kSerialLen, kSerialLen);
    }
    return Ce81Frame::kSerial;
  }
  return Ce81Frame::kStatus;
}

int8_t parseBe82RecordState(const uint8_t* data, size_t len) {
  constexpr size_t kMinRecordStateLen = 18;
  if (data == nullptr || len < kMinRecordStateLen) return -1;

  // X4-VERIFY(sniff): valid be82 responses carry 00 00 04 00 00 at
  // bytes 2-6; the response code is at byte 7.
  static const uint8_t kSignature[5] = {0x00, 0x00, 0x04, 0x00, 0x00};
  if (memcmp(data + 2, kSignature, sizeof(kSignature)) != 0) return -1;

  // X4-VERIFY(sniff): code 0x10 = record-state report, state at byte 17.
  if (data[7] != 0x10) return -1;
  return (data[17] > 0) ? 1 : 0;
}

// -----------------------------------------------------------------------
// Chunker
// -----------------------------------------------------------------------

size_t nextChunk(Chunker& c, uint8_t out[kChunkSize]) {
  if (c.buf == nullptr || c.offset >= c.len) return 0;
  size_t n = c.len - c.offset;
  if (n > kChunkSize) n = kChunkSize;
  memcpy(out, c.buf + c.offset, n);
  c.offset += n;
  return n;
}

}  // namespace insta360_protocol
