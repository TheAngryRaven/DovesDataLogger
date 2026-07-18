#include "insta360_protocol.h"

#include <math.h>
#include <string.h>

namespace insta360_protocol {

namespace {

// ---------------------------------------------------------------------
// ce82 button frames: FC EF FE 86 00 03 01 <button> <press>
// ---------------------------------------------------------------------

constexpr size_t kButtonFrameLen = 9;

// X4-CONFIRMED (2026-07-10 remote capture): ce82 button-frame preamble.
// byte[4] is a running sequence counter, filled in per-call below.
constexpr uint8_t kButtonPreamble[7] = {0xFC, 0xEF, 0xFE, 0x86,
                                        0x00, 0x03, 0x01};

size_t buildButtonFrame(uint8_t* out, size_t cap, uint8_t seq,
                        uint8_t button, uint8_t press) {
  if (out == nullptr || cap < kButtonFrameLen) return 0;
  memcpy(out, kButtonPreamble, sizeof(kButtonPreamble));
  out[4] = seq;  // running counter, +2 per frame on the real remote
  out[7] = button;
  out[8] = press;
  return kButtonFrameLen;
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
  // X4-CONFIRMED (2026-07-10 bench session): byte-for-byte the ADV_IND a
  // genuine Insta360 GPS Remote transmits, captured with nRF Connect
  // against camera "X4 34UQG5" — and replaying this exact packet from a
  // phone POWERED THE SLEEPING X4 ON. Only the 6 serial bytes at
  // mfg[14..19] (absolute offsets 19-24) vary per camera; everything
  // else is constant across Insta360 cameras.
  //
  // The payload only BORROWS the Apple/iBeacon framing (4C 00 02 15) —
  // it is NOT a real iBeacon: the "UUID" region is the Insta360 ORBIT
  // constant + the camera serial in plain ASCII, the "Major/Minor" slots
  // are 00 00 00 00, and the "TX-power" slot is the two-byte E4 01 tail.
  //
  // Flags are 0x05 (LE Limited Discoverable + BR/EDR Not Supported),
  // exactly as the real remote sends — an earlier 0x1A (from the
  // pchwalek ESP32 replication) was the one byte differing from the
  // captured ground truth.
  //
  // NOTE: wake only reaches an ARMED camera (X4: QuickCapture OFF =
  // "Bluetooth Wakeup Enabled"); armed cameras keep scanning even fully
  // powered off.
  static const uint8_t kTemplate[kWakeAdvertLen] = {
      0x02, 0x01, 0x05,                          // Flags AD (0x05, as captured)
      0x1B, 0xFF,                                // mfg AD: len 27, type 0xFF
      0x4C, 0x00,                                // company ID 0x004C (Apple, LE)
      0x02, 0x15,                                // iBeacon-style subtype + len 21
      0x09,                                      // mfg[4]
      0x4F, 0x52, 0x42, 0x49, 0x54,              // mfg[5..9]   = "ORBIT"
      0x09, 0xFF, 0x0F, 0x00,                    // mfg[10..13]
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        // mfg[14..19] = serial (ASCII)
      0x00, 0x00, 0x00, 0x00,                    // mfg[20..23]
      0xE4, 0x01,                                // mfg[24..25] tail
  };
  constexpr size_t kSerialOffset = 19;  // absolute offset of mfg[14]

  memcpy(out, kTemplate, kWakeAdvertLen);
  memcpy(out + kSerialOffset, serial, kSerialLen);
  return kWakeAdvertLen;
}

size_t buildRemoteScanResponse(uint8_t out[kRemoteScanRspLen]) {
  // X4-CONFIRMED (same capture): the genuine remote's scan response —
  // Appearance 0x0180 (Generic Remote Control) + Complete Local Name
  // "Insta360 GPS Remote". Not required for wake (the camera keys on the
  // mfg block), but part of the remote's identity for the camera's
  // reconnect/pairing scans.
  static const uint8_t kTemplate[kRemoteScanRspLen] = {
      0x03, 0x19, 0x80, 0x01,  // Appearance AD: 0x0180, remote control
      0x14, 0x09,              // Complete Local Name AD: len 20, type 0x09
      'I', 'n', 's', 't', 'a', '3', '6', '0', ' ',
      'G', 'P', 'S', ' ', 'R', 'e', 'm', 'o', 't', 'e',
  };
  memcpy(out, kTemplate, kRemoteScanRspLen);
  return kRemoteScanRspLen;
}

// -----------------------------------------------------------------------
// ce82 button frames
// -----------------------------------------------------------------------

size_t buildShutterToggle(uint8_t* out, size_t cap, uint8_t seq) {
  // X4-CONFIRMED: FC EF FE 86 <seq> 03 01 02 00 (button 0x02, tap).
  return buildButtonFrame(out, cap, seq, 0x02, 0x00);
}

size_t buildModeCycle(uint8_t* out, size_t cap, uint8_t seq) {
  // X4-CONFIRMED: FC EF FE 86 <seq> 03 01 01 00 (button 0x01, tap).
  return buildButtonFrame(out, cap, seq, 0x01, 0x00);
}

size_t buildScreenToggle(uint8_t* out, size_t cap, uint8_t seq) {
  // X4-CONFIRMED: FC EF FE 86 <seq> 03 01 00 00 (power button, single
  // tap — toggles the screen on/off; a full boot uses this too).
  return buildButtonFrame(out, cap, seq, 0x00, 0x00);
}

size_t buildPowerOff(uint8_t* out, size_t cap, uint8_t seq) {
  // X4-CONFIRMED: FC EF FE 86 <seq> 03 01 00 03 (power button, 3-second
  // HOLD). The real remote STREAMS this frame continuously with an
  // incrementing seq while the button is held — the camera powers off
  // after receiving the sustained hold; a single frame does nothing.
  return buildButtonFrame(out, cap, seq, 0x00, 0x03);
}

// -----------------------------------------------------------------------
// ce82 GPS telemetry frame (RMC-in-ASCII). Integer-only formatting so it
// is portable to Arduino (newlib-nano lacks %f) and host alike.
// -----------------------------------------------------------------------

namespace {

// Append a NUL-terminated string; returns the number of chars written.
size_t appendStr(char* buf, size_t pos, const char* s) {
  size_t n = 0;
  while (s[n] != '\0') { buf[pos + n] = s[n]; n++; }
  return n;
}

// Append `value` zero-padded to `width` digits (value assumed < 10^width).
size_t appendUintPadded(char* buf, size_t pos, uint32_t value, int width) {
  char tmp[12];
  int i = 0;
  for (int w = 0; w < width; w++) { tmp[i++] = char('0' + value % 10); value /= 10; }
  for (int j = 0; j < i; j++) buf[pos + j] = tmp[i - 1 - j];
  return (size_t)i;
}

// Append a signed fixed-point value: [-]<int>.<frac zero-padded to `dec`>.
size_t appendFixed(char* buf, size_t pos, double value, int dec) {
  const size_t start = pos;
  const bool neg = value < 0.0;
  const double a = neg ? -value : value;
  uint64_t scale = 1;
  for (int i = 0; i < dec; i++) scale *= 10;
  const uint64_t scaled = (uint64_t)llround(a * (double)scale);  // nearest
  uint64_t ip = scaled / scale;
  const uint64_t fp = scaled % scale;
  if (neg) buf[pos++] = '-';
  // integer part (at least one digit)
  char tmp[24];
  int i = 0;
  do { tmp[i++] = char('0' + ip % 10); ip /= 10; } while (ip > 0);
  for (int j = 0; j < i; j++) buf[pos++] = tmp[i - 1 - j];
  buf[pos++] = '.';
  pos += appendUintPadded(buf, pos, (uint32_t)fp, dec);
  return pos - start;
}

// decimal degrees -> ddmm.mmmm as a double (sign preserved).
double toDegMin(double deg) {
  const bool neg = deg < 0.0;
  const double a = neg ? -deg : deg;
  const double d = (double)(long)a;      // whole degrees
  const double m = (a - d) * 60.0;       // decimal minutes
  const double dm = d * 100.0 + m;
  return neg ? -dm : dm;
}

}  // namespace

size_t buildGpsRmcFrame(uint8_t* out, size_t cap, const GpsRmc& s) {
  if (out == nullptr) return 0;

  // Build the RMC body (everything between '$' and '*', exclusive) so the
  // checksum can XOR over it, then assemble the full sentence + payload.
  char body[96];
  size_t b = 0;
  b += appendStr(body, b, "GNRMC,");
  // UTC hhmmss.sss
  b += appendUintPadded(body, b, s.hour, 2);
  b += appendUintPadded(body, b, s.minute, 2);
  b += appendUintPadded(body, b, s.second, 2);
  body[b++] = '.';
  b += appendUintPadded(body, b, s.milli, 3);
  body[b++] = ',';
  body[b++] = s.valid ? 'A' : 'V';
  body[b++] = ',';
  // latitude ddmm.mmmm + N/S
  b += appendFixed(body, b, toDegMin(s.latitudeDeg) < 0 ? -toDegMin(s.latitudeDeg)
                                                        : toDegMin(s.latitudeDeg),
                   4);
  body[b++] = ',';
  body[b++] = s.latitudeDeg < 0.0 ? 'S' : 'N';
  body[b++] = ',';
  // longitude: SIGNED ddmm.mmmm, hemisphere letter always 'E' (the quirk)
  b += appendFixed(body, b, toDegMin(s.longitudeDeg), 4);
  b += appendStr(body, b, ",E,");
  b += appendFixed(body, b, s.speedKnots, 2);
  body[b++] = ',';
  b += appendFixed(body, b, s.courseDeg, 2);
  body[b++] = ',';
  // date ddmmyy
  b += appendUintPadded(body, b, s.day, 2);
  b += appendUintPadded(body, b, s.month, 2);
  b += appendUintPadded(body, b, s.year, 2);
  // constant magvar 0.0 W, mode A, extra V
  b += appendStr(body, b, ",0.0,W,A,V");

  // NMEA checksum: XOR of every char in the body.
  uint8_t cksum = 0;
  for (size_t i = 0; i < b; i++) cksum ^= (uint8_t)body[i];

  // Payload = ,26.7,<0x07>,$<body>*HH
  char payload[112];
  size_t p = 0;
  p += appendStr(payload, p, ",26.7,");
  payload[p++] = 0x07;
  payload[p++] = ',';
  payload[p++] = '$';
  memcpy(payload + p, body, b); p += b;
  payload[p++] = '*';
  static const char kHex[] = "0123456789ABCDEF";
  payload[p++] = kHex[(cksum >> 4) & 0xF];
  payload[p++] = kHex[cksum & 0xF];

  // Frame = FC EF FE 83 00 <len> <payload>
  const size_t frameLen = 6 + p;
  if (cap < frameLen) return 0;
  out[0] = 0xFC; out[1] = 0xEF; out[2] = 0xFE;
  out[3] = 0x83;   // GPS/sensor type
  out[4] = 0x00;   // SN slot: always 0 for GPS
  out[5] = (uint8_t)p;
  memcpy(out + 6, payload, p);
  return frameLen;
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

RecordObs parseRecordingState(const uint8_t* data, size_t len) {
  // Header (6): FE EF FE 10 <flag> <len>. Below 7 bytes there is no room
  // for even a 1-byte payload, so it cannot be a display string.
  if (data == nullptr || len < 7) return RecordObs::kUnknown;
  if (data[0] != 0xFE || data[1] != 0xEF || data[2] != 0xFE) {
    return RecordObs::kUnknown;
  }
  if (data[3] != 0x10) return RecordObs::kUnknown;  // not a display-string frame

  const size_t payLen = data[5];               // payload length after the header
  if (6 + payLen > len) return RecordObs::kUnknown;  // truncated write, don't guess
  // Payload = 4 control bytes + the ASCII display string. A payload without
  // room for the 4 control bytes plus text is an idle/degenerate frame.
  if (payLen <= 4) return RecordObs::kIdle;
  const uint8_t* ascii = data + 10;            // 6 header + 4 control bytes
  const size_t asciiLen = payLen - 4;

  // The elapsed-time counter ".HH:MM:SS" is the only display string that
  // carries ':' separators (the mode/battery strings never do), so two or
  // more colons is the dependable "recording" signal.
  int colons = 0;
  for (size_t i = 0; i < asciiLen; ++i) {
    if (ascii[i] == ':') ++colons;
  }
  return colons >= 2 ? RecordObs::kRecording : RecordObs::kIdle;
}

}  // namespace insta360_protocol
