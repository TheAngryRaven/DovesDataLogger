#include "insta360_protocol.h"

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

}  // namespace insta360_protocol
