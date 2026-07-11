#pragma once

///////////////////////////////////////////
// INSTA360 X4 BLE PROTOCOL
// Builders and parsers for the BLE frames that emulate the Insta360 GPS
// Remote controlling an Insta360 X4 action camera: the iBeacon-masquerade
// wake advertisement, the remote's scan response, the ce82 remote-button
// frames (all control — record via shutter toggle, power-off via streamed
// hold), and the parser for the camera's ce81 writes (serial capture). The
// BLE plumbing (services, GATT, advertising) stays in the sketch; this unit
// owns the byte layouts so they are host-testable.
//
// Byte tables come from X4-verified reference implementations
// (pchwalek / tsunghowu / btittelbach / arsfabula); every camera-facing
// table is marked `// X4-VERIFY(sniff)` until confirmed against a live
// sniff of our own hardware.
//
// All builders write nothing and return 0 when `cap` is too small.
// Multi-byte integers on the wire are little-endian.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace insta360_protocol {

// Camera serial: exactly 6 ASCII characters, uppercase A-Z or 0-9.
constexpr size_t kSerialLen = 6;

// The wake advertisement is a fixed 31-byte BLE advertising payload
// (3-byte Flags AD + 28-byte Apple-framed manufacturer AD carrying the
// 26-byte mfg-data value sniffed from the real GPS Action Remote).
constexpr size_t kWakeAdvertLen = 31;

// Validate `s` as a camera serial (exactly 6 chars, each A-Z or 0-9;
// lowercase letters are accepted and uppercased) and copy the 6
// uppercase ASCII bytes to `out`. Returns false (writing nothing) on
// null / wrong length / invalid characters.
bool parseSerial(const char* s, uint8_t out[kSerialLen]);

// Render a 6-byte serial as a NUL-terminated string (out holds 7).
void serialToString(const uint8_t serial[kSerialLen],
                    char out[kSerialLen + 1]);

// Build the 31-byte wake advertisement (iBeacon masquerade carrying the
// camera serial) that wakes a sleeping X4. X4-confirmed ground truth
// captured from a genuine GPS Remote. Returns kWakeAdvertLen.
size_t buildWakeAdvert(uint8_t out[kWakeAdvertLen],
                       const uint8_t serial[kSerialLen]);

// The genuine remote's 25-byte scan response: Appearance 0x0180 +
// Complete Local Name "Insta360 GPS Remote". Returns kRemoteScanRspLen.
constexpr size_t kRemoteScanRspLen = 25;
size_t buildRemoteScanResponse(uint8_t out[kRemoteScanRspLen]);

// ce82 remote->camera button frames (9 bytes each). `seq` is a running
// counter the real remote places at byte[4] and increments by 2 with
// every ce82 frame it sends (X4-confirmed capture: shutter=0x00,
// mode=0x02, power-click=0x04, then the power-HOLD stream 0x06, 0x08,
// 0x0A ...). The camera tracks distinct frames by it, so a held button
// must be STREAMED with a fresh seq each frame, not sent once.
//   btn:  0x02 shutter, 0x01 mode, 0x00 power
//   press: 0x00 tap/click, 0x03 3-second hold (power-off)
size_t buildShutterToggle(uint8_t* out, size_t cap, uint8_t seq);
size_t buildModeCycle(uint8_t* out, size_t cap, uint8_t seq);
size_t buildScreenToggle(uint8_t* out, size_t cap, uint8_t seq);  // power tap
size_t buildPowerOff(uint8_t* out, size_t cap, uint8_t seq);      // power hold

// ce81: frames the camera writes to our remote service.
enum class Ce81Frame : uint8_t { kUnknown, kSerial, kStatus };

// Classify a ce81 write. Returns kSerial when the frame is the
// camera-serial frame and copies the 6 ASCII serial bytes to
// `serialOut` (if non-null). `len` is the raw write length.
Ce81Frame parseCe81Frame(const uint8_t* data, size_t len,
                         uint8_t serialOut[kSerialLen]);

}  // namespace insta360_protocol
