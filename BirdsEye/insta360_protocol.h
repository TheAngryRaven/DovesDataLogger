#pragma once

///////////////////////////////////////////
// INSTA360 X4 BLE PROTOCOL
// Builders and parsers for the BLE frames that control an Insta360 X4
// action camera: the iBeacon-masquerade wake advertisement, the ce82
// remote-button frames, the be81 control frames (start/stop video,
// keep-alive, GPS injection), and the parsers for the camera's ce81
// writes and be82 notifications. The BLE plumbing (services, GATT,
// advertising) stays in the sketch; this unit owns the byte layouts so
// they are host-testable.
//
// Byte tables come from X4-verified reference implementations
// (pchwalek / tsunghowu / btittelbach / arsfabula); every camera-facing
// table is marked `// X4-VERIFY(sniff)` until confirmed against a live
// sniff of our own hardware.
//
// All builders write nothing and return 0 when `cap` is too small.
// Multi-byte integers on the wire are little-endian; doubles are
// IEEE-754 binary64 little-endian (the nRF52840 and the host test
// machines are both little-endian IEEE-754 — see the static_assert in
// the .cpp).
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace insta360_protocol {

// BLE writes to the camera are streamed in chunks of at most this many
// bytes (fits the default 23-byte MTU's 20-byte ATT payload).
constexpr size_t kChunkSize = 20;

// Camera serial: exactly 6 ASCII characters, uppercase A-Z or 0-9.
constexpr size_t kSerialLen = 6;

// The wake advertisement is a fixed 31-byte BLE advertising payload.
constexpr size_t kWakeAdvertLen = 31;

// First message counter value for be81 control frames. Incremented by
// the caller per command sent.
constexpr uint16_t kInitialMsgCounter = 0x0200;

// Validate `s` as a camera serial (exactly 6 chars, each A-Z or 0-9;
// lowercase letters are accepted and uppercased) and copy the 6
// uppercase ASCII bytes to `out`. Returns false (writing nothing) on
// null / wrong length / invalid characters.
bool parseSerial(const char* s, uint8_t out[kSerialLen]);

// Render a 6-byte serial as a NUL-terminated string (out holds 7).
void serialToString(const uint8_t serial[kSerialLen],
                    char out[kSerialLen + 1]);

// Build the 31-byte wake advertisement (iBeacon masquerade carrying the
// camera serial) that wakes a sleeping X4. Returns kWakeAdvertLen.
size_t buildWakeAdvert(uint8_t out[kWakeAdvertLen],
                       const uint8_t serial[kSerialLen]);

// ce82 remote->camera button frames (9 bytes each, static contents).
size_t buildShutterToggle(uint8_t* out, size_t cap);
size_t buildModeCycle(uint8_t* out, size_t cap);
size_t buildScreenToggle(uint8_t* out, size_t cap);
size_t buildPowerOff(uint8_t* out, size_t cap);

// be81 remote->camera control frames. `counter` is the per-command
// message counter (start at kInitialMsgCounter, increment per command).
size_t buildStartVideo(uint8_t* out, size_t cap, uint16_t counter);
size_t buildStopVideo(uint8_t* out, size_t cap, uint16_t counter);
size_t buildKeepAlive(uint8_t* out, size_t cap);

// One GPS fix to inject into the camera's recording (command 0x35).
struct GpsSample {
  uint32_t unixTimeSec = 0;
  double latitudeDeg = 0.0;   // signed
  double longitudeDeg = 0.0;  // signed
  double speedMps = 0.0;
  double headingDeg = 0.0;    // 0-360
  double altitudeM = 0.0;
};

// Build the 71-byte GPS injection frame. Latitude/longitude/speed are
// stored as absolute values with hemisphere bytes 'N'/'S' and 'E'/'W'.
size_t buildGpsFrame(uint8_t* out, size_t cap, uint16_t counter,
                     const GpsSample& s);

// ce81: frames the camera writes to our remote service.
enum class Ce81Frame : uint8_t { kUnknown, kSerial, kStatus };

// Classify a ce81 write. Returns kSerial when the frame is the
// camera-serial frame and copies the 6 ASCII serial bytes to
// `serialOut` (if non-null). `len` is the raw write length.
Ce81Frame parseCe81Frame(const uint8_t* data, size_t len,
                         uint8_t serialOut[kSerialLen]);

// be82: notifications from the camera's control service.
// Record state: -1 = not a record-state frame / unknown, 0 = stopped,
// 1 = recording.
int8_t parseBe82RecordState(const uint8_t* data, size_t len);

// Splits an arbitrary-length frame into <= kChunkSize-byte BLE writes.
struct Chunker {
  const uint8_t* buf = nullptr;
  size_t len = 0;
  size_t offset = 0;
};

// Copy the next chunk (up to kChunkSize bytes) into `out` and advance.
// Returns the number of bytes copied, 0 when the frame is exhausted.
size_t nextChunk(Chunker& c, uint8_t out[kChunkSize]);

}  // namespace insta360_protocol
