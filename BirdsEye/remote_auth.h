#pragma once

#include <stddef.h>
#include <stdint.h>

///////////////////////////////////////////
// REMOTE TRANSFER AUTH (plan 0019)
//
// Pure, Arduino-free rules for the PIN-gated remote transfer: the
// challenge-response answer, the nonce + lockout state machine, which
// commands a not-yet-authenticated peer may send, which settings are
// protected, and when the camera takes the radio back. bluetooth.ino owns
// only the radio and the clock; every decision lives here so it is
// host-tested.
//
// THE ANSWER (shared byte-for-byte with DovesDataViewer and LapWing):
//   mac    = HMAC-SHA256(key = PIN as ASCII, msg = "BEAUTH1|" + nonceHex)
//   answer = lowercase hex of the first 16 bytes of mac (32 chars)
// The nonce is 16 random bytes, sent as 32 lowercase hex chars. It is not
// bound to the device name: the nonce is already unique to this logger and
// this attempt, and the advertised name can be truncated on air, so mixing
// it in would only add a way for two correct implementations to disagree.
//
// What this does NOT protect against: the link is unencrypted Just Works,
// so a sniffer that records one handshake can brute-force a 4-digit PIN
// offline. The PIN keeps other people in the pits out; see the plan.
///////////////////////////////////////////

namespace remote_auth {

constexpr size_t kNonceLen = 16;
constexpr size_t kNonceHexLen = kNonceLen * 2;
constexpr size_t kAnswerHexLen = 32;
constexpr size_t kPinLen = 4;

// Wrong answers allowed before a lockout.
constexpr uint8_t kMaxFails = 5;
// First lockout; each further lockout doubles it, up to kLockoutMaxMs.
constexpr uint32_t kLockoutBaseMs = 60000;
constexpr uint32_t kLockoutMaxMs = 15UL * 60UL * 1000UL;
// A locked-mode peer that hasn't authenticated by now is dropped, so a
// stranger can't squat on the only peripheral connection slot.
constexpr uint32_t kAuthTimeoutMs = 20000;
// Camera wake threshold (camera_fsm kRpmOnThreshold). A paired camera
// claims the radio above it, and keeps the claim until the engine falls
// below kCameraReleaseRpm (camera_fsm kRpmOffThreshold) — without that
// hysteresis a pull-start cranking through 500 thrashes the standby advert
// on and off, an SD read and an advert rebuild each time.
constexpr uint16_t kCameraWakeRpm = 500;
constexpr uint16_t kCameraReleaseRpm = 300;
// An engine above this ends a remote session whatever the camera is doing:
// the driver is about to need logging, auto-race and the race pages, and
// the parked transfer loop runs none of them. Same number as the camera
// wake and auto-race thresholds.
constexpr uint16_t kEngineRunningRpm = 500;
// A remote session with no request traffic for this long ends. Nobody at
// the logger started it, so nobody there would think to end it, and the
// parked loop has no idle shutdown of its own.
constexpr uint32_t kRemoteIdleMs = 10UL * 60UL * 1000UL;
// After dropping a peer that never authenticated, the menu advert stays
// down this long. One slot means a squatter (or a bonded camera chasing
// our address) could otherwise hold it in a tight 20 s loop forever.
constexpr uint32_t kSquatBackoffMs = 5000;

// Exactly kPinLen ASCII digits, nothing else.
bool isValidPin(const char* pin);

// 32-bit random -> a 4-digit PIN in 1000-9999 (the range every logger has
// used since first boot). `out` holds at least kPinLen + 1 bytes.
void pinFromRandom(uint32_t r, char* out);

// Lowercase hex of `len` bytes into `out` (2*len + 1 bytes).
void toHex(const uint8_t* bytes, size_t len, char* out);

// The expected answer for (pin, nonceHex) into `out` (kAnswerHexLen + 1
// bytes).
void computeAnswer(const char* pin, const char* nonceHex, char* out);

// Case-insensitive compare of two hex strings of exactly `n` chars, in time
// independent of where they differ. False if either is shorter than `n` or
// longer than `n`.
bool hexEqualConstTime(const char* a, const char* b, size_t n);

// Per-boot auth state (RAM only — a reboot clears the lockout, which needs
// the device in hand anyway).
struct State {
  bool nonceValid = false;
  char nonceHex[kNonceHexLen + 1] = {0};
  uint8_t fails = 0;        // wrong answers since the last lockout
  uint8_t lockouts = 0;     // lockouts so far this boot (sets the length)
  bool lockActive = false;
  uint32_t lockStartMs = 0;
  uint32_t lockLenMs = 0;
  bool authed = false;      // AUTH:OK seen on the current connection
};

// Length of the next lockout given how many have already happened.
uint32_t lockoutLengthMs(uint8_t lockoutsSoFar);

// True while a lockout is running. Clears an expired lockout (and the fail
// count that caused it) as a side effect. Wrap-safe on nowMs.
bool isLocked(State& s, uint32_t nowMs);

// Whole seconds left in the current lockout, rounded up; 0 if none.
uint32_t lockSecondsLeft(State& s, uint32_t nowMs);

// Wrong answers left before a lockout.
inline uint8_t triesLeft(const State& s) {
  return s.fails >= kMaxFails ? 0 : uint8_t(kMaxFails - s.fails);
}

// Install a fresh nonce from `bytes` (kNonceLen random bytes). Any previous
// nonce is discarded.
void issueNonce(State& s, const uint8_t* bytes);

enum class Verdict : uint8_t {
  kOk,       // AUTH:OK
  kFail,     // AUTH:FAIL:<triesLeft>
  kLocked,   // AUTH:LOCKED:<lockSecondsLeft> (already locked, or this fail locked it)
  kNoNonce,  // AUTH:NO_NONCE — no AUTH? since the last answer
  kPinUnavailable,  // AUTH:BUSY — the stored PIN is unreadable or not 4 digits
};

// Check `answerHex` against the expected answer for the current nonce. The
// nonce is consumed on kOk, kFail and kLocked. kNoNonce has none to consume,
// and kPinUnavailable leaves it intact and counts nothing: an SD refusal or
// a hand-edited PIN is the logger's fault, not a wrong guess, so it must
// not burn the app's attempt or walk it toward a lockout. A malformed
// answer counts as a wrong one.
Verdict checkAnswer(State& s, const char* answerHex, const char* pin, uint32_t nowMs);

// Reset per-connection state (authed flag, nonce). The lockout survives.
void onDisconnect(State& s);

// Session modes. Open = someone started transfer on the device; locked =
// the logger is advertising from its main menu.
enum class Mode : uint8_t { kOpen, kLocked };

// May this command run? `cmd` is the trimmed text written to the request
// characteristic, compared exactly — a leading space or anything after an
// embedded NUL makes it a different (unknown) command, never an allowed one. Open: everything. Locked before AUTH:OK: only AUTH?,
// AUTH:<…> and BATT. PINGET is open-mode only — it is how an app learns the
// PIN, so it is never available to a peer that started remotely.
bool commandAllowed(Mode mode, bool authed, const char* cmd);

// Settings that SLIST must omit and SGET must refuse (SERR:PROTECTED).
bool isProtectedSettingKey(const char* key);

// What the camera is doing, as far as the radio is concerned.
struct CameraInputs {
  bool fsmIdleOrUnpaired = true;  // camera_fsm state is kUnpaired or kIdle
  bool testPageOpen = false;      // camera bench page holds the radio
  bool ownsRadio = false;         // bleOwner == CAMERA
  bool paired = false;
  uint16_t rpm = 0;
};

// The camera has, or is about to have, a use for the radio: its state
// machine is anywhere but UNPAIRED/IDLE, the bench page is open, it owns
// the radio — or a paired camera's engine has passed the wake threshold.
// That last one fires before the camera FSM's own 2 s wake debounce, so
// remote transfer has always let go of the radio before the camera asks.
// `engineLatch` is the caller's state for the engine half: set above
// kCameraWakeRpm, cleared below kCameraReleaseRpm (or when unpaired).
bool cameraWantsRadio(const CameraInputs& in, bool& engineLatch);

// When the menu standby advert should be up.
struct StandbyInputs {
  bool onMainMenu = false;
  bool radioFree = false;        // bleOwner == NONE, or already ours for standby
  bool cameraWantsRadio = false;
  bool raceActive = false;
  bool settingEnabled = false;   // remote_transfer setting
  bool squatBackoff = false;     // squatBackoffActive()
};
bool standbyWanted(const StandbyInputs& in);

// The 20 s squatting limit, wrap-safe.
inline bool authTimedOut(uint32_t connectedAtMs, uint32_t nowMs) {
  return uint32_t(nowMs - connectedAtMs) >= kAuthTimeoutMs;
}

// The post-squat quiet period, wrap-safe. `armed` is false until the first
// squat drop.
inline bool squatBackoffActive(bool armed, uint32_t droppedAtMs, uint32_t nowMs) {
  return armed && uint32_t(nowMs - droppedAtMs) < kSquatBackoffMs;
}

// Why a remote (menu-started, authenticated) session must end now.
enum class RemoteEnd : uint8_t {
  kNone,
  kEngine,  // AUTH:ENGINE — engine above kEngineRunningRpm
  kCamera,  // AUTH:CAMERA — the camera wants the radio back
  kIdle,    // AUTH:IDLE — kRemoteIdleMs without a request
};

struct RemoteSessionInputs {
  bool otaApplyPending = false;   // FWAPPLY accepted: the flasher owns the exit
  uint16_t rpm = 0;
  bool cameraWantsRadio = false;
  bool busy = false;              // a download / upload / OTA stream in flight
  uint32_t lastRequestMs = 0;     // last write to the request characteristic
  uint32_t nowMs = 0;
};

// Engine first (it also covers a paired camera's own engine wake), then the
// camera, then idleness. An in-flight transfer is activity — a long
// download is one command — but never outranks the engine or the camera.
// An accepted OTA apply is never interrupted: it reboots on its own.
RemoteEnd remoteSessionEnd(const RemoteSessionInputs& in);

// The unsolicited notice sent before the drop; nullptr for kNone.
const char* remoteEndToken(RemoteEnd e);

}  // namespace remote_auth
