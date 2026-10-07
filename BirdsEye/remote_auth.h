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
//   mac    = HMAC-SHA256(key = PIN as ASCII,
//                        msg = "BEAUTH1|" + nonceHex + "|" + bluetooth_name)
//   answer = lowercase hex of the first 16 bytes of mac (32 chars)
// The nonce is 16 random bytes, sent as 32 lowercase hex chars.
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
// Camera wake threshold (camera_fsm kRpmOnThreshold). A remote transfer
// ends above it when a camera is paired, so the camera can wake.
constexpr uint16_t kCameraWakeRpm = 500;

// Exactly kPinLen ASCII digits, nothing else.
bool isValidPin(const char* pin);

// 32-bit random -> a 4-digit PIN in 1000-9999 (the range every logger has
// used since first boot). `out` holds at least kPinLen + 1 bytes.
void pinFromRandom(uint32_t r, char* out);

// Lowercase hex of `len` bytes into `out` (2*len + 1 bytes).
void toHex(const uint8_t* bytes, size_t len, char* out);

// The expected answer for (pin, nonceHex, deviceName) into `out`
// (kAnswerHexLen + 1 bytes).
void computeAnswer(const char* pin, const char* nonceHex, const char* deviceName,
                   char* out);

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
};

// Check `answerHex` against the expected answer for the current nonce. The
// nonce is consumed on every verdict except kLocked-before-checking and
// kNoNonce. A malformed answer counts as a wrong one.
Verdict checkAnswer(State& s, const char* answerHex, const char* pin,
                    const char* deviceName, uint32_t nowMs);

// Reset per-connection state (authed flag, nonce). The lockout survives.
void onDisconnect(State& s);

// Session modes. Open = someone started transfer on the device; locked =
// the logger is advertising from its main menu.
enum class Mode : uint8_t { kOpen, kLocked };

// May this command run? `cmd` is the trimmed text written to the request
// characteristic. Open: everything. Locked before AUTH:OK: only AUTH?,
// AUTH:<…> and BATT. PINGET is open-mode only — it is how an app learns the
// PIN, so it is never available to a peer that started remotely.
bool commandAllowed(Mode mode, bool authed, const char* cmd);

// Settings that SLIST must omit and SGET must refuse (SERR:PROTECTED).
bool isProtectedSettingKey(const char* key);

// When the menu standby advert should be up.
struct StandbyInputs {
  bool onMainMenu = false;
  bool radioFree = false;     // bleOwner == NONE
  bool cameraBusy = false;    // see cameraBusy()
  bool raceActive = false;
  bool settingEnabled = false;  // remote_transfer setting
};
bool standbyWanted(const StandbyInputs& in);

// The camera "has a use for the radio": its state machine is anywhere but
// UNPAIRED/IDLE, the camera bench page is open, or it owns the radio.
bool cameraBusy(bool fsmIdleOrUnpaired, bool testPageOpen, bool ownsRadio);

// A REMOTE transfer session must hand the radio back now: the camera is
// busy, or a paired camera's engine just passed the wake threshold.
bool cameraEndsRemoteSession(bool cameraBusyNow, bool cameraPaired, uint16_t rpm);

// The 20 s squatting limit, wrap-safe.
inline bool authTimedOut(uint32_t connectedAtMs, uint32_t nowMs) {
  return uint32_t(nowMs - connectedAtMs) >= kAuthTimeoutMs;
}

}  // namespace remote_auth
