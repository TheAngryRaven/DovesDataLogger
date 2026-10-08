#include "doctest.h"

#include <cstring>

#include "remote_auth.h"

using namespace remote_auth;

namespace {

const uint8_t kNonceA[kNonceLen] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};

// The answer for kNonceA / "4821". The SAME vector is
// pinned in DovesDataViewer (src/lib/ble/auth.test.ts) and LapWing
// (loggers/doveslogger/auth.rs) — change all three together or none.
const char kGoldenAnswer[] = "4936f5dad4502101eb0f65790cca77b4";

void answerFor(const State& s, const char* pin, char* out) {
  computeAnswer(pin, s.nonceHex, out);
}

}  // namespace

TEST_CASE("answer matches the cross-implementation golden vectors") {
  char out[kAnswerHexLen + 1];
  computeAnswer("4821", "00112233445566778899aabbccddeeff", out);
  CHECK(std::strcmp(out, kGoldenAnswer) == 0);
  computeAnswer("1000", "ffffffffffffffffffffffffffffffff", out);
  CHECK(std::strcmp(out, "3db7fac4dc48559fa85594079b59fce4") == 0);
}

TEST_CASE("PIN format") {
  CHECK(isValidPin("4821"));
  CHECK(isValidPin("0000"));
  CHECK_FALSE(isValidPin("482"));
  CHECK_FALSE(isValidPin("48210"));
  CHECK_FALSE(isValidPin("48a1"));
  CHECK_FALSE(isValidPin(" 482"));
  CHECK_FALSE(isValidPin(""));
  CHECK_FALSE(isValidPin(nullptr));
}

TEST_CASE("pinFromRandom stays in 1000-9999") {
  char pin[kPinLen + 1];
  pinFromRandom(0, pin);
  CHECK(std::strcmp(pin, "1000") == 0);
  pinFromRandom(8999, pin);
  CHECK(std::strcmp(pin, "9999") == 0);
  pinFromRandom(9000, pin);
  CHECK(std::strcmp(pin, "1000") == 0);
  pinFromRandom(0xffffffffu, pin);
  CHECK(isValidPin(pin));
  CHECK(pin[0] != '0');
}

TEST_CASE("constant-time hex compare is case-insensitive and length-strict") {
  CHECK(hexEqualConstTime("abcd", "ABCD", 4));
  CHECK_FALSE(hexEqualConstTime("abce", "abcd", 4));
  CHECK_FALSE(hexEqualConstTime("abc", "abcd", 4));
  CHECK_FALSE(hexEqualConstTime("abcde", "abcd", 4));
  CHECK_FALSE(hexEqualConstTime(nullptr, "abcd", 4));
}

TEST_CASE("right answer authenticates and consumes the nonce") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(std::strcmp(s.nonceHex, "00112233445566778899aabbccddeeff") == 0);
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", 0) == Verdict::kOk);
  CHECK(s.authed);
  // Replay of the same answer: the nonce is gone.
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", 0) == Verdict::kNoNonce);
}

TEST_CASE("uppercase answer is accepted") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "4936F5DAD4502101EB0F65790CCA77B4", "4821", 0) ==
        Verdict::kOk);
}

TEST_CASE("answer is bound to the PIN") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, kGoldenAnswer, "4822", 0) == Verdict::kFail);
}

TEST_CASE("wrong answer consumes the nonce and counts down") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", 0) == Verdict::kFail);
  CHECK(triesLeft(s) == kMaxFails - 1);
  CHECK_FALSE(s.nonceValid);
  CHECK_FALSE(s.authed);
}

TEST_CASE("malformed answers count as wrong") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "short", "4821", 0) == Verdict::kFail);
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "zzc61d4ff111ba3236b50217443c3daa", "4821", 0) == Verdict::kFail);
  CHECK(triesLeft(s) == kMaxFails - 2);
}

TEST_CASE("an unreadable or invalid stored PIN never authenticates") {
  State s;
  issueNonce(s, kNonceA);
  char ans[kAnswerHexLen + 1];
  computeAnswer("", s.nonceHex, ans);
  CHECK(checkAnswer(s, ans, "", 0) == Verdict::kPinUnavailable);
  CHECK_FALSE(s.authed);
  computeAnswer("12345", s.nonceHex, ans);
  CHECK(checkAnswer(s, ans, "12345", 0) == Verdict::kPinUnavailable);
  CHECK(checkAnswer(s, ans, "12a4", 0) == Verdict::kPinUnavailable);
  CHECK_FALSE(s.authed);
}

TEST_CASE("PIN unreadable leaves the nonce intact and counts nothing") {
  State s;
  issueNonce(s, kNonceA);
  // Every try with the PIN unreadable: no failure, no lockout, nonce kept.
  for (uint8_t i = 0; i < kMaxFails + 2; ++i) {
    CHECK(checkAnswer(s, "00000000000000000000000000000000", "", 0) ==
          Verdict::kPinUnavailable);
  }
  CHECK(s.nonceValid);
  CHECK(triesLeft(s) == kMaxFails);
  CHECK_FALSE(isLocked(s, 0));
  // Once the card answers again, the SAME nonce still authenticates.
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", 0) == Verdict::kOk);
  CHECK_FALSE(s.nonceValid);
}

TEST_CASE("no nonce outranks an unreadable PIN") {
  State s;
  CHECK(checkAnswer(s, kGoldenAnswer, "", 0) == Verdict::kNoNonce);
}

TEST_CASE("five wrong answers lock for 60 s, then doubling to 15 min") {
  State s;
  uint32_t now = 1000;
  for (uint8_t i = 0; i + 1 < kMaxFails; ++i) {
    issueNonce(s, kNonceA);
    CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", now) == Verdict::kFail);
  }
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", now) == Verdict::kLocked);
  CHECK(isLocked(s, now));
  CHECK(lockSecondsLeft(s, now) == 60);
  CHECK(lockSecondsLeft(s, now + 59001) == 1);

  // Even the right answer is refused while locked, and the nonce is burnt.
  issueNonce(s, kNonceA);
  char ans[kAnswerHexLen + 1];
  answerFor(s, "4821", ans);
  CHECK(checkAnswer(s, ans, "4821", now + 1000) == Verdict::kLocked);
  CHECK_FALSE(s.nonceValid);

  now += 60000;
  CHECK_FALSE(isLocked(s, now));
  CHECK(triesLeft(s) == kMaxFails);

  CHECK(lockoutLengthMs(0) == 60000);
  CHECK(lockoutLengthMs(1) == 120000);
  CHECK(lockoutLengthMs(3) == 480000);
  CHECK(lockoutLengthMs(4) == kLockoutMaxMs);
  CHECK(lockoutLengthMs(200) == kLockoutMaxMs);
}

TEST_CASE("lockout survives millis() wrap") {
  State s;
  const uint32_t now = 0xffffffffu - 10000u;
  for (uint8_t i = 0; i < kMaxFails; ++i) {
    issueNonce(s, kNonceA);
    checkAnswer(s, "00000000000000000000000000000000", "4821", now);
  }
  CHECK(isLocked(s, now + 30000u));  // wrapped past zero, still inside 60 s
  CHECK_FALSE(isLocked(s, now + 60000u));
}

TEST_CASE("disconnect clears auth but not the lockout") {
  State s;
  issueNonce(s, kNonceA);
  checkAnswer(s, kGoldenAnswer, "4821", 0);
  onDisconnect(s);
  CHECK_FALSE(s.authed);
  CHECK_FALSE(s.nonceValid);

  for (uint8_t i = 0; i < kMaxFails; ++i) {
    issueNonce(s, kNonceA);
    checkAnswer(s, "00000000000000000000000000000000", "4821", 0);
  }
  onDisconnect(s);
  CHECK(isLocked(s, 1));
}

TEST_CASE("command gating") {
  // Open (local start): everything, including PINGET.
  CHECK(commandAllowed(Mode::kOpen, false, "LIST"));
  CHECK(commandAllowed(Mode::kOpen, false, "PINGET"));
  CHECK(commandAllowed(Mode::kOpen, false, "SRESET"));

  // Locked before auth: only the handshake and battery.
  CHECK(commandAllowed(Mode::kLocked, false, "AUTH?"));
  CHECK(commandAllowed(Mode::kLocked, false, "AUTH:abc"));
  CHECK(commandAllowed(Mode::kLocked, false, "BATT"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "LIST"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "SLIST"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "FWDFU"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "AUTHX"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "PINGET"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, nullptr));
  // Exact match only: a leading space is a different (unknown) command.
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, " LIST"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, " AUTH?"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, " BATT"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, "BATT "));
  // An embedded NUL ends the command: what follows can't ride along.
  const char authThenList[] = "AUTH?\0LIST";
  CHECK(commandAllowed(Mode::kLocked, false, authThenList));
  CHECK(std::strcmp(authThenList, "AUTH?") == 0);
  const char listAfterNul[] = "\0LIST";
  CHECK_FALSE(commandAllowed(Mode::kLocked, false, listAfterNul));
  const char pinGetNul[] = "PINGET\0x";
  CHECK_FALSE(commandAllowed(Mode::kLocked, true, pinGetNul));

  // Locked after auth: everything but PINGET.
  CHECK(commandAllowed(Mode::kLocked, true, "LIST"));
  CHECK(commandAllowed(Mode::kLocked, true, "SSET:bluetooth_pin=1234"));
  CHECK_FALSE(commandAllowed(Mode::kLocked, true, "PINGET"));
}

TEST_CASE("protected settings") {
  CHECK(isProtectedSettingKey("bluetooth_pin"));
  CHECK_FALSE(isProtectedSettingKey("bluetooth_name"));
  CHECK_FALSE(isProtectedSettingKey("bluetooth_pin2"));
  CHECK_FALSE(isProtectedSettingKey(nullptr));
}

TEST_CASE("standby advert only on a free menu with the camera idle") {
  StandbyInputs in;
  in.onMainMenu = true;
  in.radioFree = true;
  in.settingEnabled = true;
  CHECK(standbyWanted(in));

  StandbyInputs off = in;
  off.settingEnabled = false;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.onMainMenu = false;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.radioFree = false;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.cameraWantsRadio = true;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.raceActive = true;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.squatBackoff = true;
  CHECK_FALSE(standbyWanted(off));
}

TEST_CASE("squat back-off is 5 s, only once armed, and wrap-safe") {
  CHECK_FALSE(squatBackoffActive(false, 1000, 1000));
  CHECK(squatBackoffActive(true, 1000, 1000));
  CHECK(squatBackoffActive(true, 1000, 1000 + kSquatBackoffMs - 1));
  CHECK_FALSE(squatBackoffActive(true, 1000, 1000 + kSquatBackoffMs));
  CHECK(squatBackoffActive(true, 0xfffffff0u, 100));
  CHECK_FALSE(squatBackoffActive(true, 0xfffffff0u, 0xfffffff0u + kSquatBackoffMs));
}

TEST_CASE("the camera always wins") {
  bool latch = false;
  CameraInputs idle;
  CHECK_FALSE(cameraWantsRadio(idle, latch));

  CameraInputs c = idle;
  c.fsmIdleOrUnpaired = false;  // waking / recording / watching / pairing
  CHECK(cameraWantsRadio(c, latch));
  c = idle;
  c.testPageOpen = true;
  CHECK(cameraWantsRadio(c, latch));
  c = idle;
  c.ownsRadio = true;
  CHECK(cameraWantsRadio(c, latch));

  // A paired camera's engine above the wake threshold claims the radio
  // before the camera FSM's own debounce does.
  c = idle;
  c.paired = true;
  c.rpm = kCameraWakeRpm;
  CHECK_FALSE(cameraWantsRadio(c, latch));
  c.rpm = kCameraWakeRpm + 1;
  CHECK(cameraWantsRadio(c, latch));
  CHECK(latch);

  // No paired camera: the engine starting doesn't matter.
  latch = false;
  c = idle;
  c.rpm = 9000;
  CHECK_FALSE(cameraWantsRadio(c, latch));
  CHECK_FALSE(latch);
}

TEST_CASE("the engine claim holds until the engine is really off") {
  bool latch = false;
  CameraInputs c;
  c.paired = true;
  // A pull-start: cranking blips over 500 and sags back between pulls.
  c.rpm = 450;
  CHECK_FALSE(cameraWantsRadio(c, latch));
  c.rpm = 620;
  CHECK(cameraWantsRadio(c, latch));
  const uint16_t band[] = {480, 350, 300, 499, 450};
  for (uint16_t rpm : band) {
    c.rpm = rpm;
    CHECK(cameraWantsRadio(c, latch));  // no flapping between 300 and 500
  }
  c.rpm = kCameraReleaseRpm - 1;
  CHECK_FALSE(cameraWantsRadio(c, latch));
  CHECK_FALSE(latch);
  c.rpm = 450;  // back in the band, but not latched: stays released
  CHECK_FALSE(cameraWantsRadio(c, latch));

  // Unpairing releases a latched claim at once.
  c.rpm = 900;
  CHECK(cameraWantsRadio(c, latch));
  c.paired = false;
  c.rpm = 450;
  CHECK_FALSE(cameraWantsRadio(c, latch));
  CHECK_FALSE(latch);
}

TEST_CASE("a remote session ends on the engine, the camera, or idleness") {
  RemoteSessionInputs in;
  in.lastRequestMs = 1000;
  in.nowMs = 2000;
  CHECK(remoteSessionEnd(in) == RemoteEnd::kNone);

  // Engine, with or without a camera — and ahead of it.
  RemoteSessionInputs e = in;
  e.rpm = kEngineRunningRpm;
  CHECK(remoteSessionEnd(e) == RemoteEnd::kNone);
  e.rpm = kEngineRunningRpm + 1;
  CHECK(remoteSessionEnd(e) == RemoteEnd::kEngine);
  e.cameraWantsRadio = true;
  CHECK(remoteSessionEnd(e) == RemoteEnd::kEngine);
  e.busy = true;  // a download in flight doesn't hold off the engine
  CHECK(remoteSessionEnd(e) == RemoteEnd::kEngine);

  RemoteSessionInputs c = in;
  c.cameraWantsRadio = true;
  c.busy = true;
  CHECK(remoteSessionEnd(c) == RemoteEnd::kCamera);

  // Idle: 10 minutes with no request, unless something is streaming.
  RemoteSessionInputs i = in;
  i.nowMs = in.lastRequestMs + kRemoteIdleMs - 1;
  CHECK(remoteSessionEnd(i) == RemoteEnd::kNone);
  i.nowMs = in.lastRequestMs + kRemoteIdleMs;
  CHECK(remoteSessionEnd(i) == RemoteEnd::kIdle);
  i.busy = true;
  CHECK(remoteSessionEnd(i) == RemoteEnd::kNone);
  // Wrap-safe.
  i.busy = false;
  i.lastRequestMs = 0xffffff00u;
  i.nowMs = 0xffffff00u + kRemoteIdleMs;
  CHECK(remoteSessionEnd(i) == RemoteEnd::kIdle);
  i.nowMs = 100;
  CHECK(remoteSessionEnd(i) == RemoteEnd::kNone);

  // An accepted OTA apply owns the exit, whatever else is going on.
  RemoteSessionInputs o = e;
  o.otaApplyPending = true;
  o.nowMs = o.lastRequestMs + kRemoteIdleMs;
  CHECK(remoteSessionEnd(o) == RemoteEnd::kNone);
}

TEST_CASE("remote end tokens") {
  CHECK(remoteEndToken(RemoteEnd::kNone) == nullptr);
  CHECK(std::strcmp(remoteEndToken(RemoteEnd::kEngine), "AUTH:ENGINE") == 0);
  CHECK(std::strcmp(remoteEndToken(RemoteEnd::kCamera), "AUTH:CAMERA") == 0);
  CHECK(std::strcmp(remoteEndToken(RemoteEnd::kIdle), "AUTH:IDLE") == 0);
}

TEST_CASE("auth timeout is 20 s and wrap-safe") {
  CHECK_FALSE(authTimedOut(1000, 20999));
  CHECK(authTimedOut(1000, 21000));
  CHECK_FALSE(authTimedOut(0xffffff00u, 100));
  CHECK(authTimedOut(0xffffff00u, 0xffffff00u + kAuthTimeoutMs));
}
