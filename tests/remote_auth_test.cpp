#include "doctest.h"

#include <cstring>

#include "remote_auth.h"

using namespace remote_auth;

namespace {

const uint8_t kNonceA[kNonceLen] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};

// The answer for kNonceA / "4821" / "DovesDataLogger-042". The SAME vector is
// pinned in DovesDataViewer (src/lib/ble/auth.test.ts) and LapWing
// (loggers/doveslogger/auth.rs) — change all three together or none.
const char kGoldenAnswer[] = "2cc61d4ff111ba3236b50217443c3daa";

void answerFor(const State& s, const char* pin, char* out) {
  computeAnswer(pin, s.nonceHex, "DovesDataLogger-042", out);
}

}  // namespace

TEST_CASE("answer matches the cross-implementation golden vectors") {
  char out[kAnswerHexLen + 1];
  computeAnswer("4821", "00112233445566778899aabbccddeeff", "DovesDataLogger-042", out);
  CHECK(std::strcmp(out, kGoldenAnswer) == 0);
  computeAnswer("1000", "ffffffffffffffffffffffffffffffff", "BirdsEye", out);
  CHECK(std::strcmp(out, "1918b8ddb0591dd12fd31eb2ef04ba33") == 0);
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
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", "DovesDataLogger-042", 0) == Verdict::kOk);
  CHECK(s.authed);
  // Replay of the same answer: the nonce is gone.
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", "DovesDataLogger-042", 0) == Verdict::kNoNonce);
}

TEST_CASE("uppercase answer is accepted") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "2CC61D4FF111BA3236B50217443C3DAA", "4821", "DovesDataLogger-042", 0) ==
        Verdict::kOk);
}

TEST_CASE("answer is bound to the device name") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, kGoldenAnswer, "4821", "SomeOtherLogger", 0) == Verdict::kFail);
}

TEST_CASE("wrong answer consumes the nonce and counts down") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", "x", 0) == Verdict::kFail);
  CHECK(triesLeft(s) == kMaxFails - 1);
  CHECK_FALSE(s.nonceValid);
  CHECK_FALSE(s.authed);
}

TEST_CASE("malformed answers count as wrong") {
  State s;
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "short", "4821", "x", 0) == Verdict::kFail);
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "zzc61d4ff111ba3236b50217443c3daa", "4821", "x", 0) == Verdict::kFail);
  CHECK(triesLeft(s) == kMaxFails - 2);
}

TEST_CASE("a stored PIN that is not 4 digits never authenticates") {
  State s;
  issueNonce(s, kNonceA);
  char ans[kAnswerHexLen + 1];
  computeAnswer("", s.nonceHex, "x", ans);
  CHECK(checkAnswer(s, ans, "", "x", 0) == Verdict::kFail);
}

TEST_CASE("five wrong answers lock for 60 s, then doubling to 15 min") {
  State s;
  uint32_t now = 1000;
  for (uint8_t i = 0; i + 1 < kMaxFails; ++i) {
    issueNonce(s, kNonceA);
    CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", "x", now) == Verdict::kFail);
  }
  issueNonce(s, kNonceA);
  CHECK(checkAnswer(s, "00000000000000000000000000000000", "4821", "x", now) == Verdict::kLocked);
  CHECK(isLocked(s, now));
  CHECK(lockSecondsLeft(s, now) == 60);
  CHECK(lockSecondsLeft(s, now + 59001) == 1);

  // Even the right answer is refused while locked, and the nonce is burnt.
  issueNonce(s, kNonceA);
  char ans[kAnswerHexLen + 1];
  answerFor(s, "4821", ans);
  CHECK(checkAnswer(s, ans, "4821", "DovesDataLogger-042", now + 1000) == Verdict::kLocked);
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
    checkAnswer(s, "00000000000000000000000000000000", "4821", "x", now);
  }
  CHECK(isLocked(s, now + 30000u));  // wrapped past zero, still inside 60 s
  CHECK_FALSE(isLocked(s, now + 60000u));
}

TEST_CASE("disconnect clears auth but not the lockout") {
  State s;
  issueNonce(s, kNonceA);
  checkAnswer(s, kGoldenAnswer, "4821", "DovesDataLogger-042", 0);
  onDisconnect(s);
  CHECK_FALSE(s.authed);
  CHECK_FALSE(s.nonceValid);

  for (uint8_t i = 0; i < kMaxFails; ++i) {
    issueNonce(s, kNonceA);
    checkAnswer(s, "00000000000000000000000000000000", "4821", "x", 0);
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
  off.cameraBusy = true;
  CHECK_FALSE(standbyWanted(off));
  off = in;
  off.raceActive = true;
  CHECK_FALSE(standbyWanted(off));
}

TEST_CASE("the camera always wins") {
  CHECK_FALSE(cameraBusy(true, false, false));
  CHECK(cameraBusy(false, false, false));  // waking / recording / watching / pairing
  CHECK(cameraBusy(true, true, false));    // bench page open
  CHECK(cameraBusy(true, false, true));    // owns the radio

  CHECK(cameraEndsRemoteSession(true, false, 0));
  CHECK_FALSE(cameraEndsRemoteSession(false, true, kCameraWakeRpm));
  CHECK(cameraEndsRemoteSession(false, true, kCameraWakeRpm + 1));
  // No paired camera: the engine starting doesn't matter.
  CHECK_FALSE(cameraEndsRemoteSession(false, false, 9000));
}

TEST_CASE("auth timeout is 20 s and wrap-safe") {
  CHECK_FALSE(authTimedOut(1000, 20999));
  CHECK(authTimedOut(1000, 21000));
  CHECK_FALSE(authTimedOut(0xffffff00u, 100));
  CHECK(authTimedOut(0xffffff00u, 0xffffff00u + kAuthTimeoutMs));
}
