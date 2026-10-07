#include "remote_auth.h"

#include <string.h>

#include "sha256.h"

namespace remote_auth {

namespace {

const char kHexDigits[] = "0123456789abcdef";

inline char lowerAscii(char c) { return (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c; }

inline bool isHex(char c) {
  c = lowerAscii(c);
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
}

bool startsWith(const char* s, const char* prefix) {
  return strncmp(s, prefix, strlen(prefix)) == 0;
}

}  // namespace

bool isValidPin(const char* pin) {
  if (pin == nullptr) return false;
  for (size_t i = 0; i < kPinLen; ++i) {
    if (pin[i] < '0' || pin[i] > '9') return false;
  }
  return pin[kPinLen] == '\0';
}

void pinFromRandom(uint32_t r, char* out) {
  uint32_t v = 1000u + (r % 9000u);
  for (int i = int(kPinLen) - 1; i >= 0; --i) {
    out[i] = char('0' + (v % 10u));
    v /= 10u;
  }
  out[kPinLen] = '\0';
}

void toHex(const uint8_t* bytes, size_t len, char* out) {
  for (size_t i = 0; i < len; ++i) {
    out[i * 2] = kHexDigits[bytes[i] >> 4];
    out[i * 2 + 1] = kHexDigits[bytes[i] & 0x0f];
  }
  out[len * 2] = '\0';
}

void computeAnswer(const char* pin, const char* nonceHex, const char* deviceName,
                   char* out) {
  // "BEAUTH1|" + 32 + "|" + name. Names are capped well below this by the
  // settings and advert limits; truncating would only produce a mismatch.
  char msg[8 + kNonceHexLen + 1 + 64 + 1];
  size_t n = 0;
  const char* parts[] = {"BEAUTH1|", nonceHex, "|", deviceName};
  for (const char* part : parts) {
    for (size_t i = 0; part[i] != '\0' && n < sizeof(msg) - 1; ++i) msg[n++] = part[i];
  }
  msg[n] = '\0';

  uint8_t mac[sha256::kDigestLen];
  sha256::hmac(pin, strlen(pin), msg, n, mac);
  toHex(mac, kAnswerHexLen / 2, out);
}

bool hexEqualConstTime(const char* a, const char* b, size_t n) {
  if (a == nullptr || b == nullptr) return false;
  if (strlen(a) != n || strlen(b) != n) return false;
  uint8_t diff = 0;
  for (size_t i = 0; i < n; ++i) {
    diff |= uint8_t(lowerAscii(a[i]) ^ lowerAscii(b[i]));
  }
  return diff == 0;
}

uint32_t lockoutLengthMs(uint8_t lockoutsSoFar) {
  uint32_t len = kLockoutBaseMs;
  for (uint8_t i = 0; i < lockoutsSoFar && len < kLockoutMaxMs; ++i) len *= 2u;
  return len > kLockoutMaxMs ? kLockoutMaxMs : len;
}

bool isLocked(State& s, uint32_t nowMs) {
  if (!s.lockActive) return false;
  if (uint32_t(nowMs - s.lockStartMs) >= s.lockLenMs) {
    s.lockActive = false;
    s.fails = 0;
    return false;
  }
  return true;
}

uint32_t lockSecondsLeft(State& s, uint32_t nowMs) {
  if (!isLocked(s, nowMs)) return 0;
  const uint32_t left = s.lockLenMs - uint32_t(nowMs - s.lockStartMs);
  return (left + 999u) / 1000u;
}

void issueNonce(State& s, const uint8_t* bytes) {
  toHex(bytes, kNonceLen, s.nonceHex);
  s.nonceValid = true;
}

Verdict checkAnswer(State& s, const char* answerHex, const char* pin,
                    const char* deviceName, uint32_t nowMs) {
  if (isLocked(s, nowMs)) {
    s.nonceValid = false;
    return Verdict::kLocked;
  }
  if (!s.nonceValid) return Verdict::kNoNonce;

  // Single use, right or wrong.
  s.nonceValid = false;

  bool ok = false;
  if (isValidPin(pin) && answerHex != nullptr && strlen(answerHex) == kAnswerHexLen) {
    bool wellFormed = true;
    for (size_t i = 0; i < kAnswerHexLen; ++i) wellFormed = wellFormed && isHex(answerHex[i]);
    if (wellFormed) {
      char expected[kAnswerHexLen + 1];
      computeAnswer(pin, s.nonceHex, deviceName, expected);
      ok = hexEqualConstTime(answerHex, expected, kAnswerHexLen);
    }
  }
  memset(s.nonceHex, 0, sizeof(s.nonceHex));

  if (ok) {
    s.authed = true;
    s.fails = 0;
    return Verdict::kOk;
  }

  if (s.fails < kMaxFails) ++s.fails;
  if (s.fails >= kMaxFails) {
    s.lockActive = true;
    s.lockStartMs = nowMs;
    s.lockLenMs = lockoutLengthMs(s.lockouts);
    if (s.lockouts < 0xff) ++s.lockouts;
    return Verdict::kLocked;
  }
  return Verdict::kFail;
}

void onDisconnect(State& s) {
  s.authed = false;
  s.nonceValid = false;
  memset(s.nonceHex, 0, sizeof(s.nonceHex));
}

bool commandAllowed(Mode mode, bool authed, const char* cmd) {
  if (cmd == nullptr) return false;
  const bool isPinGet = strcmp(cmd, "PINGET") == 0;
  if (mode == Mode::kOpen) return true;
  if (isPinGet) return false;
  if (authed) return true;
  return strcmp(cmd, "AUTH?") == 0 || startsWith(cmd, "AUTH:") || strcmp(cmd, "BATT") == 0;
}

bool isProtectedSettingKey(const char* key) {
  return key != nullptr && strcmp(key, "bluetooth_pin") == 0;
}

bool standbyWanted(const StandbyInputs& in) {
  return in.settingEnabled && in.onMainMenu && in.radioFree && !in.cameraBusy &&
         !in.raceActive;
}

bool cameraBusy(bool fsmIdleOrUnpaired, bool testPageOpen, bool ownsRadio) {
  return !fsmIdleOrUnpaired || testPageOpen || ownsRadio;
}

bool cameraEndsRemoteSession(bool cameraBusyNow, bool cameraPaired, uint16_t rpm) {
  return cameraBusyNow || (cameraPaired && rpm > kCameraWakeRpm);
}

}  // namespace remote_auth
