#include "sensoregg_protocol.h"

#include <math.h>
#include <string.h>

namespace sensoregg_protocol {

namespace {

// Decode a little-endian int16 temperature field to degC, mapping the
// invalid sentinel to NaN. The uint16->int16 round trip is well-defined
// (two's complement) via the explicit cast.
float decodeDeciC(uint8_t lo, uint8_t hi) {
  const int16_t raw = (int16_t)((uint16_t)lo | ((uint16_t)hi << 8));
  if (raw == kInvalidSentinel) {
    return NAN;
  }
  return (float)raw / 10.0f;
}

int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

}  // namespace

bool matchesMagic(const uint8_t* data, size_t len) {
  if (data == nullptr || len < sizeof(kMagic)) {
    return false;
  }
  for (size_t i = 0; i < sizeof(kMagic); i++) {
    if (data[i] != kMagic[i]) {
      return false;
    }
  }
  return true;
}

bool parsePayload(const uint8_t* data, size_t len, Reading& out) {
  if (data == nullptr || len < kPayloadLen) {
    return false;
  }
  if (!matchesMagic(data, len)) {
    return false;
  }
  const uint8_t ver = data[4];
  if (ver != kProtocolVersion && ver != kProtocolVersionV2) {
    return false;
  }
  // A version's own length gate: a v2 frame truncated to 14-15 bytes is
  // corrupt, not a v1 frame — reject rather than mis-parse.
  if (ver == kProtocolVersionV2 && len < kPayloadLenV2) {
    return false;
  }

  out.flags = data[5];
  out.pairingActive = (out.flags & 0x01) != 0;
  out.tcFault = (out.flags & 0x02) != 0;
  out.egtC = decodeDeciC(data[6], data[7]);
  out.junctionC = decodeDeciC(data[8], data[9]);
  out.status = data[10];
  out.battery = data[11];
  out.sequence = (uint16_t)((uint16_t)data[12] | ((uint16_t)data[13] << 8));
  out.auxC = (ver >= kProtocolVersionV2) ? decodeDeciC(data[14], data[15])
                                         : NAN;
  out.protoVersion = ver;
  return true;
}

bool isFresh(uint32_t receivedAtMs, uint32_t nowMs) {
  // Unsigned subtraction is wrap-safe across the millis() rollover.
  return (uint32_t)(nowMs - receivedAtMs) < kStalenessMs;
}

float celsiusToFahrenheit(float c) {
  return c * 9.0f / 5.0f + 32.0f;
}

void seqMonitorFeed(SeqMonitor& m, uint16_t seq, uint32_t nowMs) {
  if (!m.haveSeq || seq != m.lastSeq) {
    m.lastSeq = seq;
    m.lastChangeMs = nowMs;
    m.haveSeq = true;
  }
}

bool seqMonitorLive(const SeqMonitor& m, uint32_t nowMs) {
  return m.haveSeq && isFresh(m.lastChangeMs, nowMs);
}

bool parseMac(const char* s, uint8_t outHuman[6]) {
  if (s == nullptr) {
    return false;
  }
  uint8_t tmp[6];
  size_t i = 0;
  for (int b = 0; b < 6; b++) {
    // A NUL at s[i] fails the first nibble, so s[i+1] is never read past
    // the terminator.
    const int hi = hexNibble(s[i]);
    if (hi < 0) return false;
    const int lo = hexNibble(s[i + 1]);
    if (lo < 0) return false;
    tmp[b] = (uint8_t)((hi << 4) | lo);
    i += 2;
    if (b < 5) {
      if (s[i] != ':') return false;
      i++;
    }
  }
  if (s[i] != '\0') return false;  // trailing garbage / over-length
  memcpy(outHuman, tmp, 6);
  return true;
}

void formatMac(const uint8_t human[6], char out[kMacStrLen]) {
  static const char kHex[] = "0123456789ABCDEF";
  size_t o = 0;
  for (int b = 0; b < 6; b++) {
    out[o++] = kHex[human[b] >> 4];
    out[o++] = kHex[human[b] & 0x0F];
    if (b < 5) out[o++] = ':';
  }
  out[o] = '\0';
}

void macReverse(const uint8_t in[6], uint8_t out[6]) {
  uint8_t tmp[6];
  for (int i = 0; i < 6; i++) tmp[i] = in[5 - i];
  memcpy(out, tmp, 6);
}

bool macIsWildcard(const uint8_t mac[6]) {
  for (int i = 0; i < 6; i++) {
    if (mac[i] != 0x00) return false;
  }
  return true;
}

bool macAccepts(const uint8_t filterHuman[6], const uint8_t peerLsbFirst[6]) {
  if (macIsWildcard(filterHuman)) return true;
  for (int i = 0; i < 6; i++) {
    if (peerLsbFirst[i] != filterHuman[5 - i]) return false;
  }
  return true;
}

}  // namespace sensoregg_protocol
