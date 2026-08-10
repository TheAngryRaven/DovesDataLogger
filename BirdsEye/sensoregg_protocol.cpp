#include "sensoregg_protocol.h"

#include <math.h>

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

}  // namespace sensoregg_protocol
