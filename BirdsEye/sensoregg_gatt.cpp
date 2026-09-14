#include "sensoregg_gatt.h"

#include <math.h>
#include <string.h>

namespace sensoregg_gatt {

namespace {

uint16_t getU16(const uint8_t* p) {
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t getU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

float getF32(const uint8_t* p) {
  // Host and target are both little-endian IEEE-754; memcpy avoids the
  // type-punning UB.
  float f;
  memcpy(&f, p, 4);
  return f;
}

}  // namespace

bool parseDescriptor(const uint8_t* d, size_t len, PodDescriptor& out) {
  if (d == nullptr || len < kDescHeaderLen) return false;
  const uint8_t schemaVersion = d[0];
  const uint8_t channelCount = d[4];
  const uint8_t recordLen = d[5];
  if (schemaVersion != 1) return false;
  if (recordLen < kMinRecordLen) return false;
  if (channelCount == 0 || channelCount > kMaxChannels) return false;
  if (len < kDescHeaderLen + (size_t)channelCount * recordLen) return false;

  out.schemaVersion = schemaVersion;
  out.deviceType = d[1];
  out.fwMajor = d[2];
  out.fwMinor = d[3];
  out.channelCount = channelCount;
  out.recordLen = recordLen;
  for (uint8_t i = 0; i < channelCount; i++) {
    const uint8_t* r = d + kDescHeaderLen + (size_t)i * recordLen;
    ChannelInfo& c = out.ch[i];
    c.id = r[0];
    c.quantity = r[1];
    c.periodMs = getU16(&r[2]);
    c.scale = getF32(&r[4]);
    c.offset = getF32(&r[8]);
    memcpy(c.name, &r[12], 8);
    c.name[8] = '\0';  // wire is NUL-padded, not necessarily terminated
  }
  return true;
}

bool parseSampleFrame(const uint8_t* d, size_t len, SampleFrame& out) {
  if (d == nullptr || len < kSampleHeaderLen + 2) return false;
  const uint8_t n = d[9];
  if (n == 0 || n > kMaxSamplesPerFrame) return false;
  if (len != kSampleHeaderLen + (size_t)n * 2) return false;

  out.channelId = d[0];
  out.bootId = d[1];
  out.seq = d[2];
  out.baseMs = getU32(&d[3]);
  out.intervalMs = getU16(&d[7]);
  out.n = n;
  for (uint8_t i = 0; i < n; i++) {
    out.raw[i] = (int16_t)getU16(&d[kSampleHeaderLen + (size_t)i * 2]);
  }
  return true;
}

bool parseClock(const uint8_t* d, size_t len, uint8_t& bootId,
                uint32_t& millisNow) {
  if (d == nullptr || len < kClockLen) return false;
  bootId = d[0];
  millisNow = getU32(&d[2]);
  return true;
}

float sampleToReal(int16_t raw, const ChannelInfo& c) {
  if (raw == kInvalidSentinel) return NAN;
  return (float)raw * c.scale + c.offset;
}

void mapChannels(const PodDescriptor& pd, int8_t outIdx[ROLE_COUNT]) {
  for (uint8_t r = 0; r < ROLE_COUNT; r++) outIdx[r] = -1;
  for (uint8_t i = 0; i < pd.channelCount; i++) {
    const ChannelInfo& c = pd.ch[i];
    if (c.quantity == 0x01) {  // temperature
      if (outIdx[ROLE_EGT] < 0 && strcmp(c.name, "EGT") == 0) {
        outIdx[ROLE_EGT] = (int8_t)i;
      } else if (outIdx[ROLE_CJ] < 0 && strcmp(c.name, "CJ") == 0) {
        outIdx[ROLE_CJ] = (int8_t)i;
      } else if (outIdx[ROLE_AUX] < 0 && strcmp(c.name, "IAT") == 0) {
        outIdx[ROLE_AUX] = (int8_t)i;
      }
    } else if (c.quantity == 0x08) {  // ratio -> battery percent
      if (outIdx[ROLE_BATT] < 0) outIdx[ROLE_BATT] = (int8_t)i;
    }
  }
}

int8_t fastestChannel(const PodDescriptor& pd) {
  int8_t best = 0;
  uint16_t bestPeriod = 0;
  for (uint8_t i = 0; i < pd.channelCount; i++) {
    const uint16_t p = pd.ch[i].periodMs;
    if (p == 0) continue;  // aperiodic — no heartbeat to monitor
    if (bestPeriod == 0 || p < bestPeriod) {
      bestPeriod = p;
      best = (int8_t)i;
    }
  }
  return best;
}

void clockFitAnchor(ClockFit& f, uint8_t bootId, uint32_t podMillis,
                    uint32_t reqMs, uint32_t rspMs) {
  const uint32_t rtt = (uint32_t)(rspMs - reqMs);  // wrap-safe
  f.valid = true;
  f.bootId = bootId;
  f.podMs0 = podMillis;
  f.loggerMs0 = reqMs + rtt / 2;
  f.halfRttMs = rtt / 2;
}

bool clockFitSameEpoch(const ClockFit& f, uint8_t frameBootId) {
  return f.valid && f.bootId == frameBootId;
}

uint32_t clockFitPodToLogger(const ClockFit& f, uint32_t podMs) {
  // Signed u32 delta: correct in both directions across the millis
  // wrap, for pod times within +/- ~24.8 days of the anchor.
  const int32_t delta = (int32_t)(podMs - f.podMs0);
  return f.loggerMs0 + (uint32_t)delta;
}

}  // namespace sensoregg_gatt
