#include "sat_bars.h"

namespace sat_bars {

namespace {
constexpr int kBarGapPx = 1;
constexpr int kBarMinWidthPx = 2;
// UBX-NAV-SAT can report more blocks than we could ever draw; bound the
// scratch "already picked" mask.
constexpr int kMaxObs = 64;
}  // namespace

int selectCnos(const SatObs* obs, int n, uint8_t* outCno, int cap) {
  if (obs == nullptr || outCno == nullptr || n <= 0 || cap <= 0) return 0;
  if (n > kMaxObs) n = kMaxObs;

  bool taken[kMaxObs] = {};
  int count = 0;
  // Two passes: satellites used in the nav solution first, then the
  // rest. Within each pass, repeatedly pick the strongest remaining —
  // n is at most a few dozen SVs, so O(n * cap) is trivial and avoids
  // <algorithm> on the target.
  for (int pass = 0; pass < 2 && count < cap; pass++) {
    const bool wantUsed = (pass == 0);
    while (count < cap) {
      int best = -1;
      for (int i = 0; i < n; i++) {
        if (taken[i] || obs[i].used != wantUsed || obs[i].cno == 0) continue;
        if (best < 0 || obs[i].cno > obs[best].cno) best = i;
      }
      if (best < 0) break;  // pass exhausted
      taken[best] = true;
      outCno[count++] = obs[best].cno;
    }
  }
  return count;
}

int layout(const uint8_t* cno, int count, int areaW, int areaH, Bar* out,
           int maxBars) {
  if (cno == nullptr || out == nullptr) return 0;
  if (count <= 0 || areaW <= 0 || areaH <= 0 || maxBars <= 0) return 0;
  if (count > maxBars) count = maxBars;

  // Widest equal bars that fit with 1 px gaps; shed bars if even
  // minimum-width bars would overflow the area.
  int w = (areaW - (count - 1) * kBarGapPx) / count;
  while (w < kBarMinWidthPx && count > 1) {
    count--;
    w = (areaW - (count - 1) * kBarGapPx) / count;
  }
  if (w < 1) return 0;

  for (int i = 0; i < count; i++) {
    int c = cno[i];
    if (c > kCnoCeiling) c = kCnoCeiling;
    int h = (c * areaH) / kCnoCeiling;
    out[i].x = static_cast<int16_t>(i * (w + kBarGapPx));
    out[i].w = static_cast<int16_t>(w);
    out[i].h = static_cast<int16_t>(h);
  }
  return count;
}

}  // namespace sat_bars
