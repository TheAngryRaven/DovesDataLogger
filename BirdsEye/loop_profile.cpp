#include "loop_profile.h"

namespace loop_profile {
namespace {

// Saturating add. See the header: a pegged counter is an honest "we
// stopped being able to count"; a wrapped one reads as idle.
inline uint32_t addSat(uint32_t a, uint32_t b) {
  return (a > UINT32_MAX - b) ? UINT32_MAX : (uint32_t)(a + b);
}

const char* const kTags[kSectionCount] = {
    "GPS", "TCH", "ACC", "BLE", "EGG", "TRK", "LAP",
    "IDL", "CAM", "LED", "BTN", "PGE", "DSP"};

}  // namespace

const char* sectionTag(uint8_t section) {
  if (section == kOther) return "OTH";
  if (section >= kSectionCount) return "???";
  return kTags[section];
}

uint16_t permilleOf(uint32_t part, uint32_t whole) {
  if (whole == 0) return 0;
  const uint64_t p = ((uint64_t)part * 1000ULL) / (uint64_t)whole;
  return (p > 1000ULL) ? (uint16_t)1000 : (uint16_t)p;
}

void reset(State& s, uint32_t nowTicks) {
  for (uint8_t i = 0; i < kSectionCount; i++) {
    s.sec[i].totalTicks = 0;
    s.sec[i].maxTicks = 0;
    s.sec[i].calls = 0;
  }
  s.loops = 0;
  s.loopTotalTicks = 0;
  s.loopMaxTicks = 0;
  s.windowStartTicks = nowTicks;
}

void addSection(State& s, uint8_t section, uint32_t elapsedTicks) {
  if (section >= kSectionCount) return;
  SectionStat& st = s.sec[section];
  st.totalTicks = addSat(st.totalTicks, elapsedTicks);
  st.calls = addSat(st.calls, 1);
  if (elapsedTicks > st.maxTicks) st.maxTicks = elapsedTicks;
}

void addLoop(State& s, uint32_t elapsedTicks) {
  s.loopTotalTicks = addSat(s.loopTotalTicks, elapsedTicks);
  s.loops = addSat(s.loops, 1);
  if (elapsedTicks > s.loopMaxTicks) s.loopMaxTicks = elapsedTicks;
}

bool rollup(State& s, uint32_t nowTicks, uint32_t windowTicks,
            uint32_t ticksPerUs, Report& out) {
  const uint32_t elapsed = ticksSince(s.windowStartTicks, nowTicks);
  if (elapsed < windowTicks) return false;
  if (ticksPerUs == 0) ticksPerUs = 1;  // mis-probed timebase: keep ratios

  out.valid = true;
  out.windowUs = elapsed / ticksPerUs;
  out.saturated = (s.loopTotalTicks == UINT32_MAX);

  // Section shares are of WINDOW wall time, not of loop time, so a
  // section's number means "this fraction of the last second", which is
  // what a scope on the profiling pin measures too.
  uint32_t sectionSum = 0;
  for (uint8_t i = 0; i < kSectionCount; i++) {
    const SectionStat& st = s.sec[i];
    out.permille[i] = permilleOf(st.totalTicks, elapsed);
    out.maxUs[i] = st.maxTicks / ticksPerUs;
    sectionSum = addSat(sectionSum, st.totalTicks);
    if (st.totalTicks == UINT32_MAX) out.saturated = true;
  }

  // Loop time that no section bracketed: the glue between the calls
  // (button-hold checks, idle timers, the shutdown combos). Clamped at
  // zero because the whole-iteration span and the section spans come
  // from separate reads and can disagree by a tick.
  const uint32_t otherTicks =
      (s.loopTotalTicks > sectionSum) ? (s.loopTotalTicks - sectionSum) : 0;
  out.permille[kOther] = permilleOf(otherTicks, elapsed);

  // Window time outside loop() entirely. In this firmware that is the
  // Arduino core's loop dispatch and nothing else, so it should read
  // ~0; anything bigger means work is happening where loop() cannot
  // see it.
  const uint32_t outside =
      (elapsed > s.loopTotalTicks) ? (elapsed - s.loopTotalTicks) : 0;
  out.outsideLoopPermille = permilleOf(outside, elapsed);

  out.loops = s.loops;
  out.loopMaxUs = s.loopMaxTicks / ticksPerUs;
  if (s.loops > 0) {
    out.loopMeanUs = (s.loopTotalTicks / s.loops) / ticksPerUs;
  } else {
    out.loopMeanUs = 0;
  }
  if (out.windowUs > 0) {
    out.loopRateHz = (uint32_t)(((uint64_t)s.loops * 1000000ULL +
                                 (uint64_t)(out.windowUs / 2)) /
                                (uint64_t)out.windowUs);
  } else {
    out.loopRateHz = 0;
  }

  reset(s, nowTicks);
  return true;
}

}  // namespace loop_profile
