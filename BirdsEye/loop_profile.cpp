#include "loop_profile.h"

namespace loop_profile {
namespace {

// Saturating add. See the header: a pegged counter is an honest "we
// stopped being able to count"; a wrapped one reads as idle.
inline uint32_t addSat(uint32_t a, uint32_t b) {
  return (a > UINT32_MAX - b) ? UINT32_MAX : (uint32_t)(a + b);
}

const char* const kTags[kSectionCount] = {"GPS", "TCH", "ACC", "BLE",
                                          "EGG", "TRK", "LAP", "IDL",
                                          "CAM", "LED", "BTN", "DSP"};

}  // namespace

const char* sectionTag(uint8_t section) {
  if (section == kOther) return "OTH";
  if (section == kSleep) return "SLP";
  if (section >= kSectionCount) return "???";
  return kTags[section];
}

uint16_t permilleOf(uint32_t part, uint32_t whole) {
  if (whole == 0) return 0;
  const uint64_t p = ((uint64_t)part * 1000ULL) / (uint64_t)whole;
  return (p > 1000ULL) ? (uint16_t)1000 : (uint16_t)p;
}

void reset(State& s, uint32_t nowTicks, uint32_t nowMs) {
  for (uint8_t i = 0; i < kSectionCount; i++) {
    s.sec[i].totalTicks = 0;
    s.sec[i].maxTicks = 0;
    s.sec[i].calls = 0;
  }
  s.loops = 0;
  s.loopTotalTicks = 0;
  s.loopMaxTicks = 0;
  s.windowStartTicks = nowTicks;
  s.windowStartMs = nowMs;
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

bool rollup(State& s, uint32_t nowTicks, uint32_t nowMs, uint32_t windowMs,
            uint32_t ticksPerUs, Report& out) {
  // WALL clock closes the window. Using the duration clock here was the
  // original bug: the DWT cycle counter stops when the core halts, so a
  // "one second" window was one second of CPU-awake time and every rate
  // and share downstream was inflated by the sleep factor.
  const uint32_t elapsedMs = nowMs - s.windowStartMs;  // wrap-safe
  if (elapsedMs < windowMs) return false;
  if (ticksPerUs == 0) ticksPerUs = 1;  // mis-probed timebase: keep ratios

  out.valid = true;
  // Cap before the *1000: a window longer than ~71 minutes would
  // overflow, and a bogus huge window must not divide by a wrapped
  // denominator. Nothing legitimate comes near it.
  out.windowUs =
      (elapsedMs > 4000000u) ? 4000000000u : (uint32_t)(elapsedMs * 1000u);
  out.awakeUs = ticksSince(s.windowStartTicks, nowTicks) / ticksPerUs;
  out.saturated = (s.loopTotalTicks == UINT32_MAX);

  // Shares are of WALL time, which is what a scope on the profiling pin
  // measures too. Totals are divided down to microseconds first: the
  // sub-microsecond precision that ticks buy matters per CALL, not on an
  // accumulated total, and whole microseconds keep every denominator
  // inside 32 bits.
  uint32_t sectionSumUs = 0;
  for (uint8_t i = 0; i < kSectionCount; i++) {
    const SectionStat& st = s.sec[i];
    const uint32_t us = st.totalTicks / ticksPerUs;
    out.permille[i] = permilleOf(us, out.windowUs);
    out.maxUs[i] = st.maxTicks / ticksPerUs;
    sectionSumUs = addSat(sectionSumUs, us);
    if (st.totalTicks == UINT32_MAX) out.saturated = true;
  }

  const uint32_t loopUs = s.loopTotalTicks / ticksPerUs;
  out.busyPermille = permilleOf(loopUs, out.windowUs);

  // Loop time that no section bracketed: the glue between the calls
  // (session-end checks, idle timers, the shutdown combos). Clamped at
  // zero because the whole-iteration span and the section spans come
  // from separate reads and can disagree by a tick.
  const uint32_t otherUs = (loopUs > sectionSumUs) ? (loopUs - sectionSumUs) : 0;
  out.permille[kOther] = permilleOf(otherUs, out.windowUs);

  // Everything else in the window: the core's loop dispatch, other
  // FreeRTOS tasks, and any time the CPU spent asleep. Derived from
  // busy rather than from a tick subtraction precisely BECAUSE the tick
  // counter is the thing that may have stopped.
  out.permille[kSleep] =
      (out.busyPermille >= 1000) ? 0 : (uint16_t)(1000 - out.busyPermille);

  out.loops = s.loops;
  out.loopMaxUs = s.loopMaxTicks / ticksPerUs;
  out.loopMeanUs =
      (s.loops > 0) ? ((s.loopTotalTicks / s.loops) / ticksPerUs) : 0;
  // Iterations per WALL second.
  out.loopRateHz =
      (uint32_t)(((uint64_t)s.loops * 1000ULL + (uint64_t)(elapsedMs / 2)) /
                 (uint64_t)elapsedMs);

  reset(s, nowTicks, nowMs);
  return true;
}

}  // namespace loop_profile
