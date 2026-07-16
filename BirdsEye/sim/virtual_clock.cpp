#include "virtual_clock.h"

namespace sim_clock {

static uint64_t g_nowUs = 0;

uint64_t nowUs() { return g_nowUs; }

void advanceUs(uint64_t us) { g_nowUs += us; }

void reset() { g_nowUs = 0; }

}  // namespace sim_clock
