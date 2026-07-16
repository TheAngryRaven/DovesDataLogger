#pragma once

#include <stdint.h>

///////////////////////////////////////////
// VIRTUAL CLOCK (sim)
//
// The simulator has NO internal clock — the host owns virtual time
// (see the API contract in the simulator handoff spec). millis() /
// micros() / delay() in the Arduino shim all resolve against this
// counter. delay()/delayMicroseconds() ADVANCE it (firmware blocking
// waits consume virtual time instead of hanging), and the host driver
// advances it between loop() iterations via sim_clock::advanceUs().
//
// No wall clock anywhere: two runs with the same inputs see identical
// timestamps, which is what makes the sim deterministic.
///////////////////////////////////////////

namespace sim_clock {

// Current virtual time in microseconds since sim boot.
uint64_t nowUs();

// Advance virtual time (host driver + shim delay()).
void advanceUs(uint64_t us);

// Reset to zero (fresh boot).
void reset();

}  // namespace sim_clock
