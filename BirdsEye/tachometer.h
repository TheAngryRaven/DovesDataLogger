#pragma once

///////////////////////////////////////////
// TACHOMETER MODULE
// Falling-edge ISR on D0 timestamps every pulse into a 16-entry ring
// buffer. TACH_LOOP() drains the buffer, computes mean inter-pulse
// period, and feeds the result through a 1D Kalman filter. The
// filtered RPM is published in tachLastReported.
///////////////////////////////////////////

#include <stdint.h>

// Latest filtered RPM (rounded). Written by TACH_LOOP, read by
// display/logging/sleep — volatile so cross-context reads are coherent.
extern volatile int tachLastReported;

// Set true by the ISR on any valid pulse — sleep mode uses this as a
// wake trigger; main loop should NOT clear it (ISR owns it).
extern volatile bool tachHavePeriod;

// ISR — must have C-style linkage for attachInterrupt().
void TACH_COUNT_PULSE();

// Drain ring buffer, update Kalman estimate, apply engine-stop timeout.
// Call once per main-loop iteration (~250 Hz).
void TACH_LOOP();
