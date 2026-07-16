#pragma once

// nRF watchdog register surface lives in the Arduino shim (NRF_WDT and
// the WDT_* macros) — wdtSetup() is #ifndef SIM'd out, wdtPet() writes a
// RAM struct harmlessly.
#include "Arduino.h"
