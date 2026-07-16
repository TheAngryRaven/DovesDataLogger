#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// TinyUSB shim — sim build.
// usb_msc.ino is NOT compiled into the sim; BirdsEye.ino only includes
// this header (nothing in the compiled sources references the types).
///////////////////////////////////////////
