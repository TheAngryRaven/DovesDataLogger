#include "doctest.h"
#include "wake_cause.h"

#include <cstring>

using namespace wake_cause;

// XIAO nRF52840-ish masks: tach on P0.02, buttons on P0.03/P0.28/P0.29.
// The decoder only sees masks, so the exact bits are arbitrary here.
static const PinMasks kPins = {
    /*tach0=*/1u << 2,
    /*tach1=*/0,
    /*buttons0=*/(1u << 3) | (1u << 28) | (1u << 29),
    /*buttons1=*/0,
};

// A second wiring where the wake pins live on port 1, to prove both
// latch words are honored.
static const PinMasks kPinsPort1 = {
    /*tach0=*/0,
    /*tach1=*/1u << 4,
    /*buttons0=*/0,
    /*buttons1=*/(1u << 5) | (1u << 6),
};

// ---------------------------------------------------------------------------
// System OFF GPIO wakes
// ---------------------------------------------------------------------------

TEST_CASE("decode - OFF + tach latch is a tach wake") {
    CHECK(decode({kReasOff, 1u << 2, 0}, kPins) == Cause::kTachWake);
    CHECK(decode({kReasOff, 0, 1u << 4}, kPinsPort1) == Cause::kTachWake);
}

TEST_CASE("decode - OFF + button latch is a button wake") {
    CHECK(decode({kReasOff, 1u << 3, 0}, kPins) == Cause::kButtonWake);
    CHECK(decode({kReasOff, 1u << 28, 0}, kPins) == Cause::kButtonWake);
    CHECK(decode({kReasOff, 1u << 29, 0}, kPins) == Cause::kButtonWake);
    CHECK(decode({kReasOff, 0, 1u << 6}, kPinsPort1) == Cause::kButtonWake);
}

TEST_CASE("decode - tach outranks a simultaneous button latch") {
    // Engine start with a coincidental button bounce must still boot
    // toward race mode.
    CHECK(decode({kReasOff, (1u << 2) | (1u << 3), 0}, kPins) == Cause::kTachWake);
}

TEST_CASE("decode - OFF with no known pin latched is an unknown OFF wake") {
    CHECK(decode({kReasOff, 0, 0}, kPins) == Cause::kOffWakeUnknown);
    // A latch on some unrelated pin is equally unknown.
    CHECK(decode({kReasOff, 1u << 15, 0}, kPins) == Cause::kOffWakeUnknown);
}

TEST_CASE("decode - OFF outranks VBUS when both are recorded") {
    // We never enter System OFF with VBUS present (the charging loop
    // intercepts), so OFF+VBUS means a plug-in raced the entry — the
    // GPIO wake we armed is the truth.
    CHECK(decode({kReasOff | kReasVbus, 1u << 2, 0}, kPins) == Cause::kTachWake);
    CHECK(decode({kReasOff | kReasVbus, 0, 0}, kPins) == Cause::kOffWakeUnknown);
}

// ---------------------------------------------------------------------------
// Non-GPIO causes
// ---------------------------------------------------------------------------

TEST_CASE("decode - VBUS alone is a USB wake") {
    CHECK(decode({kReasVbus, 0, 0}, kPins) == Cause::kUsbWake);
}

TEST_CASE("decode - watchdog and soft reset decode to themselves") {
    CHECK(decode({kReasDog, 0, 0}, kPins) == Cause::kWatchdog);
    CHECK(decode({kReasSreq, 0, 0}, kPins) == Cause::kSoftReset);
}

TEST_CASE("decode - reset pin and empty RESETREAS are cold boots") {
    CHECK(decode({kReasResetPin, 0, 0}, kPins) == Cause::kColdBoot);
    CHECK(decode({0, 0, 0}, kPins) == Cause::kColdBoot);
}

TEST_CASE("decode - stale latch without the OFF bit is ignored") {
    // LATCH is sticky; a latched pin from normal operation must not be
    // misread as a wake on a plain power-on.
    CHECK(decode({0, 1u << 2, 0}, kPins) == Cause::kColdBoot);
    CHECK(decode({kReasSreq, (1u << 2) | (1u << 3), 0}, kPins) == Cause::kSoftReset);
}

// ---------------------------------------------------------------------------
// System OFF wake-pin arming
//
// The invariant every one of these protects: nothing left armed may be
// asserting DETECT when System OFF is entered, because the nRF52840 turns
// that into an immediate wake — i.e. a reset. Getting the tach polarity
// wrong is exactly the bug that made every shutdown reboot the logger.
// ---------------------------------------------------------------------------

TEST_CASE("armTach - an open-collector pickup resting high wakes on the fall") {
    CHECK(armTach(true) == PinArm::kSenseLowPullUp);
}

TEST_CASE("armTach - the shipped Schmitt-inverter board rests low, wakes on the rise") {
    // PULSE_RPM is a push-pull output that idles LOW and pulses HIGH per
    // spark. SENSE_LOW here would assert DETECT forever and cost System OFF.
    CHECK(armTach(false) == PinArm::kSenseHighPullDown);
}

TEST_CASE("armTach - whichever level it rests at, the arm is not already satisfied") {
    CHECK(armHoldsDetect(armTach(/*restsHigh=*/true), /*pinIsHigh=*/true) == false);
    CHECK(armHoldsDetect(armTach(/*restsHigh=*/false), /*pinIsHigh=*/false) == false);
}

TEST_CASE("armTach - the arm fires on the opposite level") {
    CHECK(armHoldsDetect(armTach(true), /*pinIsHigh=*/false) == true);
    CHECK(armHoldsDetect(armTach(false), /*pinIsHigh=*/true) == true);
}

TEST_CASE("armButton - a released button arms active-low") {
    CHECK(armButton(true) == PinArm::kSenseLowPullUp);
    CHECK(armHoldsDetect(armButton(true), /*pinIsHigh=*/true) == false);
    CHECK(armHoldsDetect(armButton(true), /*pinIsHigh=*/false) == true);
}

TEST_CASE("armButton - a stuck/held button is skipped, never inverted") {
    // Buttons are wired one way only, so a low read means held. Arming
    // SENSE_HIGH would "wake on release" and, worse, arming SENSE_LOW
    // would block System OFF entirely. Dropping it as a wake source is
    // the only option that still lets the device sleep.
    CHECK(armButton(false) == PinArm::kSkip);
}

TEST_CASE("armHoldsDetect - a skipped pin never asserts DETECT") {
    CHECK(armHoldsDetect(PinArm::kSkip, true) == false);
    CHECK(armHoldsDetect(PinArm::kSkip, false) == false);
}

TEST_CASE("shortName - every cause has a distinct label that fits the boot line") {
    const Cause all[] = {Cause::kColdBoot,  Cause::kTachWake, Cause::kButtonWake,
                         Cause::kUsbWake,   Cause::kWatchdog, Cause::kSoftReset,
                         Cause::kOffWakeUnknown};
    for (Cause a : all) {
        const char* label = shortName(a);
        REQUIRE(label != nullptr);
        CHECK(label[0] != '\0');
        // "Mode:GPS-only W:" is 16 chars of a 21-char line.
        CHECK(std::strlen(label) <= 4);
        for (Cause b : all) {
            if (a == b) continue;
            CHECK(std::strcmp(label, shortName(b)) != 0);
        }
    }
}
