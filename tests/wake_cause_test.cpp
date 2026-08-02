#include "doctest.h"
#include "wake_cause.h"

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
// Shutdown-side tach SENSE polarity vote
// ---------------------------------------------------------------------------

TEST_CASE("tachIdleIsHigh - clear majorities decide the idle level") {
    CHECK(tachIdleIsHigh(15, 15));        // idle-high circuit -> SENSE-LOW
    CHECK(tachIdleIsHigh(14, 15));
    CHECK_FALSE(tachIdleIsHigh(0, 15));   // idle-low circuit -> SENSE-HIGH
    CHECK_FALSE(tachIdleIsHigh(1, 15));   // one noise blip doesn't flip it
    CHECK_FALSE(tachIdleIsHigh(7, 15));   // strict minority stays low
    CHECK(tachIdleIsHigh(8, 15));
}

TEST_CASE("tachIdleIsHigh - ties and empty bursts default to idle-high") {
    // The pull-up reads a floating/disconnected tach input as high, and
    // idle-high preserves the historical SENSE-LOW arm.
    CHECK(tachIdleIsHigh(5, 10));
    CHECK(tachIdleIsHigh(0, 0));
}
