#include <initializer_list>

#include "doctest.h"
#include "led_modes.h"

using led_frame::kStripCenter;
using led_frame::kStripCount;
using led_frame::Rgb;
using led_modes::pacePip;
using led_modes::PacePip;
using led_modes::ScaleSpec;
using led_modes::Source;
using led_modes::StatusAction;
using led_modes::StatusState;

static bool isRed(Rgb c) { return c.r > 0 && c.g == 0 && c.b == 0; }
static bool isGreen(Rgb c) { return c.g > 0 && c.r == 0 && c.b == 0; }
static bool isOff(Rgb c) { return c.r == 0 && c.g == 0 && c.b == 0; }

TEST_CASE("pace pip: deadband parks on the white centerline") {
  for (float pace : {0.0f, 0.125f, -0.125f, 0.05f}) {
    CAPTURE(pace);
    PacePip p = pacePip(pace);
    CHECK(p.stripIndex == kStripCenter);
    CHECK(p.color.r == 255);
    CHECK(p.color.g == 255);
    CHECK(p.color.b == 255);
  }
}

TEST_CASE("pace pip: slower goes LEFT in red, faster RIGHT in green") {
  PacePip slow = pacePip(0.3f);
  CHECK(slow.stripIndex < kStripCenter);
  CHECK(isRed(slow.color));

  PacePip fast = pacePip(-0.3f);
  CHECK(fast.stripIndex > kStripCenter);
  CHECK(isGreen(fast.color));
}

TEST_CASE("pace pip: full scale clamps to the end pixels") {
  CHECK(pacePip(1.0f).stripIndex == 0);
  CHECK(pacePip(5.0f).stripIndex == 0);
  CHECK(pacePip(-1.0f).stripIndex == kStripCount - 1);
  CHECK(pacePip(-99.0f).stripIndex == kStripCount - 1);
}

TEST_CASE("pace pip: monotonic steps, one pixel per 0.25 ms/m") {
  // Just past the deadband: first pixel out.
  CHECK(pacePip(0.13f).stripIndex == kStripCenter - 1);
  CHECK(pacePip(0.25f).stripIndex == kStripCenter - 1);
  CHECK(pacePip(0.26f).stripIndex == kStripCenter - 2);
  CHECK(pacePip(0.50f).stripIndex == kStripCenter - 2);
  CHECK(pacePip(0.75f).stripIndex == kStripCenter - 3);
  CHECK(pacePip(0.99f).stripIndex == kStripCenter - 4);
  // Mirrored on the fast side.
  CHECK(pacePip(-0.26f).stripIndex == kStripCenter + 2);
  int prev = pacePip(0.0f).stripIndex;
  for (float pace = 0.05f; pace < 1.3f; pace += 0.05f) {
    int idx = pacePip(pace).stripIndex;
    CHECK(idx <= prev);  // more positive pace never moves right
    prev = idx;
  }
}

TEST_CASE("renderPace: centerline dim white under an off-center pip") {
  Rgb out[kStripCount];
  led_modes::renderPace(0.6f, out);
  // Centerline present, dimmer than the pip.
  CHECK(out[kStripCenter].r == out[kStripCenter].g);
  CHECK(out[kStripCenter].r == out[kStripCenter].b);
  CHECK(out[kStripCenter].r > 0);
  CHECK(out[kStripCenter].r < 255);
  int pipIdx = pacePip(0.6f).stripIndex;
  CHECK(isRed(out[pipIdx]));
  for (int i = 0; i < kStripCount; i++) {
    if (i != pipIdx && i != kStripCenter) {
      CHECK(isOff(out[i]));
    }
  }
}

static const ScaleSpec kRpmSpec{0.0f, 10000.0f, led_modes::kRpmRedFrac,
                                led_frame::kGreen, led_frame::kRed};

TEST_CASE("renderScale: fill count across the range") {
  Rgb out[kStripCount];

  led_modes::renderScale(0.0f, kRpmSpec, out);
  for (int i = 0; i < kStripCount; i++) CHECK(isOff(out[i]));

  led_modes::renderScale(10000.0f, kRpmSpec, out);
  for (int i = 0; i < kStripCount; i++) CHECK(!isOff(out[i]));

  // Clamped outside the range.
  led_modes::renderScale(-500.0f, kRpmSpec, out);
  for (int i = 0; i < kStripCount; i++) CHECK(isOff(out[i]));
  led_modes::renderScale(99999.0f, kRpmSpec, out);
  for (int i = 0; i < kStripCount; i++) CHECK(!isOff(out[i]));

  // Half scale: round(0.5 * 9) = 5 pixels lit (fill is left-to-right,
  // unlit tail stays off).
  led_modes::renderScale(5000.0f, kRpmSpec, out);
  for (int i = 0; i < 5; i++) CHECK(!isOff(out[i]));
  for (int i = 5; i < kStripCount; i++) CHECK(isOff(out[i]));
}

TEST_CASE("renderScale: red past the halfway mark, green below") {
  Rgb out[kStripCount];
  led_modes::renderScale(10000.0f, kRpmSpec, out);
  // redFrac 0.5 on 9 px: indices 0..4 green, 5..8 red.
  for (int i = 0; i < 5; i++) CHECK(isGreen(out[i]));
  for (int i = 5; i < kStripCount; i++) CHECK(isRed(out[i]));
}

TEST_CASE("renderScale: degenerate span never divides by zero") {
  Rgb out[kStripCount];
  ScaleSpec bad{100.0f, 100.0f, 0.5f, led_frame::kGreen, led_frame::kRed};
  led_modes::renderScale(100.0f, bad, out);
  for (int i = 0; i < kStripCount; i++) CHECK(isOff(out[i]));
}

static const StatusAction kRevAction{Source::kRpm, 10000.0f, 9700.0f,
                                     led_frame::kRed,
                                     led_modes::kRevFlashHalfPeriodMs,
                                     led_frame::kOff};

TEST_CASE("evalStatus: threshold fires, hysteresis holds, clear releases") {
  StatusState st;
  // Below threshold: off, not latched.
  CHECK(isOff(led_modes::evalStatus(kRevAction, st, 9000.0f, true, 0)));
  CHECK(!st.active);
  // At threshold: latches. nowMs=0 is the ON flash phase.
  CHECK(isRed(led_modes::evalStatus(kRevAction, st, 10000.0f, true, 0)));
  CHECK(st.active);
  // Dips into the hysteresis band: still latched (flash phase ON).
  CHECK(isRed(led_modes::evalStatus(kRevAction, st, 9800.0f, true, 0)));
  CHECK(st.active);
  // Below clearBelow: releases.
  CHECK(isOff(led_modes::evalStatus(kRevAction, st, 9600.0f, true, 0)));
  CHECK(!st.active);
}

TEST_CASE("evalStatus: flash phase follows nowMs") {
  StatusState st;
  led_modes::evalStatus(kRevAction, st, 12000.0f, true, 0);
  REQUIRE(st.active);
  // Half-period 100 ms: 0..99 on, 100..199 off, 200..299 on...
  CHECK(isRed(led_modes::evalStatus(kRevAction, st, 12000.0f, true, 50)));
  CHECK(isOff(led_modes::evalStatus(kRevAction, st, 12000.0f, true, 150)));
  CHECK(isRed(led_modes::evalStatus(kRevAction, st, 12000.0f, true, 250)));
  CHECK(st.active);  // off-phase is flash, not release
}

TEST_CASE("evalStatus: invalid input forces off AND releases the latch") {
  StatusState st;
  led_modes::evalStatus(kRevAction, st, 12000.0f, true, 0);
  REQUIRE(st.active);
  CHECK(isOff(led_modes::evalStatus(kRevAction, st, 12000.0f, false, 0)));
  CHECK(!st.active);
  // Recovering validity below threshold stays off.
  CHECK(isOff(led_modes::evalStatus(kRevAction, st, 9000.0f, true, 0)));
  CHECK(!st.active);
}

TEST_CASE("evalStatus: kNone source is always off") {
  StatusState st;
  StatusAction none{Source::kNone, 0.0f, 0.0f, led_frame::kRed, 100,
                    led_frame::kBlue};  // invalidColor must NOT leak through
  CHECK(isOff(led_modes::evalStatus(none, st, 99999.0f, true, 0)));
  CHECK(isOff(led_modes::evalStatus(none, st, 99999.0f, false, 0)));
  CHECK(!st.active);
}

TEST_CASE("evalStatus: zero half-period means solid, not divide-by-zero") {
  StatusState st;
  StatusAction solid{Source::kRpm, 100.0f, 90.0f, led_frame::kOrange, 0,
                     led_frame::kOff};
  Rgb c = led_modes::evalStatus(solid, st, 200.0f, true, 12345);
  CHECK(c.r == led_frame::kOrange.r);
  CHECK(c.g == led_frame::kOrange.g);
  CHECK(c.b == led_frame::kOrange.b);
}

static bool isBlue(Rgb c) { return c.b > 0 && c.r == 0 && c.g == 0; }

TEST_CASE("temp tri-state: blue when invalid, off when good, red flash hot") {
  // The temp action as the glue builds it: threshold from the setting,
  // clear 20 C below, blue as the no-signal color.
  StatusAction temp{Source::kEgtC, 650.0f, 630.0f, led_frame::kRed,
                    led_modes::kEgtFlashHalfPeriodMs, led_frame::kBlue};
  StatusState st;

  // No probe signal: solid blue, latch released.
  CHECK(isBlue(led_modes::evalStatus(temp, st, 0.0f, false, 0)));
  CHECK(!st.active);

  // Good reading below limit: off.
  CHECK(isOff(led_modes::evalStatus(temp, st, 500.0f, true, 0)));

  // Hot: red flash (nowMs=0 is the ON phase).
  CHECK(isRed(led_modes::evalStatus(temp, st, 660.0f, true, 0)));
  CHECK(st.active);

  // Probe drops out mid-alert: blue AND the latch releases — a stale
  // value must never keep the alert flashing.
  CHECK(isBlue(led_modes::evalStatus(temp, st, 660.0f, false, 0)));
  CHECK(!st.active);

  // Signal returns in the hysteresis band (640): stays off — the latch
  // was released, and 640 is below the 650 fire threshold.
  CHECK(isOff(led_modes::evalStatus(temp, st, 640.0f, true, 0)));
  CHECK(!st.active);
}

TEST_CASE("overrev-style action: wide hysteresis holds the latch down to clear") {
  // Overrev fires at 8500 but clears at the NORMAL rev limit's clear
  // point (7550 * 0.97 ~ 7323) — the whole band between must stay latched.
  StatusAction over{Source::kRpm, 8500.0f, 7323.0f, led_frame::kRed,
                    led_modes::kRevFlashHalfPeriodMs, led_frame::kOff};
  StatusState st;
  CHECK(isOff(led_modes::evalStatus(over, st, 8000.0f, true, 0)));
  CHECK(!st.active);
  led_modes::evalStatus(over, st, 8600.0f, true, 0);
  CHECK(st.active);
  // Back under the trip point but above clear: still latched.
  led_modes::evalStatus(over, st, 8000.0f, true, 0);
  CHECK(st.active);
  led_modes::evalStatus(over, st, 7400.0f, true, 0);
  CHECK(st.active);
  // Below clear: releases.
  led_modes::evalStatus(over, st, 7300.0f, true, 0);
  CHECK(!st.active);
}

TEST_CASE("inverted hysteresis (clearBelow above threshold) never strobes") {
  // rev_limit and overrev_limit clamp independently, so a user can set
  // overrev_limit at or below rev_limit * kRevClearFrac. neopixel.ino then
  // builds the overrev action with clearBelow ABOVE threshold. Before the
  // clamp in evalStatus, a value inside that inverted band set the latch on
  // one frame and cleared it on the next — all 11 pixels strobing at the
  // 30 Hz frame rate instead of flashing at 100 ms.
  //
  // Concretely: rev_limit 15000 (clear 14550), overrev_limit 12000.
  StatusAction over{Source::kRpm, 12000.0f, 14550.0f, led_frame::kRed,
                    led_modes::kRevFlashHalfPeriodMs, led_frame::kOff};
  StatusState st;

  // At 13000 RPM: above the 12000 trip, below the bogus 14550 release.
  led_modes::evalStatus(over, st, 13000.0f, true, 0);
  CHECK(st.active);
  // The latch must HOLD across successive frames, not alternate.
  for (uint32_t f = 1; f < 20; f++) {
    led_modes::evalStatus(over, st, 13000.0f, true, f * 33);
    CAPTURE(f);
    CHECK(st.active);
  }
  // It still releases below the (collapsed) trip point.
  led_modes::evalStatus(over, st, 11999.0f, true, 0);
  CHECK(!st.active);
  // And re-arms cleanly.
  led_modes::evalStatus(over, st, 12000.0f, true, 0);
  CHECK(st.active);

  // A sane configuration is untouched by the clamp: overrev above rev.
  StatusAction sane{Source::kRpm, 16000.0f, 14550.0f, led_frame::kRed,
                    led_modes::kRevFlashHalfPeriodMs, led_frame::kOff};
  StatusState st2;
  led_modes::evalStatus(sane, st2, 16100.0f, true, 0);
  CHECK(st2.active);
  led_modes::evalStatus(sane, st2, 15000.0f, true, 0);
  CHECK(st2.active);  // wide band still latched
  led_modes::evalStatus(sane, st2, 14000.0f, true, 0);
  CHECK(!st2.active);
}

TEST_CASE("search pip: one green pixel, bounces to both ends, deterministic") {
  Rgb out[kStripCount];
  auto litIndex = [&](uint32_t t) {
    led_modes::renderSearchPip(t, out);
    int idx = -1;
    int lit = 0;
    for (int i = 0; i < kStripCount; i++) {
      if (!isOff(out[i])) {
        lit++;
        idx = i;
        CHECK(isGreen(out[i]));
      }
    }
    CHECK(lit == 1);  // exactly one pixel, always
    return idx;
  };

  // Endpoints: start of the period at px 0, half-period at px 8.
  CHECK(litIndex(0) == 0);
  CHECK(litIndex(led_modes::kSearchBouncePeriodMs / 2) == kStripCount - 1);
  // Periodic: one full period later, same position.
  for (uint32_t t = 0; t < led_modes::kSearchBouncePeriodMs; t += 37) {
    CHECK(litIndex(t) == litIndex(t + led_modes::kSearchBouncePeriodMs));
  }
  // Deterministic: same t, same frame.
  CHECK(litIndex(12345) == litIndex(12345));
  // Sweeps: every pixel is visited somewhere in one period.
  bool seen[kStripCount] = {};
  for (uint32_t t = 0; t < led_modes::kSearchBouncePeriodMs; t += 10) {
    seen[litIndex(t)] = true;
  }
  for (int i = 0; i < kStripCount; i++) {
    CAPTURE(i);
    CHECK(seen[i]);
  }
  // Triangle symmetry: out and back visit mirrored positions.
  uint32_t const q = led_modes::kSearchBouncePeriodMs / 4;
  CHECK(litIndex(q) == litIndex(led_modes::kSearchBouncePeriodMs - q));
}
