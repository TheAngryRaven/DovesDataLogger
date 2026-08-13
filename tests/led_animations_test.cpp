#include <initializer_list>

#include "doctest.h"
#include "led_animations.h"

using led_frame::Frame;
using led_frame::kPixelCount;
using led_frame::Rgb;

static bool framesEqual(const Frame& a, const Frame& b) {
  for (int i = 0; i < kPixelCount; i++) {
    if (a.px[i].r != b.px[i].r || a.px[i].g != b.px[i].g ||
        a.px[i].b != b.px[i].b) {
      return false;
    }
  }
  return true;
}

static bool frameLit(const Frame& f) {
  for (int i = 0; i < kPixelCount; i++) {
    if (f.px[i].r || f.px[i].g || f.px[i].b) {
      return true;
    }
  }
  return false;
}

TEST_CASE("mix avalanches consecutive inputs apart") {
  CHECK(led_animations::mix(1) != led_animations::mix(2));
  CHECK(led_animations::mix(0) != 0);
  // Stateless: same input, same output.
  CHECK(led_animations::mix(12345) == led_animations::mix(12345));
}

TEST_CASE("hueToRgb: wheel anchors") {
  Rgb r = led_animations::hueToRgb(0);
  CHECK(r.r == 255);
  CHECK(r.g == 0);
  CHECK(r.b == 0);
  Rgb g = led_animations::hueToRgb(85);
  CHECK(g.r == 0);
  CHECK(g.g == 255);
  CHECK(g.b == 0);
  Rgb b = led_animations::hueToRgb(170);
  CHECK(b.r == 0);
  CHECK(b.g == 0);
  CHECK(b.b == 255);
}

TEST_CASE("renderBoot: deterministic in (t, seed)") {
  const uint32_t seed = 0xC0FFEE;
  for (uint32_t t : {0U, 137U, 800U, 1000U, 1499U, 1800U, 2300U, 2599U}) {
    CAPTURE(t);
    Frame a{};
    Frame b{};
    led_animations::renderBoot(t, seed, a);
    led_animations::renderBoot(t, seed, b);
    CHECK(framesEqual(a, b));
  }
}

TEST_CASE("renderBoot: running while t < duration, done at duration") {
  Frame f{};
  CHECK(led_animations::renderBoot(0, 1, f));
  CHECK(frameLit(f));  // comet visible immediately
  CHECK(led_animations::renderBoot(led_animations::kBootDurationMs - 1, 1, f));
  CHECK(!led_animations::renderBoot(led_animations::kBootDurationMs, 1, f));
  CHECK(!frameLit(f));  // done clears the frame
  CHECK(!led_animations::renderBoot(999999, 1, f));
}

TEST_CASE("renderBoot: comet moves over time") {
  const uint32_t seed = 7;
  Frame a{};
  Frame b{};
  led_animations::renderBoot(100, seed, a);
  led_animations::renderBoot(400, seed, b);
  CHECK(!framesEqual(a, b));
}

TEST_CASE("renderBoot: different seeds differ in the sparkle phase") {
  // Pick a time in the sparkle window past the comet fade so only
  // sparkles are on — seed is the only variable.
  const uint32_t t = led_animations::kBootFadeEndMs + 100;
  bool differed = false;
  for (uint32_t s = 0; s < 8 && !differed; s++) {
    Frame a{};
    Frame b{};
    led_animations::renderBoot(t + (uint32_t)(s * led_animations::kSparkleSlotMs),
                               0x1111, a);
    led_animations::renderBoot(t + (uint32_t)(s * led_animations::kSparkleSlotMs),
                               0x2222, b);
    if (!framesEqual(a, b)) {
      differed = true;
    }
  }
  CHECK(differed);
}

TEST_CASE("renderPurple: deterministic, lifecycle, wave from center") {
  const uint32_t seed = 42;
  Frame a{};
  Frame b{};
  led_animations::renderPurple(200, seed, a);
  led_animations::renderPurple(200, seed, b);
  CHECK(framesEqual(a, b));

  // Early wave: center chain pixel lit, chain ends not yet reached.
  Frame w{};
  CHECK(led_animations::renderPurple(60, seed, w));
  const int center = led_frame::kStripFirst + led_frame::kStripCenter;
  CHECK((w.px[center].r || w.px[center].b));
  CHECK(!(w.px[0].r || w.px[0].g || w.px[0].b));
  CHECK(!(w.px[kPixelCount - 1].r || w.px[kPixelCount - 1].g ||
          w.px[kPixelCount - 1].b));

  // Hold phase: everything lit (status pixels included — celebration).
  Frame h{};
  CHECK(led_animations::renderPurple(800, seed, h));
  for (int i = 0; i < kPixelCount; i++) {
    CAPTURE(i);
    CHECK((h.px[i].r || h.px[i].g || h.px[i].b));
  }

  // Fade phase dimmer than hold.
  Frame fade{};
  CHECK(led_animations::renderPurple(1550, seed, fade));
  CHECK(fade.px[center].b < h.px[center].b);

  // Done at duration.
  Frame d{};
  CHECK(!led_animations::renderPurple(led_animations::kPurpleDurationMs, seed, d));
  CHECK(!frameLit(d));
}

TEST_CASE("golden frames: fixed (t, seed) triplets stay stable") {
  // Locks the deterministic contract — if an intentional visual tweak
  // changes these, re-record the numbers in the same commit.
  Frame f{};
  led_animations::renderBoot(0, 0xBEEF, f);
  // t=0: comet head at px 0, full red (hue 0), tail wrapping behind.
  CHECK(f.px[0].r == 255);
  CHECK(f.px[0].g == 0);
  CHECK(f.px[0].b == 0);

  Frame p{};
  led_animations::renderPurple(700, 0xBEEF, p);
  // Hold phase base color is kPurple where no sparkle landed. At least
  // the majority of pixels must be exactly base purple.
  int base = 0;
  for (int i = 0; i < kPixelCount; i++) {
    if (p.px[i].r == led_frame::kPurple.r && p.px[i].g == led_frame::kPurple.g &&
        p.px[i].b == led_frame::kPurple.b) {
      base++;
    }
  }
  CHECK(base >= kPixelCount / 2);
}
