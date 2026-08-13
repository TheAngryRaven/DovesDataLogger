#include <initializer_list>

#include "doctest.h"
#include "led_frame.h"

using led_frame::Frame;
using led_frame::Rgb;

TEST_CASE("layout constants describe the 11-px chain") {
  CHECK(led_frame::kPixelCount == 11);
  CHECK(led_frame::kStatusLeft == 0);
  CHECK(led_frame::kStatusRight == 10);
  CHECK(led_frame::kStripFirst == 1);
  CHECK(led_frame::kStripCount == 9);
  // Strip-relative center maps to chain pixel 5, the middle of 1..9.
  CHECK(led_frame::kStripFirst + led_frame::kStripCenter == 5);
}

TEST_CASE("clear turns every pixel off") {
  Frame f{};
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    f.px[i] = Rgb{255, 128, 7};
  }
  led_frame::clear(f);
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    CHECK(f.px[i].r == 0);
    CHECK(f.px[i].g == 0);
    CHECK(f.px[i].b == 0);
  }
}

TEST_CASE("scale: identity at 255, off at 0, rounds to nearest") {
  Rgb c{255, 100, 1};
  Rgb id = led_frame::scale(c, 255);
  CHECK(id.r == 255);
  CHECK(id.g == 100);
  CHECK(id.b == 1);

  Rgb off = led_frame::scale(c, 0);
  CHECK(off.r == 0);
  CHECK(off.g == 0);
  CHECK(off.b == 0);

  // 255 * 128 / 255 = 128 exactly; 100 * 128 / 255 = 50.19 -> 50.
  Rgb half = led_frame::scale(c, 128);
  CHECK(half.r == 128);
  CHECK(half.g == 50);
  // 1 * 128 / 255 = 0.50 -> rounds to 1, not truncated to 0.
  CHECK(half.b == 1);
}

TEST_CASE("applyCap: no channel of any pixel ever exceeds the cap") {
  const uint8_t caps[] = {0, 1, 64, 128, 254, 255};
  for (uint8_t cap : caps) {
    CAPTURE((int)cap);
    Frame f{};
    // Worst-case saturated frame plus a mixed gradient.
    for (int i = 0; i < led_frame::kPixelCount; i++) {
      f.px[i] = (i % 2 == 0) ? Rgb{255, 255, 255}
                             : Rgb{(uint8_t)(i * 23), 255, (uint8_t)(255 - i)};
    }
    led_frame::applyCap(f, cap);
    for (int i = 0; i < led_frame::kPixelCount; i++) {
      CHECK(f.px[i].r <= cap);
      CHECK(f.px[i].g <= cap);
      CHECK(f.px[i].b <= cap);
    }
  }
}

TEST_CASE("applyCap: 255 is identity, 0 is all-off") {
  Frame f{};
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    f.px[i] = Rgb{(uint8_t)(i * 20), (uint8_t)(255 - i * 10), 33};
  }
  Frame copy = f;
  led_frame::applyCap(f, 255);
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    CHECK(f.px[i].r == copy.px[i].r);
    CHECK(f.px[i].g == copy.px[i].g);
    CHECK(f.px[i].b == copy.px[i].b);
  }
  led_frame::applyCap(f, 0);
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    CHECK(f.px[i].r == 0);
    CHECK(f.px[i].g == 0);
    CHECK(f.px[i].b == 0);
  }
}

TEST_CASE("applyCap: full-white maps to exactly the cap") {
  Frame f{};
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    f.px[i] = led_frame::kWhite;
  }
  led_frame::applyCap(f, 64);
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    CHECK(f.px[i].r == 64);
    CHECK(f.px[i].g == 64);
    CHECK(f.px[i].b == 64);
  }
}

TEST_CASE("add saturates at 255") {
  Rgb s = led_frame::add(Rgb{200, 100, 0}, Rgb{100, 100, 5});
  CHECK(s.r == 255);
  CHECK(s.g == 200);
  CHECK(s.b == 5);
}
