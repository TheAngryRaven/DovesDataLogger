#include "doctest.h"

#include "pin_page.h"

using namespace pin_page;

namespace {

Action press(State& s, int row, uint32_t t, bool held = true, bool other = false) {
  Inputs in;
  in.selectHeld = held;
  in.otherButtonHeld = other;
  in.row = row;
  in.nowMs = t;
  return step(s, in);
}

}  // namespace

TEST_CASE("the press that opened the page never counts") {
  State s;
  begin(s);
  for (uint32_t t = 0; t <= 5000; t += 100) CHECK(press(s, kRowShow, t) == Action::kNone);
  CHECK(holdSecondsLeft(s, 5000) == 0);
}

TEST_CASE("hold Select 3 s on Show reveals, once per hold") {
  State s;
  begin(s);
  press(s, kRowShow, 0, false);
  CHECK(press(s, kRowShow, 1000) == Action::kNone);
  CHECK(holdSecondsLeft(s, 1000) == 3);
  CHECK(holdSecondsLeft(s, 3500) == 1);
  CHECK(press(s, kRowShow, 3999) == Action::kNone);
  CHECK(press(s, kRowShow, 4000) == Action::kReveal);
  // Still held: no repeat.
  CHECK(press(s, kRowShow, 9000) == Action::kNone);
}

TEST_CASE("hold on New PIN asks for a new PIN") {
  State s;
  begin(s);
  press(s, kRowNewPin, 0, false);
  press(s, kRowNewPin, 10);
  CHECK(press(s, kRowNewPin, 3010) == Action::kNewPin);
}

TEST_CASE("release, row change or a side button cancels with no credit") {
  State s;
  begin(s);
  press(s, kRowShow, 0, false);
  press(s, kRowShow, 100);
  press(s, kRowShow, 2000, false);  // released
  press(s, kRowShow, 2100);
  CHECK(press(s, kRowShow, 4000) == Action::kNone);  // only 1.9 s since re-hold

  begin(s);
  press(s, kRowShow, 0, false);
  press(s, kRowShow, 100);
  press(s, kRowNewPin, 2000);  // moved
  CHECK(press(s, kRowNewPin, 4000) == Action::kNone);
  CHECK(press(s, kRowNewPin, 5000) == Action::kNewPin);

  begin(s);
  press(s, kRowShow, 0, false);
  press(s, kRowShow, 100);
  press(s, kRowShow, 1000, true, true);  // reboot combo
  CHECK(press(s, kRowShow, 3200) == Action::kNone);
}

TEST_CASE("Back never arms a hold") {
  State s;
  begin(s);
  press(s, kRowBack, 0, false);
  for (uint32_t t = 0; t <= 6000; t += 100) CHECK(press(s, kRowBack, t) == Action::kNone);
}

TEST_CASE("reveal lasts 15 s and wraps safely") {
  State s;
  begin(s);
  CHECK_FALSE(isRevealed(s, 0));
  reveal(s, 1000);
  CHECK(isRevealed(s, 1000));
  CHECK(revealSecondsLeft(s, 1000) == 15);
  CHECK(revealSecondsLeft(s, 15999) == 1);
  CHECK_FALSE(isRevealed(s, 16000));
  CHECK(revealSecondsLeft(s, 16000) == 0);

  reveal(s, 0xffffffffu - 1000u);
  CHECK(isRevealed(s, 5000u));  // wrapped, 6 s in
  CHECK_FALSE(isRevealed(s, 14000u));
}

TEST_CASE("begin hides the PIN") {
  State s;
  reveal(s, 0);
  begin(s);
  CHECK_FALSE(isRevealed(s, 1));
}
