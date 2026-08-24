#include <initializer_list>
#include <cstring>

#include "doctest.h"
#include "led_status.h"

using led_frame::Rgb;
using led_status::CamLevel;
using led_status::GpsLevel;
using led_status::Inputs;
using led_status::Mode;
using led_status::State;
using led_status::Verdict;

static bool same(Rgb a, Rgb b) {
  return a.r == b.r && a.g == b.g && a.b == b.b;
}

// A snapshot with nothing interesting happening: engine off, no fix, no
// camera, no egg. Tests turn on only what they care about.
static Inputs base() {
  Inputs in;
  in.targetRpm = 15000.0f;
  in.targetSpeedMph = 60.0f;
  in.egtAlertC = 650.0f;
  return in;
}

///////////////////////////////////////////
// Mode names: the wire format
///////////////////////////////////////////

TEST_CASE("every mode's name parses back to itself") {
  for (int i = 0; i < (int)Mode::kCount; i++) {
    const Mode m = (Mode)i;
    Mode out = Mode::kCount;
    REQUIRE(led_status::parseMode(led_status::modeName(m), &out));
    CHECK(out == m);
  }
}

TEST_CASE("parseMode is case-insensitive and exact") {
  Mode m = Mode::kOff;
  CHECK(led_status::parseMode("RPM", &m));
  CHECK(m == Mode::kRpm);
  CHECK(led_status::parseMode("Sector", &m));
  CHECK(m == Mode::kSector);

  // A prefix, a suffix, whitespace and junk must all be REFUSED rather
  // than resolved to something. False means "keep the compiled-in
  // default", so a typo leaves the LED doing what it did before instead
  // of silently going dark or changing job.
  for (const char* bad : {"", " ", "rp", "rpmm", "off ", " off", "laps",
                          "purple", "1", "OFFF"}) {
    Mode before = Mode::kCamera;
    m = before;
    CHECK_FALSE(led_status::parseMode(bad, &m));
    CHECK(m == before);  // untouched on failure
  }
  CHECK_FALSE(led_status::parseMode(nullptr, &m));
}

TEST_CASE("modeName never returns null, even out of range") {
  CHECK(led_status::modeName(Mode::kCount) != nullptr);
  CHECK(strcmp(led_status::modeName(Mode::kCount), "off") == 0);
  CHECK(strcmp(led_status::modeName((Mode)200), "off") == 0);
}

TEST_CASE("only gps and camera stay lit outside a session") {
  CHECK(led_status::activeOutsideRace(Mode::kGps));
  CHECK(led_status::activeOutsideRace(Mode::kCamera));
  for (Mode m : {Mode::kOff, Mode::kRpm, Mode::kSpeed, Mode::kLap,
                 Mode::kSector, Mode::kEgt}) {
    CHECK_FALSE(led_status::activeOutsideRace(m));
  }
}

///////////////////////////////////////////
// off
///////////////////////////////////////////

TEST_CASE("off mode is dark and holds no latch") {
  State s;
  Inputs in = base();
  in.rpm = 20000.0f;  // would be screaming in rpm mode
  s.threshold.active = true;
  CHECK(same(led_status::evalMode(Mode::kOff, in, s, 0), led_frame::kOff));
  CHECK_FALSE(s.threshold.active);
}

///////////////////////////////////////////
// Threshold modes
///////////////////////////////////////////

TEST_CASE("rpm mode reproduces the pre-0013 rev flasher") {
  State s;
  Inputs in = base();

  in.rpm = 14000.0f;
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kOff));

  in.rpm = 15000.0f;  // at the target
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kRed));
  CHECK(s.threshold.active);
  // 100 ms half-period: off phase is a FLASH, not a release.
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 100), led_frame::kOff));
  CHECK(s.threshold.active);
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 200), led_frame::kRed));

  // Hysteresis: 97 % of 15000 = 14550. Just above holds.
  in.rpm = 14600.0f;
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kRed));
  in.rpm = 14000.0f;
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kOff));
  CHECK_FALSE(s.threshold.active);
}

TEST_CASE("speed mode flashes at the target and goes dark without a fix") {
  State s;
  Inputs in = base();
  in.speedValid = true;

  in.speedMph = 55.0f;
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 0), led_frame::kOff));

  in.speedMph = 60.0f;
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 0), led_frame::kRed));
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 250), led_frame::kOff));
  CHECK(s.threshold.active);  // still latched, just mid-flash

  // Fixed 2 mph clear delta (GPS speed noise is absolute, not a
  // percentage): 58.5 holds, 57.9 releases.
  in.speedMph = 58.5f;
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 0), led_frame::kRed));
  in.speedMph = 57.9f;
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 0), led_frame::kOff));

  // No fix -> OFF, not a "no signal" colour. The strip's search pip
  // already reports the missing lock; a second lit pixel saying the same
  // thing is noise.
  in.speedMph = 90.0f;
  in.speedValid = false;
  CHECK(same(led_status::evalMode(Mode::kSpeed, in, s, 0), led_frame::kOff));
  CHECK_FALSE(s.threshold.active);
}

///////////////////////////////////////////
// GPS
///////////////////////////////////////////

TEST_CASE("gps ladder classifies sats / fix / time lock") {
  CHECK(led_status::gpsLevel(0, false, false) == GpsLevel::kNoSats);
  CHECK(led_status::gpsLevel(4, false, false) == GpsLevel::kSats);
  CHECK(led_status::gpsLevel(8, true, false) == GpsLevel::kFix);
  CHECK(led_status::gpsLevel(8, true, true) == GpsLevel::kLocked);
  // A fix with a stale sat count is still a fix — fix wins over sats.
  CHECK(led_status::gpsLevel(0, true, true) == GpsLevel::kLocked);
}

TEST_CASE("gps mode is steady red / yellow / blue / green") {
  State s;
  Inputs in = base();

  CHECK(same(led_status::evalMode(Mode::kGps, in, s, 0), led_frame::kRed));

  in.gpsSats = 5;
  CHECK(same(led_status::evalMode(Mode::kGps, in, s, 0), led_frame::kYellow));

  in.gpsFix = true;
  CHECK(same(led_status::evalMode(Mode::kGps, in, s, 0), led_frame::kBlue));

  in.gpsTimeValid = true;
  CHECK(same(led_status::evalMode(Mode::kGps, in, s, 0), led_frame::kGreen));

  // Steady, not flashing: identical at every phase of every flash rate.
  for (uint32_t t : {0u, 100u, 250u, 400u, 999u, 123456u}) {
    CHECK(same(led_status::evalMode(Mode::kGps, in, s, t), led_frame::kGreen));
  }
}

///////////////////////////////////////////
// Camera
///////////////////////////////////////////

TEST_CASE("camera ladder: pairing, session, link, subscribe, recording") {
  // Unpaired is unpaired no matter what else is true.
  CHECK(led_status::cameraLevel(false, true, true, true, true) ==
        CamLevel::kUnpaired);
  // Paired but parked with no session: nothing is wrong yet.
  CHECK(led_status::cameraLevel(true, false, false, false, false) ==
        CamLevel::kUnpaired);
  // Session running, camera not up: that IS worth flagging.
  CHECK(led_status::cameraLevel(true, true, false, false, false) ==
        CamLevel::kIdle);
  CHECK(led_status::cameraLevel(true, true, true, false, false) ==
        CamLevel::kLinkPartial);
  CHECK(led_status::cameraLevel(true, true, true, true, false) ==
        CamLevel::kLinkReady);
  // Recording outranks the link ladder — if it is rolling, say so.
  CHECK(led_status::cameraLevel(true, true, true, false, true) ==
        CamLevel::kRecording);
}

TEST_CASE("camera mode: flashing blue means we cannot command it") {
  State s;
  Inputs in = base();

  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 0), led_frame::kOff));

  in.cameraPaired = true;
  in.sessionActive = true;
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 0),
             led_frame::kYellow));

  // Linked but the camera never subscribed to ce82: our button frames go
  // nowhere, so this FLASHES rather than sitting steady.
  in.cameraLinkUp = true;
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 0), led_frame::kBlue));
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 400),
             led_frame::kOff));
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 800),
             led_frame::kBlue));

  // Subscribed: steady blue — identical at both phases above, which is
  // exactly how a driver tells the two states apart.
  in.cameraSubscribed = true;
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 0), led_frame::kBlue));
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 400),
             led_frame::kBlue));

  in.cameraRecording = true;
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 0), led_frame::kRed));
  CHECK(same(led_status::evalMode(Mode::kCamera, in, s, 400), led_frame::kRed));
}

///////////////////////////////////////////
// EGT — including the bug this plan exists to kill
///////////////////////////////////////////

TEST_CASE("egt mode is the Temp1 tri-state on a SensorEgg build") {
  State s;
  Inputs in = base();
  in.eggSupported = true;

  // No probe signal -> solid blue. A dropout is information.
  in.egtValid = false;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kBlue));
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 250), led_frame::kBlue));

  in.egtValid = true;
  in.egtC = 400.0f;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kOff));

  in.egtC = 700.0f;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kRed));
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 250), led_frame::kOff));
  CHECK(s.threshold.active);

  // A dropout mid-alert releases the latch — never latch stale data.
  in.egtValid = false;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kBlue));
  CHECK_FALSE(s.threshold.active);
}

TEST_CASE("egt mode on a build WITHOUT SensorEgg support is OFF, never blue") {
  // The regression test for the shipped bug: before plan 0013 the right
  // status pixel was hardwired to Temp1, so on every master/release
  // image sensoreggEgtC() answered a permanent NaN and the LED sat
  // solid blue — "no probe signal" — for the whole of every race. There
  // is no probe on such a build, so there is no signal to be missing.
  State s;
  Inputs in = base();
  in.eggSupported = false;

  in.egtValid = false;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kOff));

  // Even handed a hot, VALID reading it stays dark — the gate is the
  // build, not the NaN path, and this proves the two are independent.
  in.egtValid = true;
  in.egtC = 900.0f;
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 0), led_frame::kOff));
  CHECK(same(led_status::evalMode(Mode::kEgt, in, s, 250), led_frame::kOff));
  CHECK_FALSE(s.threshold.active);
}

///////////////////////////////////////////
// Lap / sector verdict colours
///////////////////////////////////////////

TEST_CASE("lap and sector modes map verdicts and never flash") {
  State s;
  Inputs in = base();

  struct { Verdict v; Rgb want; } cases[] = {
      {Verdict::kNone, led_frame::kOff},
      {Verdict::kBetter, led_frame::kGreen},
      {Verdict::kWorse, led_frame::kRed},
      {Verdict::kBest, led_frame::kPurple},
  };
  for (auto c : cases) {
    in.lapVerdict = c.v;
    in.sectorVerdict = c.v;
    for (uint32_t t : {0u, 100u, 250u, 400u, 98765u}) {
      CHECK(same(led_status::evalMode(Mode::kLap, in, s, t), c.want));
      CHECK(same(led_status::evalMode(Mode::kSector, in, s, t), c.want));
    }
  }
}

TEST_CASE("lap and sector are independent of each other") {
  State s;
  Inputs in = base();
  in.lapVerdict = Verdict::kWorse;
  in.sectorVerdict = Verdict::kBetter;
  CHECK(same(led_status::evalMode(Mode::kLap, in, s, 0), led_frame::kRed));
  CHECK(same(led_status::evalMode(Mode::kSector, in, s, 0), led_frame::kGreen));
}

///////////////////////////////////////////
// Latch lifetime
///////////////////////////////////////////

TEST_CASE("reset releases the threshold latch") {
  State s;
  Inputs in = base();
  in.rpm = 16000.0f;
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kRed));
  REQUIRE(s.threshold.active);

  led_status::reset(s);
  CHECK_FALSE(s.threshold.active);

  // Back under the threshold after the reset: stays off rather than
  // resuming a flash the previous session started.
  in.rpm = 14000.0f;
  CHECK(same(led_status::evalMode(Mode::kRpm, in, s, 0), led_frame::kOff));
}

TEST_CASE("switching a pixel to a non-threshold mode drops the latch") {
  State s;
  Inputs in = base();
  in.rpm = 16000.0f;
  led_status::evalMode(Mode::kRpm, in, s, 0);
  REQUIRE(s.threshold.active);

  in.gpsSats = 6;
  led_status::evalMode(Mode::kGps, in, s, 0);
  CHECK_FALSE(s.threshold.active);
}

TEST_CASE("kYellow is distinguishable from kOrange and kGreen") {
  // The gps/camera ladders lean on yellow being its own colour on a
  // WS2812, not "orange, but more so".
  CHECK(led_frame::kYellow.g > led_frame::kOrange.g);
  CHECK(led_frame::kYellow.r > led_frame::kYellow.g);
  CHECK(led_frame::kYellow.b == 0);
}
