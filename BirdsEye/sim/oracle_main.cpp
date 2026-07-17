///////////////////////////////////////////
// oracle_main.cpp — Phase-3 lap-timing oracle.
//
// Two modes:
//
// (default) SYNTHETIC ORACLE — constructs a constant-speed circular
//   trace through the OKC "Normal" start/finish line whose lap period
//   is known EXACTLY by construction (constant angular rate: crossings
//   happen at the same angle every revolution, so the period is pure
//   math, independent of geo projection error). The trace is injected
//   at 25 Hz through the real onPVTReceived(); the tach runs through
//   the real ISR. The firmware must: leave the boot GPS page into race
//   mode, haversine-detect OKC, parse the track, have CourseDetector
//   pick "Normal" by length, and produce lap times matching the
//   constructed period within one GPS frame (40 ms).
//
// --dovex <file> HARDWARE ORACLE — replays a real .dovex log (rows
//   injected at their own timestamps, RPM column driving the tach) and
//   compares the sim's laps against the hardware-computed lap list in
//   the file's 1 KB header: same code, same answer, within 40 ms.
//   Wire this into CI once a hardware-recorded OKC session is committed
//   under sim/fixtures/.
///////////////////////////////////////////

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dovex_header.h"
#include "sim_host.h"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kToleranceMs = 40;  // one GPS frame at 25 Hz

// OKC "Normal" start line (from the track fixture).
constexpr double kLineALat = 28.4127081705638, kLineALng = -81.3797326641803;
constexpr double kLineBLat = 28.4127303867932, kLineBLng = -81.3795704875378;

// Fixed session epoch: 2026-03-14 15:00:00 UTC.
constexpr unsigned long long kEpochMs = 1773500400000ULL;

bool nearlyEqualMs(uint32_t a, double b) {
  return std::fabs((double)a - b) <= (double)kToleranceMs;
}

int failures = 0;
void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}

// ---------------------------------------------------------- synthetic mode
int runSynthetic() {
  sim_init();

  // Local flat-earth frame around the start-line midpoint M.
  const double mLat = (kLineALat + kLineBLat) / 2.0;
  const double mLng = (kLineALng + kLineBLng) / 2.0;
  const double mPerDegLat = 111320.0;
  const double mPerDegLng = 111320.0 * std::cos(mLat * kPi / 180.0);

  // Circle through M, center due west, radius sized so the lap length
  // equals the "Normal" course (3383 ft) — CourseDetector must rank it
  // above Pro/Short/Ten2one/xShort.
  const double lapMeters = 3383.0 * 0.3048;
  const double r = lapMeters / (2.0 * kPi);
  const double speedMph = 35.0;  // > waypoint_speed(30) and >= 10 auto-race
  const double speedMs = speedMph * 0.44704;
  const double periodMs = lapMeters / speedMs * 1000.0;  // EXACT lap time
  const double omega = 2.0 * kPi / (periodMs / 1000.0);  // rad/s

  // First crossing (theta = 0, heading due north through the east-west
  // line) at t = 20 s — room for the boot page, race entry, detection.
  const double theta0 = -omega * 20.0;

  std::printf("synthetic: lap %.3f m, period %.1f ms, r %.2f m\n", lapMeters,
              periodMs, r);

  // Boot pre-roll: 2 s of no-fix frames through the JSON spelling (both
  // injection paths get exercised). Firmware sits on the GPS status page.
  for (int i = 0; i < 10; i++) {
    char json[256];
    std::snprintf(json, sizeof(json),
                  "{\"timestamp\":%llu,\"lat\":0,\"lng\":0,\"sats\":%d,"
                  "\"hdop\":9.9,\"speed_mph\":0,\"altitude_m\":0,"
                  "\"heading_deg\":0,\"h_acc_m\":50,\"fix\":false,"
                  "\"accelX\":0,\"accelY\":0,\"accelZ\":1}",
                  kEpochMs + (unsigned long long)(i * 200), 3 + (i % 3));
    if (!sim_inject_pvt_json(json)) {
      std::printf("FAIL json injection\n");
      return 1;
    }
    sim_step_millis(200);
  }
  check(sim_current_page() == 900, "boot holds GPS status page (no fix)");
  check(sim_gps_fix() == 0, "no fix during pre-roll");

  // Engine on: real ISR pulses at 6000 RPM.
  sim_set_rpm(6000);

  // Stream the lap trace at 25 Hz. The status page auto-closes 3 s after
  // fix+timeValid and exits into race mode (engine running).
  const int seconds = 360;  // ~5.2 laps after the first crossing at 20 s
  const int frames = seconds * 25;
  bool sawRaceEntry = false;
  for (int i = 0; i < frames; i++) {
    const double tSec = i * 0.040;
    const double theta = theta0 + omega * tSec;
    const double x = -r + r * std::cos(theta);  // meters east of M
    const double y = r * std::sin(theta);       // meters north of M
    // CCW tangent (-sin, cos): heading in compass degrees.
    double heading =
        std::atan2(-std::sin(theta), std::cos(theta)) * 180.0 / kPi;
    if (heading < 0) heading += 360.0;

    SimPvt p = {};
    p.timestamp_ms = kEpochMs + 2000ULL + (unsigned long long)(tSec * 1000.0);
    p.lat = mLat + y / mPerDegLat;
    p.lng = mLng + x / mPerDegLng;
    p.altitude_m = 25.0;
    p.speed_mph = speedMph;
    p.heading_deg = heading;
    p.h_acc_m = 1.2;
    p.hdop = 0.8;
    p.sats = 12;
    p.fix = 1;
    p.accel_x = (float)(speedMs * speedMs / r / 9.81);  // steady lateral g
    p.accel_y = 0.0f;
    p.accel_z = 1.0f;
    sim_inject_pvt(&p);
    sim_step_millis(40);

    if (!sawRaceEntry && sim_race_active()) {
      sawRaceEntry = true;
      std::printf("race mode entered at sim t=%u ms (page %d)\n",
                  sim_millis(), sim_current_page());
    }
  }

  check(sawRaceEntry, "race mode auto-entered from the GPS status page");
  check(sim_gps_fix() == 1, "fix held");
  check(sim_rpm() > 5500 && sim_rpm() < 6500, "Kalman RPM tracks 6000");
  check(sim_track_detected() == 1, "OKC detected by proximity");
  check(sim_lap_anything_active() == 0, "course timer active (not fallback)");
  check(std::strcmp(sim_course_name(), "Normal") == 0,
        "CourseDetector picked 'Normal' by length");
  check(sim_logging_active() == 1, "DOVEX logging active (time lock)");

  const int laps = sim_lap_count();
  std::printf("laps recorded: %d (expected period %.1f ms)\n", laps,
              periodMs);
  check(laps >= 3, "at least 3 laps recorded");
  for (int i = 0; i < laps; i++) {
    const uint32_t lap = sim_lap_time_ms(i);
    char buf[80];
    std::snprintf(buf, sizeof(buf), "lap %d = %u ms within %u ms of truth",
                  i + 1, lap, kToleranceMs);
    check(nearlyEqualMs(lap, periodMs), buf);
  }

  return failures ? 1 : 0;
}

// ------------------------------------------------------------- dovex mode
struct DovexRow {
  unsigned long long ts;
  double lat, lng, hdop, speed, alt, heading, hacc;
  int sats, rpm;
  float ax, ay, az;
};

int runDovex(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) {
    std::printf("cannot open %s\n", path);
    return 2;
  }
  std::vector<char> header(dovex_header::kHeaderSize);
  if (std::fread(header.data(), 1, header.size(), f) != header.size()) {
    std::printf("short dovex header\n");
    std::fclose(f);
    return 2;
  }
  // Parse the reserved header with the firmware's OWN parser — the lap
  // list in it is the hardware-computed ground truth.
  dovex_header::ParsedHeader meta;
  unsigned long hwLapBuf[1000];
  size_t hwLapCount = 0;
  if (!dovex_header::parse(header.data(), header.size(), meta, hwLapBuf,
                           1000, hwLapCount)) {
    std::printf("dovex header did not parse (crashed session?)\n");
    std::fclose(f);
    return 2;
  }
  std::vector<uint32_t> hwLaps(hwLapBuf, hwLapBuf + hwLapCount);
  std::printf("hardware session: %s / %s / course '%s' best %s — %zu laps\n",
              meta.datetime, meta.driver, meta.course, meta.bestLap,
              hwLaps.size());

  std::vector<DovexRow> rows;
  char line[512];
  // Skip the CSV column header line.
  if (!std::fgets(line, sizeof(line), f)) {
    std::fclose(f);
    return 2;
  }
  while (std::fgets(line, sizeof(line), f)) {
    DovexRow r;
    if (std::sscanf(line,
                    "%llu,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%f,%f,%f",
                    &r.ts, &r.sats, &r.hdop, &r.lat, &r.lng, &r.speed,
                    &r.alt, &r.heading, &r.hacc, &r.rpm, &r.ax, &r.ay,
                    &r.az) == 13) {
      rows.push_back(r);
    }
  }
  std::fclose(f);
  std::printf("dovex rows: %zu\n", rows.size());
  if (rows.empty()) return 2;

  sim_init();

  // Boot pre-roll: no-fix frames so the status page runs its real UX.
  for (int i = 0; i < 10; i++) {
    SimPvt p = {};
    p.timestamp_ms = rows[0].ts - 2000 + i * 200;
    p.sats = 3;
    p.hdop = 9.9;
    p.h_acc_m = 50;
    sim_inject_pvt(&p);
    sim_step_millis(200);
  }

  // Accumulate laps as they appear. lapHistory is cleared if the sim's
  // own auto-idle/session-end logic fires during the replay (exactly as
  // the hardware's did at the end of the recorded session), so the
  // comparison list is collected live rather than read at the end.
  std::vector<uint32_t> simLaps;
  int prevCount = 0;
  std::string courseSeen;

  unsigned long long prevTs = rows[0].ts;
  for (const DovexRow& r : rows) {
    SimPvt p = {};
    p.timestamp_ms = r.ts;
    p.lat = r.lat;
    p.lng = r.lng;
    p.altitude_m = r.alt;
    p.speed_mph = r.speed;
    p.heading_deg = r.heading;
    p.h_acc_m = r.hacc;
    p.hdop = r.hdop;
    p.sats = r.sats;
    p.fix = 1;  // rows only exist in the log when the fix was valid
    p.accel_x = r.ax;
    p.accel_y = r.ay;
    p.accel_z = r.az;
    sim_set_rpm(r.rpm);
    sim_inject_pvt(&p);
    unsigned long long dt = r.ts - prevTs;
    if (dt < 1) dt = 1;
    if (dt > 1000) dt = 1000;  // bridge gaps without stalling
    sim_step_millis((uint32_t)dt);
    prevTs = r.ts;

    const int count = sim_lap_count();
    if (count > prevCount) {
      for (int i = prevCount; i < count; i++) {
        simLaps.push_back(sim_lap_time_ms(i));
      }
    }
    prevCount = count;
    if (courseSeen.empty() && sim_course_name()[0]) {
      courseSeen = sim_course_name();
    }
  }

  std::printf("sim laps: %zu, course '%s'\n", simLaps.size(),
              courseSeen.c_str());
  {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "course matches header ('%s')",
                  meta.course);
    check(courseSeen == meta.course, buf);
  }
  check(simLaps.size() == hwLaps.size(), "lap count matches hardware header");
  for (size_t i = 0; i < hwLaps.size() && i < simLaps.size(); i++) {
    char buf[96];
    std::snprintf(buf, sizeof(buf),
                  "lap %zu: sim %u ms vs hardware %u ms (±%u)", i + 1,
                  simLaps[i], hwLaps[i], kToleranceMs);
    check(nearlyEqualMs(simLaps[i], (double)hwLaps[i]), buf);
  }
  return failures ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 3 && !std::strcmp(argv[1], "--dovex")) {
    return runDovex(argv[2]);
  }
  if (argc != 1) {
    std::fprintf(stderr, "usage: %s [--dovex <file.dovex>]\n", argv[0]);
    return 2;
  }
  const int rc = runSynthetic();
  std::printf(rc == 0 ? "--- oracle ok ---\n" : "--- oracle FAILED ---\n");
  return rc;
}
