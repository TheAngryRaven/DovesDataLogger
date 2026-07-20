///////////////////////////////////////////
// trackfile_main.cpp — track-file parsing test driver.
//
// Exercises buildTrackList() + parseTrackFile()/parseTrackFileEntry()
// against the REAL sd_functions.ino code over the sim VFS:
//
//  - the single-track object format (the preloaded OKC.json asset),
//  - the multi-track HackTheTrack export (root keyed by track name),
//    written into the VFS at runtime — this is the format that was
//    silently invisible to the firmware (2026-07-19 field incident:
//    a 4.2 KB two-track export both overflowed the old 4 KB JSON
//    buffer and had no root "courses" key, so it produced zero
//    manifest entries and no parse error anywhere).
//
// ArduinoJson-bound logic can't live in the tests/ host harness, so this
// driver is its coverage.
///////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "Arduino.h"
#include "SdFat.h"
#include "project.h"
#include "sd_functions.h"
#include "sim_host.h"

// Globals defined in BirdsEye.ino (compiled into the sim core TU).
extern SdFat SD;
extern TrackManifestEntry trackManifest[];
extern int trackManifestCount;
extern int numOfTracks;
extern char tracks[][MAX_LAYOUT_LENGTH];
extern TrackLayout trackLayouts[];
extern TrackMetadata activeTrackMetadata;

namespace {

int failures = 0;
void check(bool ok, const char* what) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}

// A real two-track HackTheTrack multi-track export (CKC + OKC), verbatim
// from the 2026-07-19 field session. 4262 bytes — over the old 4 KB buffer.
const char kMultiTrackJson[] = R"json({
  "Colorado Karting Circuit": {
    "shortName": "CKC",
    "defaultCourse": "Colorado Karting Circuit CCW",
    "courses": [
      {
        "name": "Colorado Karting Circuit CCW",
        "lengthFt": 4201,
        "start_a_lat": 39.5698074116402,
        "start_a_lng": -104.831080287695,
        "start_b_lat": 39.5698694389592,
        "start_b_lng": -104.831054806709,
        "sector_2_a_lat": 39.5702612435752,
        "sector_2_a_lng": -104.830867052078,
        "sector_2_b_lat": 39.5701940476902,
        "sector_2_b_lng": -104.830901920795,
        "sector_3_a_lat": 39.5701682031017,
        "sector_3_a_lng": -104.831551015377,
        "sector_3_b_lat": 39.570219892269,
        "sector_3_b_lng": -104.831508100033
      }
    ]
  },
  "Orlando Kart Center": {
    "shortName": "OKC",
    "defaultCourse": "Normal",
    "courses": [
      {
        "name": "Normal",
        "lengthFt": 3383,
        "start_a_lat": 28.4127081705638,
        "start_a_lng": -81.3797326641803,
        "start_b_lat": 28.4127303867932,
        "start_b_lng": -81.3795704875378,
        "sector_2_a_lat": 28.4119049886871,
        "sector_2_a_lng": -81.3790708193926,
        "sector_2_b_lat": 28.4118316342961,
        "sector_2_b_lng": -81.3791856652217,
        "sector_3_a_lat": 28.4115010664104,
        "sector_3_a_lng": -81.3799856475317,
        "sector_3_b_lat": 28.4115084390461,
        "sector_3_b_lng": -81.3798064021136
      },
      {
        "name": "Pro",
        "lengthFt": 3828,
        "start_a_lat": 28.4127081705638,
        "start_a_lng": -81.3797326641803,
        "start_b_lat": 28.4127303867932,
        "start_b_lng": -81.3795704875378,
        "sector_2_a_lat": 28.4119049886871,
        "sector_2_a_lng": -81.3790708193926,
        "sector_2_b_lat": 28.4118316342961,
        "sector_2_b_lng": -81.3791856652217,
        "sector_3_a_lat": 28.4115010664104,
        "sector_3_a_lng": -81.3799856475317,
        "sector_3_b_lat": 28.4115084390461,
        "sector_3_b_lng": -81.3798064021136
      },
      {
        "name": "Short",
        "lengthFt": 2338,
        "start_a_lat": 28.411993499165,
        "start_a_lng": -81.3799588509719,
        "start_b_lat": 28.4119938453891,
        "start_b_lng": -81.379864836741,
        "sector_2_a_lat": 28.4114927631029,
        "sector_2_a_lng": -81.3795585429157,
        "sector_2_b_lat": 28.4115458438609,
        "sector_2_b_lng": -81.3793814715123,
        "sector_3_a_lat": 28.4112084857017,
        "sector_3_a_lng": -81.3793065225231,
        "sector_3_b_lat": 28.4111329928196,
        "sector_3_b_lng": -81.3794042747997
      },
      {
        "name": "Ten2one",
        "lengthFt": 2603,
        "start_a_lat": 28.4119841399534,
        "start_a_lng": -81.3799676299095,
        "start_b_lat": 28.4119765649198,
        "start_b_lng": -81.3798238636838,
        "sector_2_a_lat": 28.4121297304433,
        "sector_2_a_lng": -81.3791039555996,
        "sector_2_b_lat": 28.4121769130568,
        "sector_2_b_lng": -81.3789296049969,
        "sector_3_a_lat": 28.4110763731228,
        "sector_3_a_lng": -81.3793199065039,
        "sector_3_b_lat": 28.4112143835809,
        "sector_3_b_lng": -81.3793037069974
      },
      {
        "name": "xShort",
        "lengthFt": 1865,
        "start_a_lat": 28.411993499165,
        "start_a_lng": -81.3799588509719,
        "start_b_lat": 28.4119938453891,
        "start_b_lng": -81.379864836741,
        "sector_2_a_lat": 28.4114444006113,
        "sector_2_a_lng": -81.3794848698762,
        "sector_2_b_lat": 28.4114974813936,
        "sector_2_b_lng": -81.3793667850187,
        "sector_3_a_lat": 28.4110928872042,
        "sector_3_a_lng": -81.3792012636115,
        "sector_3_b_lat": 28.4111966899424,
        "sector_3_b_lng": -81.3791992181244
      }
    ]
  }
})json";

const TrackManifestEntry* findEntry(const char* key) {
  for (int i = 0; i < trackManifestCount; i++) {
    if (std::strcmp(trackManifest[i].trackKey, key) == 0) {
      return &trackManifest[i];
    }
  }
  return nullptr;
}

bool near(double a, double b) { return a > b - 1e-9 && a < b + 1e-9; }

}  // namespace

int main() {
  sim_init();  // boots the firmware; buildTrackList() ran with OKC.json only

  check(trackManifestCount == 1, "boot manifest: OKC.json only");

  // Drop the multi-track export into the VFS the way a user drops it onto
  // the card, then rescan.
  {
    File f;
    if (!f.open("/TRACKS/MULTI.json", O_WRITE | O_CREAT | O_TRUNC)) {
      std::printf("FAIL cannot create /TRACKS/MULTI.json\n");
      return 1;
    }
    f.write(kMultiTrackJson, std::strlen(kMultiTrackJson));
    f.close();
  }
  check(buildTrackList(), "rescan after adding MULTI.json");
  std::printf("manifest entries: %d\n", trackManifestCount);
  check(trackManifestCount == 3,
        "multi-track file adds one manifest entry per contained track");

  const TrackManifestEntry* single = findEntry("");
  check(single != nullptr && std::strcmp(single->filename, "OKC") == 0,
        "single-track entry keeps an empty trackKey");

  const TrackManifestEntry* ckc = findEntry("Colorado Karting Circuit");
  check(ckc != nullptr, "CKC manifest entry exists");
  if (ckc) {
    check(std::strcmp(ckc->filename, "MULTI") == 0, "CKC entry names its file");
    check(near(ckc->lat, 39.5698074116402) && near(ckc->lon, -104.831080287695),
          "CKC entry carries the first course's coordinates");
  }

  const TrackManifestEntry* okc = findEntry("Orlando Kart Center");
  check(okc != nullptr, "OKC (multi) manifest entry exists");

  // Parse the OKC member out of the multi-track file.
  char path[64];
  std::strcpy(path, "/TRACKS/MULTI.json");
  check(parseTrackFileEntry(path, "Orlando Kart Center") == PARSE_STATUS_GOOD,
        "parse multi-track member: Orlando Kart Center");
  check(numOfTracks == 5, "OKC member yields 5 courses");
  check(std::strcmp(activeTrackMetadata.longName, "Orlando Kart Center") == 0,
        "longName is the member key");
  check(std::strcmp(activeTrackMetadata.shortName, "OKC") == 0,
        "shortName parsed from the member");
  check(std::strcmp(activeTrackMetadata.defaultCourse, "Normal") == 0,
        "defaultCourse parsed from the member");
  check(numOfTracks >= 1 && std::strcmp(tracks[0], "Normal") == 0 &&
            near(activeTrackMetadata.courseLengthFt[0], 3383.0f),
        "course 0 is Normal @ 3383 ft");
  check(numOfTracks >= 2 && near(activeTrackMetadata.courseLengthFt[1], 3828.0f),
        "course 1 is Pro @ 3828 ft");
  check(numOfTracks >= 1 && trackLayouts[0].hasSector2 && trackLayouts[0].hasSector3,
        "Normal sector lines parsed");

  // Parse the CKC member.
  check(parseTrackFileEntry(path, "Colorado Karting Circuit") == PARSE_STATUS_GOOD,
        "parse multi-track member: Colorado Karting Circuit");
  check(numOfTracks == 1 && near(activeTrackMetadata.courseLengthFt[0], 4201.0f),
        "CKC member yields 1 course @ 4201 ft");

  // A key that isn't in the file must fail loudly, not fall through.
  check(parseTrackFileEntry(path, "Nope Raceway") == PARSE_STATUS_PARSE_FAILED,
        "unknown track key is a parse failure");

  // Keyless parse of a multi-track file: no root courses array, so it
  // parses clean but yields zero courses (callers already guard on that).
  check(parseTrackFile(path) == PARSE_STATUS_GOOD && numOfTracks == 0,
        "keyless parse of a multi-track file yields zero courses");

  // The preloaded single-track asset still parses the old way.
  std::strcpy(path, "/TRACKS/OKC.json");
  check(parseTrackFile(path) == PARSE_STATUS_GOOD && numOfTracks == 5,
        "single-track OKC.json unaffected");
  check(std::strcmp(activeTrackMetadata.shortName, "OKC") == 0,
        "single-track shortName intact");

  std::printf(failures ? "--- trackfile FAILED ---\n" : "--- trackfile ok ---\n");
  return failures ? 1 : 0;
}
