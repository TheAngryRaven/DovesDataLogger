#pragma once

///////////////////////////////////////////
// DOVEX HEADER FORMAT
// Pure-string layout helpers for the reserved 1 KB at the start of
// every .dovex log file. Both the firmware (writeDovexHeader in
// BirdsEye.ino, parseDovexHeader in replay.ino) and host unit tests
// operate through this interface — the file I/O stays out of here.
//
// On-disk layout (DOVEX_HEADER_SIZE = 1024 bytes total):
//   Line 1 (\r\n):  datetime,driver,course,short_name,best_lap_ms,optimal_ms
//   Line 2 (\r\n):  <metadata values>
//   Line 3 (\r\n):  laps_ms
//   Line 4 (\r\n):  <lap1_ms,lap2_ms,...,lapN_ms>
//   Remaining:      \n padding until byte 1024
//
// The header is written when a session ends. If the device crashes
// mid-session the bytes 0..1023 are left blank but everything after
// (the streaming GPS rows) is still valid.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace dovex_header {

constexpr size_t kHeaderSize = 1024;

// Caps on string field lengths inside the parsed struct. These match
// the on-device globals declared in BirdsEye.ino.
constexpr size_t kDatetimeLen  = 24;
constexpr size_t kDriverLen    = 32;
constexpr size_t kCourseLen    = 32;
constexpr size_t kShortNameLen = 16;
constexpr size_t kLapStrLen    = 16;  // "N/A" or numeric lap time as text

// Input to format(). All const char* fields must be non-null and
// well-formed; format() does NOT escape commas inside them.
struct Metadata {
  const char*   datetime;     // e.g. "2025-03-11 14:30:00"
  const char*   driver;       // "Driver"
  const char*   course;       // "Normal"
  const char*   shortName;    // "OKC"
  unsigned long bestLapMs;    // 0 -> printed as "N/A"
  unsigned long optimalMs;    // 0 -> printed as "N/A"
};

// Output of parse(). Strings are always null-terminated.
struct ParsedHeader {
  char datetime[kDatetimeLen];
  char driver[kDriverLen];
  char course[kCourseLen];
  char shortName[kShortNameLen];
  char bestLap[kLapStrLen];   // text — may be "N/A"
  char optimal[kLapStrLen];   // text — may be "N/A"
};

// Render metadata + lap times into the first kHeaderSize bytes of
// `buf`. Pads the rest with '\n'. Returns true if everything fit;
// false if the lap list was truncated (the buffer is still valid
// 1024 bytes, just with fewer laps).
//
// bufSize must be >= kHeaderSize. Crashes (returns false without
// writing) if not.
bool format(char* buf, size_t bufSize,
            const Metadata& meta,
            const unsigned long* lapTimes, size_t lapCount);

// Parse the first kHeaderSize bytes of `buf` into `outMeta` plus up
// to `maxLaps` lap times in `outLaps`. Writes the actual lap count
// to `outLapCount`. Returns false on:
//   - bufSize < kHeaderSize
//   - empty line 1 (header was never written — incomplete session)
//   - missing line 2
// Lap parsing is best-effort; non-numeric / zero-valued tokens are
// silently skipped (matches the firmware's existing behavior).
bool parse(const char* buf, size_t bufSize,
           ParsedHeader& outMeta,
           unsigned long* outLaps, size_t maxLaps,
           size_t& outLapCount);

}  // namespace dovex_header
