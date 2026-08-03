#include "track_json.h"

#include <string.h>

namespace track_json {

using course_creator::CourseKind;
using course_creator::Line;
using course_creator::LineId;
using course_creator::lineDone;
using course_creator::lineOf;

namespace {

// Small append helper shared by the emitters. Tracks a write cursor and
// latches failure, so each emitter can append freely and check once at the
// end instead of testing every call.
struct Writer {
  char* out;
  size_t size;
  size_t used = 0;
  bool overflowed = false;

  Writer(char* o, size_t s) : out(o), size(s) {
    if (out == nullptr || size == 0) overflowed = true;
  }

  void put(const char* text, size_t len) {
    if (overflowed) return;
    if (used + len + 1 > size) {  // +1 keeps room for the NUL
      overflowed = true;
      return;
    }
    memcpy(out + used, text, len);
    used += len;
  }

  void put(const char* text) { put(text, strlen(text)); }

  void coord(double value) {
    if (overflowed) return;
    char buf[kCoordSize];
    const int n = formatFixed(buf, sizeof(buf), value, kCoordDecimals);
    if (n < 0) {
      overflowed = true;
      return;
    }
    put(buf, static_cast<size_t>(n));
  }

  // "key":value — the only shape this file emits for numbers.
  void coordField(const char* key, double value) {
    put("\"");
    put(key);
    put("\":");
    coord(value);
  }

  void stringField(const char* key, const char* value) {
    put("\"");
    put(key);
    put("\":\"");
    put(value ? value : "");
    put("\"");
  }

  int finish() {
    if (overflowed) return -1;
    out[used] = '\0';
    return static_cast<int>(used);
  }
};

// Longest field name emitted: "sector_2_a_lat" (14) + NUL.
constexpr size_t kFieldKeySize = 20;

// prefix + suffix into `key`. Truncation is impossible with the prefixes
// this file uses, but the bound keeps that true if one is ever added.
void buildKey(char* key, size_t keySize, const char* prefix, const char* suffix) {
  const size_t p = strlen(prefix);
  const size_t s = strlen(suffix);
  if (p + s + 1 > keySize) {
    key[0] = '\0';
    return;
  }
  memcpy(key, prefix, p);
  memcpy(key + p, suffix, s);
  key[p + s] = '\0';
}

// Emit the four coordinates of one line under a field prefix, e.g.
// "sector_2" -> sector_2_a_lat / _a_lng / _b_lat / _b_lng.
void putLine(Writer& w, const char* prefix, const Line& line) {
  static const char* const kSuffixes[] = {"_a_lat", "_a_lng", "_b_lat", "_b_lng"};
  const double values[] = {line.aLat, line.aLon, line.bLat, line.bLon};

  char key[kFieldKeySize];
  for (uint8_t i = 0; i < 4; i++) {
    if (i > 0) w.put(",");
    buildKey(key, sizeof(key), prefix, kSuffixes[i]);
    w.coordField(key, values[i]);
  }
}

}  // namespace

int formatFixed(char* out, size_t outSize, double value, uint8_t decimals) {
  if (out == nullptr || outSize == 0 || decimals > 9) return -1;

  bool negative = value < 0.0;
  if (negative) value = -value;

  // 10^decimals — small enough that the loop beats pulling in pow().
  int64_t scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10;

  // Round half away from zero. Coordinates scaled by 1e8 stay far inside
  // the range where a double holds an integer exactly, so this is lossless.
  const int64_t scaled = static_cast<int64_t>(value * static_cast<double>(scale) + 0.5);
  const int64_t whole = scaled / scale;
  int64_t frac = scaled % scale;

  // Build back-to-front into a scratch buffer, then copy.
  char tmp[32];
  size_t n = 0;

  for (uint8_t i = 0; i < decimals; i++) {
    tmp[n++] = static_cast<char>('0' + static_cast<int>(frac % 10));
    frac /= 10;
  }
  if (decimals > 0) tmp[n++] = '.';

  int64_t w = whole;
  if (w == 0) {
    tmp[n++] = '0';
  } else {
    while (w > 0 && n < sizeof(tmp)) {
      tmp[n++] = static_cast<char>('0' + static_cast<int>(w % 10));
      w /= 10;
    }
  }
  // A value that rounds to zero is not negative — "-0.00000000" would be
  // read back as a real coordinate at the equator, which it is.
  if (negative && scaled != 0) tmp[n++] = '-';

  if (n + 1 > outSize) return -1;
  for (size_t i = 0; i < n; i++) out[i] = tmp[n - 1 - i];
  out[n] = '\0';
  return static_cast<int>(n);
}

int formatCourse(char* out, size_t outSize,
                 const course_creator::State& course,
                 const char* courseName,
                 const char* dateCreated) {
  Writer w(out, outSize);
  const bool sprint = course.kind == CourseKind::kSprint;

  w.put("{");
  w.stringField("name", courseName);
  w.put(",");

  putLine(w, "start", lineOf(course, LineId::kStart));

  // Optional lines are emitted only when captured — parseTrackFile()
  // probes for them with containsKey(), so an absent line and a line of
  // zeroes are very different things.
  const Line& s2 = lineOf(course, LineId::kSector2);
  if (lineDone(s2)) {
    w.put(",");
    putLine(w, "sector_2", s2);
  }
  const Line& s3 = lineOf(course, LineId::kSector3);
  if (lineDone(s3)) {
    w.put(",");
    putLine(w, "sector_3", s3);
  }
  if (sprint) {
    const Line& fin = lineOf(course, LineId::kFinish);
    if (lineDone(fin)) {
      w.put(",");
      putLine(w, "finish", fin);
    }
    // Sprint-only by contract: the webapp documents Course.dateCreated as
    // sprint-only, and sprint_select is the only reader.
    if (dateCreated != nullptr && dateCreated[0] != '\0') {
      w.put(",");
      w.stringField("date_created", dateCreated);
    }
  }

  w.put("}");
  return w.finish();
}

int formatTrackFile(char* out, size_t outSize,
                    const char* longName, const char* shortName,
                    const course_creator::State& course,
                    const char* courseName,
                    const char* dateCreated) {
  Writer w(out, outSize);
  const bool sprint = course.kind == CourseKind::kSprint;

  w.put("{");
  w.stringField("longName", longName);
  w.put(",");
  w.stringField("shortName", shortName);
  w.put(",");
  if (sprint) {
    // Redundant with the folder (which is authoritative) but the webapp
    // validates against it, so a file that claims what it is travels
    // correctly even if someone moves it by hand.
    w.stringField("type", "sprint");
    w.put(",");
  }
  w.stringField("defaultCourse", courseName);
  w.put(",\"courses\":[");
  if (w.finish() < 0) return -1;

  const int courseLen = formatCourse(out + w.used, outSize - w.used,
                                     course, courseName, dateCreated);
  if (courseLen < 0) return -1;
  w.used += static_cast<size_t>(courseLen);

  w.put("]}");
  return w.finish();
}

}  // namespace track_json
