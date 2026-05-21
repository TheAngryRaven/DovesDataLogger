#include "dovex_header.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace dovex_header {

namespace {

// Copy a substring [src, src+len) into dst (sized dstSize), null-
// terminating. Truncates if needed.
void boundedCopy(char* dst, size_t dstSize, const char* src, size_t len) {
  if (dstSize == 0) return;
  const size_t toCopy = (len < dstSize - 1) ? len : (dstSize - 1);
  memcpy(dst, src, toCopy);
  dst[toCopy] = '\0';
}

// Lap-number formatter: "N/A" if zero, else the decimal value. Returns
// the number of characters written (excluding null terminator).
int formatLapField(char* out, size_t outSize, unsigned long value) {
  if (value == 0) {
    if (outSize < 4) { if (outSize) out[0] = '\0'; return 0; }
    out[0] = 'N'; out[1] = '/'; out[2] = 'A'; out[3] = '\0';
    return 3;
  }
  return snprintf(out, outSize, "%lu", value);
}

}  // namespace

bool format(char* buf, size_t bufSize,
            const Metadata& meta,
            const unsigned long* lapTimes, size_t lapCount) {
  if (buf == nullptr || bufSize < kHeaderSize) return false;

  char* const end = buf + kHeaderSize;
  char* p = buf;

  // Helper: append a string. Returns false if it doesn't fit.
  auto append = [&](const char* s) -> bool {
    const size_t len = strlen(s);
    if (p + len > end) return false;
    // Intentionally NOT null-terminated: we're building a fixed 1024-byte
    // region that gets \n padding, not a C string.
    memcpy(p, s, len);  // NOLINT(bugprone-not-null-terminated-result)
    p += len;
    return true;
  };

  // Line 1: column labels (CRLF — matches Arduino Print::println).
  if (!append("datetime,driver,course,short_name,best_lap_ms,optimal_ms\r\n")) {
    return false;
  }

  // Line 2: metadata values.
  char bestStr[kLapStrLen];
  char optStr[kLapStrLen];
  formatLapField(bestStr, sizeof(bestStr), meta.bestLapMs);
  formatLapField(optStr,  sizeof(optStr),  meta.optimalMs);

  const int n = snprintf(p, static_cast<size_t>(end - p),
                          "%s,%s,%s,%s,%s,%s\r\n",
                          meta.datetime ? meta.datetime : "",
                          meta.driver   ? meta.driver   : "",
                          meta.course   ? meta.course   : "",
                          meta.shortName? meta.shortName: "",
                          bestStr, optStr);
  if (n < 0 || p + n > end) return false;
  p += n;

  // Line 3: lap column label.
  if (!append("laps_ms\r\n")) return false;

  // Line 4: comma-separated lap times. Truncate gracefully — bail out
  // the moment the next entry would overflow, but always leave room
  // for the trailing CRLF + padding.
  bool truncated = false;
  for (size_t i = 0; i < lapCount; i++) {
    char lapBuf[24];
    int lapLen;
    if (i == 0) {
      lapLen = snprintf(lapBuf, sizeof(lapBuf), "%lu", lapTimes[i]);
    } else {
      lapLen = snprintf(lapBuf, sizeof(lapBuf), ",%lu", lapTimes[i]);
    }
    if (lapLen < 0) { truncated = true; break; }
    // Reserve 2 bytes at the end for the CRLF after the lap list.
    if (p + lapLen + 2 > end) { truncated = true; break; }
    memcpy(p, lapBuf, lapLen);
    p += lapLen;
  }
  if (p + 2 <= end) {
    *p++ = '\r';
    *p++ = '\n';
  }

  // Pad remaining bytes with '\n' (matches the existing firmware).
  while (p < end) {
    *p++ = '\n';
  }

  return !truncated;
}

bool parse(const char* buf, size_t bufSize,
           ParsedHeader& outMeta,
           unsigned long* outLaps, size_t maxLaps,
           size_t& outLapCount) {
  outLapCount = 0;
  // Pre-clear strings so partial parses don't leak stale data.
  outMeta.datetime[0]  = '\0';
  outMeta.driver[0]    = '\0';
  outMeta.course[0]    = '\0';
  outMeta.shortName[0] = '\0';
  outMeta.bestLap[0]   = '\0';
  outMeta.optimal[0]   = '\0';

  if (buf == nullptr || bufSize < kHeaderSize) return false;

  // Walk by lines. \r is dropped, \n terminates. End-of-region is
  // the kHeaderSize boundary.
  const char* const end = buf + kHeaderSize;
  const char* p = buf;

  auto nextLine = [&](const char*& outBegin, size_t& outLen) -> bool {
    if (p >= end) return false;
    const char* start = p;
    while (p < end && *p != '\n') p++;
    outBegin = start;
    outLen = static_cast<size_t>(p - start);
    // Strip a trailing \r.
    if (outLen > 0 && outBegin[outLen - 1] == '\r') outLen--;
    if (p < end) p++;  // step past \n
    return true;
  };

  const char* lineBegin;
  size_t      lineLen;

  // Line 1 (column labels). Reject when blank — that means the
  // metadata was never written (crash mid-session) and we should bail.
  if (!nextLine(lineBegin, lineLen)) return false;
  if (lineLen == 0) return false;

  // Line 2 (metadata values).
  if (!nextLine(lineBegin, lineLen)) return false;

  // Split line 2 by commas into a local copy we can NUL-poke.
  char metaBuf[256];
  boundedCopy(metaBuf, sizeof(metaBuf), lineBegin, lineLen);

  char* tok = strtok(metaBuf, ",");
  if (tok) boundedCopy(outMeta.datetime,  sizeof(outMeta.datetime),  tok, strlen(tok));
  tok = strtok(nullptr, ",");
  if (tok) boundedCopy(outMeta.driver,    sizeof(outMeta.driver),    tok, strlen(tok));
  tok = strtok(nullptr, ",");
  if (tok) boundedCopy(outMeta.course,    sizeof(outMeta.course),    tok, strlen(tok));
  tok = strtok(nullptr, ",");
  if (tok) boundedCopy(outMeta.shortName, sizeof(outMeta.shortName), tok, strlen(tok));
  tok = strtok(nullptr, ",");
  if (tok) boundedCopy(outMeta.bestLap,   sizeof(outMeta.bestLap),   tok, strlen(tok));
  tok = strtok(nullptr, ",");
  if (tok) boundedCopy(outMeta.optimal,   sizeof(outMeta.optimal),   tok, strlen(tok));

  // Line 3 (lap column label) — skip; tolerate missing.
  if (!nextLine(lineBegin, lineLen)) return true;  // no laps recorded

  // Line 4 (lap times).
  if (!nextLine(lineBegin, lineLen)) return true;  // no laps recorded
  if (lineLen == 0) return true;

  // Sized to hold any lap line that can fit in the header — up to
  // kHeaderSize minus the four other lines' bytes.
  char lapsBuf[kHeaderSize];
  boundedCopy(lapsBuf, sizeof(lapsBuf), lineBegin, lineLen);

  tok = strtok(lapsBuf, ",");
  while (tok != nullptr && outLapCount < maxLaps) {
    const unsigned long v = strtoul(tok, nullptr, 10);
    if (v > 0) {
      outLaps[outLapCount++] = v;
    }
    tok = strtok(nullptr, ",");
  }

  return true;
}

}  // namespace dovex_header
