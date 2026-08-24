#include "setting_parse.h"

#include <limits.h>

namespace setting_parse {
namespace {

bool isSpace(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' ||
         c == '\f';
}

}  // namespace

bool parseIntSetting(const char* s, int* out) {
  if (s == nullptr || out == nullptr) {
    return false;
  }
  const char* p = s;
  while (isSpace(*p)) {
    p++;
  }
  bool negative = false;
  if (*p == '+' || *p == '-') {
    negative = (*p == '-');
    p++;
  }
  if (*p < '0' || *p > '9') {
    return false;  // no digits at all — "" / " " / "abc" / "-" / "+"
  }
  // Accumulate in long long so overflow is detected rather than wrapped.
  // A settings file is hand-editable, so "99999999999" must be rejected,
  // not folded into some in-range value.
  long long acc = 0;
  while (*p >= '0' && *p <= '9') {
    acc = acc * 10 + (*p - '0');
    if (acc > 4294967296LL) {
      return false;  // far past any int; stop before acc itself overflows
    }
    p++;
  }
  while (isSpace(*p)) {
    p++;
  }
  if (*p != '\0') {
    return false;  // trailing junk — "12abc", "1.5", "0x10"
  }
  const long long value = negative ? -acc : acc;
  if (value < (long long)INT_MIN || value > (long long)INT_MAX) {
    return false;
  }
  *out = (int)value;
  return true;
}

}  // namespace setting_parse
