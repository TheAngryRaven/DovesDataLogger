#include "crc32.h"

namespace crc32 {

uint32_t update(uint32_t crc, const void* data, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < len; ++i) {
    crc ^= p[i];
    for (int bit = 0; bit < 8; ++bit) {
      // Branchless reflected division step: subtract the polynomial only
      // when the low bit is set. (~(crc & 1) + 1) is 0xFFFFFFFF if the low
      // bit is 1, else 0 — a portable mask with no conditional branch.
      crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
    }
  }
  return crc;
}

uint32_t compute(const void* data, size_t len) {
  return finalize(update(kInit, data, len));
}

char* toHex(uint32_t crc, char* out) {
  static const char kHex[] = "0123456789abcdef";
  for (int i = 0; i < 8; ++i) {
    out[i] = kHex[(crc >> ((7 - i) * 4)) & 0xFu];
  }
  out[8] = '\0';
  return out;
}

bool fromHex(const char* str, uint32_t& out) {
  if (str == nullptr) return false;
  uint32_t value = 0;
  for (int i = 0; i < 8; ++i) {
    const char c = str[i];
    uint32_t digit;
    if (c >= '0' && c <= '9') {
      digit = static_cast<uint32_t>(c - '0');
    } else if (c >= 'a' && c <= 'f') {
      digit = static_cast<uint32_t>(c - 'a' + 10);
    } else if (c >= 'A' && c <= 'F') {
      digit = static_cast<uint32_t>(c - 'A' + 10);
    } else {
      return false;  // not a hex digit (also catches a short string's NUL)
    }
    value = (value << 4) | digit;
  }
  if (str[8] != '\0') return false;  // reject anything longer than 8 digits
  out = value;
  return true;
}

}  // namespace crc32
