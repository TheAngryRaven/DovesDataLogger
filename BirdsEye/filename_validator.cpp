#include "filename_validator.h"

#include <string.h>

namespace filename_validator {

bool isValidFilename(const char* name, size_t max_len) {
  if (name == nullptr) return false;

  const size_t len = strlen(name);
  if (len == 0) return false;
  if (len > max_len) return false;

  // Leading dot covers ".", "..", and hidden dotfiles. Combined with
  // the no-slash rule below this makes "../" traversal impossible.
  if (name[0] == '.') return false;

  for (size_t i = 0; i < len; i++) {
    const unsigned char c = static_cast<unsigned char>(name[i]);
    const bool ok = (c >= 'A' && c <= 'Z') ||
                    (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') ||
                    c == '.' || c == '_' || c == '-';
    if (!ok) return false;
  }

  return true;
}

}  // namespace filename_validator
