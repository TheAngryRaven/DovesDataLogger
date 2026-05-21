#pragma once

///////////////////////////////////////////
// FILENAME VALIDATION
// Gate for any filename that arrives from an untrusted source (today:
// the BLE file/track commands). BLE delivers raw bytes straight into
// SD.open() / SD.remove() / "/TRACKS/%s" path building, so without
// this an attacker in radio range could walk the filesystem
// (../, absolute paths) or wedge FAT with reserved characters.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>

namespace filename_validator {

// Maximum filename length accepted from BLE. Must fit the smallest
// on-device filename buffer (trackUploadFilename[25] -> 24 usable).
// Every real filename the device produces is far shorter: a DOVEX log
// is "20YYMMDD_HHMM.dovex" (19 chars) and tracks are like "OKC.json".
constexpr size_t kMaxBleFilenameLen = 24;

// A name is valid iff ALL of:
//   - non-null and non-empty
//   - length <= max_len
//   - first character is NOT '.' (rejects ".", "..", and dotfiles, and
//     thereby any leading-dot path-traversal attempt)
//   - every character is in the FAT-safe set [A-Za-z0-9._-]
//     (rejects '/' and '\\' path separators, drive colons, wildcards,
//     spaces, control bytes, and any high-bit byte)
//
// Note: because '/' is rejected outright there are no path components,
// so a "../" sequence can never form — the leading-dot rule plus the
// no-slash rule together make directory traversal impossible.
bool isValidFilename(const char* name, size_t max_len);

}  // namespace filename_validator
