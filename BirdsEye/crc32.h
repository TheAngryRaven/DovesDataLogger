#pragma once

#include <stddef.h>
#include <stdint.h>

///////////////////////////////////////////
// CRC-32 / IEEE 802.3 (zlib)
//
// Pure, Arduino-free implementation extracted so the firmware OTA path and
// the host unit tests share ONE byte-for-byte definition. The web companion
// (DovesDataViewer) computes the firmware image CRC with the same algorithm,
// so the firmware MUST match it exactly:
//
//   reflected polynomial 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF,
//   reflected in/out.
//
// Sanity vector: crc32::compute("123456789", 9) == 0xcbf43926.
//
// The wire format is the CRC as a lowercase, zero-padded 8-char hex string
// (e.g. "0a1b2c3d"), compared case-insensitively.
///////////////////////////////////////////

namespace crc32 {

// Seed value for an incremental CRC. Feed chunks through update(), then
// finalize() the running state to get the output CRC. Incremental update
// lets us CRC a multi-hundred-KB SD file in fixed-size reads without ever
// holding the whole image in RAM.
constexpr uint32_t kInit = 0xFFFFFFFFu;

// Fold `len` bytes of `data` into the running CRC state and return the new
// state. `state` starts at kInit.
uint32_t update(uint32_t state, const void* data, size_t len);

// Apply the final XOR to a running state to produce the output CRC.
inline uint32_t finalize(uint32_t state) { return state ^ 0xFFFFFFFFu; }

// One-shot CRC over a single contiguous buffer.
uint32_t compute(const void* data, size_t len);

// Format `crc` as a lowercase, zero-padded 8-char hex string into `out`,
// which must hold at least 9 bytes (8 digits + NUL). Returns `out`.
char* toHex(uint32_t crc, char* out);

// Parse exactly 8 hex digits (case-insensitive) from `str` into `out`.
// Returns false unless `str` is precisely 8 hex characters followed by NUL.
bool fromHex(const char* str, uint32_t& out);

}  // namespace crc32
