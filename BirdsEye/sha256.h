#pragma once

#include <stddef.h>
#include <stdint.h>

///////////////////////////////////////////
// SHA-256 + HMAC-SHA256 (FIPS 180-4, RFC 2104)
//
// Pure, Arduino-free. Exists for the remote-transfer PIN handshake
// (plan 0019): the logger and both companion apps (WebCrypto in the
// viewer, the hmac/sha2 crates in LapWing) must agree on the answer
// byte-for-byte, so this unit is pinned to the FIPS 180-4 and RFC 4231
// test vectors in tests/sha256_test.cpp.
//
// Small and slow on purpose: it runs once per authentication attempt,
// never in a hot path, so code size wins over speed.
///////////////////////////////////////////

namespace sha256 {

constexpr size_t kDigestLen = 32;
constexpr size_t kBlockLen = 64;

struct Ctx {
  uint32_t h[8];
  uint64_t bitLen;
  uint8_t block[kBlockLen];
  size_t blockLen;
};

void init(Ctx& ctx);
void update(Ctx& ctx, const void* data, size_t len);
void finish(Ctx& ctx, uint8_t out[kDigestLen]);

// One-shot digest.
void hash(const void* data, size_t len, uint8_t out[kDigestLen]);

// HMAC-SHA256. Keys longer than one block are hashed first, per RFC 2104.
void hmac(const void* key, size_t keyLen, const void* msg, size_t msgLen,
          uint8_t out[kDigestLen]);

}  // namespace sha256
