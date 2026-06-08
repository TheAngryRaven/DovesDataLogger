#include "doctest.h"

#include <cstring>

#include "crc32.h"

// The firmware OTA CRC must match the web companion byte-for-byte. These
// tests pin the algorithm to CRC-32 / IEEE 802.3 (zlib) and the lowercase
// 8-char hex wire format the protocol uses.

TEST_CASE("crc32 canonical sanity vector") {
  // The check value every CRC-32/ISO-HDLC implementation agrees on.
  CHECK(crc32::compute("123456789", 9) == 0xcbf43926u);
}

TEST_CASE("crc32 of empty input is zero") {
  CHECK(crc32::compute("", 0) == 0u);
}

TEST_CASE("crc32 incremental matches one-shot") {
  const char* s = "The quick brown fox jumps over the lazy dog";
  const size_t n = std::strlen(s);
  const uint32_t oneShot = crc32::compute(s, n);

  // Feed it in three uneven slices — mimics fixed-size SD reads.
  uint32_t state = crc32::kInit;
  state = crc32::update(state, s, 1);
  state = crc32::update(state, s + 1, 10);
  state = crc32::update(state, s + 11, n - 11);
  CHECK(crc32::finalize(state) == oneShot);
}

TEST_CASE("crc32 incremental with empty slices is a no-op") {
  const char* s = "123456789";
  uint32_t state = crc32::kInit;
  state = crc32::update(state, s, 0);
  state = crc32::update(state, s, 9);
  state = crc32::update(state, s + 9, 0);
  CHECK(crc32::finalize(state) == 0xcbf43926u);
}

TEST_CASE("toHex is lowercase and zero-padded to 8 chars") {
  char buf[9];

  CHECK(std::strcmp(crc32::toHex(0xcbf43926u, buf), "cbf43926") == 0);
  CHECK(std::strcmp(crc32::toHex(0x0a1b2c3du, buf), "0a1b2c3d") == 0);
  CHECK(std::strcmp(crc32::toHex(0x00000000u, buf), "00000000") == 0);
  CHECK(std::strcmp(crc32::toHex(0xffffffffu, buf), "ffffffff") == 0);
}

TEST_CASE("fromHex parses and validates 8 hex digits") {
  uint32_t v = 0;

  CHECK(crc32::fromHex("cbf43926", v));
  CHECK(v == 0xcbf43926u);

  // Case-insensitive (the protocol compares case-insensitively).
  CHECK(crc32::fromHex("CBF43926", v));
  CHECK(v == 0xcbf43926u);
  CHECK(crc32::fromHex("0A1b2C3d", v));
  CHECK(v == 0x0a1b2c3du);

  // Rejections.
  CHECK_FALSE(crc32::fromHex("cbf4392", v));    // too short
  CHECK_FALSE(crc32::fromHex("cbf439268", v));  // too long
  CHECK_FALSE(crc32::fromHex("xyz12345", v));   // non-hex
  CHECK_FALSE(crc32::fromHex("", v));           // empty
  CHECK_FALSE(crc32::fromHex(nullptr, v));      // null
}

TEST_CASE("toHex / fromHex round-trip") {
  const uint32_t values[] = {0u, 1u, 0xcbf43926u, 0xdeadbeefu, 0xffffffffu};
  for (uint32_t want : values) {
    char buf[9];
    crc32::toHex(want, buf);
    uint32_t got = 0;
    CHECK(crc32::fromHex(buf, got));
    CHECK(got == want);
  }
}
