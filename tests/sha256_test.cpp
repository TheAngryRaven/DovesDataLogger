#include "doctest.h"

#include <cstring>
#include <string>

#include "remote_auth.h"
#include "sha256.h"

// The remote-transfer handshake (plan 0019) must agree byte-for-byte with
// WebCrypto in the viewer and the hmac/sha2 crates in LapWing, so the
// primitives are pinned to the published vectors.

namespace {

std::string hex(const uint8_t* d, size_t n) {
  char buf[2 * sha256::kDigestLen + 1];
  remote_auth::toHex(d, n, buf);
  return buf;
}

std::string digestOf(const std::string& s) {
  uint8_t out[sha256::kDigestLen];
  sha256::hash(s.data(), s.size(), out);
  return hex(out, sizeof(out));
}

std::string hmacOf(const std::string& key, const std::string& msg) {
  uint8_t out[sha256::kDigestLen];
  sha256::hmac(key.data(), key.size(), msg.data(), msg.size(), out);
  return hex(out, sizeof(out));
}

}  // namespace

TEST_CASE("sha256 FIPS 180-4 vectors") {
  CHECK(digestOf("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  CHECK(digestOf("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  CHECK(digestOf("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq") ==
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST_CASE("sha256 one million 'a' through incremental updates") {
  sha256::Ctx ctx;
  sha256::init(ctx);
  const std::string chunk(1000, 'a');
  for (int i = 0; i < 1000; ++i) sha256::update(ctx, chunk.data(), chunk.size());
  uint8_t out[sha256::kDigestLen];
  sha256::finish(ctx, out);
  CHECK(hex(out, sizeof(out)) ==
        "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_CASE("sha256 padding boundaries (55/56/64 bytes)") {
  // 55 bytes fits the length in the same block; 56 spills into a second.
  CHECK(digestOf(std::string(55, 'a')) ==
        "9f4390f8d30c2dd92ec9f095b65e2b9ae9b0a925a5258e241c9f1e910f734318");
  CHECK(digestOf(std::string(56, 'a')) ==
        "b35439a4ac6f0948b6d6f9e3c6af0f5f590ce20f1bde7090ef7970686ec6738a");
  CHECK(digestOf(std::string(64, 'a')) ==
        "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");
}

TEST_CASE("hmac-sha256 RFC 4231 test cases") {
  // Case 1
  CHECK(hmacOf(std::string(20, '\x0b'), "Hi There") ==
        "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
  // Case 2
  CHECK(hmacOf("Jefe", "what do ya want for nothing?") ==
        "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
  // Case 3
  CHECK(hmacOf(std::string(20, '\xaa'), std::string(50, '\xdd')) ==
        "773ea91e36800e46854db8ebd09181a72959098b3ef8c122d9635514ced565fe");
  // Case 6: key longer than one block is hashed first
  CHECK(hmacOf(std::string(131, '\xaa'),
               "Test Using Larger Than Block-Size Key - Hash Key First") ==
        "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
}
