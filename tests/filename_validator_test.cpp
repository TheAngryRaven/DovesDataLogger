#include "doctest.h"
#include "filename_validator.h"

#include <string>

using filename_validator::isValidFilename;
using filename_validator::kMaxBleFilenameLen;

// Default to the production limit unless a test cares about the bound.
static bool valid(const char* s) {
    return isValidFilename(s, kMaxBleFilenameLen);
}

// ---------------------------------------------------------------------------
// Accepted names — the real filenames the device actually handles.
// ---------------------------------------------------------------------------

TEST_CASE("isValidFilename - accepts real DOVEX log names") {
    CHECK(valid("20240115_1430.dovex"));
    CHECK(valid("20251231_2359.dovex"));
}

TEST_CASE("isValidFilename - accepts real track names") {
    CHECK(valid("OKC.json"));
    CHECK(valid("BMP.json"));
    CHECK(valid("PIQUET.json"));
    CHECK(valid("Silverstone.json"));
}

TEST_CASE("isValidFilename - accepts the FAT-safe character set") {
    CHECK(valid("a"));
    CHECK(valid("A"));
    CHECK(valid("0"));
    CHECK(valid("file_name-1.2.3.ext"));
    CHECK(valid("UPPER_lower-123.EXT"));
    CHECK(valid("a.b.c.d.e"));         // multiple dots are fine
    CHECK(valid("track--name__x"));    // repeated separators are fine
}

TEST_CASE("isValidFilename - dots are fine when not leading") {
    CHECK(valid("foo..bar"));   // embedded double-dot is just a name
    CHECK(valid("v1.0.dovex"));
    CHECK(valid("a."));          // trailing dot (odd but FAT-safe)
}

// ---------------------------------------------------------------------------
// Rejected — empty / null / length
// ---------------------------------------------------------------------------

TEST_CASE("isValidFilename - rejects null and empty") {
    CHECK_FALSE(isValidFilename(nullptr, kMaxBleFilenameLen));
    CHECK_FALSE(valid(""));
}

TEST_CASE("isValidFilename - enforces max length boundary") {
    // Exactly max_len passes; one more fails.
    const std::string atLimit(kMaxBleFilenameLen, 'a');
    const std::string overLimit(kMaxBleFilenameLen + 1, 'a');
    CHECK(isValidFilename(atLimit.c_str(), kMaxBleFilenameLen));
    CHECK_FALSE(isValidFilename(overLimit.c_str(), kMaxBleFilenameLen));
}

TEST_CASE("isValidFilename - respects a caller-supplied smaller limit") {
    CHECK(isValidFilename("abcd", 4));
    CHECK_FALSE(isValidFilename("abcde", 4));
}

// ---------------------------------------------------------------------------
// Rejected — path traversal & separators (the whole point of this gate)
// ---------------------------------------------------------------------------

TEST_CASE("isValidFilename - rejects leading dot (covers . .. dotfiles)") {
    CHECK_FALSE(valid("."));
    CHECK_FALSE(valid(".."));
    CHECK_FALSE(valid(".hidden"));
    CHECK_FALSE(valid(".ssh"));
}

TEST_CASE("isValidFilename - rejects directory traversal attempts") {
    CHECK_FALSE(valid("../secret"));
    CHECK_FALSE(valid("../../etc/passwd"));
    CHECK_FALSE(valid("..%2f"));       // also fails on the '%'
}

TEST_CASE("isValidFilename - rejects any slash") {
    CHECK_FALSE(valid("/absolute"));
    CHECK_FALSE(valid("sub/dir"));
    CHECK_FALSE(valid("TRACKS/OKC.json"));
    CHECK_FALSE(valid("back\\slash"));  // backslash separator too
}

// ---------------------------------------------------------------------------
// Rejected — reserved / unsafe characters
// ---------------------------------------------------------------------------

TEST_CASE("isValidFilename - rejects spaces") {
    CHECK_FALSE(valid("my file.json"));
    CHECK_FALSE(valid(" leading"));
    CHECK_FALSE(valid("trailing "));
}

TEST_CASE("isValidFilename - rejects Windows-reserved and wildcard chars") {
    CHECK_FALSE(valid("name:stream"));
    CHECK_FALSE(valid("wild*card"));
    CHECK_FALSE(valid("ques?tion"));
    CHECK_FALSE(valid("pipe|d"));
    CHECK_FALSE(valid("quo\"te"));
    CHECK_FALSE(valid("less<than"));
    CHECK_FALSE(valid("great>er"));
}

TEST_CASE("isValidFilename - rejects control bytes and high-bit bytes") {
    CHECK_FALSE(valid("tab\there"));
    CHECK_FALSE(valid("newline\nhere"));
    char ctrl[] = {'a', 0x01, 'b', '\0'};
    CHECK_FALSE(valid(ctrl));
    char highbit[] = {'a', static_cast<char>(0x80), 'b', '\0'};
    CHECK_FALSE(valid(highbit));
    char del[] = {'a', static_cast<char>(0x7F), '\0'};  // DEL
    CHECK_FALSE(valid(del));
}
