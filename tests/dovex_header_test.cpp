#include "doctest.h"
#include "dovex_header.h"

#include <cstring>
#include <string>

using dovex_header::kHeaderSize;
using dovex_header::Metadata;
using dovex_header::ParsedHeader;

namespace {

// Helper: turn the first n bytes of a buffer into a std::string for
// easier ASSERT messages — but only stops at the first \0 byte, which
// is fine since the header is all printable + \r + \n with no NULs.
std::string asString(const char* buf, size_t n) {
    return std::string(buf, n);
}

// Helper: build a sane Metadata pointing at static strings.
Metadata fixtureMeta() {
    return Metadata{
        "2025-03-11 14:30:00",  // datetime
        "Driver",               // driver
        "Normal",               // course
        "OKC",                  // shortName
        62345UL,                // bestLapMs
        61890UL                 // optimalMs
    };
}

}  // namespace

// ---------------------------------------------------------------------------
// format() — basic structural checks
// ---------------------------------------------------------------------------

TEST_CASE("format - writes exactly kHeaderSize bytes, padded with \\n") {
    char buf[kHeaderSize];
    std::memset(buf, 'X', sizeof(buf));  // pre-fill so we can detect over/under-write

    const unsigned long laps[] = {65432, 63210, 62345};
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(),
                                 laps, 3));

    // No 'X' must survive — every byte of the header must have been written.
    for (size_t i = 0; i < kHeaderSize; i++) {
        CAPTURE(i);
        REQUIRE(buf[i] != 'X');
    }
    // Last byte should be padding \n.
    CHECK(buf[kHeaderSize - 1] == '\n');
}

TEST_CASE("format - line 1 is the column label with CRLF") {
    char buf[kHeaderSize];
    const unsigned long laps[] = {1000};
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(), laps, 1));

    const std::string expected =
        "datetime,driver,course,short_name,best_lap_ms,optimal_ms\r\n";
    CHECK(asString(buf, expected.size()) == expected);
}

TEST_CASE("format - line 2 fields appear in order") {
    char buf[kHeaderSize];
    const unsigned long laps[] = {1000};
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(), laps, 1));

    // The metadata line follows line 1's CRLF and ends with its own CRLF.
    const std::string head = asString(buf, kHeaderSize);
    CHECK(head.find("2025-03-11 14:30:00,Driver,Normal,OKC,62345,61890\r\n")
          != std::string::npos);
}

TEST_CASE("format - bestLap / optimal of 0 become 'N/A'") {
    Metadata m = fixtureMeta();
    m.bestLapMs = 0;
    m.optimalMs = 0;

    char buf[kHeaderSize];
    const unsigned long laps[] = {1000};
    REQUIRE(dovex_header::format(buf, sizeof(buf), m, laps, 1));

    const std::string head = asString(buf, kHeaderSize);
    CHECK(head.find(",N/A,N/A\r\n") != std::string::npos);
}

TEST_CASE("format - empty lap list still produces valid 1024-byte buffer") {
    char buf[kHeaderSize];
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(),
                                 nullptr, 0));
    // Layout: label1 \r\n metadata \r\n label2 \r\n \r\n then \n padding.
    const std::string head = asString(buf, kHeaderSize);
    CHECK(head.find("laps_ms\r\n\r\n") != std::string::npos);
    // Trailing padding is \n.
    CHECK(buf[kHeaderSize - 1] == '\n');
}

TEST_CASE("format - rejects too-small destination") {
    char small[kHeaderSize - 1];
    const unsigned long laps[] = {1000};
    CHECK_FALSE(dovex_header::format(small, sizeof(small),
                                     fixtureMeta(), laps, 1));
}

TEST_CASE("format - lap list truncation returns false but still pads") {
    // ~125 laps of 8-digit values won't fit in 1024 bytes.
    unsigned long manyLaps[200];
    for (size_t i = 0; i < 200; i++) manyLaps[i] = 12345678UL;

    char buf[kHeaderSize];
    const bool ok = dovex_header::format(buf, sizeof(buf), fixtureMeta(),
                                         manyLaps, 200);
    CHECK_FALSE(ok);  // truncated

    // Truncated or not, the buffer is still a complete 1024 bytes
    // and the last byte is the \n padding fill.
    CHECK(buf[kHeaderSize - 1] == '\n');

    // The truncated buffer should still parse — partial lap list is
    // recoverable data, not garbage. This is the actual contract we
    // care about: a truncated DOVEX file isn't corrupt.
    ParsedHeader meta;
    unsigned long readLaps[200];
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 200, readCount));
    CHECK(readCount > 0u);          // at least one lap survived
    CHECK(readCount < 200u);        // but not all of them
    // Every surviving lap matches the value we wrote.
    for (size_t i = 0; i < readCount; i++) {
        CHECK(readLaps[i] == 12345678UL);
    }
}

// ---------------------------------------------------------------------------
// parse() — happy path & round-trip
// ---------------------------------------------------------------------------

TEST_CASE("parse - round-trips format() output") {
    char buf[kHeaderSize];
    const unsigned long writeLaps[] = {65432UL, 63210UL, 62345UL, 64567UL};
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(),
                                 writeLaps, 4));

    ParsedHeader meta;
    unsigned long readLaps[10] = {0};
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 10, readCount));

    CHECK(std::string(meta.datetime)  == "2025-03-11 14:30:00");
    CHECK(std::string(meta.driver)    == "Driver");
    CHECK(std::string(meta.course)    == "Normal");
    CHECK(std::string(meta.shortName) == "OKC");
    CHECK(std::string(meta.bestLap)   == "62345");
    CHECK(std::string(meta.optimal)   == "61890");

    CHECK(readCount == 4u);
    CHECK(readLaps[0] == 65432UL);
    CHECK(readLaps[1] == 63210UL);
    CHECK(readLaps[2] == 62345UL);
    CHECK(readLaps[3] == 64567UL);
}

TEST_CASE("parse - round-trips zero-lap session") {
    char buf[kHeaderSize];
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(),
                                 nullptr, 0));

    ParsedHeader meta;
    unsigned long readLaps[10] = {0};
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 10, readCount));

    CHECK(std::string(meta.driver) == "Driver");
    CHECK(readCount == 0u);
}

TEST_CASE("parse - 'N/A' best/optimal pass through as strings") {
    Metadata m = fixtureMeta();
    m.bestLapMs = 0;
    m.optimalMs = 0;

    char buf[kHeaderSize];
    const unsigned long laps[] = {5000};
    REQUIRE(dovex_header::format(buf, sizeof(buf), m, laps, 1));

    ParsedHeader meta;
    unsigned long readLaps[10];
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 10, readCount));
    CHECK(std::string(meta.bestLap) == "N/A");
    CHECK(std::string(meta.optimal) == "N/A");
}

// ---------------------------------------------------------------------------
// parse() — error & degenerate cases
// ---------------------------------------------------------------------------

TEST_CASE("parse - empty header (all \\n padding) rejected") {
    // What an incomplete-session DOVEX file looks like: 1024 bytes of \n
    // because the firmware pre-fills before any metadata is written.
    char buf[kHeaderSize];
    std::memset(buf, '\n', sizeof(buf));

    ParsedHeader meta;
    unsigned long readLaps[10];
    size_t readCount = 999;  // sentinel
    CHECK_FALSE(dovex_header::parse(buf, sizeof(buf),
                                    meta, readLaps, 10, readCount));
    CHECK(readCount == 0u);
}

TEST_CASE("parse - rejects undersized buffer") {
    char small[kHeaderSize - 1];
    std::memset(small, 'a', sizeof(small));
    ParsedHeader meta;
    unsigned long readLaps[10];
    size_t readCount = 999;
    CHECK_FALSE(dovex_header::parse(small, sizeof(small),
                                    meta, readLaps, 10, readCount));
}

TEST_CASE("parse - missing lap line is OK (returns true, lapCount=0)") {
    // Build a buffer where line 3 (laps_ms) is present but line 4 is
    // empty — represents a session that ended before any laps.
    char buf[kHeaderSize];
    std::memset(buf, '\n', sizeof(buf));
    const char* hdr =
        "datetime,driver,course,short_name,best_lap_ms,optimal_ms\r\n"
        "2025-03-11 14:30:00,Driver,Normal,OKC,N/A,N/A\r\n"
        "laps_ms\r\n"
        "\r\n";  // empty lap list
    std::memcpy(buf, hdr, std::strlen(hdr));

    ParsedHeader meta;
    unsigned long readLaps[10];
    size_t readCount = 999;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 10, readCount));
    CHECK(readCount == 0u);
    CHECK(std::string(meta.driver) == "Driver");
}

TEST_CASE("parse - clamps to maxLaps") {
    // Format with 5 laps but only let parse store 3.
    char buf[kHeaderSize];
    const unsigned long laps[] = {1000, 2000, 3000, 4000, 5000};
    REQUIRE(dovex_header::format(buf, sizeof(buf), fixtureMeta(), laps, 5));

    ParsedHeader meta;
    unsigned long readLaps[3] = {0, 0, 0};
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 3, readCount));
    CHECK(readCount == 3u);
    CHECK(readLaps[0] == 1000UL);
    CHECK(readLaps[1] == 2000UL);
    CHECK(readLaps[2] == 3000UL);
}

TEST_CASE("parse - skips zero-value lap tokens (matches firmware behavior)") {
    // Build a header with a 0 in the middle of the lap list. The
    // firmware drops zeros (they shouldn't occur, but be defensive).
    char buf[kHeaderSize];
    std::memset(buf, '\n', sizeof(buf));
    const char* hdr =
        "datetime,driver,course,short_name,best_lap_ms,optimal_ms\r\n"
        "2025-03-11 14:30:00,Driver,Normal,OKC,1000,1000\r\n"
        "laps_ms\r\n"
        "1000,0,2000,0,3000\r\n";
    std::memcpy(buf, hdr, std::strlen(hdr));

    ParsedHeader meta;
    unsigned long readLaps[10];
    size_t readCount = 0;
    REQUIRE(dovex_header::parse(buf, sizeof(buf),
                                meta, readLaps, 10, readCount));
    CHECK(readCount == 3u);
    CHECK(readLaps[0] == 1000UL);
    CHECK(readLaps[1] == 2000UL);
    CHECK(readLaps[2] == 3000UL);
}
