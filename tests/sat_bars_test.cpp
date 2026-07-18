#include "doctest.h"
#include "sat_bars.h"

using namespace sat_bars;

// ---------------------------------------------------------------------------
// selectCnos — which satellites make the display
// ---------------------------------------------------------------------------

TEST_CASE("selectCnos - empty and degenerate inputs return 0") {
    uint8_t out[kMaxSats];
    SatObs one = {40, true};
    CHECK(selectCnos(nullptr, 4, out, kMaxSats) == 0);
    CHECK(selectCnos(&one, 0, out, kMaxSats) == 0);
    CHECK(selectCnos(&one, 1, nullptr, kMaxSats) == 0);
    CHECK(selectCnos(&one, 1, out, 0) == 0);
}

TEST_CASE("selectCnos - single satellite passes through") {
    SatObs obs[] = {{37, true}};
    uint8_t out[kMaxSats];
    REQUIRE(selectCnos(obs, 1, out, kMaxSats) == 1);
    CHECK(out[0] == 37);
}

TEST_CASE("selectCnos - untracked (cno 0) satellites are skipped") {
    SatObs obs[] = {{0, true}, {30, false}, {0, false}};
    uint8_t out[kMaxSats];
    REQUIRE(selectCnos(obs, 3, out, kMaxSats) == 1);
    CHECK(out[0] == 30);
}

TEST_CASE("selectCnos - used-in-nav come first, each group strongest-first") {
    SatObs obs[] = {
        {25, false}, {45, true}, {50, false}, {30, true}, {40, true},
    };
    uint8_t out[kMaxSats];
    REQUIRE(selectCnos(obs, 5, out, kMaxSats) == 5);
    // Used group sorted desc, then unused group sorted desc.
    CHECK(out[0] == 45);
    CHECK(out[1] == 40);
    CHECK(out[2] == 30);
    CHECK(out[3] == 50);
    CHECK(out[4] == 25);
}

TEST_CASE("selectCnos - cap keeps the strongest used satellites") {
    SatObs obs[20];
    for (int i = 0; i < 20; i++) {
        obs[i] = {static_cast<uint8_t>(20 + i), true};  // 20..39, all used
    }
    uint8_t out[kMaxSats];
    REQUIRE(selectCnos(obs, 20, out, kMaxSats) == kMaxSats);
    // Strongest 16 of 20 survive, descending: 39..24.
    for (int i = 0; i < kMaxSats; i++) {
        CHECK(out[i] == 39 - i);
    }
}

// ---------------------------------------------------------------------------
// layout — bar geometry on the 128x30 bottom half
// ---------------------------------------------------------------------------

TEST_CASE("layout - degenerate inputs return 0") {
    uint8_t cno[] = {40};
    Bar out[kMaxSats];
    CHECK(layout(nullptr, 1, 128, 30, out, kMaxSats) == 0);
    CHECK(layout(cno, 0, 128, 30, out, kMaxSats) == 0);
    CHECK(layout(cno, 1, 0, 30, out, kMaxSats) == 0);
    CHECK(layout(cno, 1, 128, 0, out, kMaxSats) == 0);
    CHECK(layout(cno, 1, 128, 30, nullptr, kMaxSats) == 0);
    CHECK(layout(cno, 1, 128, 30, out, 0) == 0);
}

TEST_CASE("layout - bars always fit the area for 1..16 sats on 128x30") {
    uint8_t cno[kMaxSats];
    for (int i = 0; i < kMaxSats; i++) cno[i] = 45;
    Bar out[kMaxSats];
    for (int count = 1; count <= kMaxSats; count++) {
        const int n = layout(cno, count, 128, 30, out, kMaxSats);
        REQUIRE(n == count);
        for (int i = 0; i < n; i++) {
            CHECK(out[i].w >= 2);
            CHECK(out[i].x + out[i].w <= 128);
            CHECK(out[i].h <= 30);
            if (i > 0) CHECK(out[i].x >= out[i - 1].x + out[i - 1].w + 1);
        }
    }
}

TEST_CASE("layout - height scales with CNO and clamps at the ceiling") {
    uint8_t cno[] = {0, 25, static_cast<uint8_t>(kCnoCeiling),
                     static_cast<uint8_t>(kCnoCeiling + 10)};
    Bar out[kMaxSats];
    REQUIRE(layout(cno, 4, 128, 30, out, kMaxSats) == 4);
    CHECK(out[0].h == 0);          // untracked → no bar
    CHECK(out[1].h == 15);         // 25/50 of 30
    CHECK(out[2].h == 30);         // full height
    CHECK(out[3].h == 30);         // clamped, never taller than the area
}

TEST_CASE("layout - count above maxBars is truncated") {
    uint8_t cno[20];
    for (int i = 0; i < 20; i++) cno[i] = 40;
    Bar out[kMaxSats];
    CHECK(layout(cno, 20, 128, 30, out, kMaxSats) == kMaxSats);
}

TEST_CASE("layout - impossibly narrow area sheds bars instead of overflowing") {
    uint8_t cno[] = {40, 40, 40, 40};
    Bar out[kMaxSats];
    // 5 px wide: only one 2px+ bar fits sensibly.
    const int n = layout(cno, 4, 5, 30, out, kMaxSats);
    REQUIRE(n >= 1);
    for (int i = 0; i < n; i++) {
        CHECK(out[i].x + out[i].w <= 5);
    }
}
