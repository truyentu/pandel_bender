#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/zobrist_table.h"
#include <unordered_set>

using namespace openpanelcam::phase3;

TEST_CASE("ZobristTable produces consistent hashes", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState state;
    state.bentMask = 0b1010;
    state.orientation = Orientation::DEG_90;
    state.abaConfig = 100;

    uint64_t hash1 = table.hash(state);
    uint64_t hash2 = table.hash(state);

    REQUIRE(hash1 == hash2);
}

TEST_CASE("ZobristTable different states have different hashes", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.bentMask = 0b1010;
    s2.bentMask = 0b1011;

    REQUIRE(table.hash(s1) != table.hash(s2));
}

TEST_CASE("ZobristTable orientation changes hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.orientation = Orientation::DEG_0;
    s2.orientation = Orientation::DEG_90;

    REQUIRE(table.hash(s1) != table.hash(s2));
}

TEST_CASE("ZobristTable incremental update equals full hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState before;
    before.bentMask = 0b1010;
    before.orientation = Orientation::DEG_0;

    uint64_t hashBefore = table.hash(before);

    // Apply bend 2
    SearchState after = before;
    after.markBent(2);

    uint64_t hashFull = table.hash(after);
    uint64_t hashIncremental = table.updateBend(hashBefore, 2);

    REQUIRE(hashFull == hashIncremental);
}

TEST_CASE("ZobristTable orientation update is incremental", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState state;
    state.orientation = Orientation::DEG_0;

    uint64_t h1 = table.hash(state);

    state.orientation = Orientation::DEG_90;
    uint64_t h2Full = table.hash(state);

    uint64_t h2Inc = table.updateOrientation(h1, Orientation::DEG_0, Orientation::DEG_90);

    REQUIRE(h2Full == h2Inc);
}

TEST_CASE("ZobristTable low collision rate", "[phase3][zobrist]") {
    ZobristTable table(12345);
    std::unordered_set<uint64_t> hashes;

    // Generate 1024 different states
    for (int mask = 0; mask < 256; mask++) {
        for (int ori = 0; ori < 4; ori++) {
            SearchState state;
            state.bentMask = mask;
            state.orientation = static_cast<Orientation>(ori);
            hashes.insert(table.hash(state));
        }
    }

    // Allow up to 1% collision rate
    REQUIRE(hashes.size() >= 1000);
}

TEST_CASE("ZobristTable ABA config affects hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.abaConfig = 100;
    s2.abaConfig = 200;

    REQUIRE(table.hash(s1) != table.hash(s2));
}

TEST_CASE("ZobristTable ABA incremental update", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState state;
    state.abaConfig = 50;
    uint64_t h1 = table.hash(state);

    state.abaConfig = 150;
    uint64_t h2Full = table.hash(state);

    uint64_t h2Inc = table.updateAba(h1, 50, 150);

    REQUIRE(h2Full == h2Inc);
}
