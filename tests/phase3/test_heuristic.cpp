#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/heuristic.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ===== Helper =====

static std::vector<BendFeature> makeBends(int count, double length = 100.0) {
    std::vector<BendFeature> bends(count);
    for (int i = 0; i < count; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = length;
        bends[i].direction = {1.0, 0.0, 0.0};  // All same direction
    }
    return bends;
}

// ===== Heuristic::getRemainingBends =====

TEST_CASE("getRemainingBends filters correctly", "[phase3][heuristic]") {
    auto bends = makeBends(5);

    SearchState state;
    state.bentMask = 0b01010;  // Bends 1 and 3 done

    auto remaining = Heuristic::getRemainingBends(state, bends);

    REQUIRE(remaining.size() == 3);

    bool has0 = false, has2 = false, has4 = false;
    for (const auto& b : remaining) {
        if (b.id == 0) has0 = true;
        if (b.id == 2) has2 = true;
        if (b.id == 4) has4 = true;
    }
    REQUIRE(has0);
    REQUIRE(has2);
    REQUIRE(has4);
}

TEST_CASE("getRemainingBends returns empty when all bent", "[phase3][heuristic]") {
    auto bends = makeBends(3);

    SearchState state;
    state.bentMask = 0b111;

    auto remaining = Heuristic::getRemainingBends(state, bends);
    REQUIRE(remaining.empty());
}

// ===== H1: RotationalEntropyHeuristic =====

TEST_CASE("H1 returns 0 for no remaining bends", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    auto bends = makeBends(3);
    SearchState state;
    state.bentMask = 0b111;

    REQUIRE(h1.estimate(state, bends) == 0.0);
}

TEST_CASE("H1 returns 0 for same orientation", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    // All bends same direction -> same orientation
    auto bends = makeBends(4);

    SearchState state;  // Nothing bent
    REQUIRE(h1.estimate(state, bends) == 0.0);
}

TEST_CASE("H1 penalizes diverse orientations", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};   // DEG_0
    bends[1].id = 1; bends[1].direction = {0, 1, 0};   // DEG_90
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};  // DEG_180
    bends[3].id = 3; bends[3].direction = {0, -1, 0};  // DEG_270

    SearchState state;

    double h = h1.estimate(state, bends);
    // 4 distinct orientations -> (4-1) * 0.8 = 2.4
    REQUIRE(h == Approx(2.4));
}

TEST_CASE("H1 decreases as bends complete", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};
    bends[3].id = 3; bends[3].direction = {0, -1, 0};

    SearchState s0;  // 0 bent
    SearchState s1;
    s1.markBent(0);  // 1 bent -> 3 remaining with 3 distinct

    double h0 = h1.estimate(s0, bends);
    double h1val = h1.estimate(s1, bends);

    REQUIRE(h0 >= h1val);
}

TEST_CASE("H1 with custom weight", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1(2.0);

    std::vector<BendFeature> bends(2);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};

    SearchState state;
    double h = h1.estimate(state, bends);
    // 2 distinct -> (2-1) * 2.0 = 2.0
    REQUIRE(h == Approx(2.0));
}

// ===== H2: ToolingVarianceHeuristic =====

TEST_CASE("H2 returns 0 for single remaining bend", "[phase3][heuristic][h2]") {
    ToolingVarianceHeuristic h2;

    auto bends = makeBends(3);
    SearchState state;
    state.bentMask = 0b110;  // Only bend 0 remaining

    REQUIRE(h2.estimate(state, bends) == 0.0);
}

TEST_CASE("H2 returns 0 for same lengths", "[phase3][heuristic][h2]") {
    ToolingVarianceHeuristic h2;

    auto bends = makeBends(4, 100.0);  // All same length

    SearchState state;
    REQUIRE(h2.estimate(state, bends) == 0.0);
}

TEST_CASE("H2 penalizes different lengths", "[phase3][heuristic][h2]") {
    ToolingVarianceHeuristic h2;

    std::vector<BendFeature> bends(3);
    bends[0].id = 0; bends[0].length = 50.0;
    bends[1].id = 1; bends[1].length = 200.0;
    bends[2].id = 2; bends[2].length = 500.0;

    SearchState state;
    double h = h2.estimate(state, bends);
    REQUIRE(h > 0.0);
}

TEST_CASE("H2 higher variance = higher estimate", "[phase3][heuristic][h2]") {
    ToolingVarianceHeuristic h2;

    // Low variance
    std::vector<BendFeature> lowVar(3);
    lowVar[0].id = 0; lowVar[0].length = 100.0;
    lowVar[1].id = 1; lowVar[1].length = 110.0;
    lowVar[2].id = 2; lowVar[2].length = 105.0;

    // High variance
    std::vector<BendFeature> highVar(3);
    highVar[0].id = 0; highVar[0].length = 50.0;
    highVar[1].id = 1; highVar[1].length = 500.0;
    highVar[2].id = 2; highVar[2].length = 200.0;

    SearchState state;
    REQUIRE(h2.estimate(state, highVar) > h2.estimate(state, lowVar));
}

// ===== H3: GraspFragmentationHeuristic =====

TEST_CASE("H3 returns 0 for 0-1 remaining bends", "[phase3][heuristic][h3]") {
    GraspFragmentationHeuristic h3;

    auto bends = makeBends(3);

    SearchState allDone;
    allDone.bentMask = 0b111;
    REQUIRE(h3.estimate(allDone, bends) == 0.0);

    SearchState oneLeft;
    oneLeft.bentMask = 0b110;
    REQUIRE(h3.estimate(oneLeft, bends) == 0.0);
}

TEST_CASE("H3 increases with more remaining bends", "[phase3][heuristic][h3]") {
    GraspFragmentationHeuristic h3;

    auto bends = makeBends(9);

    SearchState few;       // 6 remaining
    few.bentMask = 0b111;

    SearchState many;      // 9 remaining
    // nothing bent

    REQUIRE(h3.estimate(many, bends) >= h3.estimate(few, bends));
}

TEST_CASE("H3 with 6 remaining bends", "[phase3][heuristic][h3]") {
    GraspFragmentationHeuristic h3;

    auto bends = makeBends(6);
    SearchState state;

    double h = h3.estimate(state, bends);
    // 6 / 3 = 2, * 2.0 = 4.0
    REQUIRE(h == Approx(4.0));
}

// ===== CombinedHeuristic =====

TEST_CASE("CombinedHeuristic returns 0 at goal", "[phase3][heuristic][combined]") {
    CombinedHeuristic combined;

    auto bends = makeBends(3);
    SearchState goal;
    goal.bentMask = 0b111;

    REQUIRE(combined.estimate(goal, bends) == 0.0);
}

TEST_CASE("CombinedHeuristic sums all three", "[phase3][heuristic][combined]") {
    CombinedHeuristic combined;
    RotationalEntropyHeuristic h1;
    ToolingVarianceHeuristic h2;
    GraspFragmentationHeuristic h3;

    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};  bends[0].length = 100.0;
    bends[1].id = 1; bends[1].direction = {0, 1, 0};  bends[1].length = 200.0;
    bends[2].id = 2; bends[2].direction = {-1, 0, 0}; bends[2].length = 300.0;
    bends[3].id = 3; bends[3].direction = {0, -1, 0}; bends[3].length = 400.0;

    SearchState state;

    double expected = h1.estimate(state, bends)
                    + h2.estimate(state, bends)
                    + h3.estimate(state, bends);

    REQUIRE(combined.estimate(state, bends) == Approx(expected));
}

TEST_CASE("CombinedHeuristic is non-negative", "[phase3][heuristic][combined]") {
    CombinedHeuristic combined;

    auto bends = makeBends(7);

    // Test various states
    for (uint32_t mask = 0; mask < 128; mask += 13) {
        SearchState state;
        state.bentMask = mask;

        REQUIRE(combined.estimate(state, bends) >= 0.0);
    }
}

TEST_CASE("CombinedHeuristic monotonically decreases toward goal", "[phase3][heuristic][combined]") {
    CombinedHeuristic combined;

    auto bends = makeBends(4);
    for (auto& b : bends) {
        b.direction = {1, 0, 0};
        b.length = 100.0;
    }

    SearchState s0;
    SearchState s1; s1.markBent(0);
    SearchState s2; s2.markBent(0); s2.markBent(1);
    SearchState s3; s3.markBent(0); s3.markBent(1); s3.markBent(2);
    SearchState s4; s4.markBent(0); s4.markBent(1); s4.markBent(2); s4.markBent(3);

    double h0 = combined.estimate(s0, bends);
    double h1 = combined.estimate(s1, bends);
    double h2 = combined.estimate(s2, bends);
    double h3 = combined.estimate(s3, bends);
    double h4 = combined.estimate(s4, bends);

    REQUIRE(h0 >= h1);
    REQUIRE(h1 >= h2);
    REQUIRE(h2 >= h3);
    REQUIRE(h3 >= h4);
    REQUIRE(h4 == 0.0);
}
