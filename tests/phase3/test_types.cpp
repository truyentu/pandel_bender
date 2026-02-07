#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/types.h"

using namespace openpanelcam::phase3;

TEST_CASE("Orientation enum values", "[phase3][types]") {
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_0) == 0);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_90) == 1);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_180) == 2);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_270) == 3);
}

TEST_CASE("orientationToDegrees converts correctly", "[phase3][types]") {
    REQUIRE(orientationToDegrees(Orientation::DEG_0) == 0.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_90) == 90.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_180) == 180.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_270) == 270.0);
}

TEST_CASE("rotateClockwise cycles through orientations", "[phase3][types]") {
    REQUIRE(rotateClockwise(Orientation::DEG_0) == Orientation::DEG_90);
    REQUIRE(rotateClockwise(Orientation::DEG_90) == Orientation::DEG_180);
    REQUIRE(rotateClockwise(Orientation::DEG_180) == Orientation::DEG_270);
    REQUIRE(rotateClockwise(Orientation::DEG_270) == Orientation::DEG_0);
}

TEST_CASE("rotateOpposite returns 180 degree rotation", "[phase3][types]") {
    REQUIRE(rotateOpposite(Orientation::DEG_0) == Orientation::DEG_180);
    REQUIRE(rotateOpposite(Orientation::DEG_90) == Orientation::DEG_270);
    REQUIRE(rotateOpposite(Orientation::DEG_180) == Orientation::DEG_0);
    REQUIRE(rotateOpposite(Orientation::DEG_270) == Orientation::DEG_90);
}

TEST_CASE("ActionType enum exists", "[phase3][types]") {
    ActionType bend = ActionType::BEND;
    ActionType rotate = ActionType::ROTATE;
    ActionType aba = ActionType::ABA_RECONFIG;
    ActionType repo = ActionType::REPOSITION;

    REQUIRE(bend != rotate);
    REQUIRE(rotate != aba);
    REQUIRE(aba != repo);
}

TEST_CASE("RepoReason enum exists", "[phase3][types]") {
    REQUIRE(RepoReason::NONE != RepoReason::GRIP_AREA_EXHAUSTED);
    REQUIRE(RepoReason::BOX_CLOSING != RepoReason::COM_OUTSIDE_GRIP);
}

// ===== SearchState Tests =====

TEST_CASE("SearchState default initialization", "[phase3][types][state]") {
    SearchState state;

    REQUIRE(state.bentMask == 0);
    REQUIRE(state.orientation == Orientation::DEG_0);
    REQUIRE(state.abaConfig == 0);
    REQUIRE(state.needsRepo == false);
    REQUIRE(state.repoReason == RepoReason::NONE);
}

TEST_CASE("SearchState isBent checks bitmask", "[phase3][types][state]") {
    SearchState state;
    state.bentMask = 0b1010;  // Bends 1 and 3 are done

    REQUIRE(state.isBent(0) == false);
    REQUIRE(state.isBent(1) == true);
    REQUIRE(state.isBent(2) == false);
    REQUIRE(state.isBent(3) == true);
    REQUIRE(state.isBent(4) == false);

    // Edge cases
    REQUIRE(state.isBent(-1) == false);
    REQUIRE(state.isBent(32) == false);
}

TEST_CASE("SearchState markBent sets bit", "[phase3][types][state]") {
    SearchState state;

    state.markBent(0);
    REQUIRE(state.bentMask == 0b0001);

    state.markBent(2);
    REQUIRE(state.bentMask == 0b0101);

    state.markBent(2);  // Already set, no change
    REQUIRE(state.bentMask == 0b0101);

    // Edge cases - should not crash
    state.markBent(-1);
    state.markBent(32);
    REQUIRE(state.bentMask == 0b0101);
}

TEST_CASE("SearchState bentCount counts bits", "[phase3][types][state]") {
    SearchState state;

    REQUIRE(state.bentCount() == 0);

    state.bentMask = 0b1010;
    REQUIRE(state.bentCount() == 2);

    state.bentMask = 0b11111111;
    REQUIRE(state.bentCount() == 8);

    state.bentMask = 0xFFFFFFFF;
    REQUIRE(state.bentCount() == 32);
}

TEST_CASE("SearchState isGoal checks completion", "[phase3][types][state]") {
    SearchState state;

    // 3 bends total
    REQUIRE(state.isGoal(3) == false);

    state.bentMask = 0b011;  // Only 2 done
    REQUIRE(state.isGoal(3) == false);

    state.bentMask = 0b111;  // All 3 done
    REQUIRE(state.isGoal(3) == true);

    // Edge cases
    REQUIRE(state.isGoal(0) == false);
    REQUIRE(state.isGoal(-1) == false);
    REQUIRE(state.isGoal(33) == false);
}

TEST_CASE("SearchState equality ignores grip", "[phase3][types][state]") {
    SearchState s1, s2;

    s1.bentMask = 0b101;
    s1.orientation = Orientation::DEG_90;
    s1.abaConfig = 42;
    s1.gripCenterX = 100.0;
    s1.gripCenterY = 200.0;

    s2.bentMask = 0b101;
    s2.orientation = Orientation::DEG_90;
    s2.abaConfig = 42;
    s2.gripCenterX = 999.0;  // Different grip
    s2.gripCenterY = 999.0;

    REQUIRE(s1 == s2);  // Still equal

    s2.bentMask = 0b111;
    REQUIRE(s1 != s2);  // Now different
}
