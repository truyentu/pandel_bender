#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
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

// ===== SearchNode Tests =====

TEST_CASE("SearchNode default values", "[phase3][types][node]") {
    SearchNode node;

    REQUIRE(node.g == 0.0);
    REQUIRE(node.h == 0.0);
    REQUIRE(node.f() == 0.0);
    REQUIRE(node.parentId == -1);
    REQUIRE(node.lastBendId == -1);
}

TEST_CASE("SearchNode f() computes g + h", "[phase3][types][node]") {
    SearchNode node;
    node.g = 10.5;
    node.h = 5.3;

    REQUIRE(node.f() == Catch::Approx(15.8));
}

TEST_CASE("SearchNode comparison for min-heap", "[phase3][types][node]") {
    SearchNode low, high;
    low.g = 5.0;
    low.h = 2.0;  // f = 7

    high.g = 10.0;
    high.h = 3.0;  // f = 13

    REQUIRE(high > low);
    REQUIRE(!(low > high));
}

TEST_CASE("SearchNode createInitial returns flat state", "[phase3][types][node]") {
    auto node = SearchNode::createInitial();

    REQUIRE(node.state.bentMask == 0);
    REQUIRE(node.state.orientation == Orientation::DEG_0);
    REQUIRE(node.g == 0.0);
    REQUIRE(node.h == 0.0);
    REQUIRE(node.parentId == -1);
    REQUIRE(node.lastBendId == -1);
}

TEST_CASE("SequenceAction stores action details", "[phase3][types][action]") {
    SequenceAction action;
    action.type = ActionType::BEND;
    action.bendId = 3;
    action.duration = 2.5;
    action.description = "Bend flange 3 at 90 degrees";

    REQUIRE(action.type == ActionType::BEND);
    REQUIRE(action.bendId == 3);
    REQUIRE(action.duration == Catch::Approx(2.5));
}

// ===== SequencerStatistics and Phase3Output Tests =====

TEST_CASE("SequencerStatistics default values", "[phase3][types][stats]") {
    SequencerStatistics stats;

    REQUIRE(stats.nodesExpanded == 0);
    REQUIRE(stats.nodesGenerated == 0);
    REQUIRE(stats.searchTimeMs == 0.0);
}

TEST_CASE("Phase3Output default values", "[phase3][types][output]") {
    Phase3Output output;

    REQUIRE(output.bendSequence.empty());
    REQUIRE(output.actions.empty());
    REQUIRE(output.totalCycleTime == 0.0);
    REQUIRE(output.repoCount == 0);
    REQUIRE(output.success == false);
    REQUIRE(output.optimal == false);
}

TEST_CASE("Phase3Output generateSummary", "[phase3][types][output]") {
    Phase3Output output;
    output.success = true;
    output.bendSequence = {0, 2, 1, 3};
    output.totalCycleTime = 15.5;
    output.repoCount = 1;
    output.stats.nodesExpanded = 150;
    output.stats.searchTimeMs = 25.3;

    std::string summary = output.generateSummary();

    REQUIRE(summary.find("SUCCESS") != std::string::npos);
    REQUIRE(summary.find("4") != std::string::npos);
    REQUIRE(summary.find("150") != std::string::npos);
}
