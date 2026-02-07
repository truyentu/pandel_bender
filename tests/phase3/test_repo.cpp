#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/successor_generator.h"
#include "openpanelcam/phase3/sequencer.h"
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ===== RepoTriggerDetector Tests =====

TEST_CASE("RepoTriggerDetector returns NONE for simple cases", "[phase3][repo]") {
    RepoTriggerDetector detector;

    SearchState state;
    state.markBent(0);

    std::vector<BendFeature> bends(3);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};

    std::vector<GraspConstraint> constraints;

    RepoReason reason = detector.check(state, 0, bends, constraints);
    REQUIRE(reason == RepoReason::NONE);
}

TEST_CASE("RepoTriggerDetector detects box closing", "[phase3][repo]") {
    RepoTriggerDetector detector;

    // 4 bends forming a box: +X, +Y, -X, -Y
    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};
    bends[3].id = 3; bends[3].direction = {0, -1, 0};

    // All 4 bent -> box closing on last bend
    SearchState state;
    state.markBent(0);
    state.markBent(1);
    state.markBent(2);
    state.markBent(3);

    std::vector<GraspConstraint> constraints;

    RepoReason reason = detector.check(state, 3, bends, constraints);
    REQUIRE(reason == RepoReason::BOX_CLOSING);
}

TEST_CASE("RepoTriggerDetector no box closing with <4 bends", "[phase3][repo]") {
    RepoTriggerDetector detector;

    // Only 3 bends - not enough for box
    std::vector<BendFeature> bends(3);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};

    SearchState state;
    state.markBent(0);
    state.markBent(1);
    state.markBent(2);

    std::vector<GraspConstraint> constraints;

    RepoReason reason = detector.check(state, 2, bends, constraints);
    REQUIRE(reason == RepoReason::NONE);
}

TEST_CASE("RepoTriggerDetector detects grip area exhaustion", "[phase3][repo]") {
    RepoTriggerDetector detector(100.0); // minGripArea = 100

    std::vector<BendFeature> bends(2);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};

    SearchState state;
    state.markBent(0);
    state.markBent(1);

    // Create constraint matching this state with insufficient grip
    GraspConstraint constraint;
    constraint.bentBends = {0, 1};
    constraint.hasValidGrip = false;
    constraint.validArea = 50.0; // Below minimum

    std::vector<GraspConstraint> constraints = {constraint};

    RepoReason reason = detector.check(state, 1, bends, constraints);
    REQUIRE(reason == RepoReason::GRIP_AREA_EXHAUSTED);
}

TEST_CASE("RepoTriggerDetector valid grip passes", "[phase3][repo]") {
    RepoTriggerDetector detector(100.0);

    std::vector<BendFeature> bends(2);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].direction = {0, 1, 0};

    SearchState state;
    state.markBent(0);

    // Constraint with sufficient grip area
    GraspConstraint constraint;
    constraint.bentBends = {0};
    constraint.hasValidGrip = true;
    constraint.validArea = 200.0; // Above minimum

    std::vector<GraspConstraint> constraints = {constraint};

    RepoReason reason = detector.check(state, 0, bends, constraints);
    REQUIRE(reason == RepoReason::NONE);
}

// ===== Repo Integration in Sequencer Tests =====

static Phase2Output makePhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) p2.precedenceGraph.addNode(i);
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static std::vector<BendFeature> makeBoxBends() {
    // 4 bends forming a box pattern
    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].angle = 90.0; bends[0].length = 100.0;
    bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].angle = 90.0; bends[1].length = 100.0;
    bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].angle = 90.0; bends[2].length = 100.0;
    bends[2].direction = {-1, 0, 0};
    bends[3].id = 3; bends[3].angle = 90.0; bends[3].length = 100.0;
    bends[3].direction = {0, -1, 0};
    return bends;
}

TEST_CASE("Sequencer detects repo in box closing scenario", "[phase3][repo][integration]") {
    Sequencer seq;
    auto p2 = makePhase2(4);
    auto bends = makeBoxBends();

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 4);
    // Box closing should trigger at least 1 repo
    REQUIRE(result.repoCount >= 1);
    REQUIRE(!result.repoAfterBends.empty());
}

TEST_CASE("Sequencer repo cost increases total cycle time", "[phase3][repo][integration]") {
    Sequencer seq;

    // L-bracket (2 bends, no repo expected)
    auto p2a = makePhase2(2);
    std::vector<BendFeature> bendsA(2);
    bendsA[0].id = 0; bendsA[0].angle = 90.0; bendsA[0].length = 100.0;
    bendsA[0].direction = {1, 0, 0};
    bendsA[1].id = 1; bendsA[1].angle = 90.0; bendsA[1].length = 100.0;
    bendsA[1].direction = {0, 1, 0};
    auto resultA = seq.sequence(p2a, bendsA);

    // Box (4 bends, repo expected)
    auto p2b = makePhase2(4);
    auto bendsB = makeBoxBends();
    auto resultB = seq.sequence(p2b, bendsB);

    REQUIRE(resultA.success == true);
    REQUIRE(resultB.success == true);

    // Box with repo should have higher total time
    if (resultB.repoCount > 0) {
        REQUIRE(resultB.totalCycleTime > resultA.totalCycleTime);
    }
}

TEST_CASE("Sequencer tracks repo actions in output", "[phase3][repo][integration]") {
    Sequencer seq;
    auto p2 = makePhase2(4);
    auto bends = makeBoxBends();

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);

    // Count REPOSITION actions
    int repoActions = 0;
    for (const auto& action : result.actions) {
        if (action.type == ActionType::REPOSITION) {
            repoActions++;
            REQUIRE(action.duration > 0.0);
        }
    }
    REQUIRE(repoActions == result.repoCount);
}
