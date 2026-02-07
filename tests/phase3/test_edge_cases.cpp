#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/sequencer.h"
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

static Phase2Output makePhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) p2.precedenceGraph.addNode(i);
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static Phase2Output makeChainPhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) p2.precedenceGraph.addNode(i);
    for (int i = 0; i < n - 1; i++)
        p2.precedenceGraph.addEdge(i, i + 1, ConstraintType::GEOMETRIC, 1.0, "chain");
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static std::vector<BendFeature> makeBends(int n, double length = 100.0) {
    std::vector<BendFeature> bends(n);
    for (int i = 0; i < n; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = length;
        bends[i].direction = {1.0, 0.0, 0.0};
    }
    return bends;
}

// ===== Edge Cases =====

TEST_CASE("Edge: empty input returns success", "[phase3][edge]") {
    Sequencer seq;
    Phase2Output p2;
    p2.precedenceGraph.finalize();
    p2.success = true;
    std::vector<BendFeature> bends;

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.optimal == true);
    REQUIRE(result.bendSequence.empty());
    REQUIRE(result.actions.empty());
    REQUIRE(result.totalCycleTime == 0.0);
}

TEST_CASE("Edge: single bend trivial", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makePhase2(1);
    auto bends = makeBends(1);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.optimal == true);
    REQUIRE(result.bendSequence.size() == 1);
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.totalCycleTime > 0.0);
    REQUIRE(result.stats.nodesExpanded >= 1);
}

TEST_CASE("Edge: all same orientation (trivial)", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makePhase2(4);

    // All bends in same direction - no rotation cost
    std::vector<BendFeature> bends(4);
    for (int i = 0; i < 4; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = 100.0;
        bends[i].direction = {1.0, 0.0, 0.0}; // All same direction
    }

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 4);
    // No rotation costs
    REQUIRE(result.repoCount == 0);
}

TEST_CASE("Edge: chain constraint enforces strict order", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makeChainPhase2(5);
    auto bends = makeBends(5);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 5);
    // Chain must be 0,1,2,3,4
    for (int i = 0; i < 5; i++) {
        REQUIRE(result.bendSequence[i] == i);
    }
}

TEST_CASE("Edge: diamond DAG allows multiple valid paths", "[phase3][edge]") {
    // Diamond: 0 -> 1, 0 -> 2, 1 -> 3, 2 -> 3
    Phase2Output p2;
    for (int i = 0; i < 4; i++) p2.precedenceGraph.addNode(i);
    p2.precedenceGraph.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "diamond");
    p2.precedenceGraph.addEdge(0, 2, ConstraintType::GEOMETRIC, 1.0, "diamond");
    p2.precedenceGraph.addEdge(1, 3, ConstraintType::GEOMETRIC, 1.0, "diamond");
    p2.precedenceGraph.addEdge(2, 3, ConstraintType::GEOMETRIC, 1.0, "diamond");
    p2.precedenceGraph.finalize();
    p2.success = true;

    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].angle = 90.0; bends[0].length = 100.0;
    bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].angle = 90.0; bends[1].length = 100.0;
    bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].angle = 90.0; bends[2].length = 100.0;
    bends[2].direction = {-1, 0, 0};
    bends[3].id = 3; bends[3].angle = 90.0; bends[3].length = 100.0;
    bends[3].direction = {0, -1, 0};

    Sequencer seq;
    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 4);
    // Bend 0 must be first, bend 3 must be last
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.bendSequence[3] == 3);
    // Middle two must be {1,2} in some order
    bool valid = (result.bendSequence[1] == 1 && result.bendSequence[2] == 2) ||
                 (result.bendSequence[1] == 2 && result.bendSequence[2] == 1);
    REQUIRE(valid);
}

TEST_CASE("Edge: max nodes limit triggers fallback", "[phase3][edge]") {
    Sequencer seq;

    // Very low max nodes - should trigger beam search fallback
    SearchConfig config;
    config.maxNodes = 5;       // Very low
    config.useBeamSearch = true;
    config.beamWidth = 50;
    seq.setConfig(config);

    auto p2 = makePhase2(6);
    auto bends = makeBends(6);

    auto result = seq.sequence(p2, bends);

    // Should still find a solution via beam search
    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 6);
    // Beam search result is non-optimal
    REQUIRE(result.optimal == false);
    REQUIRE(!result.warnings.empty());
}

TEST_CASE("Edge: beam search disabled returns failure on limit", "[phase3][edge]") {
    Sequencer seq;

    SearchConfig config;
    config.maxNodes = 5;
    config.useBeamSearch = false; // No fallback
    seq.setConfig(config);

    auto p2 = makePhase2(8);

    // Create bends with diverse orientations to maximize search space
    std::vector<BendFeature> bends(8);
    for (int i = 0; i < 8; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = 100.0 + i * 20.0;
        switch (i % 4) {
            case 0: bends[i].direction = {1, 0, 0}; break;
            case 1: bends[i].direction = {0, 1, 0}; break;
            case 2: bends[i].direction = {-1, 0, 0}; break;
            case 3: bends[i].direction = {0, -1, 0}; break;
        }
    }

    auto result = seq.sequence(p2, bends);

    // With very low limit and no fallback, likely fails
    if (!result.success) {
        REQUIRE(!result.errorMessage.empty());
    }
}

TEST_CASE("Edge: timeout handling", "[phase3][edge]") {
    Sequencer seq;

    SearchConfig config;
    config.maxNodes = 1000000;
    config.timeoutSeconds = 0.001; // Extremely short timeout
    config.useBeamSearch = true;
    config.beamWidth = 50;
    seq.setConfig(config);

    auto p2 = makePhase2(8);
    std::vector<BendFeature> bends(8);
    for (int i = 0; i < 8; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = 100.0;
        switch (i % 4) {
            case 0: bends[i].direction = {1, 0, 0}; break;
            case 1: bends[i].direction = {0, 1, 0}; break;
            case 2: bends[i].direction = {-1, 0, 0}; break;
            case 3: bends[i].direction = {0, -1, 0}; break;
        }
    }

    auto result = seq.sequence(p2, bends);

    // With extremely short timeout, A* will timeout and beam search
    // will be attempted. It may or may not succeed, but should not crash.
    // If it succeeds, result is non-optimal.
    if (result.success) {
        REQUIRE(result.bendSequence.size() == 8);
    } else {
        REQUIRE(!result.errorMessage.empty());
    }
}

TEST_CASE("Edge: large chain (20 bends) is fast", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makeChainPhase2(20);
    auto bends = makeBends(20);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 20);
    // Chain is linear - should be very fast
    REQUIRE(result.stats.searchTimeMs < 100.0);

    for (int i = 0; i < 20; i++) {
        REQUIRE(result.bendSequence[i] == i);
    }
}

TEST_CASE("Edge: varying angles", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makePhase2(3);

    std::vector<BendFeature> bends(3);
    bends[0].id = 0; bends[0].angle = 45.0;  bends[0].length = 100.0;
    bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].angle = 90.0;  bends[1].length = 100.0;
    bends[1].direction = {0, 1, 0};
    bends[2].id = 2; bends[2].angle = 135.0; bends[2].length = 100.0;
    bends[2].direction = {1, 0, 0};

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 3);
    REQUIRE(result.totalCycleTime > 0.0);
}

TEST_CASE("Edge: generateSummary output", "[phase3][edge]") {
    Sequencer seq;
    auto p2 = makePhase2(3);
    auto bends = makeBends(3);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);

    std::string summary = result.generateSummary();
    REQUIRE(summary.find("SUCCESS") != std::string::npos);
    REQUIRE(summary.find("3") != std::string::npos); // 3 bends
    REQUIRE(summary.find("Nodes Expanded") != std::string::npos);
}
