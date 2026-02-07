#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/sequencer.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// Helper: create Phase2Output
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

// ===== Sequencer Basic Tests =====

TEST_CASE("Sequencer empty input", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makePhase2(0);
    std::vector<BendFeature> bends;

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.empty());
}

TEST_CASE("Sequencer single bend", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makePhase2(1);
    auto bends = makeBends(1);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 1);
    REQUIRE(result.totalCycleTime > 0.0);
}

TEST_CASE("Sequencer L-bracket (2 bends)", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makePhase2(2);

    std::vector<BendFeature> bends(2);
    bends[0].id = 0; bends[0].angle = 90.0; bends[0].length = 150.0;
    bends[0].direction = {1, 0, 0};
    bends[1].id = 1; bends[1].angle = 90.0; bends[1].length = 100.0;
    bends[1].direction = {0, 1, 0};

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 2);
    REQUIRE(result.totalCycleTime > 0.0);

    std::string summary = result.generateSummary();
    REQUIRE(summary.find("SUCCESS") != std::string::npos);
}

TEST_CASE("Sequencer U-channel (3 bends chain)", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makeChainPhase2(3);
    auto bends = makeBends(3);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 3);
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.bendSequence[1] == 1);
    REQUIRE(result.bendSequence[2] == 2);
}

TEST_CASE("Sequencer 5 bends performance", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makePhase2(5);
    auto bends = makeBends(5);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 5);
    REQUIRE(result.stats.searchTimeMs < 1000.0);
}

TEST_CASE("Sequencer setConfig works", "[phase3][sequencer]") {
    Sequencer seq;

    SearchConfig config;
    config.maxNodes = 500000;
    config.timeoutSeconds = 10.0;
    seq.setConfig(config);

    auto p2 = makePhase2(3);
    auto bends = makeBends(3);

    auto result = seq.sequence(p2, bends);
    REQUIRE(result.success == true);
}

TEST_CASE("Sequencer getStatistics after search", "[phase3][sequencer]") {
    Sequencer seq;
    auto p2 = makePhase2(4);
    auto bends = makeBends(4);

    seq.sequence(p2, bends);
    const auto& stats = seq.getStatistics();

    REQUIRE(stats.nodesExpanded > 0);
    REQUIRE(stats.nodesGenerated > 0);
    REQUIRE(stats.searchTimeMs >= 0.0);
}

TEST_CASE("Sequencer reuse (multiple calls)", "[phase3][sequencer]") {
    Sequencer seq;

    // First call
    auto p2a = makePhase2(2);
    auto bendsA = makeBends(2);
    auto resultA = seq.sequence(p2a, bendsA);
    REQUIRE(resultA.success == true);

    // Second call with different input
    auto p2b = makeChainPhase2(3);
    auto bendsB = makeBends(3);
    auto resultB = seq.sequence(p2b, bendsB);
    REQUIRE(resultB.success == true);
    REQUIRE(resultB.bendSequence.size() == 3);
}

// ===== Integration: Phase 2 -> Phase 3 =====

TEST_CASE("Integration: ConstraintSolver -> Sequencer", "[phase3][integration]") {
    // Phase 2: solve constraints
    ConstraintSolver solver;

    std::vector<BendFeature> bends(3);
    bends[0].id = 0; bends[0].angle = 90.0; bends[0].length = 100.0;
    bends[0].position = {0, 0, 0}; bends[0].direction = {1, 0, 0};
    bends[0].normal = {0, 0, 1};
    bends[1].id = 1; bends[1].angle = 90.0; bends[1].length = 150.0;
    bends[1].position = {500, 0, 0}; bends[1].direction = {0, 1, 0};
    bends[1].normal = {0, 0, 1};
    bends[2].id = 2; bends[2].angle = 90.0; bends[2].length = 120.0;
    bends[2].position = {0, 500, 0}; bends[2].direction = {-1, 0, 0};
    bends[2].normal = {0, 0, 1};

    Phase2Output p2 = solver.solve(bends);
    REQUIRE(p2.success == true);

    // Phase 3: find optimal sequence
    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);

    REQUIRE(p3.success == true);
    REQUIRE(p3.bendSequence.size() == 3);
    REQUIRE(p3.totalCycleTime > 0.0);
    REQUIRE(p3.actions.size() >= 3); // May include REPOSITION actions
    REQUIRE(p3.stats.nodesExpanded > 0);
}
