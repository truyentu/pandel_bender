#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/negative_sequence.h"

using namespace openpanelcam::phase4;
using namespace openpanelcam::phase1;
using Catch::Approx;

static BendFeature makeBend(int id, double angle, double length,
                             double px, double py, double pz,
                             double dx, double dy, double dz) {
    BendFeature b;
    b.id = id;
    b.angle = angle;
    b.length = length;
    b.position = {px, py, pz};
    b.direction = {dx, dy, dz};
    b.normal = {0, 0, 1};
    return b;
}

// ===== NegativeSequenceAnalyzer Construction =====

TEST_CASE("NegativeSequenceAnalyzer default construction", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    // Should construct without error
    REQUIRE(true);
}

TEST_CASE("NegativeSequenceAnalyzer custom config", "[phase4][negative]") {
    ValidatorConfig config;
    config.collisionMargin = 5.0;
    NegativeSequenceAnalyzer analyzer(config);
    REQUIRE(true);
}

// ===== Empty Input =====

TEST_CASE("NegativeSequence empty bends succeeds", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends;

    auto result = analyzer.analyze(bends);

    REQUIRE(result.success == true);
    REQUIRE(result.forwardSequence.empty());
    REQUIRE(result.reversedSequence.empty());
}

// ===== Single Bend =====

TEST_CASE("NegativeSequence single bend succeeds", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = analyzer.analyze(bends);

    REQUIRE(result.success == true);
    REQUIRE(result.reversedSequence.size() == 1);
    REQUIRE(result.forwardSequence.size() == 1);
    REQUIRE(result.forwardSequence[0] == 0);
}

// ===== Distant Bends (no collision) =====

TEST_CASE("NegativeSequence distant bends all feasible", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 2000, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 100.0, 4000, 0, 0, 1, 0, 0)
    };

    auto result = analyzer.analyze(bends);

    REQUIRE(result.success == true);
    REQUIRE(result.forwardSequence.size() == 3);
    REQUIRE(result.infeasibleBends.empty());
}

// ===== Forward sequence is reverse of reversed =====

TEST_CASE("NegativeSequence forward is reverse of reversed", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 80.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 80.0, 1000, 0, 0, 0, 1, 0)
    };

    auto result = analyzer.analyze(bends);

    REQUIRE(result.success == true);
    REQUIRE(result.reversedSequence.size() == result.forwardSequence.size());

    // Forward should be exact reverse of reversed
    auto fwd = result.forwardSequence;
    auto rev = result.reversedSequence;
    std::reverse(rev.begin(), rev.end());
    REQUIRE(fwd == rev);
}

// ===== Iteration count =====

TEST_CASE("NegativeSequence tracks iterations", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 2000, 0, 0, 0, 1, 0)
    };

    auto result = analyzer.analyze(bends);

    REQUIRE(result.iterationsUsed >= 1);
}

// ===== IsBendFeasible direct test =====

TEST_CASE("IsBendFeasible returns true for isolated bend", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    BentState emptyState;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    REQUIRE(analyzer.isBendFeasible(bend, emptyState) == true);
}

TEST_CASE("IsBendFeasible returns false for colliding bend", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    BentState state;
    state.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    // Same position -> collision expected
    REQUIRE(analyzer.isBendFeasible(bend, state) == false);
}

TEST_CASE("IsBendFeasible returns true for distant bend", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    BentState state;
    state.addBentFlange(makeBend(1, 90.0, 100.0, 2000, 2000, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    REQUIRE(analyzer.isBendFeasible(bend, state) == true);
}

// ===== Multiple iterations =====

TEST_CASE("NegativeSequence processes all bends in multiple iterations", "[phase4][negative]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 45.0, 50.0, 500, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 50.0, 1000, 0, 0, 1, 0, 0),
        makeBend(3, 60.0, 50.0, 1500, 0, 0, 0, 1, 0)
    };

    auto result = analyzer.analyze(bends);

    REQUIRE(result.success == true);
    REQUIRE(result.forwardSequence.size() == 4);

    // All bends should appear exactly once
    std::set<int> seen(result.forwardSequence.begin(), result.forwardSequence.end());
    REQUIRE(seen.size() == 4);
    REQUIRE(seen.count(0) == 1);
    REQUIRE(seen.count(1) == 1);
    REQUIRE(seen.count(2) == 1);
    REQUIRE(seen.count(3) == 1);
}

// ===== Tool collision tests =====

TEST_CASE("NegativeSequence with tool collision blocks bend", "[phase4][negative][tool]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    // Tool overlapping with bend position
    std::vector<ToolGeometry> tools;
    ToolGeometry tool;
    tool.valid = true;
    tool.toolId = 0;
    tool.punchAABB = AABB(-50, -50, -50, 50, 50, 50);  // Covers bend
    tool.dieAABB = AABB(-50, -50, -200, 50, 50, -100);
    tools.push_back(tool);

    auto result = analyzer.analyze(bends, tools);
    // Bend should be infeasible due to tool collision
    REQUIRE(result.success == false);
    REQUIRE(result.infeasibleBends.size() == 1);
}

TEST_CASE("NegativeSequence with distant tool succeeds", "[phase4][negative][tool]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0)
    };

    // Tool far away from bend
    std::vector<ToolGeometry> tools;
    ToolGeometry tool;
    tool.valid = true;
    tool.toolId = 0;
    tool.punchAABB = AABB(5000, 5000, 5000, 5100, 5100, 5100);
    tool.dieAABB = AABB(5000, 5000, 4800, 5100, 5100, 4900);
    tools.push_back(tool);

    auto result = analyzer.analyze(bends, tools);
    REQUIRE(result.success == true);
}

TEST_CASE("NegativeSequence with invalid tool is skipped", "[phase4][negative][tool]") {
    NegativeSequenceAnalyzer analyzer;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0)
    };

    // Invalid tool should be skipped
    std::vector<ToolGeometry> tools;
    ToolGeometry tool;
    tool.valid = false;  // Invalid
    tool.punchAABB = AABB(-50, -50, -50, 50, 50, 50);
    tools.push_back(tool);

    auto result = analyzer.analyze(bends, tools);
    REQUIRE(result.success == true);
}

TEST_CASE("IsBendFeasible with tool collision returns false", "[phase4][negative][tool]") {
    NegativeSequenceAnalyzer analyzer;
    BentState emptyState;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    std::vector<ToolGeometry> tools;
    ToolGeometry tool;
    tool.valid = true;
    tool.punchAABB = AABB(-50, -50, -50, 50, 50, 50);
    tool.dieAABB = AABB(1000, 1000, 1000, 1100, 1100, 1100);
    tools.push_back(tool);

    REQUIRE(analyzer.isBendFeasible(bend, emptyState, tools) == false);
}

TEST_CASE("IsBendFeasible without tools still works", "[phase4][negative][tool]") {
    NegativeSequenceAnalyzer analyzer;
    BentState emptyState;
    auto bend = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);

    std::vector<ToolGeometry> emptyTools;
    REQUIRE(analyzer.isBendFeasible(bend, emptyState, emptyTools) == true);
}
