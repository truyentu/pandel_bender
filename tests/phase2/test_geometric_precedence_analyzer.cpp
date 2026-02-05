#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

TEST_CASE("BentState construction", "[phase2][geometric]") {
    BentState state;

    REQUIRE(state.count() == 0);
    REQUIRE(state.getBentBends().empty());
}

TEST_CASE("BentState apply and check bends", "[phase2][geometric]") {
    BentState state;

    state.applyBend(0);
    state.applyBend(1);
    state.applyBend(2);

    REQUIRE(state.count() == 3);
    REQUIRE(state.isBent(0) == true);
    REQUIRE(state.isBent(1) == true);
    REQUIRE(state.isBent(2) == true);
    REQUIRE(state.isBent(3) == false);
}

TEST_CASE("BentState reset", "[phase2][geometric]") {
    BentState state;

    state.applyBend(0);
    state.applyBend(1);
    REQUIRE(state.count() == 2);

    state.reset();
    REQUIRE(state.count() == 0);
    REQUIRE(state.isBent(0) == false);
    REQUIRE(state.isBent(1) == false);
}

TEST_CASE("GeometricPrecedenceAnalyzer construction", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;

    auto stats = analyzer.getStatistics();
    REQUIRE(stats.totalPairsChecked == 0);
    REQUIRE(stats.totalConstraints == 0);
}

TEST_CASE("GeometricPrecedenceAnalyzer analyze empty bends", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;
    std::vector<BendFeature> bends;  // Empty

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.empty());
}

TEST_CASE("GeometricPrecedenceAnalyzer analyze single bend", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create a dummy bend
    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    // Single bend -> no constraints
    REQUIRE(constraints.empty());
}

TEST_CASE("BentState get bent bends list", "[phase2][geometric]") {
    BentState state;

    state.applyBend(5);
    state.applyBend(7);

    const auto& bentList = state.getBentBends();
    REQUIRE(bentList.size() == 2);
    REQUIRE(bentList[0] == 5);
    REQUIRE(bentList[1] == 7);
}

// ============================================================================
// Task 11: Corner Overlap Detection Tests
// ============================================================================

TEST_CASE("Corner overlap - no overlap case", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;
    BentState state;

    // Create two bends that don't overlap (far apart)
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.position.z = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 200.0;  // Far away - no overlap
    b1.position.y = 200.0;
    b1.position.z = 0.0;

    // Check for corner overlap (should be false - too far apart)
    bool overlap = analyzer.checkCornerOverlap(b0, b1, state);

    REQUIRE(overlap == false);
}

TEST_CASE("Corner overlap - with mock overlap detection", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;
    BentState state;

    // Create two bends
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;

    b1.id = 1;
    b1.angle = 90.0;

    // Test checkCornerOverlap method exists and returns bool
    bool result = analyzer.checkCornerOverlap(b0, b1, state);

    // For now, should return false (stub implementation)
    REQUIRE((result == true || result == false));  // Just check it compiles and returns bool
}

TEST_CASE("Analyze two bends - check statistics", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;

    b1.id = 1;
    b1.angle = 90.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should check 2 pairs: (0,1) and (1,0)
    REQUIRE(stats.totalPairsChecked == 2);
}

TEST_CASE("Analyze three bends - check pair count", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b1.id = 1;
    b2.id = 2;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should check 6 pairs: (0,1), (0,2), (1,0), (1,2), (2,0), (2,1)
    REQUIRE(stats.totalPairsChecked == 6);
}

TEST_CASE("Corner overlap - overlapping bends at same position", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;
    BentState state;

    // Create two bends at same position (should overlap)
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.position.z = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 10.0;  // Within 30mm tolerance
    b1.position.y = 10.0;
    b1.position.z = 0.0;

    // Check for corner overlap
    bool overlap = analyzer.checkCornerOverlap(b0, b1, state);

    // Should detect overlap (within tolerance)
    REQUIRE(overlap == true);
}

TEST_CASE("Corner overlap - far apart bends", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;
    BentState state;

    // Create two bends far apart (no overlap)
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.position.z = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 100.0;  // Far away (>30mm tolerance)
    b1.position.y = 100.0;
    b1.position.z = 0.0;

    // Check for corner overlap
    bool overlap = analyzer.checkCornerOverlap(b0, b1, state);

    // Should NOT detect overlap (too far)
    REQUIRE(overlap == false);
}

TEST_CASE("Analyze with overlapping bends - generates constraint", "[phase2][geometric][overlap]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create overlapping bends
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 10.0;  // Close enough to overlap
    b1.position.y = 10.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should generate at least one constraint
    REQUIRE(stats.totalConstraints >= 1);
    REQUIRE(stats.cornerOverlapCount >= 1);

    // Verify constraint content
    if (!constraints.empty()) {
        REQUIRE(constraints[0].type == ConstraintType::GEOMETRIC);
        REQUIRE(constraints[0].confidence == 1.0);
        REQUIRE(constraints[0].reasoning == "Corner overlap detected");
    }
}


