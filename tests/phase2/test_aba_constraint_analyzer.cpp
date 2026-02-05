#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/aba_constraint_analyzer.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 21: ABAConstraintAnalyzer - Basic Setup & Constructor
// ============================================================================

TEST_CASE("ABAConstraintAnalyzer - constructor", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    // Should construct successfully
    REQUIRE(true);
}

TEST_CASE("ABAConstraintAnalyzer - analyze empty bends", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;  // Empty

    auto constraints = analyzer.analyze(bends);

    // Empty analysis should return empty constraints
    REQUIRE(constraints.empty());
}

TEST_CASE("ABAConstraintAnalyzer - analyze single short bend", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 50.0;  // Short bend (< 100mm)

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    // Should have 1 constraint for this bend
    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Verify basic properties
    REQUIRE(constraint.bendId == 0);
    REQUIRE(constraint.bendLength == 50.0);
    REQUIRE(constraint.requiredWidth >= 50.0);  // At least bend length
}

TEST_CASE("ABAConstraintAnalyzer - analyze single long bend", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 275.0;  // Long bend

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Required width should account for clearance
    REQUIRE(constraint.requiredWidth >= 275.0);
    REQUIRE(constraint.bendLength == 275.0);
}

TEST_CASE("ABAConstraintAnalyzer - feasibility check", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should have feasibility determination
    // feasible = true/false based on whether ABA segments can cover width
    REQUIRE((constraint.feasible == true || constraint.feasible == false));

    if (constraint.feasible) {
        // If feasible, should have segment solution
        REQUIRE(!constraint.segmentSolution.empty());
        REQUIRE(constraint.totalSegments > 0);
    } else {
        // If not feasible, should have reason
        REQUIRE(!constraint.reason.empty());
    }
}

TEST_CASE("ABAConstraintAnalyzer - multiple bends", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 150.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);

    // Should have 3 constraints (one per bend)
    REQUIRE(constraints.size() == 3);

    // Each should have correct bend ID
    REQUIRE(constraints[0].bendId == 0);
    REQUIRE(constraints[1].bendId == 1);
    REQUIRE(constraints[2].bendId == 2);

    // Each should have correct length
    REQUIRE(constraints[0].bendLength == 100.0);
    REQUIRE(constraints[1].bendLength == 200.0);
    REQUIRE(constraints[2].bendLength == 150.0);
}

TEST_CASE("ABAConstraintAnalyzer - statistics tracking", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;

    std::vector<BendFeature> bends = { b0, b1 };

    analyzer.analyze(bends);

    auto stats = analyzer.getStatistics();

    // Should have basic statistics
    REQUIRE(stats.totalBendsAnalyzed == 2);
    REQUIRE(stats.feasibleCount >= 0);
    REQUIRE(stats.infeasibleCount >= 0);
    REQUIRE(stats.feasibleCount + stats.infeasibleCount == stats.totalBendsAnalyzed);
}

TEST_CASE("ABAConstraintAnalyzer - box closing flag", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    // Should have box closing flag (true or false)
    const auto& constraint = constraints[0];
    REQUIRE((constraint.isBoxClosing == true || constraint.isBoxClosing == false));
}

TEST_CASE("ABAConstraintAnalyzer - clearance calculation", "[phase2][aba][basic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should have clearance value
    REQUIRE(constraint.clearance >= 0.0);

    // Required width should be >= bend length + clearance
    REQUIRE(constraint.requiredWidth >= constraint.bendLength + constraint.clearance);
}
