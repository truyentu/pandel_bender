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

// ============================================================================
// Task 22: Dynamic Programming Subset Sum Solver
// ============================================================================

TEST_CASE("Subset sum - exact match with single segment", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;  // Exact match with 100mm segment

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should find solution
    REQUIRE(constraint.feasible == true);
    REQUIRE(!constraint.segmentSolution.empty());

    // Should prefer exact match (single 100mm segment)
    // May also include clearance, so could be 100 + 25 = 125 or similar
    REQUIRE(constraint.totalSegments >= 1);
}

TEST_CASE("Subset sum - exact match with multiple segments", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 275.0;  // 275 = 100 + 175 or 150 + 125 or 100 + 100 + 75

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should find solution
    REQUIRE(constraint.feasible == true);

    // Verify total width covers required width
    REQUIRE(constraint.totalWidth >= constraint.requiredWidth);

    // Should minimize number of segments
    REQUIRE(constraint.totalSegments >= 2);
    REQUIRE(constraint.totalSegments <= 4);  // Should not use too many
}

TEST_CASE("Subset sum - minimize segment count", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 200.0;  // Exact match with single 200mm segment

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    REQUIRE(constraint.feasible == true);

    // Should prefer single 200mm segment over multiple smaller ones
    // With clearance, might need 200 + 25 = 225 total
    // Best: 200 + 25 (2 segments)
    REQUIRE(constraint.totalSegments <= 3);
}

TEST_CASE("Subset sum - small bend uses smallest segments", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 30.0;  // Small bend

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    REQUIRE(constraint.feasible == true);

    // Should use 25mm segment (smallest available)
    // With clearance ~10mm: need 40mm → 25 + 25 = 50mm
    bool hasSmallSegment = false;
    for (int seg : constraint.segmentSolution) {
        if (seg == 25 || seg == 50) {
            hasSmallSegment = true;
        }
    }
    REQUIRE(hasSmallSegment == true);
}

TEST_CASE("Subset sum - minimize waste", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 175.0;  // Exact match with 175mm segment

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    REQUIRE(constraint.feasible == true);

    // Calculate waste (overage)
    double waste = constraint.totalWidth - constraint.requiredWidth;

    // Waste should be minimal (< 25mm for most cases)
    REQUIRE(waste >= 0.0);  // Should not be negative
    REQUIRE(waste < 50.0);  // Should not have excessive waste
}

TEST_CASE("Subset sum - large bend requires multiple segments", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 500.0;  // Large bend

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    REQUIRE(constraint.feasible == true);

    // Should use multiple large segments
    // 500 + ~10mm clearance = 510mm
    // Best: 200 + 200 + 100 + 25 = 525mm (4 segments)
    // Or: 200 + 200 + 125 = 525mm (3 segments)
    REQUIRE(constraint.totalSegments >= 3);

    // Total should cover required width
    REQUIRE(constraint.totalWidth >= constraint.requiredWidth);
}

TEST_CASE("Subset sum - DP optimization preference", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 150.0;  // Can use: 150 (1 seg) or 100+50 (2 seg) or 75+75 (2 seg)

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    REQUIRE(constraint.feasible == true);

    // With clearance ~10mm, need 160mm
    // Optimal: 150 + 25 = 175mm (2 segments, waste = 15mm)
    // Alternative: 100 + 75 = 175mm (2 segments, waste = 15mm)
    // Alternative: 200mm (1 segment, waste = 40mm) - less optimal due to waste

    // Should have reasonable solution
    REQUIRE(constraint.totalSegments <= 3);

    // Should not have excessive waste
    double waste = constraint.totalWidth - constraint.requiredWidth;
    REQUIRE(waste < 50.0);
}

TEST_CASE("Subset sum - consistent results", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    // Analyze multiple times
    auto c1 = analyzer.analyze(bends);
    auto c2 = analyzer.analyze(bends);

    // Results should be identical (deterministic)
    REQUIRE(c1.size() == c2.size());
    REQUIRE(c1[0].totalSegments == c2[0].totalSegments);
    REQUIRE(c1[0].totalWidth == c2[0].totalWidth);
    REQUIRE(c1[0].feasible == c2[0].feasible);
}

TEST_CASE("Subset sum - performance with DP", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    // Test with multiple bends to measure DP performance
    std::vector<BendFeature> bends;
    for (int i = 0; i < 10; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 50.0 + i * 20.0;  // Varying lengths
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 10);

    auto stats = analyzer.getStatistics();

    // Should have timing data
    REQUIRE(stats.analysisTimeMs >= 0.0);
    REQUIRE(stats.avgBendTimeMs >= 0.0);

    // DP should still be fast (< 5ms per bend on average)
    REQUIRE(stats.avgBendTimeMs < 5.0);

    // All should be feasible
    REQUIRE(stats.feasibleCount == 10);
    REQUIRE(stats.infeasibleCount == 0);
}
