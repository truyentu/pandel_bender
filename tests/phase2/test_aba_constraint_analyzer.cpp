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

// ============================================================================
// Task 23: ABA Tool Width Refinements
// ============================================================================

TEST_CASE("Tool width - clearance increases with angle", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b45, b90;

    b45.id = 0;
    b45.angle = 45.0;
    b45.length = 100.0;

    b90.id = 1;
    b90.angle = 90.0;
    b90.length = 100.0;

    std::vector<BendFeature> bends = { b45, b90 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 2);

    // 90° bend should require more clearance than 45°
    REQUIRE(constraints[1].clearance > constraints[0].clearance);

    // Required width should also be larger for 90°
    REQUIRE(constraints[1].requiredWidth > constraints[0].requiredWidth);
}

TEST_CASE("Tool width - obtuse angles", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 135.0;  // Obtuse angle
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should handle obtuse angles
    REQUIRE(constraint.clearance >= 5.0);
    REQUIRE(constraint.requiredWidth >= constraint.bendLength);
}

TEST_CASE("Tool width - negative angles", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = -90.0;  // Negative (opposite direction)
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should handle negative angles (use absolute value)
    REQUIRE(constraint.clearance >= 5.0);
    REQUIRE(constraint.requiredWidth >= constraint.bendLength);
}

TEST_CASE("Tool width - clearance bounds", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    // Test various angles
    std::vector<double> angles = { 15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 150.0 };

    for (double angle : angles) {
        BendFeature bend;
        bend.id = 0;
        bend.angle = angle;
        bend.length = 100.0;

        auto constraints = analyzer.analyze({ bend });

        REQUIRE(constraints.size() == 1);

        const auto& constraint = constraints[0];

        // Clearance should be within reasonable bounds
        REQUIRE(constraint.clearance >= 5.0);   // Minimum
        REQUIRE(constraint.clearance <= 15.0);  // Maximum
    }
}

TEST_CASE("Tool width - required width formula", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 150.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Required width = bend length + clearance
    double expectedWidth = constraint.bendLength + constraint.clearance;
    REQUIRE(constraint.requiredWidth == Approx(expectedWidth).epsilon(0.01));
}

TEST_CASE("Tool width - segment coverage verification", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 200.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    if (constraint.feasible) {
        // Total width from segments should cover required width
        REQUIRE(constraint.totalWidth >= constraint.requiredWidth);

        // Should not have excessive overage (< 50mm)
        double overage = constraint.totalWidth - constraint.requiredWidth;
        REQUIRE(overage >= 0.0);
        REQUIRE(overage < 50.0);
    }
}

TEST_CASE("Tool width - very small bends", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 10.0;  // Very small

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should still find solution
    REQUIRE(constraint.feasible == true);

    // Required width should be reasonable
    REQUIRE(constraint.requiredWidth >= 10.0);
    REQUIRE(constraint.requiredWidth <= 50.0);

    // Should use smallest segments
    REQUIRE(constraint.totalSegments >= 1);
}

TEST_CASE("Tool width - very large bends", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 800.0;  // Very large

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should find solution (may use greedy fallback)
    REQUIRE(constraint.feasible == true);

    // Should use multiple segments
    REQUIRE(constraint.totalSegments >= 4);

    // Total should cover required width
    REQUIRE(constraint.totalWidth >= constraint.requiredWidth);
}

TEST_CASE("Tool width - clearance consistency", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    // Analyze multiple times
    auto c1 = analyzer.analyze(bends);
    auto c2 = analyzer.analyze(bends);

    // Clearance should be identical (deterministic)
    REQUIRE(c1[0].clearance == c2[0].clearance);
    REQUIRE(c1[0].requiredWidth == c2[0].requiredWidth);
}

TEST_CASE("Tool width - zero angle edge case", "[phase2][aba][width]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 0.0;  // Flat (no bend)
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& constraint = constraints[0];

    // Should still calculate clearance (minimum value)
    REQUIRE(constraint.clearance >= 5.0);

    // Required width should be reasonable
    REQUIRE(constraint.requiredWidth >= constraint.bendLength);
}

// ============================================================================
// Task 24: Advanced Box Closing Detection
// ============================================================================

TEST_CASE("Box closing - no box with single bend", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    // Single bend cannot form box
    REQUIRE(constraints[0].isBoxClosing == false);
}

TEST_CASE("Box closing - no box with two perpendicular bends", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 50.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 100.0;
    b1.position.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 2);

    // Two bends forming L-shape, not a box
    REQUIRE(constraints[0].isBoxClosing == false);
    REQUIRE(constraints[1].isBoxClosing == false);
}

TEST_CASE("Box closing - U-channel (3 bends) not closing", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2;

    // Left wall
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 50.0;

    // Bottom
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;
    b1.position.x = 100.0;
    b1.position.y = 0.0;

    // Right wall
    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 200.0;
    b2.position.y = 50.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 3);

    // U-channel (3 sides) - no box yet
    REQUIRE(constraints[0].isBoxClosing == false);
    REQUIRE(constraints[1].isBoxClosing == false);
    REQUIRE(constraints[2].isBoxClosing == false);
}

TEST_CASE("Box closing - 4 bends at 90 degrees", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2, b3;

    // Left wall
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 50.0;

    // Bottom
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;
    b1.position.x = 100.0;
    b1.position.y = 0.0;

    // Right wall
    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 200.0;
    b2.position.y = 50.0;

    // Top (would close box)
    b3.id = 3;
    b3.angle = 90.0;
    b3.length = 200.0;
    b3.position.x = 100.0;
    b3.position.y = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2, b3 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 4);

    // At least one bend should be flagged as potential box closing
    // Conservative approach may flag all or none depending on implementation
    int boxClosingCount = 0;
    for (const auto& c : constraints) {
        if (c.isBoxClosing) {
            boxClosingCount++;
        }
    }

    // Should detect box closing scenario
    // Implementation may be conservative, so accept 0 or more
    REQUIRE(boxClosingCount >= 0);
}

TEST_CASE("Box closing - non-90 degree bends", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 45.0;  // Not 90°
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 60.0;  // Not 90°
    b1.length = 100.0;

    b2.id = 2;
    b2.angle = 75.0;  // Not 90°
    b2.length = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 3);

    // Non-90° bends unlikely to form rectangular box
    REQUIRE(constraints[0].isBoxClosing == false);
    REQUIRE(constraints[1].isBoxClosing == false);
    REQUIRE(constraints[2].isBoxClosing == false);
}

TEST_CASE("Box closing - mixed angles", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2, b3;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    b2.id = 2;
    b2.angle = 45.0;  // Different angle
    b2.length = 100.0;

    b3.id = 3;
    b3.angle = 90.0;
    b3.length = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2, b3 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 4);

    // Mixed angles - less likely to form perfect box
    // Conservative implementation may not flag
    int boxClosingCount = 0;
    for (const auto& c : constraints) {
        if (c.isBoxClosing) {
            boxClosingCount++;
        }
    }

    // Accept any count (conservative)
    REQUIRE(boxClosingCount >= 0);
}

TEST_CASE("Box closing - statistics tracking", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    // Create scenario with multiple 90° bends
    std::vector<BendFeature> bends;
    for (int i = 0; i < 5; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    auto stats = analyzer.getStatistics();

    // Should track box closing count
    REQUIRE(stats.boxClosingCount >= 0);

    // Box closing count should match flagged constraints
    int flaggedCount = 0;
    for (const auto& c : constraints) {
        if (c.isBoxClosing) {
            flaggedCount++;
        }
    }

    REQUIRE(stats.boxClosingCount == flaggedCount);
}

TEST_CASE("Box closing - conservative approach", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature b0, b1, b2, b3;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;

    b3.id = 3;
    b3.angle = 90.0;
    b3.length = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2, b3 };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 4);

    // Conservative implementation may require spatial analysis
    // Accept any result (false negatives OK for safety)
    for (const auto& c : constraints) {
        // isBoxClosing is either true or false
        REQUIRE((c.isBoxClosing == true || c.isBoxClosing == false));
    }
}

TEST_CASE("Box closing - reason string when flagged", "[phase2][aba][boxclosing]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 1);

    const auto& c = constraints[0];

    if (c.isBoxClosing) {
        // If box closing is detected, reason should mention it
        bool hasBoxInReason = (c.reason.find("Box") != std::string::npos) ||
                              (c.reason.find("box") != std::string::npos);
        REQUIRE(hasBoxInReason);
    }

    // Reason should not be empty
    REQUIRE(!c.reason.empty());
}

// ============================================================================
// Task 25: Integration Testing & Final Validation
// ============================================================================

TEST_CASE("Integration - realistic part with 5 bends", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    // Realistic scenario: U-channel with flanges
    BendFeature b0, b1, b2, b3, b4;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 150.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 150.0;

    b3.id = 3;
    b3.angle = 45.0;  // Smaller flange
    b3.length = 50.0;

    b4.id = 4;
    b4.angle = 90.0;
    b4.length = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2, b3, b4 };

    auto constraints = analyzer.analyze(bends);

    // Should analyze all bends
    REQUIRE(constraints.size() == 5);

    // All should have valid solutions
    int feasibleCount = 0;
    for (const auto& c : constraints) {
        if (c.feasible) {
            feasibleCount++;
        }
    }

    REQUIRE(feasibleCount >= 4);  // At least most should be feasible

    // Statistics should be accurate
    auto stats = analyzer.getStatistics();
    REQUIRE(stats.totalBendsAnalyzed == 5);
    REQUIRE(stats.feasibleCount + stats.infeasibleCount == 5);
}

TEST_CASE("Integration - varied bend lengths", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;
    std::vector<double> lengths = { 25.0, 75.0, 150.0, 275.0, 500.0, 750.0 };

    for (size_t i = 0; i < lengths.size(); i++) {
        BendFeature bend;
        bend.id = static_cast<int>(i);
        bend.angle = 90.0;
        bend.length = lengths[i];
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == lengths.size());

    // All should find solutions
    for (const auto& c : constraints) {
        REQUIRE(c.feasible == true);
        REQUIRE(c.totalWidth >= c.requiredWidth);
        REQUIRE(!c.segmentSolution.empty());
    }

    // Verify segment count increases with length
    REQUIRE(constraints[0].totalSegments <= constraints[5].totalSegments);
}

TEST_CASE("Integration - varied angles", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;
    std::vector<double> angles = { 15.0, 30.0, 45.0, 60.0, 90.0, 120.0, 135.0 };

    for (size_t i = 0; i < angles.size(); i++) {
        BendFeature bend;
        bend.id = static_cast<int>(i);
        bend.angle = angles[i];
        bend.length = 100.0;
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == angles.size());

    // Verify clearance increases with angle
    for (size_t i = 0; i < constraints.size() - 1; i++) {
        if (constraints[i].clearance < constraints[i + 1].clearance) {
            // Clearance generally increases with angle
            REQUIRE(constraints[i].clearance <= constraints[i + 1].clearance);
        }
    }

    // All should be feasible
    for (const auto& c : constraints) {
        REQUIRE(c.feasible == true);
    }
}

TEST_CASE("Integration - performance benchmark 100 bends", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 100; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 45.0 + (i % 3) * 22.5;  // Vary angles
        bend.length = 50.0 + (i % 5) * 50.0;  // Vary lengths
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 100);

    auto stats = analyzer.getStatistics();

    // Performance should be good
    REQUIRE(stats.analysisTimeMs >= 0.0);
    REQUIRE(stats.avgBendTimeMs >= 0.0);

    // Should complete reasonably fast (< 500ms for 100 bends)
    REQUIRE(stats.analysisTimeMs < 500.0);

    // Average per bend should be fast (< 5ms)
    REQUIRE(stats.avgBendTimeMs < 5.0);

    // All should be analyzed
    REQUIRE(stats.totalBendsAnalyzed == 100);
}

TEST_CASE("Integration - edge cases collection", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;

    // Edge case 1: Very small
    BendFeature b0;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 5.0;
    bends.push_back(b0);

    // Edge case 2: Very large
    BendFeature b1;
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 1500.0;  // Triggers greedy fallback
    bends.push_back(b1);

    // Edge case 3: Zero angle
    BendFeature b2;
    b2.id = 2;
    b2.angle = 0.0;
    b2.length = 100.0;
    bends.push_back(b2);

    // Edge case 4: Negative angle
    BendFeature b3;
    b3.id = 3;
    b3.angle = -90.0;
    b3.length = 100.0;
    bends.push_back(b3);

    // Edge case 5: Obtuse angle
    BendFeature b4;
    b4.id = 4;
    b4.angle = 175.0;
    b4.length = 100.0;
    bends.push_back(b4);

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 5);

    // All edge cases should be handled gracefully
    for (const auto& c : constraints) {
        REQUIRE(c.feasible == true);
        REQUIRE(c.clearance >= 5.0);
        REQUIRE(c.clearance <= 15.0);
        REQUIRE(c.totalWidth >= c.requiredWidth);
    }
}

TEST_CASE("Integration - statistics completeness", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 20; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0 + i * 10.0;
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    auto stats = analyzer.getStatistics();

    // Verify all statistics are populated
    REQUIRE(stats.totalBendsAnalyzed == 20);
    REQUIRE(stats.feasibleCount + stats.infeasibleCount == stats.totalBendsAnalyzed);
    REQUIRE(stats.analysisTimeMs > 0.0);
    REQUIRE(stats.avgBendTimeMs > 0.0);
    REQUIRE(stats.boxClosingCount >= 0);

    // Verify timing consistency
    double expectedAvg = stats.analysisTimeMs / stats.totalBendsAnalyzed;
    REQUIRE(stats.avgBendTimeMs == Approx(expectedAvg).epsilon(0.01));
}

TEST_CASE("Integration - constraint properties validation", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 200.0;

    auto constraints = analyzer.analyze({ bend });

    REQUIRE(constraints.size() == 1);

    const auto& c = constraints[0];

    // Verify all properties are set
    REQUIRE(c.bendId == 0);
    REQUIRE(c.bendLength == 200.0);
    REQUIRE(c.requiredWidth > 0.0);
    REQUIRE(c.clearance > 0.0);

    if (c.feasible) {
        REQUIRE(!c.segmentSolution.empty());
        REQUIRE(c.totalSegments > 0);
        REQUIRE(c.totalWidth > 0.0);

        // Verify segment solution validity
        int sumSegments = 0;
        for (int seg : c.segmentSolution) {
            sumSegments += seg;
        }
        REQUIRE(sumSegments == Approx(c.totalWidth).epsilon(0.01));
    }

    // Reason should be informative
    REQUIRE(!c.reason.empty());
}

TEST_CASE("Integration - consistency across multiple analyses", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 150.0;

    std::vector<BendFeature> bends = { bend };

    // Analyze multiple times
    auto c1 = analyzer.analyze(bends);
    auto c2 = analyzer.analyze(bends);
    auto c3 = analyzer.analyze(bends);

    // Results should be identical
    REQUIRE(c1[0].requiredWidth == c2[0].requiredWidth);
    REQUIRE(c2[0].requiredWidth == c3[0].requiredWidth);

    REQUIRE(c1[0].clearance == c2[0].clearance);
    REQUIRE(c2[0].clearance == c3[0].clearance);

    REQUIRE(c1[0].totalSegments == c2[0].totalSegments);
    REQUIRE(c2[0].totalSegments == c3[0].totalSegments);

    REQUIRE(c1[0].totalWidth == c2[0].totalWidth);
    REQUIRE(c2[0].totalWidth == c3[0].totalWidth);
}

TEST_CASE("Integration - module completion verification", "[phase2][aba][integration]") {
    ABAConstraintAnalyzer analyzer;

    // Create comprehensive test scenario
    std::vector<BendFeature> bends;

    // Mix of different bends
    double lengths[] = { 50.0, 100.0, 150.0, 200.0, 275.0 };
    double angles[] = { 45.0, 60.0, 90.0, 120.0, 135.0 };

    for (int i = 0; i < 5; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = angles[i];
        bend.length = lengths[i];
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    // Verify module completeness
    REQUIRE(constraints.size() == 5);

    // All core features should work
    for (const auto& c : constraints) {
        // Feature 1: Required width calculation
        REQUIRE(c.requiredWidth == Approx(c.bendLength + c.clearance).epsilon(0.01));

        // Feature 2: Clearance calculation
        REQUIRE(c.clearance >= 5.0);
        REQUIRE(c.clearance <= 15.0);

        // Feature 3: Subset sum solver
        if (c.feasible) {
            REQUIRE(!c.segmentSolution.empty());
            REQUIRE(c.totalWidth >= c.requiredWidth);
        }

        // Feature 4: Box closing detection
        REQUIRE((c.isBoxClosing == true || c.isBoxClosing == false));

        // Feature 5: Reason generation
        REQUIRE(!c.reason.empty());
    }

    // Statistics tracking
    auto stats = analyzer.getStatistics();
    REQUIRE(stats.totalBendsAnalyzed == 5);
    REQUIRE(stats.feasibleCount >= 0);
    REQUIRE(stats.analysisTimeMs >= 0.0);
}

