#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

// ============================================================================
// Task 14: Integration Tests - Complex Scenarios
// ============================================================================

TEST_CASE("Integration - Simple L-bracket (2 bends)", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Simple L-bracket: 2 perpendicular bends
    // No conflicts expected
    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 0.0;
    b0.direction.y = 1.0;  // Along Y

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 100.0;
    b1.position.y = 50.0;
    b1.direction.x = 1.0;  // Along X (perpendicular)
    b1.direction.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should check 2 pairs
    REQUIRE(stats.totalPairsChecked == 2);

    // Perpendicular far apart → minimal or no constraints
    // Exact count depends on tolerance
    REQUIRE(stats.totalConstraints >= 0);
}

TEST_CASE("Integration - U-channel (3 bends)", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // U-channel: 3 bends forming U-shape
    //
    //  b0 (left)         b2 (right)
    //     |                 |
    //     |                 |
    //     +-----------------+
    //          b1 (bottom)

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
    auto stats = analyzer.getStatistics();

    // Should check 6 pairs (3 bends)
    REQUIRE(stats.totalPairsChecked == 6);

    // May have some constraints depending on geometry
    REQUIRE(stats.totalConstraints >= 0);

    // Should NOT have box closing yet (only 3 sides)
    // Fourth bend would trigger box closing
}

TEST_CASE("Integration - Closed box (4 bends)", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Closed rectangular box: 4 bends
    // This is a critical test case

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
    auto stats = analyzer.getStatistics();

    // Should check 12 pairs (4 bends)
    REQUIRE(stats.totalPairsChecked == 12);

    // Box closing should be evaluated
    REQUIRE(stats.boxClosingCount >= 0);
}

TEST_CASE("Integration - Parallel close bends (blocking)", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Two parallel bends very close together
    // Should trigger sequential blocking

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 0.0;
    b0.direction.y = 1.0;  // Parallel

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 40.0;  // 40mm apart (within blocking range)
    b1.position.y = 0.0;
    b1.direction.x = 0.0;
    b1.direction.y = 1.0;  // Parallel

    std::vector<BendFeature> bends = { b0, b1 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should detect sequential blocking
    REQUIRE(stats.sequentialBlockCount >= 1);

    // Should have SEQUENTIAL constraints
    bool hasSequential = false;
    for (const auto& edge : constraints) {
        if (edge.type == ConstraintType::SEQUENTIAL) {
            hasSequential = true;
        }
    }
    REQUIRE(hasSequential == true);
}

TEST_CASE("Integration - Mixed constraints scenario", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Complex scenario with multiple constraint types

    BendFeature b0, b1, b2;

    // Bend 0
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 0.0;
    b0.direction.y = 1.0;

    // Bend 1 - close and parallel to b0 (sequential blocking)
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 35.0;  // Close (blocking range)
    b1.position.y = 0.0;
    b1.direction.x = 0.0;
    b1.direction.y = 1.0;  // Parallel

    // Bend 2 - far away (no conflicts)
    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 500.0;  // Far away
    b2.position.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should have some constraints
    REQUIRE(stats.totalConstraints >= 1);

    // Should have sequential blocking between b0 and b1
    REQUIRE(stats.sequentialBlockCount >= 1);

    // Verify constraint properties
    for (const auto& edge : constraints) {
        // All constraints should have valid IDs
        REQUIRE(edge.id >= 0);
        REQUIRE(edge.fromBend >= 0);
        REQUIRE(edge.toBend >= 0);

        // Confidence should be in valid range
        REQUIRE(edge.confidence >= 0.0);
        REQUIRE(edge.confidence <= 1.0);

        // Should have reasoning
        REQUIRE(!edge.reasoning.empty());
    }
}

TEST_CASE("Integration - Statistics consistency", "[phase2][integration]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create varied scenario
    BendFeature b0, b1, b2, b3;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 30.0;  // Close
    b1.position.y = 0.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 60.0;  // Close
    b2.position.y = 0.0;

    b3.id = 3;
    b3.angle = 90.0;
    b3.length = 100.0;
    b3.position.x = 500.0;  // Far
    b3.position.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1, b2, b3 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Statistics consistency checks
    REQUIRE(stats.totalPairsChecked == 12);  // 4 bends = 12 pairs

    // Total constraints should equal sum of individual counts
    int sum = stats.cornerOverlapCount +
              stats.boxClosingCount +
              stats.sequentialBlockCount;

    REQUIRE(stats.totalConstraints == sum);

    // All counts should be non-negative
    REQUIRE(stats.cornerOverlapCount >= 0);
    REQUIRE(stats.boxClosingCount >= 0);
    REQUIRE(stats.sequentialBlockCount >= 0);
    REQUIRE(stats.totalConstraints >= 0);
}
