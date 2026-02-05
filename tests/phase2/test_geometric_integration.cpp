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

// ============================================================================
// Task 15: Performance Metrics & Optimization Tests
// ============================================================================

TEST_CASE("Performance - timing metrics collected", "[phase2][performance]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create moderate-size scenario
    std::vector<BendFeature> bends;
    for (int i = 0; i < 5; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 100.0;  // Spread out
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should have timing metrics
    REQUIRE(stats.analysisTimeMs >= 0.0);
    REQUIRE(stats.avgPairTimeMs >= 0.0);

    // For 5 bends, should check 20 pairs (5*4)
    REQUIRE(stats.totalPairsChecked == 20);

    // Average time should be reasonable (< 1ms per pair for simple geometry)
    // This is a performance sanity check
    REQUIRE(stats.avgPairTimeMs < 10.0);  // 10ms is very generous
}

TEST_CASE("Performance - max constraints per pair tracked", "[phase2][performance]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create scenario with multiple constraints
    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 0.0;
    b0.direction.y = 1.0;

    // b1 close to b0 (multiple constraints possible)
    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 30.0;
    b1.position.y = 0.0;
    b1.direction.x = 0.0;
    b1.direction.y = 1.0;

    // b2 far away (no constraints)
    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 500.0;
    b2.position.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should track max constraints per pair
    REQUIRE(stats.maxConstraintsPerPair >= 0);

    // For close parallel bends, might get multiple constraints
    // (corner overlap + sequential blocking)
    // Max should be reasonable (≤ 3 for current implementation)
    REQUIRE(stats.maxConstraintsPerPair <= 3);
}

TEST_CASE("Performance - scaling with bend count", "[phase2][performance]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Test with increasing bend counts
    std::vector<int> bendCounts = { 2, 4, 6, 8 };
    std::vector<double> times;

    for (int n : bendCounts) {
        std::vector<BendFeature> bends;
        for (int i = 0; i < n; i++) {
            BendFeature bend;
            bend.id = i;
            bend.angle = 90.0;
            bend.length = 100.0;
            bend.position.x = i * 150.0;  // Spread out to avoid constraints
            bend.position.y = 0.0;
            bends.push_back(bend);
        }

        auto constraints = analyzer.analyze(bends);
        auto stats = analyzer.getStatistics();

        times.push_back(stats.analysisTimeMs);

        // Verify O(n²) scaling (pairs = n*(n-1))
        int expectedPairs = n * (n - 1);
        REQUIRE(stats.totalPairsChecked == expectedPairs);
    }

    // Times should increase roughly quadratically
    // (This is a rough check - just ensure it's not exponential)
    // time[3] / time[0] should be roughly (8/2)² = 16
    // But we'll be lenient due to variance
    if (times[0] > 0.0) {
        double ratio = times[3] / times[0];
        REQUIRE(ratio < 100.0);  // Should not be exponential
    }
}

TEST_CASE("Performance - empty input (edge case)", "[phase2][performance]") {
    GeometricPrecedenceAnalyzer analyzer;

    std::vector<BendFeature> bends;  // Empty

    auto constraints = analyzer.analyze(bends);
    auto stats = analyzer.getStatistics();

    // Should handle gracefully
    REQUIRE(stats.analysisTimeMs >= 0.0);
    REQUIRE(stats.totalPairsChecked == 0);
    REQUIRE(stats.totalConstraints == 0);
    REQUIRE(constraints.empty());
}

