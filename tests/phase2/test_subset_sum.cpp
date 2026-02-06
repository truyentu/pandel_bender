#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/aba_constraint_analyzer.h"
#include <numeric>

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

// ============================================================================
// Task 34: Enhanced Subset Sum Algorithm Tests
// ============================================================================

TEST_CASE("Subset sum finds exact match", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 190.0;  // 190 + 10 clearance = 200
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints.size() == 1);
    REQUIRE(constraints[0].feasible == true);

    int total = std::accumulate(
        constraints[0].segmentSolution.begin(),
        constraints[0].segmentSolution.end(), 0
    );
    REQUIRE(total >= 200);
    REQUIRE(total <= 225);  // Minimal waste
}

TEST_CASE("Subset sum minimizes segment count", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 290.0;  // 290 + 10 = 300
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);
    REQUIRE(constraints[0].totalSegments <= 3);
}

TEST_CASE("Subset sum handles large width", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 990.0;  // Large bend
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);

    int total = std::accumulate(
        constraints[0].segmentSolution.begin(),
        constraints[0].segmentSolution.end(), 0
    );
    REQUIRE(total >= 1000);
}

TEST_CASE("Subset sum handles small width", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 15.0;  // 15 + 5 clearance = 20
    bend.angle = 45.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);
    REQUIRE(constraints[0].segmentSolution.size() >= 1);
}

TEST_CASE("Subset sum uses largest segments first", "[phase2][aba][subsetsum]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 390.0;  // Need 400
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);
    // Should use 200 + 200 (2 segments) instead of smaller segments
    REQUIRE(constraints[0].totalSegments <= 3);
}

// ============================================================================
// Additional edge case tests for subset sum robustness
// ============================================================================

TEST_CASE("Subset sum handles exact segment size match", "[phase2][aba][subsetsum][edge]") {
    ABAConstraintAnalyzer analyzer;

    // Test exact matches with segment sizes: 25, 50, 75, 100, 125, 150, 175, 200
    std::vector<double> exactSizes = {25.0, 50.0, 75.0, 100.0, 125.0, 150.0, 175.0, 200.0};

    for (double size : exactSizes) {
        BendFeature bend;
        bend.id = 0;
        bend.angle = 45.0;  // Lower angle for smaller clearance
        bend.length = size - 5.0;  // Account for clearance

        auto constraints = analyzer.analyze({bend});

        REQUIRE(constraints.size() == 1);
        REQUIRE(constraints[0].feasible == true);
    }
}

TEST_CASE("Subset sum handles boundary between DP and greedy", "[phase2][aba][subsetsum][edge]") {
    ABAConstraintAnalyzer analyzer;

    // Test around the 1000mm boundary where greedy fallback kicks in
    std::vector<double> boundaryLengths = {980.0, 990.0, 1000.0, 1010.0, 1020.0};

    for (double length : boundaryLengths) {
        BendFeature bend;
        bend.id = 0;
        bend.angle = 90.0;
        bend.length = length;

        auto constraints = analyzer.analyze({bend});

        REQUIRE(constraints.size() == 1);
        REQUIRE(constraints[0].feasible == true);
        REQUIRE(constraints[0].totalWidth >= constraints[0].requiredWidth);
    }
}

TEST_CASE("Subset sum maintains solution optimality", "[phase2][aba][subsetsum][optimization]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 250.0;  // 250 + clearance ~ 260
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);

    // Optimal solution should use fewest segments
    // 200 + 75 = 275 (2 segments) is better than 100 + 100 + 75 (3 segments)
    REQUIRE(constraints[0].totalSegments <= 2);

    // Waste should be minimal
    double waste = constraints[0].totalWidth - constraints[0].requiredWidth;
    REQUIRE(waste >= 0.0);
    REQUIRE(waste < 25.0);  // Less than smallest segment
}

TEST_CASE("Subset sum handles multiple bends efficiently", "[phase2][aba][subsetsum][performance]") {
    ABAConstraintAnalyzer analyzer;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 50; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 50.0 + (i * 15.0);  // Varying lengths from 50 to 785
        bends.push_back(bend);
    }

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.size() == 50);

    // All should be feasible
    for (const auto& c : constraints) {
        REQUIRE(c.feasible == true);
    }

    // Performance check
    auto stats = analyzer.getStatistics();
    REQUIRE(stats.avgBendTimeMs < 10.0);  // Should be fast
}

TEST_CASE("Subset sum handles zero-length bend gracefully", "[phase2][aba][subsetsum][edge]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 0.0;  // Zero length edge case
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints.size() == 1);
    // Should still find a solution using smallest segments
    REQUIRE(constraints[0].feasible == true);
}

TEST_CASE("Subset sum solutions are deterministic", "[phase2][aba][subsetsum][deterministic]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 175.0;
    bend.angle = 90.0;

    // Run multiple times
    std::vector<std::vector<int>> solutions;
    for (int i = 0; i < 5; i++) {
        auto constraints = analyzer.analyze({bend});
        solutions.push_back(constraints[0].segmentSolution);
    }

    // All solutions should be identical
    for (size_t i = 1; i < solutions.size(); i++) {
        REQUIRE(solutions[i] == solutions[0]);
    }
}

TEST_CASE("Subset sum handles prime-like widths", "[phase2][aba][subsetsum][edge]") {
    ABAConstraintAnalyzer analyzer;

    // Test widths that are hard to compose from available segments
    std::vector<double> primeishLengths = {47.0, 67.0, 83.0, 97.0, 113.0, 127.0};

    for (double length : primeishLengths) {
        BendFeature bend;
        bend.id = 0;
        bend.angle = 90.0;
        bend.length = length;

        auto constraints = analyzer.analyze({bend});

        REQUIRE(constraints.size() == 1);
        REQUIRE(constraints[0].feasible == true);
        // Solution may have some waste but should still cover
        REQUIRE(constraints[0].totalWidth >= constraints[0].requiredWidth);
    }
}

TEST_CASE("Subset sum validates segment solution consistency", "[phase2][aba][subsetsum][validation]") {
    ABAConstraintAnalyzer analyzer;

    BendFeature bend;
    bend.id = 0;
    bend.length = 300.0;
    bend.angle = 90.0;

    auto constraints = analyzer.analyze({bend});

    REQUIRE(constraints[0].feasible == true);

    // Verify totalWidth matches sum of segments
    int sumOfSegments = std::accumulate(
        constraints[0].segmentSolution.begin(),
        constraints[0].segmentSolution.end(), 0
    );

    REQUIRE(sumOfSegments == static_cast<int>(constraints[0].totalWidth));

    // Verify totalSegments matches solution size
    REQUIRE(constraints[0].totalSegments ==
            static_cast<int>(constraints[0].segmentSolution.size()));
}
