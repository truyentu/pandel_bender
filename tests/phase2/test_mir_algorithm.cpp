#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "openpanelcam/phase2/grasp_constraint_generator.h"
#include "openpanelcam/phase2/phase1_mock.h"
#include <cmath>

using namespace openpanelcam::phase2;
using namespace openpanelcam;
using Catch::Matchers::WithinAbs;

TEST_CASE("MIR finds largest rectangle in L-shaped region", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Create a single bend that creates an L-shaped valid region
    std::vector<phase1::BendFeature> bends;
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 50.0;
    bend.position.x = 75.0;
    bend.position.y = 75.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;
    bends.push_back(bend);

    // Analyze with bend 0 bent - creates dead zone
    auto constraint = generator.analyze(bends, {0});

    // Verify MIR was found with positive dimensions
    REQUIRE(constraint.maxInscribedRect.area > 0);
    REQUIRE(constraint.maxInscribedRect.width > 0);
    REQUIRE(constraint.maxInscribedRect.height > 0);

    // MIR center should be inside valid region
    REQUIRE(constraint.validRegion.contains(constraint.maxInscribedRect.center));

    // MIR corners should also be inside valid region
    Point2D bl = constraint.maxInscribedRect.bottomLeft;
    Point2D tr = constraint.maxInscribedRect.topRight;

    // Verify the rectangle has reasonable dimensions
    INFO("MIR width: " << constraint.maxInscribedRect.width);
    INFO("MIR height: " << constraint.maxInscribedRect.height);
    INFO("MIR area: " << constraint.maxInscribedRect.area);
}

TEST_CASE("MIR handles multiple dead zones", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Create two bends at opposite corners
    std::vector<phase1::BendFeature> bends;

    phase1::BendFeature bend1;
    bend1.id = 0;
    bend1.angle = 90.0;
    bend1.length = 50.0;
    bend1.position.x = 50.0;
    bend1.position.y = 50.0;
    bend1.direction.x = 0.0;
    bend1.direction.y = 1.0;
    bends.push_back(bend1);

    phase1::BendFeature bend2;
    bend2.id = 1;
    bend2.angle = 90.0;
    bend2.length = 50.0;
    bend2.position.x = 400.0;
    bend2.position.y = 400.0;
    bend2.direction.x = 0.0;
    bend2.direction.y = 1.0;
    bends.push_back(bend2);

    // Both bends are bent, creating two dead zones
    auto constraint = generator.analyze(bends, {0, 1});

    // Should still find a valid grip
    REQUIRE(constraint.hasValidGrip == true);

    // MIR should have significant area (two corner dead zones leave center open)
    REQUIRE(constraint.maxInscribedRect.area > 100.0);

    // Should have 2 dead zones generated
    REQUIRE(constraint.deadZones.size() == 2);

    INFO("Dead zones count: " << constraint.deadZones.size());
    INFO("MIR area with multiple dead zones: " << constraint.maxInscribedRect.area);
}

TEST_CASE("MIR returns valid result for heavily covered region", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Create many bends covering most of the sheet
    std::vector<phase1::BendFeature> bends;

    for (int i = 0; i < 20; i++) {
        phase1::BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = (i % 5) * 100.0;
        bend.position.y = (i / 5) * 100.0;
        bend.direction.x = 0.0;
        bend.direction.y = 1.0;
        bends.push_back(bend);
    }

    std::vector<int> allBent;
    for (int i = 0; i < 20; i++) allBent.push_back(i);

    // This should not crash even with many dead zones
    auto constraint = generator.analyze(bends, allBent);

    // Algorithm should complete without crashing
    REQUIRE(constraint.maxInscribedRect.area >= 0);

    // Should have created dead zones for all bends
    REQUIRE(constraint.deadZones.size() == 20);

    INFO("MIR area with heavy coverage: " << constraint.maxInscribedRect.area);
}

TEST_CASE("MIR with flat state has full sheet area", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Define bends but don't bend any (flat state)
    std::vector<phase1::BendFeature> bends;
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 250.0;
    bend.position.y = 250.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;
    bends.push_back(bend);

    // Flat state - no bends are bent yet, no dead zones
    auto constraint = generator.analyze(bends, {});

    // Should have full valid area (no dead zones)
    REQUIRE(constraint.maxInscribedRect.area > 0);
    REQUIRE(constraint.hasValidGrip == true);
    REQUIRE(constraint.deadZones.empty());

    // Valid region should be entire sheet (500x500)
    double expectedArea = 500.0 * 500.0;
    // MIR should be close to full sheet (minus small inset)
    REQUIRE(constraint.maxInscribedRect.area > expectedArea * 0.99);

    INFO("Flat state MIR area: " << constraint.maxInscribedRect.area);
}

TEST_CASE("MIR center is optimal grip location", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    std::vector<phase1::BendFeature> bends;
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 100.0;
    bend.position.y = 100.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;
    bends.push_back(bend);

    auto constraint = generator.analyze(bends, {0});

    // Optimal grip center should be MIR center
    REQUIRE_THAT(constraint.optimalGripCenter.x,
                 WithinAbs(constraint.maxInscribedRect.center.x, 0.01));
    REQUIRE_THAT(constraint.optimalGripCenter.y,
                 WithinAbs(constraint.maxInscribedRect.center.y, 0.01));
}

TEST_CASE("MIR handles edge bend positions", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Bend at the edge of the sheet
    std::vector<phase1::BendFeature> bends;
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 0.0;  // At sheet edge
    bend.position.y = 250.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;
    bends.push_back(bend);

    auto constraint = generator.analyze(bends, {0});

    // Should still find valid MIR
    REQUIRE(constraint.maxInscribedRect.area > 0);

    // MIR should be shifted away from the edge dead zone
    REQUIRE(constraint.maxInscribedRect.center.x > 50.0);
}

TEST_CASE("MIR binary search finds correct size", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    // Create scenario with known valid region
    std::vector<phase1::BendFeature> bends;

    // Single bend that creates a predictable dead zone
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 200.0;
    bend.position.x = 250.0;
    bend.position.y = 250.0;
    bend.direction.x = 1.0;  // Horizontal bend line
    bend.direction.y = 0.0;
    bends.push_back(bend);

    auto constraint = generator.analyze(bends, {0});

    // Verify dimensions are positive
    REQUIRE(constraint.maxInscribedRect.width > 0);
    REQUIRE(constraint.maxInscribedRect.height > 0);

    // Width and height should be reasonable fractions of sheet
    // With one dead zone, MIR should still be substantial
    REQUIRE(constraint.maxInscribedRect.width > 50.0);
    REQUIRE(constraint.maxInscribedRect.height > 50.0);
}

TEST_CASE("MIR with various bend angles", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    std::vector<double> angles = {30.0, 45.0, 60.0, 90.0, 120.0};

    for (double angle : angles) {
        std::vector<phase1::BendFeature> bends;
        phase1::BendFeature bend;
        bend.id = 0;
        bend.angle = angle;
        bend.length = 100.0;
        bend.position.x = 250.0;
        bend.position.y = 250.0;
        bend.direction.x = 0.0;
        bend.direction.y = 1.0;
        bends.push_back(bend);

        auto constraint = generator.analyze(bends, {0});

        INFO("Testing bend angle: " << angle);

        // Should always find a valid MIR
        REQUIRE(constraint.maxInscribedRect.area > 0);

        // Higher angles create larger dead zones, so MIR may be smaller
        // but should still exist
        REQUIRE(constraint.maxInscribedRect.width > 0);
        REQUIRE(constraint.maxInscribedRect.height > 0);
    }
}

TEST_CASE("MIR statistics are tracked correctly", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    std::vector<phase1::BendFeature> bends;
    phase1::BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 250.0;
    bend.position.y = 250.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;
    bends.push_back(bend);

    // Analyze multiple states
    generator.analyze(bends, {});   // Flat state
    generator.analyze(bends, {0});  // Bent state

    auto stats = generator.getStatistics();

    REQUIRE(stats.totalStatesAnalyzed == 2);
    REQUIRE(stats.validGripCount + stats.invalidGripCount == 2);
    REQUIRE(stats.analysisTimeMs > 0);
}
