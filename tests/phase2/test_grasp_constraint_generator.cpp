#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/grasp_constraint_generator.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

// ============================================================================
// Task 16: GraspConstraintGenerator - Basic Setup & Constructor
// ============================================================================

TEST_CASE("GraspConstraintGenerator - constructor", "[phase2][grasp][basic]") {
    GraspConstraintGenerator generator;

    // Should construct successfully
    REQUIRE(true);
}

TEST_CASE("GraspConstraintGenerator - analyze empty bends", "[phase2][grasp][basic]") {
    GraspConstraintGenerator generator;

    std::vector<BendFeature> bends;  // Empty
    std::vector<int> bentBends;      // Empty state

    auto constraint = generator.analyze(bends, bentBends);

    // Empty analysis should return default constraint
    REQUIRE(constraint.stateId == 0);
    REQUIRE(constraint.bentBends.empty());
    REQUIRE(constraint.hasValidGrip == false);
}

TEST_CASE("GraspConstraintGenerator - analyze single bend (flat state)", "[phase2][grasp][basic]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;
    bend.position.z = 0.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;  // Not bent yet (flat state)

    auto constraint = generator.analyze(bends, bentBends);

    // Flat state should have valid grip (entire sheet available)
    REQUIRE(constraint.stateId == 0);
    REQUIRE(constraint.hasValidGrip == true);
    REQUIRE(constraint.deadZones.empty());  // No dead zones in flat state
}

TEST_CASE("GraspConstraintGenerator - analyze single bent bend", "[phase2][grasp][basic]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };  // Already bent

    auto constraint = generator.analyze(bends, bentBends);

    // Should have dead zone from standing flange
    REQUIRE(constraint.stateId >= 0);
    REQUIRE(constraint.bentBends.size() == 1);
    REQUIRE(constraint.deadZones.size() >= 1);  // At least 1 dead zone

    // Dead zone should be STANDING_FLANGE type
    REQUIRE(constraint.deadZones[0].type == DeadZoneType::STANDING_FLANGE);
    REQUIRE(constraint.deadZones[0].causedByBend == 0);
}

TEST_CASE("GraspConstraintGenerator - statistics tracking", "[phase2][grasp][basic]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;

    generator.analyze(bends, bentBends);

    auto stats = generator.getStatistics();

    // Should have basic statistics
    REQUIRE(stats.totalStatesAnalyzed == 1);
    REQUIRE(stats.totalDeadZonesGenerated >= 0);
    REQUIRE(stats.validGripCount >= 0);
    REQUIRE(stats.invalidGripCount >= 0);
}
