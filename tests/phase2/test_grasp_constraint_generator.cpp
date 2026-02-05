#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/grasp_constraint_generator.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

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

// ============================================================================
// Task 17: Dead Zone Calculation Improvements
// ============================================================================

TEST_CASE("DeadZone - area calculation from polygon", "[phase2][grasp][deadzone]") {
    DeadZone zone;
    zone.id = 0;
    zone.type = DeadZoneType::STANDING_FLANGE;

    // Create simple square 10x10mm
    zone.polygon.vertices = {
        Point2D(0, 0),
        Point2D(10, 0),
        Point2D(10, 10),
        Point2D(0, 10)
    };

    double area = zone.area();
    REQUIRE(area == 100.0);  // 10 * 10
}

TEST_CASE("DeadZone - contains point test", "[phase2][grasp][deadzone]") {
    DeadZone zone;
    zone.polygon.vertices = {
        Point2D(0, 0),
        Point2D(10, 0),
        Point2D(10, 10),
        Point2D(0, 10)
    };

    // Inside
    REQUIRE(zone.contains(Point2D(5, 5)) == true);

    // Outside
    REQUIRE(zone.contains(Point2D(15, 5)) == false);
    REQUIRE(zone.contains(Point2D(5, 15)) == false);

    // Edge cases
    REQUIRE(zone.contains(Point2D(0, 0)) == true);  // Vertex
    REQUIRE(zone.contains(Point2D(5, 0)) == true);   // Edge
}

TEST_CASE("Dead zone - 90 degree bend creates accurate footprint", "[phase2][grasp][deadzone]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;
    bend.position.z = 0.0;
    bend.direction.x = 0.0;
    bend.direction.y = 1.0;  // Along Y
    bend.normal.x = 0.0;
    bend.normal.y = 0.0;
    bend.normal.z = 1.0;     // Rotate around Z

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };  // Already bent

    auto constraint = generator.analyze(bends, bentBends);

    // Should have 1 dead zone
    REQUIRE(constraint.deadZones.size() == 1);

    const auto& dz = constraint.deadZones[0];

    // Verify dead zone properties
    REQUIRE(dz.type == DeadZoneType::STANDING_FLANGE);
    REQUIRE(dz.causedByBend == 0);
    REQUIRE(dz.safetyMargin == 5.0);

    // Dead zone should have rectangular footprint
    REQUIRE(dz.polygon.vertices.size() == 4);

    // Area should be (flange_width + 2*safety_margin) * (bend_length + 2*safety_margin)
    // Flange width = 50mm, safety margin = 5mm, length = 100mm
    // Projected width = 50 * sin(90°) = 50mm
    // With margin: (50 + 10) * (100 + 10) = 60 * 110 = 6600mm²
    double area = dz.area();
    REQUIRE(area > 0.0);
    REQUIRE(area <= 7000.0);  // Should be around 6600mm²
    REQUIRE(area >= 6000.0);  // Lower bound
}

TEST_CASE("Dead zone - 45 degree bend creates smaller footprint", "[phase2][grasp][deadzone]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 45.0;  // Smaller angle
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // Should still have dead zone, but smaller
    REQUIRE(constraint.deadZones.size() == 1);

    double area45 = constraint.deadZones[0].area();

    // Compare with 90 degree
    bend.angle = 90.0;
    auto constraint90 = generator.analyze(bends, bentBends);
    double area90 = constraint90.deadZones[0].area();

    // 45 degree should have smaller footprint than 90 degree
    // (because flange doesn't stand as tall)
    REQUIRE(area45 <= area90);
}

TEST_CASE("Dead zone - safety margin expands footprint", "[phase2][grasp][deadzone]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // Dead zone should have safety margin
    REQUIRE(constraint.deadZones[0].safetyMargin > 0.0);

    // Polygon should be expanded by safety margin
    // (Check by verifying polygon is larger than base flange)
    double dzArea = constraint.deadZones[0].area();

    // Base flange area (50mm x 100mm)
    double baseFlangeArea = 50.0 * 100.0;

    // Dead zone should be larger due to safety margin
    REQUIRE(dzArea >= baseFlangeArea);
}

TEST_CASE("Dead zone - multiple bends create multiple zones", "[phase2][grasp][deadzone]") {
    GraspConstraintGenerator generator;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 50.0;
    b0.position.y = 50.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 150.0;  // Different position
    b1.position.y = 50.0;

    std::vector<BendFeature> bends = { b0, b1 };
    std::vector<int> bentBends = { 0, 1 };  // Both bent

    auto constraint = generator.analyze(bends, bentBends);

    // Should have 2 dead zones
    REQUIRE(constraint.deadZones.size() == 2);

    // Each should be from different bend
    REQUIRE(constraint.deadZones[0].causedByBend == 0);
    REQUIRE(constraint.deadZones[1].causedByBend == 1);

    // Both should be STANDING_FLANGE type
    REQUIRE(constraint.deadZones[0].type == DeadZoneType::STANDING_FLANGE);
    REQUIRE(constraint.deadZones[1].type == DeadZoneType::STANDING_FLANGE);
}

TEST_CASE("Dead zone - partial bend state", "[phase2][grasp][deadzone]") {
    GraspConstraintGenerator generator;

    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };
    std::vector<int> bentBends = { 0, 2 };  // Only 0 and 2 bent, not 1

    auto constraint = generator.analyze(bends, bentBends);

    // Should have 2 dead zones (only for bent bends)
    REQUIRE(constraint.deadZones.size() == 2);

    // Should be from bends 0 and 2
    bool hasZone0 = false;
    bool hasZone2 = false;
    bool hasZone1 = false;

    for (const auto& dz : constraint.deadZones) {
        if (dz.causedByBend == 0) hasZone0 = true;
        if (dz.causedByBend == 1) hasZone1 = true;
        if (dz.causedByBend == 2) hasZone2 = true;
    }

    REQUIRE(hasZone0 == true);
    REQUIRE(hasZone2 == true);
    REQUIRE(hasZone1 == false);  // Bend 1 not bent
}

// ============================================================================
// Task 18: 2D Projection & Valid Region Calculation
// ============================================================================

TEST_CASE("Valid region - flat state has full sheet available", "[phase2][grasp][region]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;  // Flat state

    auto constraint = generator.analyze(bends, bentBends);

    // Flat state should have large valid region (full sheet)
    double validArea = constraint.validRegion.area();
    REQUIRE(validArea > 200000.0);  // 500x500 = 250000mm²

    // Should have no dead zones
    REQUIRE(constraint.deadZones.empty());
}

TEST_CASE("Valid region - single bent bend reduces available area", "[phase2][grasp][region]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 250.0;  // Center of sheet
    bend.position.y = 250.0;

    std::vector<BendFeature> bends = { bend };

    // Compare flat vs bent
    std::vector<int> flatState;
    std::vector<int> bentState = { 0 };

    auto flatConstraint = generator.analyze(bends, flatState);
    auto bentConstraint = generator.analyze(bends, bentState);

    double flatArea = flatConstraint.validRegion.area();
    double bentArea = bentConstraint.validRegion.area();

    // Bent state should have smaller valid region
    REQUIRE(bentArea < flatArea);

    // Bent state should have at least 1 dead zone
    REQUIRE(bentConstraint.deadZones.size() >= 1);
}

TEST_CASE("Valid region - multiple dead zones further reduce area", "[phase2][grasp][region]") {
    GraspConstraintGenerator generator;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 100.0;
    b0.position.y = 250.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 400.0;
    b1.position.y = 250.0;

    std::vector<BendFeature> bends = { b0, b1 };

    // Compare: no bends, 1 bend, 2 bends
    auto state0 = generator.analyze(bends, {});
    auto state1 = generator.analyze(bends, {0});
    auto state2 = generator.analyze(bends, {0, 1});

    double area0 = state0.validRegion.area();
    double area1 = state1.validRegion.area();
    double area2 = state2.validRegion.area();

    // More dead zones → less valid area
    REQUIRE(area0 > area1);
    REQUIRE(area1 > area2);
}

TEST_CASE("Valid region - maintains rectangular shape (simplified)", "[phase2][grasp][region]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // Valid region should be rectangular (4 vertices for simplified implementation)
    REQUIRE(constraint.validRegion.vertices.size() == 4);

    // Verify it's axis-aligned rectangle
    const auto& vr = constraint.validRegion;

    // All x-coordinates should be one of two values (left or right edge)
    // All y-coordinates should be one of two values (top or bottom edge)
    double minX = vr.vertices[0].x;
    double maxX = vr.vertices[0].x;
    double minY = vr.vertices[0].y;
    double maxY = vr.vertices[0].y;

    for (const auto& v : vr.vertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.y > maxY) maxY = v.y;
    }

    // Width and height should be positive
    REQUIRE(maxX > minX);
    REQUIRE(maxY > minY);
}

TEST_CASE("Valid region - centroid calculation", "[phase2][grasp][region]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;

    auto constraint = generator.analyze(bends, bentBends);

    // Flat state valid region should be centered around sheet center
    Point2D centroid = constraint.validRegion.centroid();

    // For 500x500 sheet, centroid should be around (250, 250)
    REQUIRE(centroid.x > 200.0);
    REQUIRE(centroid.x < 300.0);
    REQUIRE(centroid.y > 200.0);
    REQUIRE(centroid.y < 300.0);
}

TEST_CASE("MIR - fits within valid region", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // MIR should fit within valid region
    Rectangle2D mir = constraint.maxInscribedRect;

    // MIR corners should be contained in valid region
    Point2D bl = mir.bottomLeft;
    Point2D tr = mir.topRight;
    Point2D br(tr.x, bl.y);
    Point2D tl(bl.x, tr.y);

    REQUIRE(constraint.validRegion.contains(bl));
    REQUIRE(constraint.validRegion.contains(tr));
    REQUIRE(constraint.validRegion.contains(br));
    REQUIRE(constraint.validRegion.contains(tl));
}

TEST_CASE("MIR - area is positive and reasonable", "[phase2][grasp][mir]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // MIR should have positive area
    REQUIRE(constraint.maxInscribedRect.area > 0.0);

    // MIR area should be <= valid region area
    double mirArea = constraint.maxInscribedRect.area;
    double validArea = constraint.validRegion.area();

    REQUIRE(mirArea <= validArea);
}

TEST_CASE("Grip validation - minimum area threshold", "[phase2][grasp][validation]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;

    auto constraint = generator.analyze(bends, bentBends);

    // Flat state should have valid grip
    REQUIRE(constraint.hasValidGrip == true);

    // Valid region area should be above minimum threshold
    double validArea = constraint.validRegion.area();
    REQUIRE(validArea >= constraint.minRequiredArea);

    // MIR area should also be sufficient
    REQUIRE(constraint.maxInscribedRect.area >= 100.0);
}

TEST_CASE("Grip validation - COM within grip region", "[phase2][grasp][validation]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 250.0;
    bend.position.y = 250.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // COM should be calculated
    REQUIRE(constraint.centerOfMass.x > 0.0);
    REQUIRE(constraint.centerOfMass.y > 0.0);

    // For valid grip, COM should be reasonably close to MIR center
    Point2D mirCenter = constraint.maxInscribedRect.center;
    Point2D com = constraint.centerOfMass;

    double dx = com.x - mirCenter.x;
    double dy = com.y - mirCenter.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    // Distance should be reasonable (< 150mm for this case)
    REQUIRE(distance < 150.0);
}

// ============================================================================
// Task 19: True MIR (Maximum Inscribed Rectangle) Algorithm
// ============================================================================

TEST_CASE("MIR - simple rectangular valid region", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;  // Flat state

    auto constraint = generator.analyze(bends, bentBends);

    // For rectangular valid region, MIR should equal the region (minus inset)
    Rectangle2D mir = constraint.maxInscribedRect;

    // MIR should be close to valid region size
    double validArea = constraint.validRegion.area();
    double mirArea = mir.area;

    // With 0.1mm inset, area loss is minimal
    // For 500x500 rect: (500-0.2) * (500-0.2) = 249800.04
    // Area ratio should be > 0.999
    double ratio = mirArea / validArea;
    REQUIRE(ratio > 0.999);
}

TEST_CASE("MIR - center aligned with valid region center", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;

    auto constraint = generator.analyze(bends, bentBends);

    // MIR center should be close to valid region centroid
    Point2D mirCenter = constraint.maxInscribedRect.center;
    Point2D validCentroid = constraint.validRegion.centroid();

    double dx = std::abs(mirCenter.x - validCentroid.x);
    double dy = std::abs(mirCenter.y - validCentroid.y);

    // Should be very close (within 1mm)
    REQUIRE(dx < 1.0);
    REQUIRE(dy < 1.0);
}

TEST_CASE("MIR - optimal grip point set correctly", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    // Optimal grip point should be MIR center
    Point2D optimalGrip = constraint.optimalGripCenter;
    Point2D mirCenter = constraint.maxInscribedRect.center;

    REQUIRE(optimalGrip.x == mirCenter.x);
    REQUIRE(optimalGrip.y == mirCenter.y);
}

TEST_CASE("MIR - dimensions are positive and reasonable", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    auto constraint = generator.analyze(bends, bentBends);

    Rectangle2D mir = constraint.maxInscribedRect;

    // Width and height should be positive
    REQUIRE(mir.width > 0.0);
    REQUIRE(mir.height > 0.0);

    // Dimensions should be reasonable (< sheet size)
    REQUIRE(mir.width <= 500.0);
    REQUIRE(mir.height <= 500.0);

    // Area should match width * height
    REQUIRE(mir.area == Approx(mir.width * mir.height).epsilon(0.01));
}

TEST_CASE("MIR - consistent across multiple analyses", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends = { 0 };

    // Analyze multiple times
    auto c1 = generator.analyze(bends, bentBends);
    auto c2 = generator.analyze(bends, bentBends);

    // MIR should be identical
    REQUIRE(c1.maxInscribedRect.width == c2.maxInscribedRect.width);
    REQUIRE(c1.maxInscribedRect.height == c2.maxInscribedRect.height);
    REQUIRE(c1.maxInscribedRect.area == c2.maxInscribedRect.area);
    REQUIRE(c1.maxInscribedRect.center.x == c2.maxInscribedRect.center.x);
    REQUIRE(c1.maxInscribedRect.center.y == c2.maxInscribedRect.center.y);
}

TEST_CASE("MIR - aspect ratio reasonable", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    std::vector<BendFeature> bends = { bend };
    std::vector<int> bentBends;

    auto constraint = generator.analyze(bends, bentBends);

    Rectangle2D mir = constraint.maxInscribedRect;

    // For square valid region (500x500), aspect ratio should be close to 1
    double aspectRatio = mir.width / mir.height;

    // Should be reasonably square (between 0.9 and 1.1)
    REQUIRE(aspectRatio > 0.9);
    REQUIRE(aspectRatio < 1.1);
}

TEST_CASE("MIR - area maximization", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 250.0;
    bend.position.y = 250.0;

    std::vector<BendFeature> bends = { bend };

    // Compare flat vs bent state MIR
    auto flatConstraint = generator.analyze(bends, {});
    auto bentConstraint = generator.analyze(bends, {0});

    // Flat state should have larger MIR
    REQUIRE(flatConstraint.maxInscribedRect.area > bentConstraint.maxInscribedRect.area);

    // Both should have reasonable areas
    REQUIRE(flatConstraint.maxInscribedRect.area > 200000.0);  // Large
    REQUIRE(bentConstraint.maxInscribedRect.area > 10000.0);   // Smaller but still significant
}

TEST_CASE("MIR - handles small valid regions", "[phase2][grasp][mir][advanced]") {
    GraspConstraintGenerator generator;

    // Create scenario with multiple bends to heavily constrain valid region
    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 100.0;
    b0.position.y = 250.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 250.0;
    b1.position.y = 250.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 400.0;
    b2.position.y = 250.0;

    std::vector<BendFeature> bends = { b0, b1, b2 };
    std::vector<int> bentBends = { 0, 1, 2 };

    auto constraint = generator.analyze(bends, bentBends);

    // Even with heavily constrained region, MIR should be valid
    REQUIRE(constraint.maxInscribedRect.area > 0.0);
    REQUIRE(constraint.maxInscribedRect.width > 0.0);
    REQUIRE(constraint.maxInscribedRect.height > 0.0);

    // All corners should be inside valid region
    Rectangle2D mir = constraint.maxInscribedRect;
    REQUIRE(constraint.validRegion.contains(mir.bottomLeft));
    REQUIRE(constraint.validRegion.contains(mir.topRight));
}
