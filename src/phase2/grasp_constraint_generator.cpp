#include "openpanelcam/phase2/grasp_constraint_generator.h"
#include <chrono>
#include <cmath>

namespace openpanelcam {
namespace phase2 {

GraspConstraintGenerator::GraspConstraintGenerator() {
    m_stats = Statistics();
}

GraspConstraint GraspConstraintGenerator::analyze(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
) {
    auto startTime = std::chrono::high_resolution_clock::now();

    GraspConstraint constraint;
    constraint.stateId = m_stats.totalStatesAnalyzed;
    constraint.bentBends = bentBends;

    // Handle empty case
    if (bends.empty()) {
        m_stats.totalStatesAnalyzed++;
        m_stats.invalidGripCount++;
        constraint.hasValidGrip = false;
        return constraint;
    }

    // Step 1: Calculate dead zones from bent flanges
    constraint.deadZones = calculateDeadZones(bends, bentBends);
    m_stats.totalDeadZonesGenerated += static_cast<int>(constraint.deadZones.size());

    // Step 2: Calculate valid grip region
    // Simplified: Assume sheet size 500x500mm for now
    Point2D sheetSize(500.0, 500.0);
    constraint.validRegion = calculateValidRegion(sheetSize, constraint.deadZones);

    // Step 3: Find Maximum Inscribed Rectangle (MIR)
    constraint.maxInscribedRect = findMaxInscribedRect(constraint.validRegion);

    // Step 4: Validate grip physics
    bool physicsValid = validateGripPhysics(
        constraint.maxInscribedRect,
        bends,
        bentBends
    );

    // Step 5: Calculate center of mass
    constraint.centerOfMass = calculateCenterOfMass(bends, bentBends);

    // Step 6: Determine if grip is valid
    // Criteria:
    // - Valid region area > 100mm²
    // - MIR area > 0
    // - Physics validation passed
    double validArea = constraint.validRegion.area();
    double mirArea = constraint.maxInscribedRect.area;

    constraint.hasValidGrip = (validArea >= constraint.minRequiredArea) &&
                              (mirArea > 0.0) &&
                              physicsValid;

    // Set optimal grip center to MIR center
    constraint.optimalGripCenter = constraint.maxInscribedRect.center;

    // Update statistics
    m_stats.totalStatesAnalyzed++;
    if (constraint.hasValidGrip) {
        m_stats.validGripCount++;
    } else {
        m_stats.invalidGripCount++;
    }

    // Track timing
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        endTime - startTime
    );
    double stateTimeMs = duration.count() / 1000.0;
    m_stats.analysisTimeMs += stateTimeMs;
    if (m_stats.totalStatesAnalyzed > 0) {
        m_stats.avgStateTimeMs = m_stats.analysisTimeMs / m_stats.totalStatesAnalyzed;
    }

    return constraint;
}

std::vector<DeadZone> GraspConstraintGenerator::calculateDeadZones(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
) {
    std::vector<DeadZone> deadZones;

    // For flat state (no bends bent), no dead zones
    if (bentBends.empty()) {
        return deadZones;
    }

    // For each bent bend, create dead zone from standing flange
    for (int bentId : bentBends) {
        // Find the bend feature
        const phase1::BendFeature* bendPtr = nullptr;
        for (const auto& bend : bends) {
            if (bend.id == bentId) {
                bendPtr = &bend;
                break;
            }
        }

        if (bendPtr == nullptr) {
            continue;  // Bend not found
        }

        // Create dead zone
        DeadZone zone;
        zone.id = static_cast<int>(deadZones.size());
        zone.type = DeadZoneType::STANDING_FLANGE;
        zone.causedByBend = bentId;
        zone.safetyMargin = 5.0;  // 5mm safety margin

        // Project flange to base plane
        zone.polygon = projectFlangeToBasePlane(*bendPtr);

        deadZones.push_back(zone);
    }

    return deadZones;
}

Polygon2D GraspConstraintGenerator::projectFlangeToBasePlane(
    const phase1::BendFeature& bend
) {
    // Simplified projection: Create rectangle at bend line
    // Real implementation would use OCCT to project bent flange

    Polygon2D poly;

    double halfLength = bend.length / 2.0;
    double flangeWidth = 50.0;  // Assume 50mm flange width after bending

    // Create rectangle perpendicular to bend line
    // Centered at bend position
    poly.vertices.push_back(Point2D(
        bend.position.x - flangeWidth/2,
        bend.position.y - halfLength
    ));
    poly.vertices.push_back(Point2D(
        bend.position.x + flangeWidth/2,
        bend.position.y - halfLength
    ));
    poly.vertices.push_back(Point2D(
        bend.position.x + flangeWidth/2,
        bend.position.y + halfLength
    ));
    poly.vertices.push_back(Point2D(
        bend.position.x - flangeWidth/2,
        bend.position.y + halfLength
    ));

    return poly;
}

Polygon2D GraspConstraintGenerator::calculateValidRegion(
    const Point2D& sheetSize,
    const std::vector<DeadZone>& deadZones
) {
    // Simplified: Start with full sheet rectangle
    Polygon2D validRegion;

    // Full sheet (0,0) to (sheetSize.x, sheetSize.y)
    validRegion.vertices.push_back(Point2D(0, 0));
    validRegion.vertices.push_back(Point2D(sheetSize.x, 0));
    validRegion.vertices.push_back(Point2D(sheetSize.x, sheetSize.y));
    validRegion.vertices.push_back(Point2D(0, sheetSize.y));

    // Real implementation would subtract dead zone polygons
    // For now, just return full sheet if no dead zones
    if (deadZones.empty()) {
        return validRegion;
    }

    // Simplified subtraction: Shrink region if dead zones exist
    // Real implementation would use polygon Boolean operations
    double shrinkFactor = 0.8;  // Conservative shrink
    double centerX = sheetSize.x / 2.0;
    double centerY = sheetSize.y / 2.0;
    double newWidth = sheetSize.x * shrinkFactor;
    double newHeight = sheetSize.y * shrinkFactor;

    validRegion.vertices.clear();
    validRegion.vertices.push_back(Point2D(
        centerX - newWidth/2, centerY - newHeight/2
    ));
    validRegion.vertices.push_back(Point2D(
        centerX + newWidth/2, centerY - newHeight/2
    ));
    validRegion.vertices.push_back(Point2D(
        centerX + newWidth/2, centerY + newHeight/2
    ));
    validRegion.vertices.push_back(Point2D(
        centerX - newWidth/2, centerY + newHeight/2
    ));

    return validRegion;
}

Rectangle2D GraspConstraintGenerator::findMaxInscribedRect(
    const Polygon2D& validRegion
) {
    // Simplified MIR: Use bounding box of valid region
    // Real implementation would use rotating calipers or sweep line

    Rectangle2D mir;

    if (validRegion.vertices.empty()) {
        mir.bottomLeft = Point2D(0, 0);
        mir.topRight = Point2D(0, 0);
        mir.width = 0;
        mir.height = 0;
        mir.area = 0;
        return mir;
    }

    // Find bounding box
    double minX = validRegion.vertices[0].x;
    double maxX = validRegion.vertices[0].x;
    double minY = validRegion.vertices[0].y;
    double maxY = validRegion.vertices[0].y;

    for (const auto& v : validRegion.vertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.y > maxY) maxY = v.y;
    }

    // MIR is the bounding box (simplified)
    mir.bottomLeft = Point2D(minX, minY);
    mir.topRight = Point2D(maxX, maxY);
    mir.width = maxX - minX;
    mir.height = maxY - minY;
    mir.area = mir.width * mir.height;
    mir.center = Point2D((minX + maxX) / 2.0, (minY + maxY) / 2.0);

    return mir;
}

bool GraspConstraintGenerator::validateGripPhysics(
    const Rectangle2D& gripRect,
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
) {
    // Simplified physics validation

    // Criterion 1: Grip area must be > 100mm²
    double gripArea = gripRect.area;
    if (gripArea < 100.0) {
        return false;
    }

    // Criterion 2: Center of mass should be within grip region
    Point2D com = calculateCenterOfMass(bends, bentBends);
    Point2D gripCenter = gripRect.center;

    // Distance from COM to grip center
    double dx = com.x - gripCenter.x;
    double dy = com.y - gripCenter.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    // COM should be within 100mm of grip center (simplified)
    if (distance > 100.0) {
        return false;
    }

    // Physics valid
    return true;
}

Point2D GraspConstraintGenerator::calculateCenterOfMass(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
) {
    // Simplified COM calculation
    // Real implementation would calculate based on actual bent geometry

    if (bends.empty()) {
        return Point2D(0, 0);
    }

    // For flat state, COM is at sheet center
    if (bentBends.empty()) {
        return Point2D(250.0, 250.0);  // Center of 500x500 sheet
    }

    // For bent state, shift COM slightly based on bend positions
    double comX = 250.0;
    double comY = 250.0;

    // Average bend positions (simplified)
    for (int bentId : bentBends) {
        for (const auto& bend : bends) {
            if (bend.id == bentId) {
                comX = (comX + bend.position.x) / 2.0;
                comY = (comY + bend.position.y) / 2.0;
                break;
            }
        }
    }

    return Point2D(comX, comY);
}

} // namespace phase2
} // namespace openpanelcam
