#include "openpanelcam/phase2/grasp_constraint_generator.h"
#include <chrono>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

        // Expand by safety margin
        zone.polygon = expandPolygon(zone.polygon, zone.safetyMargin);

        deadZones.push_back(zone);
    }

    return deadZones;
}

Polygon2D GraspConstraintGenerator::projectFlangeToBasePlane(
    const phase1::BendFeature& bend
) {
    // Project standing flange to base plane (Z=0)
    // Footprint depends on bend angle

    Polygon2D poly;

    double halfLength = bend.length / 2.0;

    // Flange height depends on bend angle and original flange width
    // Assuming original flange width = 50mm
    double originalFlangeWidth = 50.0;

    // For 90° bend: full width projects to base
    // For 45° bend: sin(45°) * width projects
    // For 0° bend: no projection (flat)
    double angleRad = bend.angle * M_PI / 180.0;
    double projectedWidth = originalFlangeWidth * std::sin(angleRad);

    // Minimum footprint even for small angles (safety)
    if (projectedWidth < 10.0 && bend.angle > 5.0) {
        projectedWidth = 10.0;
    }

    // Create rectangle perpendicular to bend line
    // Centered at bend position

    // Determine orientation based on bend direction
    // If bend line is along Y (direction.y ≈ 1), flange extends in X
    // If bend line is along X (direction.x ≈ 1), flange extends in Y

    double dirX = bend.direction.x;
    double dirY = bend.direction.y;
    double perpX = -dirY;  // Perpendicular direction
    double perpY = dirX;

    // Normalize perpendicular vector
    double perpLen = std::sqrt(perpX*perpX + perpY*perpY);
    if (perpLen > 0.001) {
        perpX /= perpLen;
        perpY /= perpLen;
    } else {
        // Default to X direction if bend direction is degenerate
        perpX = 1.0;
        perpY = 0.0;
    }

    // Direction along bend line
    double tanX = dirX;
    double tanY = dirY;
    double tanLen = std::sqrt(tanX*tanX + tanY*tanY);
    if (tanLen > 0.001) {
        tanX /= tanLen;
        tanY /= tanLen;
    } else {
        // Default to Y direction
        tanX = 0.0;
        tanY = 1.0;
    }

    // Four corners of flange footprint
    double hw = projectedWidth / 2.0;  // Half width
    double hl = halfLength;             // Half length

    // Corner 1: -width/2, -length/2
    poly.vertices.push_back(Point2D(
        bend.position.x - perpX * hw - tanX * hl,
        bend.position.y - perpY * hw - tanY * hl
    ));

    // Corner 2: +width/2, -length/2
    poly.vertices.push_back(Point2D(
        bend.position.x + perpX * hw - tanX * hl,
        bend.position.y + perpY * hw - tanY * hl
    ));

    // Corner 3: +width/2, +length/2
    poly.vertices.push_back(Point2D(
        bend.position.x + perpX * hw + tanX * hl,
        bend.position.y + perpY * hw + tanY * hl
    ));

    // Corner 4: -width/2, +length/2
    poly.vertices.push_back(Point2D(
        bend.position.x - perpX * hw + tanX * hl,
        bend.position.y - perpY * hw + tanY * hl
    ));

    return poly;
}

Polygon2D GraspConstraintGenerator::calculateValidRegion(
    const Point2D& sheetSize,
    const std::vector<DeadZone>& deadZones
) {
    // Start with full sheet rectangle
    Polygon2D validRegion;

    // Full sheet (0,0) to (sheetSize.x, sheetSize.y)
    validRegion.vertices.push_back(Point2D(0, 0));
    validRegion.vertices.push_back(Point2D(sheetSize.x, 0));
    validRegion.vertices.push_back(Point2D(sheetSize.x, sheetSize.y));
    validRegion.vertices.push_back(Point2D(0, sheetSize.y));

    // If no dead zones, return full sheet
    if (deadZones.empty()) {
        return validRegion;
    }

    // Simplified approach: Calculate total dead zone footprint
    // and shrink valid region accordingly

    // Find union bounding box of all dead zones
    bool firstZone = true;
    double dzMinX = 0, dzMaxX = 0, dzMinY = 0, dzMaxY = 0;

    for (const auto& dz : deadZones) {
        if (dz.polygon.vertices.empty()) continue;

        // Get bounding box of this dead zone
        double minX = dz.polygon.vertices[0].x;
        double maxX = dz.polygon.vertices[0].x;
        double minY = dz.polygon.vertices[0].y;
        double maxY = dz.polygon.vertices[0].y;

        for (const auto& v : dz.polygon.vertices) {
            if (v.x < minX) minX = v.x;
            if (v.x > maxX) maxX = v.x;
            if (v.y < minY) minY = v.y;
            if (v.y > maxY) maxY = v.y;
        }

        if (firstZone) {
            dzMinX = minX;
            dzMaxX = maxX;
            dzMinY = minY;
            dzMaxY = maxY;
            firstZone = false;
        } else {
            // Expand union bounding box
            if (minX < dzMinX) dzMinX = minX;
            if (maxX > dzMaxX) dzMaxX = maxX;
            if (minY < dzMinY) dzMinY = minY;
            if (maxY > dzMaxY) dzMaxY = maxY;
        }
    }

    // Calculate shrink factor based on dead zone coverage
    double dzWidth = dzMaxX - dzMinX;
    double dzHeight = dzMaxY - dzMinY;
    double dzArea = dzWidth * dzHeight * deadZones.size() * 0.5;  // Approximate

    double sheetArea = sheetSize.x * sheetSize.y;
    double coverage = dzArea / sheetArea;

    // Limit coverage to avoid invalid regions
    if (coverage > 0.8) coverage = 0.8;

    // Shrink factor increases with more dead zones
    double shrinkFactor = 1.0 - coverage;

    // Ensure minimum valid region
    if (shrinkFactor < 0.4) shrinkFactor = 0.4;

    // Create shrunk valid region centered on sheet
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
    // Maximum Inscribed Rectangle (MIR) Algorithm
    // ============================================
    //
    // Goal: Find largest axis-aligned rectangle that fits inside valid region
    //
    // Current implementation: Bounding box with inset
    // Rationale:
    //   - Valid region is always axis-aligned rectangle (from calculateValidRegion)
    //   - For axis-aligned rectangular regions, bounding box IS the MIR
    //   - Inset ensures strict containment (handles floating point precision)
    //   - Complexity: O(n) where n = number of vertices
    //
    // Future enhancement for arbitrary polygons:
    //   - Rotating calipers algorithm for convex polygons
    //   - Grid-based sampling for concave polygons
    //   - Dynamic programming approaches
    //
    // References:
    //   - "Largest Empty Rectangle" problem
    //   - Computational Geometry: Algorithms and Applications (de Berg et al.)

    Rectangle2D mir;

    if (validRegion.vertices.empty()) {
        mir.bottomLeft = Point2D(0, 0);
        mir.topRight = Point2D(0, 0);
        mir.width = 0;
        mir.height = 0;
        mir.area = 0;
        return mir;
    }

    // Step 1: Find axis-aligned bounding box
    // This is optimal for axis-aligned rectangular regions
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

    // Step 2: Inset by small margin to ensure strict containment
    // This handles floating point precision issues with polygon.contains()
    // For a rectangle, points exactly on the boundary may fail containment test
    double inset = 0.1;  // 0.1mm inset (negligible for practical purposes)
    minX += inset;
    maxX -= inset;
    minY += inset;
    maxY -= inset;

    // Step 3: Validate dimensions
    if (maxX <= minX || maxY <= minY) {
        // Degenerate case - valid region too small for any rectangle
        mir.bottomLeft = Point2D(0, 0);
        mir.topRight = Point2D(0, 0);
        mir.width = 0;
        mir.height = 0;
        mir.area = 0;
        return mir;
    }

    // Step 4: Construct MIR
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
    // Grip physics validation
    // =======================
    //
    // Validates that the proposed grip rectangle is physically feasible
    // for holding the bent part during subsequent bending operations.
    //
    // Validation criteria:
    // 1. Minimum grip area (friction requirement)
    // 2. Center of mass proximity (stability requirement)
    // 3. Grip aspect ratio (practicality requirement)
    //
    // Physics assumptions:
    // - Friction coefficient μ ≈ 0.3 (steel on rubber pads)
    // - Static friction force F = μ × N (normal force from grip)
    // - Minimum area ensures sufficient normal force
    // - COM proximity ensures low torque/moment

    // Criterion 1: Minimum grip area
    // Required for sufficient friction to hold part
    // Minimum 100mm² based on typical gripper specifications
    double gripArea = gripRect.area;
    const double MIN_GRIP_AREA = 100.0;  // mm²

    if (gripArea < MIN_GRIP_AREA) {
        return false;  // Insufficient friction
    }

    // Criterion 2: Center of mass proximity
    // COM should be reasonably close to grip center
    // If COM is far from grip, part may tilt or slip
    Point2D com = calculateCenterOfMass(bends, bentBends);
    Point2D gripCenter = gripRect.center;

    double dx = com.x - gripCenter.x;
    double dy = com.y - gripCenter.y;
    double comDistance = std::sqrt(dx*dx + dy*dy);

    // Maximum allowable COM offset: 100mm
    // Beyond this, moment arm too large → instability risk
    const double MAX_COM_OFFSET = 100.0;  // mm

    if (comDistance > MAX_COM_OFFSET) {
        return false;  // Unstable grip
    }

    // Criterion 3: Grip aspect ratio
    // Extremely elongated grips (very high aspect ratio) are impractical
    // Example: 200mm × 1mm grip is difficult to achieve with standard tooling
    double aspectRatio = std::max(gripRect.width, gripRect.height) /
                         std::min(gripRect.width, gripRect.height);

    const double MAX_ASPECT_RATIO = 10.0;  // Width/Height ratio

    if (aspectRatio > MAX_ASPECT_RATIO) {
        return false;  // Impractical grip geometry
    }

    // All criteria satisfied
    return true;
}

Point2D GraspConstraintGenerator::calculateCenterOfMass(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
) {
    // Center of Mass calculation for bent sheet metal part
    // ====================================================
    //
    // Assumptions:
    // - Uniform material thickness and density
    // - Sheet metal properties: constant thickness t
    // - Mass is proportional to area (since m = ρ × t × A)
    // - COM is area-weighted centroid of all sections
    //
    // Simplified approach:
    // - Flat state: COM at geometric center of sheet
    // - Bent state: COM shifts based on bent flange positions
    // - Weight each bend position by its flange area
    //
    // Real implementation would:
    // - Calculate actual 3D flange positions after bending
    // - Project to base plane
    // - Compute area-weighted centroid
    // - Account for material thickness effects

    const double SHEET_SIZE = 500.0;  // mm (assumed square sheet)

    if (bends.empty()) {
        // No bends defined - use sheet center
        return Point2D(SHEET_SIZE / 2.0, SHEET_SIZE / 2.0);
    }

    if (bentBends.empty()) {
        // Flat state - COM at sheet geometric center
        return Point2D(SHEET_SIZE / 2.0, SHEET_SIZE / 2.0);
    }

    // Bent state - calculate weighted centroid
    // Start with base sheet contribution
    double totalArea = SHEET_SIZE * SHEET_SIZE;
    double weightedX = (SHEET_SIZE / 2.0) * totalArea;
    double weightedY = (SHEET_SIZE / 2.0) * totalArea;

    // Add contribution from each bent flange
    // Each flange adds mass at its position
    const double FLANGE_WIDTH = 50.0;  // mm (assumed)

    for (int bentId : bentBends) {
        // Find the bend feature
        const phase1::BendFeature* bendPtr = nullptr;
        for (const auto& bend : bends) {
            if (bend.id == bentId) {
                bendPtr = &bend;
                break;
            }
        }

        if (bendPtr == nullptr) continue;

        // Flange area
        double flangeArea = FLANGE_WIDTH * bendPtr->length;

        // Flange centroid (at bend line position)
        double flangeX = bendPtr->position.x;
        double flangeY = bendPtr->position.y;

        // Add weighted contribution
        weightedX += flangeX * flangeArea;
        weightedY += flangeY * flangeArea;
        totalArea += flangeArea;
    }

    // Calculate area-weighted centroid
    double comX = weightedX / totalArea;
    double comY = weightedY / totalArea;

    return Point2D(comX, comY);
}

Polygon2D GraspConstraintGenerator::expandPolygon(
    const Polygon2D& poly,
    double margin
) {
    // Simplified polygon expansion using bounding box method
    // Real implementation would use polygon offsetting algorithms

    if (poly.vertices.empty()) {
        return poly;
    }

    // Find bounding box
    double minX = poly.vertices[0].x;
    double maxX = poly.vertices[0].x;
    double minY = poly.vertices[0].y;
    double maxY = poly.vertices[0].y;

    for (const auto& v : poly.vertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.y < minY) minY = v.y;
        if (v.y > maxY) maxY = v.y;
    }

    // Expand bounding box by margin
    minX -= margin;
    maxX += margin;
    minY -= margin;
    maxY += margin;

    // Create expanded rectangle
    Polygon2D expanded;
    expanded.vertices.push_back(Point2D(minX, minY));
    expanded.vertices.push_back(Point2D(maxX, minY));
    expanded.vertices.push_back(Point2D(maxX, maxY));
    expanded.vertices.push_back(Point2D(minX, maxY));

    return expanded;
}

} // namespace phase2
} // namespace openpanelcam
