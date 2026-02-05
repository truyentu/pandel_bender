#include "openpanelcam/phase2/geometric_precedence_analyzer.h"
#include <algorithm>
#include <cmath>

namespace openpanelcam {
namespace phase2 {

//==============================================================================
// BentState Implementation
//==============================================================================

BentState::BentState() {
}

void BentState::applyBend(int bendId) {
    // Check if already bent
    auto it = std::find(m_bentBends.begin(), m_bentBends.end(), bendId);
    if (it == m_bentBends.end()) {
        m_bentBends.push_back(bendId);
    }
}

bool BentState::isBent(int bendId) const {
    return std::find(m_bentBends.begin(), m_bentBends.end(), bendId) != m_bentBends.end();
}

void BentState::reset() {
    m_bentBends.clear();
}

//==============================================================================
// GeometricPrecedenceAnalyzer Implementation
//==============================================================================

GeometricPrecedenceAnalyzer::GeometricPrecedenceAnalyzer() {
    m_stats = Statistics();
}

std::vector<PrecedenceEdge> GeometricPrecedenceAnalyzer::analyze(
    const std::vector<phase1::BendFeature>& bends
) {
    std::vector<PrecedenceEdge> constraints;

    if (bends.empty()) {
        return constraints;
    }

    // Reset statistics
    m_stats = Statistics();

    // For each pair of bends (bi, bj)
    for (size_t i = 0; i < bends.size(); i++) {
        for (size_t j = 0; j < bends.size(); j++) {
            if (i == j) continue;  // Skip self

            m_stats.totalPairsChecked++;

            const auto& bi = bends[i];
            const auto& bj = bends[j];

            // Create bent state for testing
            BentState state;

            // Type 1: Check corner overlap
            // If bi bending would cause corners to overlap bj's flange
            // then bi must bend before bj
            if (checkCornerOverlap(bi, bj, state)) {
                PrecedenceEdge edge;
                edge.id = static_cast<int>(constraints.size());
                edge.fromBend = bi.id;
                edge.toBend = bj.id;
                edge.type = ConstraintType::GEOMETRIC;
                edge.confidence = 1.0;
                edge.reasoning = "Corner overlap detected";

                constraints.push_back(edge);
                m_stats.cornerOverlapCount++;
                m_stats.totalConstraints++;
            }

            // Type 2: Check if bending would close a box
            // Note: Box closing is checked per bend, not per pair
            // We'll handle this in a separate loop
        }
    }

    // Type 2: Box closing check (per bend)
    for (const auto& bend : bends) {
        // Create state with all other bends
        BentState state;
        for (const auto& otherBend : bends) {
            if (otherBend.id != bend.id) {
                state.applyBend(otherBend.id);
            }
        }

        if (isBoxClosing(bend, state)) {
            // This bend would close a box - mark as constraint
            // (In real implementation, this would create multiple constraints)
            m_stats.boxClosingCount++;
        }
    }

    // Type 3: Sequential blocking
    // Already partially covered in corner overlap check

    return constraints;
}

bool GeometricPrecedenceAnalyzer::checkCornerOverlap(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
) {
    // Simplified corner overlap detection
    // Real implementation would:
    // 1. Simulate bending bi to its final angle
    // 2. Get 4 corner points of bi's flange in bent state
    // 3. For each corner, cast ray along motion path
    // 4. Check if ray intersects bj's flange

    // For now, implement simplified version with mock geometry

    // Get corners of bi's flange (in flat state initially)
    std::vector<Point3D> corners = getFlangeCorners(bi);

    // For each corner, check if bending would cause overlap
    for (const auto& corner : corners) {
        // Predict motion path during bending
        Point3D motionDir = predictMotionPath(bi, corner);

        // Cast ray and check intersection with bj's flange
        if (rayIntersectsFlange(corner, motionDir, bj)) {
            // Overlap detected!
            return true;
        }
    }

    // No overlap found
    return false;
}

std::vector<Point3D> GeometricPrecedenceAnalyzer::getFlangeCorners(
    const phase1::BendFeature& bend
) {
    std::vector<Point3D> corners;

    // Simplified: Generate 4 corners of rectangular flange
    // based on bend line position, direction, and length

    double halfLength = bend.length / 2.0;

    // Bend line along Y-axis (default)
    Point3D center(bend.position.x, bend.position.y, bend.position.z);

    // Assume flange extends 50mm in X direction
    double flangeWidth = 50.0;

    // Corner 1: (-width/2, -length/2)
    corners.push_back(Point3D(
        center.x - flangeWidth/2,
        center.y - halfLength,
        center.z
    ));

    // Corner 2: (+width/2, -length/2)
    corners.push_back(Point3D(
        center.x + flangeWidth/2,
        center.y - halfLength,
        center.z
    ));

    // Corner 3: (+width/2, +length/2)
    corners.push_back(Point3D(
        center.x + flangeWidth/2,
        center.y + halfLength,
        center.z
    ));

    // Corner 4: (-width/2, +length/2)
    corners.push_back(Point3D(
        center.x - flangeWidth/2,
        center.y + halfLength,
        center.z
    ));

    return corners;
}

Point3D GeometricPrecedenceAnalyzer::predictMotionPath(
    const phase1::BendFeature& bend,
    const Point3D& point
) {
    // Simplified: Motion is circular arc around bend line
    // Direction depends on bend angle and normal

    // For positive angle, motion is in direction of normal
    if (bend.angle > 0) {
        return Point3D(bend.normal.x, bend.normal.y, bend.normal.z);
    } else {
        return Point3D(-bend.normal.x, -bend.normal.y, -bend.normal.z);
    }
}

bool GeometricPrecedenceAnalyzer::rayIntersectsFlange(
    const Point3D& rayOrigin,
    const Point3D& rayDirection,
    const phase1::BendFeature& flangeBend
) {
    // Simplified ray-flange intersection test
    // Real implementation would use BRepIntCurveSurface_Inter from OCCT

    // For simplified test: Check if ray origin is near the flange region
    // Flange region is defined as rectangle around bend line

    double flangeHalfWidth = 25.0;  // 50mm / 2
    double flangeHalfLength = flangeBend.length / 2.0;

    // Calculate distance from ray origin to flange center
    double distX = std::abs(rayOrigin.x - flangeBend.position.x);
    double distY = std::abs(rayOrigin.y - flangeBend.position.y);

    // Check if ray origin is within flange bounding box (with tolerance)
    double tolerance = 5.0;  // 5mm extra tolerance

    if (distX < (flangeHalfWidth + tolerance) &&
        distY < (flangeHalfLength + tolerance)) {
        return true;
    }

    // Also check if flange corners are near ray origin
    // (reverse check - does flange overlap with ray start point)
    std::vector<Point3D> flangeCorners = getFlangeCorners(flangeBend);
    for (const auto& corner : flangeCorners) {
        double dx = std::abs(corner.x - rayOrigin.x);
        double dy = std::abs(corner.y - rayOrigin.y);

        if (dx < tolerance && dy < tolerance) {
            return true;
        }
    }

    return false;
}

bool GeometricPrecedenceAnalyzer::isBoxClosing(
    const phase1::BendFeature& bend,
    const BentState& state
) {
    // Simplified implementation
    // Real implementation would:
    // 1. Project all bent flanges to 2D
    // 2. Check if they form 3 sides of a box
    // 3. Check if next bend would close 4th side

    // For now, return false (no box closing)
    return false;
}

bool GeometricPrecedenceAnalyzer::isBlocked(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
) {
    // Simplified implementation
    // Real implementation would:
    // 1. Simulate bending bj
    // 2. Check if bi's bend line is still accessible

    // For now, return false (not blocked)
    return false;
}

} // namespace phase2
} // namespace openpanelcam
