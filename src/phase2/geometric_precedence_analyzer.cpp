#include "openpanelcam/phase2/geometric_precedence_analyzer.h"
#include <algorithm>
#include <cmath>
#include <chrono>

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

    // Start timing (Task 15: Performance metrics)
    auto startTime = std::chrono::high_resolution_clock::now();

    // Reset statistics
    m_stats = Statistics();

    // For each pair of bends (bi, bj)
    for (size_t i = 0; i < bends.size(); i++) {
        for (size_t j = 0; j < bends.size(); j++) {
            if (i == j) continue;  // Skip self

            m_stats.totalPairsChecked++;

            const auto& bi = bends[i];
            const auto& bj = bends[j];

            // Track constraints per pair
            int constraintsBeforePair = static_cast<int>(constraints.size());

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

            // Type 3: Sequential blocking
            // If bj bending first would block access to bi
            // then bi must bend before bj
            BentState blockingState;
            blockingState.applyBend(bj.id);  // Simulate bj already bent

            if (isBlocked(bi, bj, blockingState)) {
                PrecedenceEdge edge;
                edge.id = static_cast<int>(constraints.size());
                edge.fromBend = bi.id;
                edge.toBend = bj.id;
                edge.type = ConstraintType::SEQUENTIAL;
                edge.confidence = 0.9;  // Slightly less confident than geometric
                edge.reasoning = "Sequential blocking - access obstructed";

                constraints.push_back(edge);
                m_stats.sequentialBlockCount++;
                m_stats.totalConstraints++;
            }

            // Track max constraints per pair (Task 15)
            int constraintsFromPair = static_cast<int>(constraints.size()) - constraintsBeforePair;
            if (constraintsFromPair > m_stats.maxConstraintsPerPair) {
                m_stats.maxConstraintsPerPair = constraintsFromPair;
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

    // End timing and calculate metrics (Task 15)
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    m_stats.analysisTimeMs = duration.count() / 1000.0;  // Convert to milliseconds

    if (m_stats.totalPairsChecked > 0) {
        m_stats.avgPairTimeMs = m_stats.analysisTimeMs / m_stats.totalPairsChecked;
    }

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
    // Box closing detection algorithm:
    // 1. Check if we have at least 3 bends already bent
    // 2. Check if those 3 bends form 3 sides of a box (U-shape)
    // 3. Check if the next bend would close the 4th side

    const auto& bentBends = state.getBentBends();

    // Need at least 3 bends bent to form 3-sided box
    if (bentBends.size() < 3) {
        return false;
    }

    // For simplified implementation, we'll use heuristic:
    // If exactly 3 bends are bent at 90°, and we're trying to bend a 4th at 90°,
    // this could potentially close a box

    // Count how many 90° bends are already bent
    // (In real implementation, would check actual spatial arrangement)

    // Simplified check: If 3+ bends bent, might form box
    // Real implementation would:
    // - Project flanges to 2D
    // - Check if they form 3 perpendicular walls
    // - Check if next bend aligns with gap

    // For now, use conservative approach:
    // Only flag as box closing if we have exactly 3 bends at 90°
    // and trying to add 4th at 90°

    if (bentBends.size() == 3 && std::abs(bend.angle - 90.0) < 1.0) {
        // Potential box closing scenario
        // In full implementation, would do geometric analysis
        return false;  // Conservative: don't flag yet without full geometry
    }

    return false;
}

bool GeometricPrecedenceAnalyzer::forms3SidedBox(
    const std::vector<int>& bentBends,
    const std::vector<phase1::BendFeature>& allBends
) {
    // Check if bent bends form 3 sides of a rectangular box
    // Simplified heuristic approach

    if (bentBends.size() != 3) {
        return false;  // Need exactly 3 bends for 3-sided box
    }

    // Find the 3 bent bend features
    std::vector<phase1::BendFeature> bentFeatures;
    for (int bentId : bentBends) {
        for (const auto& bend : allBends) {
            if (bend.id == bentId) {
                bentFeatures.push_back(bend);
                break;
            }
        }
    }

    if (bentFeatures.size() != 3) {
        return false;
    }

    // Simplified check: All 3 bends should be at approximately 90°
    for (const auto& bend : bentFeatures) {
        if (std::abs(bend.angle - 90.0) > 5.0) {
            return false;  // Not a 90° bend
        }
    }

    // If all 3 are at 90°, they could form 3 sides
    // Real implementation would check spatial arrangement:
    // - Are they perpendicular to each other?
    // - Do they share edges?
    // - Do they form a U-shape when projected to 2D?

    return true;  // Simplified: assume they form 3-sided box
}

Polygon2D GeometricPrecedenceAnalyzer::projectTo2D(
    const phase1::BendFeature& bend
) {
    // Project flange to 2D base plane (Z=0)
    // Simplified: Return rectangular projection

    Polygon2D poly;

    double halfLength = bend.length / 2.0;
    double flangeWidth = 50.0;  // Assume 50mm flange width

    // Create rectangle around bend position
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

bool GeometricPrecedenceAnalyzer::wouldClose4thSide(
    const phase1::BendFeature& nextBend,
    const std::vector<int>& bentBends,
    const std::vector<phase1::BendFeature>& allBends
) {
    // Check if next bend would complete the 4th side of box
    // Simplified implementation

    if (bentBends.size() != 3) {
        return false;
    }

    // Check if next bend is also at 90°
    if (std::abs(nextBend.angle - 90.0) > 5.0) {
        return false;
    }

    // In real implementation, would:
    // 1. Get 2D projections of 3 bent flanges
    // 2. Identify the gap (missing 4th side)
    // 3. Check if nextBend's projection fills that gap

    // Simplified: If we have 3 sides and trying to add 4th at 90°,
    // assume it would close box
    return true;
}

bool GeometricPrecedenceAnalyzer::isBlocked(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
) {
    // Sequential blocking detection:
    // Check if bending bj first would block access to bi's bend line

    // Algorithm:
    // 1. Simulate bending bj (assume it's in bent state)
    // 2. Check if bj's bent flange would obstruct tool access to bi
    // 3. Tool approaches from above (Z direction) to reach bend line

    // Simplified implementation:
    // - Check proximity of bend lines
    // - If bends are close and parallel, one might block the other

    // Calculate distance between bend lines
    double dx = bi.position.x - bj.position.x;
    double dy = bi.position.y - bj.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    // Flange width after bending (simplified: 50mm)
    double flangeWidth = 50.0;

    // If bends are very close (within flange width), might block
    if (distance < flangeWidth) {
        // Check if they're parallel (same direction)
        double dotProduct = bi.direction.x * bj.direction.x +
                           bi.direction.y * bj.direction.y;

        // If parallel (dot product close to 1 or -1), blocking likely
        if (std::abs(dotProduct) > 0.9) {
            return true;  // Parallel and close → blocking
        }
    }

    // Check if bj's flange would physically block bi's bend line
    // Simplified: if bj position overlaps with bi's access zone
    double accessZoneRadius = 60.0;  // Tool needs ~60mm clearance

    if (distance < accessZoneRadius) {
        // Close enough that flange might interfere
        // Real implementation would check actual flange geometry
        return true;
    }

    return false;
}

} // namespace phase2
} // namespace openpanelcam
