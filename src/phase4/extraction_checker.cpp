#include "openpanelcam/phase4/extraction_checker.h"
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

// ===== ExtractionChecker =====

ExtractionChecker::ExtractionChecker(const ValidatorConfig& config)
    : m_config(config) {}

ExtractionResult ExtractionChecker::check(
    const phase1::BendFeature& bend,
    const BentState& stateAfter
) const {
    ExtractionResult result;

    // Estimate tool position for this bend
    AABB toolAABB = estimateToolAABB(bend);

    // Get all occupied volumes (excluding the bend we just did)
    std::vector<AABB> obstacles;
    for (const auto& flange : stateAfter.getBentFlanges()) {
        if (flange.bendId == bend.id) continue; // Skip the current bend
        obstacles.push_back(flange.occupiedVolume);
    }

    // Check if tool can extract vertically
    if (!canExtractVertically(toolAABB, obstacles)) {
        result.canExtract = false;
        result.trappedAtBend = bend.id;
        result.description = "Tool trapped at bend " +
            std::to_string(bend.id) +
            " - vertical extraction blocked by bent flanges";
    }

    return result;
}

bool ExtractionChecker::canExtractVertically(
    const AABB& toolPosition,
    const std::vector<AABB>& obstacles
) const {
    // The tool extracts by moving upward along Z-axis
    // Check if any obstacle blocks the path above the tool
    //
    // An obstacle blocks extraction if:
    // 1. It overlaps with the tool in X and Y
    // 2. Its Z range is above the tool's top
    //
    // We project the tool's XY footprint upward and check for blockers

    for (const auto& obs : obstacles) {
        // Check XY overlap (tool footprint vs obstacle footprint)
        bool xOverlap = toolPosition.minX < obs.maxX &&
                        toolPosition.maxX > obs.minX;
        bool yOverlap = toolPosition.minY < obs.maxY &&
                        toolPosition.maxY > obs.minY;

        if (!xOverlap || !yOverlap) continue;

        // Check if obstacle is above the tool (blocks vertical extraction)
        // The obstacle blocks if its bottom is above tool's bottom
        // and it extends into the extraction path
        if (obs.maxZ > toolPosition.maxZ &&
            obs.minZ < toolPosition.maxZ + 500.0) {
            // Obstacle is in the extraction path above the tool
            return false;
        }
    }

    return true;
}

AABB ExtractionChecker::estimateToolAABB(
    const phase1::BendFeature& bend
) const {
    // Tool (punch) position is centered at the bend line
    // Tool width ≈ bend length
    // Tool depth ≈ 30mm (typical V-die width)
    // Tool height ≈ 50mm

    double px = bend.position.x;
    double py = bend.position.y;
    double pz = bend.position.z;

    double halfLen = bend.length / 2.0;
    double toolDepth = 15.0; // half of 30mm
    double toolHeight = 25.0; // half of 50mm

    // Tool extends along bend direction
    double dx = bend.direction.x;
    double dy = bend.direction.y;
    double dz = bend.direction.z;

    double extX = std::abs(dx) * halfLen + toolDepth;
    double extY = std::abs(dy) * halfLen + toolDepth;
    double extZ = std::abs(dz) * halfLen + toolHeight;

    extX = std::max(extX, 10.0);
    extY = std::max(extY, 10.0);
    extZ = std::max(extZ, 10.0);

    return AABB(px - extX, py - extY, pz - extZ,
                px + extX, py + extY, pz + extZ);
}

// ===== SpringbackCompensator =====

SpringbackCompensator::SpringbackCompensator(const ValidatorConfig& config)
    : m_config(config) {}

SpringbackData SpringbackCompensator::compensate(
    const phase1::BendFeature& bend
) const {
    SpringbackData data;
    data.bendId = bend.id;
    data.targetAngle = bend.angle;

    if (!m_config.enableSpringback) {
        data.compensatedAngle = bend.angle;
        data.springbackAngle = 0.0;
        return data;
    }

    // Springback model:
    // springback = baseDeg + perMm * thickness
    // The springback is proportional to the bend angle
    // (larger angles have proportionally more springback)
    double baseSpringback = m_config.springbackBaseDeg +
                            m_config.springbackPerMm * m_config.materialThickness;

    // Scale springback with angle (relative to 90°)
    double angleFactor = std::abs(bend.angle) / 90.0;
    data.springbackAngle = baseSpringback * angleFactor;

    // Machine must overbend to compensate
    // If target is 90° and springback is 2.75°, machine bends to 92.75°
    double sign = (bend.angle >= 0) ? 1.0 : -1.0;
    data.compensatedAngle = bend.angle + sign * data.springbackAngle;

    return data;
}

std::vector<SpringbackData> SpringbackCompensator::compensateAll(
    const std::vector<phase1::BendFeature>& bends
) const {
    std::vector<SpringbackData> results;
    results.reserve(bends.size());
    for (const auto& bend : bends) {
        results.push_back(compensate(bend));
    }
    return results;
}

} // namespace phase4
} // namespace openpanelcam
