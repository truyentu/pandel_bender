#include "openpanelcam/phase4/collision_detector.h"

namespace openpanelcam {
namespace phase4 {

CollisionDetector::CollisionDetector(const ValidatorConfig& config)
    : m_config(config) {}

CollisionResult CollisionDetector::checkStep(const SweptVolume& swept,
                                              const BentState& bentState) const {
    CollisionResult result;

    const auto& flanges = bentState.getBentFlanges();

    for (const auto& flange : flanges) {
        // Skip self-collision (same bend)
        if (flange.bendId == swept.bendId) continue;

        // Level 1: AABB fast rejection (with safety margin)
        AABB expandedSwept = swept.aabb.expand(m_config.collisionMargin);

        if (!expandedSwept.overlaps(flange.occupiedVolume)) {
            continue; // No overlap → no collision
        }

        // Level 2: OBB tighter check (if enabled)
        if (m_config.useOBB && m_config.collisionMargin == 0.0) {
            if (!swept.obb.overlaps(flange.occupiedOBB)) {
                continue; // OBB says no collision
            }
        } else if (m_config.useOBB && m_config.collisionMargin > 0.0) {
            // Expand OBB by margin before checking
            OBB expandedOBB = swept.obb;
            expandedOBB.halfExtentX += m_config.collisionMargin;
            expandedOBB.halfExtentY += m_config.collisionMargin;
            expandedOBB.halfExtentZ += m_config.collisionMargin;
            if (!expandedOBB.overlaps(flange.occupiedOBB)) {
                continue;
            }
        }

        // Collision detected
        result.hasCollision = true;
        result.type = CollisionType::SWEPT_VS_FIXED;
        result.bendId = swept.bendId;
        result.collidingBendId = flange.bendId;
        result.description = "Swept volume of bend " +
            std::to_string(swept.bendId) + " collides with bent flange " +
            std::to_string(flange.bendId);

        // Estimate penetration depth from AABB overlap
        double overlapX = std::min(expandedSwept.maxX, flange.occupiedVolume.maxX) -
                          std::max(expandedSwept.minX, flange.occupiedVolume.minX);
        double overlapY = std::min(expandedSwept.maxY, flange.occupiedVolume.maxY) -
                          std::max(expandedSwept.minY, flange.occupiedVolume.minY);
        double overlapZ = std::min(expandedSwept.maxZ, flange.occupiedVolume.maxZ) -
                          std::max(expandedSwept.minZ, flange.occupiedVolume.minZ);
        result.penetrationDepth = std::min({overlapX, overlapY, overlapZ});

        return result; // Return first collision found
    }

    return result; // No collision
}

CollisionResult CollisionDetector::checkAgainstVolumes(
    const SweptVolume& swept,
    const std::vector<AABB>& obstacles
) const {
    CollisionResult result;

    AABB expandedSwept = swept.aabb.expand(m_config.collisionMargin);

    for (size_t i = 0; i < obstacles.size(); i++) {
        if (expandedSwept.overlaps(obstacles[i])) {
            result.hasCollision = true;
            result.type = CollisionType::SWEPT_VS_FIXED;
            result.bendId = swept.bendId;
            result.collidingBendId = static_cast<int>(i);
            result.description = "Swept volume collides with obstacle " +
                std::to_string(i);
            return result;
        }
    }

    return result;
}

} // namespace phase4
} // namespace openpanelcam
