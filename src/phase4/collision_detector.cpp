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

CollisionResult CollisionDetector::checkIntervals(
    const SweptVolume& swept,
    const BentState& bentState) const
{
    CollisionResult result;

    if (swept.intervalAABBs.empty()) {
        return result;  // No intervals to check
    }

    const auto& flanges = bentState.getBentFlanges();

    for (size_t i = 0; i < swept.intervalAABBs.size(); i++) {
        AABB intervalBox = swept.intervalAABBs[i].expand(m_config.collisionMargin);

        for (const auto& flange : flanges) {
            if (flange.bendId == swept.bendId) continue;

            if (intervalBox.overlaps(flange.occupiedVolume)) {
                result.hasCollision = true;
                result.type = CollisionType::SWEPT_VS_FIXED;
                result.bendId = swept.bendId;
                result.collidingBendId = flange.bendId;
                result.description = "Swept interval " +
                    std::to_string(i) + "/" +
                    std::to_string(swept.intervalAABBs.size()) +
                    " of bend " + std::to_string(swept.bendId) +
                    " collides with bent flange " +
                    std::to_string(flange.bendId);

                // Penetration depth from interval AABB
                double overlapX = std::min(intervalBox.maxX, flange.occupiedVolume.maxX) -
                                  std::max(intervalBox.minX, flange.occupiedVolume.minX);
                double overlapY = std::min(intervalBox.maxY, flange.occupiedVolume.maxY) -
                                  std::max(intervalBox.minY, flange.occupiedVolume.minY);
                double overlapZ = std::min(intervalBox.maxZ, flange.occupiedVolume.maxZ) -
                                  std::max(intervalBox.minZ, flange.occupiedVolume.minZ);
                result.penetrationDepth = std::min({overlapX, overlapY, overlapZ});

                return result;
            }
        }
    }

    return result;
}

CollisionResult CollisionDetector::checkDualState(
    const phase1::BendFeature& bend,
    const BentState& foldedState,
    const BentState& unfoldedState) const
{
    SweptVolumeGenerator gen;
    SweptVolume swept = gen.generate(bend);

    // STEP 1: Check collision in FOLDED state (post-bend)
    CollisionResult foldedResult = checkStep(swept, foldedState);
    if (foldedResult.hasCollision) {
        foldedResult.description = "[Folded] " + foldedResult.description;
        return foldedResult;
    }

    // STEP 2: Check collision in UNFOLDED state (pre-bend)
    CollisionResult unfoldedResult = checkStep(swept, unfoldedState);
    if (unfoldedResult.hasCollision) {
        unfoldedResult.description = "[Unfolded] " + unfoldedResult.description;
        return unfoldedResult;
    }

    // No collision in either state
    return CollisionResult();
}

} // namespace phase4
} // namespace openpanelcam
