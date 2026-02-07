#pragma once

#include "types.h"
#include "swept_volume.h"
#include "bent_state.h"
#include <vector>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Hierarchical collision detector
 *
 * Uses a two-level approach:
 * 1. AABB overlap test (fast rejection, O(1))
 * 2. OBB/SAT overlap test (tighter fit, if enabled)
 *
 * Checks swept volume of a new bend against all existing bent flanges.
 */
class CollisionDetector {
public:
    explicit CollisionDetector(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Check if a bend step causes collision
     * @param swept Swept volume of the bend being performed
     * @param bentState Current state of all previously bent flanges
     * @return CollisionResult with details
     */
    CollisionResult checkStep(const SweptVolume& swept,
                              const BentState& bentState) const;

    /**
     * @brief Check swept volume against a list of AABBs
     */
    CollisionResult checkAgainstVolumes(const SweptVolume& swept,
                                        const std::vector<AABB>& obstacles) const;

private:
    ValidatorConfig m_config;
};

} // namespace phase4
} // namespace openpanelcam
