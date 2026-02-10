#pragma once

#include "types.h"
#include "swept_volume.h"
#include "bent_state.h"
#include "../phase2/phase1_mock.h"
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

    /**
     * @brief Check angular interval AABBs against bent flanges
     *
     * Paper approach: check I+1 intermediate configurations for finer
     * collision detection during the sweep trajectory.
     *
     * @param swept Swept volume with intervalAABBs populated
     * @param bentState Current bent state
     * @return CollisionResult (first interval collision found)
     */
    CollisionResult checkIntervals(const SweptVolume& swept,
                                   const BentState& bentState) const;

    /**
     * @brief Dual-state collision check (paper: fold + unfold)
     *
     * Paper pseudocode (Prasanth & Shunmugam):
     *   STEP 1: check for collision in the folded state
     *   STEP 2: unfold, then check for collision in the unfolded state
     *
     * @param bend The bend being performed
     * @param foldedState State with this bend folded (post-bend)
     * @param unfoldedState State without this bend folded (pre-bend)
     * @return CollisionResult from whichever state detects collision first
     */
    CollisionResult checkDualState(const phase1::BendFeature& bend,
                                   const BentState& foldedState,
                                   const BentState& unfoldedState) const;

private:
    ValidatorConfig m_config;
};

} // namespace phase4
} // namespace openpanelcam
