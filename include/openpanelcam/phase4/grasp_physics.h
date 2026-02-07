#pragma once

#include "types.h"
#include "bent_state.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <cmath>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Engine for validating grasp physics during bending
 *
 * Checks:
 * - Grip area: enough vacuum cup contact area remains
 * - Center of mass: part COM stays close to grip center
 * - Torque: bending forces don't exceed grip torque capacity
 * - Shear: lateral forces stay within friction limits
 */
class GraspPhysicsEngine {
public:
    explicit GraspPhysicsEngine(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Full grasp validation for a bend step
     * @param state Current bent state (already bent flanges)
     * @param nextBend The bend about to be performed
     * @return GraspValidation with all check results
     */
    GraspValidation validate(const BentState& state,
                             const phase1::BendFeature& nextBend) const;

    /**
     * @brief Compute remaining grip area after bending
     * Grip area decreases as flanges are bent up/away from the grip plane
     */
    double computeGripArea(const BentState& state,
                           const phase1::BendFeature& nextBend) const;

    /**
     * @brief Compute center of mass distance from grip center
     * As flanges bend, COM shifts away from the original flat position
     */
    double computeComDistance(const BentState& state,
                             const phase1::BendFeature& nextBend) const;

    /**
     * @brief Compute torque about the grip point
     * torque = mass * g * comDistance
     */
    double computeTorque(const BentState& state,
                         const phase1::BendFeature& nextBend) const;

    /**
     * @brief Compute shear force during bending
     * Lateral force from bending operation
     */
    double computeShearForce(const phase1::BendFeature& nextBend) const;

    const ValidatorConfig& config() const { return m_config; }

private:
    ValidatorConfig m_config;

    /**
     * @brief Estimate mass of a flange from its geometry
     */
    double estimateFlangeMass(const phase1::BendFeature& bend) const;

    /**
     * @brief Estimate initial flat part area from bend features
     */
    double estimatePartArea(const std::vector<phase1::BendFeature>& bends) const;
};

} // namespace phase4
} // namespace openpanelcam
