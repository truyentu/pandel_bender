#pragma once

#include "types.h"
#include "swept_volume.h"
#include "bent_state.h"
#include "collision_detector.h"
#include "grasp_physics.h"
#include "extraction_checker.h"
#include "../phase3/types.h"
#include "../phase2/constraint_solver.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <chrono>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Main validation pipeline for Phase 4
 *
 * Validates an entire bend sequence step by step:
 * 1. Generate swept volume for each bend
 * 2. Check collision with all previously bent flanges
 * 3. Validate grasp physics (area, COM, torque, shear)
 * 4. Check tool extraction feasibility
 * 5. Compute springback compensation
 * 6. Update bent state
 * 7. Generate feedback for failures
 */
class SequenceValidator {
public:
    explicit SequenceValidator(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Validate entire bend sequence
     * @param sequence Phase 3 output with bend order
     * @param bends All bend features
     * @return Phase4Output with step-by-step results
     */
    Phase4Output validate(const phase3::Phase3Output& sequence,
                          const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Validate a single step
     * @param stepIndex Index in the sequence
     * @param bend The bend to perform
     * @param state Current bent state
     * @return StepValidation result
     */
    StepValidation validateStep(int stepIndex,
                                const phase1::BendFeature& bend,
                                BentState& state);

    /**
     * @brief Update configuration
     */
    void setConfig(const ValidatorConfig& config);

    const ValidatorConfig& config() const { return m_config; }

private:
    ValidatorConfig m_config;
    SweptVolumeGenerator m_sweptGen;
    CollisionDetector m_collisionDetector;
    GraspPhysicsEngine m_graspEngine;
    ExtractionChecker m_extractionChecker;
    SpringbackCompensator m_springbackComp;

    /**
     * @brief Generate feedback suggestions for a failed step
     */
    void generateFeedback(Phase4Output& output,
                          const StepValidation& stepResult,
                          int stepIndex) const;

    /**
     * @brief Find bend by ID in the bends list
     */
    static const phase1::BendFeature* findBend(
        const std::vector<phase1::BendFeature>& bends, int bendId);
};

} // namespace phase4
} // namespace openpanelcam
