#pragma once

#include "types.h"
#include "bent_state.h"
#include "../phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Checks if the bending tool can be extracted after a bend
 *
 * After bending, the tool (punch/die) must be able to move away
 * from the part without colliding with bent flanges.
 * Typical extraction direction is vertical (Z-axis).
 */
class ExtractionChecker {
public:
    explicit ExtractionChecker(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Check if tool can be extracted after this bend
     * @param bend The bend just performed
     * @param stateAfter Bent state including this bend
     * @return ExtractionResult
     */
    ExtractionResult check(const phase1::BendFeature& bend,
                           const BentState& stateAfter) const;

    /**
     * @brief Check if a tool at given position can extract vertically
     * @param toolPosition AABB of the tool
     * @param obstacles All occupied volumes that could block extraction
     * @return true if vertical extraction path is clear
     */
    bool canExtractVertically(const AABB& toolPosition,
                              const std::vector<AABB>& obstacles) const;

private:
    ValidatorConfig m_config;

    /**
     * @brief Estimate tool AABB from bend parameters
     */
    AABB estimateToolAABB(const phase1::BendFeature& bend) const;
};

/**
 * @brief Compensates for material springback
 *
 * After bending, the material springs back slightly.
 * The machine must overbend to achieve the target angle.
 * Model: springback = baseDeg + perMm * thickness
 */
class SpringbackCompensator {
public:
    explicit SpringbackCompensator(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Compute springback compensation for a single bend
     */
    SpringbackData compensate(const phase1::BendFeature& bend) const;

    /**
     * @brief Compute springback for all bends
     */
    std::vector<SpringbackData> compensateAll(
        const std::vector<phase1::BendFeature>& bends) const;

private:
    ValidatorConfig m_config;
};

} // namespace phase4
} // namespace openpanelcam
