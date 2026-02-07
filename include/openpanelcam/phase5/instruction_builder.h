#pragma once

#include "types.h"
#include "../phase4/types.h"
#include "../phase3/types.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <string>
#include <ctime>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Transforms Phase 4 validated output into machine instructions
 *
 * Converts the abstract bend sequence into concrete machine-level
 * instructions: bend operations with springback compensation,
 * rotation steps, ABA reconfigurations, and repositions.
 */
class InstructionBuilder {
public:
    explicit InstructionBuilder(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Build complete machine program from validated sequence
     */
    MachineProgram build(const phase4::Phase4Output& validated,
                         const phase3::Phase3Output& sequence,
                         const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Build a single bend instruction
     */
    MachineInstruction buildBendInstruction(
        int stepId,
        const phase1::BendFeature& bend,
        const phase4::SpringbackData& springback);

    /**
     * @brief Build a rotation instruction
     */
    MachineInstruction buildRotationInstruction(
        int stepId,
        const phase3::SequenceAction& action);

    /**
     * @brief Build an ABA setup instruction
     */
    MachineInstruction buildABAInstruction(
        int stepId,
        const phase3::SequenceAction& action);

    /**
     * @brief Build a reposition instruction
     */
    MachineInstruction buildRepositionInstruction(
        int stepId,
        const phase3::SequenceAction& action);

    /**
     * @brief Compute bending force in kN
     * F = (sigma_y * L * t^2) / (4 * W) / 1000
     * W = 8 * t (V-die opening)
     */
    double computeBendForce(double length, double thickness,
                            double yieldStrength) const;

    const PostProcessorConfig& config() const { return m_config; }

private:
    PostProcessorConfig m_config;

    std::string generateTimestamp() const;

    const phase1::BendFeature* findBend(
        const std::vector<phase1::BendFeature>& bends, int bendId) const;

    const phase4::SpringbackData* findSpringback(
        const phase4::Phase4Output& validated, int bendId) const;
};

} // namespace phase5
} // namespace openpanelcam
