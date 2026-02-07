#pragma once

#include "openpanelcam/phase5/types.h"
#include <string>
#include <vector>
#include <set>
#include <cmath>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Validates MachineProgram against machine limits and sanity checks
 */
class OutputValidator {
public:
    struct ValidationResult {
        bool valid = true;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    explicit OutputValidator(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Run all validation checks on program
     */
    ValidationResult validate(const MachineProgram& program);

    /**
     * @brief Check bend angle within machine limits [-135, +135]
     */
    bool checkAngleLimits(const MachineInstruction& instr) const;

    /**
     * @brief Check bend force within machine capacity
     */
    bool checkForceLimits(const MachineInstruction& instr) const;

    /**
     * @brief Check part dimensions within machine limits
     */
    bool checkPartSize(const MachineProgram& program) const;

    /**
     * @brief Check total cycle time equals sum of step durations
     */
    bool checkCycleTimeConsistency(const MachineProgram& program) const;

    /**
     * @brief Check no duplicate step IDs
     */
    bool checkStepIdUniqueness(const MachineProgram& program) const;

private:
    PostProcessorConfig config_;
};

} // namespace phase5
} // namespace openpanelcam
