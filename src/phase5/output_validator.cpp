#include "openpanelcam/phase5/output_validator.h"
#include <sstream>
#include <cmath>

namespace openpanelcam {
namespace phase5 {

OutputValidator::OutputValidator(const PostProcessorConfig& config)
    : config_(config) {}

OutputValidator::ValidationResult OutputValidator::validate(const MachineProgram& program) {
    ValidationResult result;

    // Check each instruction
    for (const auto& instr : program.instructions) {
        if (instr.type == InstructionType::BEND) {
            if (!checkAngleLimits(instr)) {
                result.valid = false;
                std::ostringstream oss;
                oss << "Step " << instr.stepId
                    << ": bend angle " << instr.targetAngle
                    << " exceeds limits [" << config_.minBendAngle
                    << ", " << config_.maxBendAngle << "]";
                result.errors.push_back(oss.str());
            }

            if (!checkForceLimits(instr)) {
                result.valid = false;
                std::ostringstream oss;
                oss << "Step " << instr.stepId
                    << ": bend force " << instr.bendForce
                    << " kN exceeds max " << config_.maxForce << " kN";
                result.errors.push_back(oss.str());
            }
        }
    }

    // Part size check
    if (!checkPartSize(program)) {
        result.warnings.push_back("Part dimensions may exceed machine limits");
    }

    // Cycle time consistency
    if (!checkCycleTimeConsistency(program)) {
        result.warnings.push_back("Total cycle time does not match sum of step durations");
    }

    // Step ID uniqueness
    if (!checkStepIdUniqueness(program)) {
        result.valid = false;
        result.errors.push_back("Duplicate step IDs detected");
    }

    return result;
}

bool OutputValidator::checkAngleLimits(const MachineInstruction& instr) const {
    if (instr.type != InstructionType::BEND) return true;
    return instr.targetAngle >= config_.minBendAngle &&
           instr.targetAngle <= config_.maxBendAngle;
}

bool OutputValidator::checkForceLimits(const MachineInstruction& instr) const {
    if (instr.type != InstructionType::BEND) return true;
    return instr.bendForce <= config_.maxForce;
}

bool OutputValidator::checkPartSize(const MachineProgram& program) const {
    // Check bend line extents against machine limits
    for (const auto& instr : program.instructions) {
        if (instr.type == InstructionType::BEND) {
            double maxX = std::max(std::abs(instr.startX), std::abs(instr.endX));
            double maxY = std::max(std::abs(instr.startY), std::abs(instr.endY));
            if (maxX > config_.maxPartLength || maxY > config_.maxPartWidth) {
                return false;
            }
        }
    }
    return true;
}

bool OutputValidator::checkCycleTimeConsistency(const MachineProgram& program) const {
    if (program.instructions.empty()) return true;

    double sumDurations = 0.0;
    for (const auto& instr : program.instructions) {
        sumDurations += instr.duration;
    }

    // Allow 1% tolerance or 0.1s absolute
    double tolerance = std::max(0.1, program.totalCycleTime * 0.01);
    return std::abs(program.totalCycleTime - sumDurations) <= tolerance;
}

bool OutputValidator::checkStepIdUniqueness(const MachineProgram& program) const {
    std::set<int> ids;
    for (const auto& instr : program.instructions) {
        if (instr.stepId >= 0) {
            if (!ids.insert(instr.stepId).second) {
                return false;
            }
        }
    }
    return true;
}

} // namespace phase5
} // namespace openpanelcam
