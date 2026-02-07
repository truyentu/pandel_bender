#include "openpanelcam/phase5/instruction_builder.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace openpanelcam {
namespace phase5 {

InstructionBuilder::InstructionBuilder(const PostProcessorConfig& config)
    : m_config(config) {}

MachineProgram InstructionBuilder::build(
    const phase4::Phase4Output& validated,
    const phase3::Phase3Output& sequence,
    const std::vector<phase1::BendFeature>& bends
) {
    MachineProgram program;
    program.partName = "PART";
    program.createdBy = "OpenPanelCAM";
    program.timestamp = generateTimestamp();
    program.machineModel = "P4";
    program.materialThickness = m_config.defaultThickness;
    program.materialType = m_config.defaultMaterial;
    program.yieldStrength = m_config.defaultYieldStrength;
    program.adaptiveEnabled = m_config.includeAdaptiveControl;
    program.forceLimit = m_config.maxForce;

    int stepId = 1;
    int bendCount = 0;
    int repoCount = 0;
    int rotCount = 0;
    double totalTime = 0.0;

    // Process each action from Phase 3
    for (const auto& action : sequence.actions) {
        MachineInstruction instr;

        switch (action.type) {
            case phase3::ActionType::BEND: {
                const phase1::BendFeature* bend = findBend(bends, action.bendId);
                if (!bend) continue;

                const phase4::SpringbackData* sb = findSpringback(validated, action.bendId);
                phase4::SpringbackData defaultSb;
                defaultSb.bendId = action.bendId;
                defaultSb.targetAngle = bend->angle;
                defaultSb.compensatedAngle = bend->angle;
                defaultSb.springbackAngle = 0.0;

                instr = buildBendInstruction(
                    stepId, *bend, sb ? *sb : defaultSb);
                bendCount++;
                break;
            }

            case phase3::ActionType::ROTATE:
                instr = buildRotationInstruction(stepId, action);
                rotCount++;
                break;

            case phase3::ActionType::ABA_RECONFIG:
                instr = buildABAInstruction(stepId, action);
                break;

            case phase3::ActionType::REPOSITION:
                instr = buildRepositionInstruction(stepId, action);
                repoCount++;
                break;

            default:
                continue;
        }

        totalTime += instr.duration;
        program.instructions.push_back(instr);
        stepId++;
    }

    program.totalCycleTime = totalTime;
    program.bendCount = bendCount;
    program.repositionCount = repoCount;
    program.rotationCount = rotCount;

    return program;
}

MachineInstruction InstructionBuilder::buildBendInstruction(
    int stepId,
    const phase1::BendFeature& bend,
    const phase4::SpringbackData& springback
) {
    MachineInstruction instr;
    instr.stepId = stepId;
    instr.type = InstructionType::BEND;
    instr.bendId = bend.id;
    instr.targetAngle = springback.targetAngle;
    instr.compensatedAngle = springback.compensatedAngle;
    instr.springbackAngle = springback.springbackAngle;
    instr.bendLength = bend.length;

    // Bend line endpoints
    double halfLen = bend.length / 2.0;
    instr.startX = bend.position.x - bend.direction.x * halfLen;
    instr.startY = bend.position.y - bend.direction.y * halfLen;
    instr.endX = bend.position.x + bend.direction.x * halfLen;
    instr.endY = bend.position.y + bend.direction.y * halfLen;

    // Force calculation
    instr.bendForce = computeBendForce(
        bend.length, m_config.defaultThickness, m_config.defaultYieldStrength);

    // Duration estimate: ~2.5s for typical bend
    instr.duration = 2.5;

    instr.description = "Bend " + std::to_string(bend.id) +
        ": " + std::to_string(springback.targetAngle) + "deg" +
        " (comp: " + std::to_string(springback.compensatedAngle) + "deg)";

    return instr;
}

MachineInstruction InstructionBuilder::buildRotationInstruction(
    int stepId,
    const phase3::SequenceAction& action
) {
    MachineInstruction instr;
    instr.stepId = stepId;
    instr.type = InstructionType::ROTATE;
    instr.rotationAngle = static_cast<int>(action.newOrientation) * 90;
    instr.duration = 1.5;
    instr.description = "Rotate " + std::to_string(instr.rotationAngle) + "deg";
    return instr;
}

MachineInstruction InstructionBuilder::buildABAInstruction(
    int stepId,
    const phase3::SequenceAction& action
) {
    MachineInstruction instr;
    instr.stepId = stepId;
    instr.type = InstructionType::ABA_SETUP;
    instr.abaConfig = action.newAbaConfig;
    instr.duration = 0.8;
    instr.description = "ABA config: " + std::to_string(action.newAbaConfig);
    return instr;
}

MachineInstruction InstructionBuilder::buildRepositionInstruction(
    int stepId,
    const phase3::SequenceAction& action
) {
    MachineInstruction instr;
    instr.stepId = stepId;
    instr.type = InstructionType::REPOSITION;
    instr.duration = action.duration > 0 ? action.duration : 3.0;
    instr.description = action.description.empty()
        ? "Reposition part" : action.description;
    return instr;
}

double InstructionBuilder::computeBendForce(
    double length, double thickness, double yieldStrength
) const {
    // F = (sigma_y * L * t^2) / (4 * W) / 1000.0 [kN]
    // W = 8 * t (V-die opening width)
    double W = 8.0 * thickness;
    double forceN = (yieldStrength * length * thickness * thickness) / (4.0 * W);
    return forceN / 1000.0; // Convert N to kN
}

std::string InstructionBuilder::generateTimestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf;
#ifdef _WIN32
    localtime_s(&tm_buf, &time_t);
#else
    localtime_r(&time_t, &tm_buf);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
    return oss.str();
}

const phase1::BendFeature* InstructionBuilder::findBend(
    const std::vector<phase1::BendFeature>& bends, int bendId
) const {
    for (const auto& b : bends) {
        if (b.id == bendId) return &b;
    }
    return nullptr;
}

const phase4::SpringbackData* InstructionBuilder::findSpringback(
    const phase4::Phase4Output& validated, int bendId
) const {
    for (const auto& sb : validated.springbackTable) {
        if (sb.bendId == bendId) return &sb;
    }
    return nullptr;
}

} // namespace phase5
} // namespace openpanelcam
