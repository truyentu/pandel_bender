#include "openpanelcam/phase4/sequence_validator.h"
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

SequenceValidator::SequenceValidator(const ValidatorConfig& config)
    : m_config(config)
    , m_collisionDetector(config)
    , m_graspEngine(config)
    , m_extractionChecker(config)
    , m_springbackComp(config) {}

Phase4Output SequenceValidator::validate(
    const phase3::Phase3Output& sequence,
    const std::vector<phase1::BendFeature>& bends
) {
    Phase4Output output;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Validate input
    if (sequence.bendSequence.empty()) {
        output.success = true;
        output.allStepsValid = true;
        return output;
    }

    if (!sequence.success) {
        output.success = false;
        output.errors.push_back("Phase 3 sequence was not successful");
        return output;
    }

    // Process each step in the sequence
    BentState state;
    bool allValid = true;

    for (size_t i = 0; i < sequence.bendSequence.size(); i++) {
        int bendId = sequence.bendSequence[i];
        const phase1::BendFeature* bend = findBend(bends, bendId);

        if (!bend) {
            output.errors.push_back("Bend ID " + std::to_string(bendId) +
                                    " not found in bend list");
            allValid = false;
            continue;
        }

        // Validate this step
        StepValidation stepResult = validateStep(
            static_cast<int>(i), *bend, state);

        output.stepResults.push_back(stepResult);

        // Track failures
        if (!stepResult.isValid()) {
            allValid = false;
            generateFeedback(output, stepResult, static_cast<int>(i));
        }

        if (stepResult.collision.hasCollision) {
            output.collisionsDetected++;
        }
        if (!stepResult.grasp.isValid()) {
            output.graspFailures++;
        }
        if (!stepResult.extraction.canExtract) {
            output.extractionFailures++;
        }

        // Add springback to table
        output.springbackTable.push_back(stepResult.springback);

        // Update bent state for next step
        state.addBentFlange(*bend);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    output.validationTimeMs = std::chrono::duration<double, std::milli>(
        endTime - startTime).count();

    output.success = allValid;
    output.allStepsValid = allValid;

    if (allValid) {
        output.warnings.push_back("All " +
            std::to_string(sequence.bendSequence.size()) +
            " steps validated successfully");
    }

    return output;
}

StepValidation SequenceValidator::validateStep(
    int stepIndex,
    const phase1::BendFeature& bend,
    BentState& state
) {
    StepValidation result;
    result.stepIndex = stepIndex;
    result.bendId = bend.id;

    // 1. Generate swept volume
    SweptVolume swept = m_sweptGen.generate(bend);

    // 2. Check collision with bent flanges
    result.collision = m_collisionDetector.checkStep(swept, state);

    // 3. Validate grasp physics
    result.grasp = m_graspEngine.validate(state, bend);

    // 4. Check tool extraction
    // Create a temporary state with this bend added for extraction check
    BentState stateAfter = state;
    stateAfter.addBentFlange(bend);
    result.extraction = m_extractionChecker.check(bend, stateAfter);

    // 5. Compute springback compensation
    result.springback = m_springbackComp.compensate(bend);

    return result;
}

void SequenceValidator::setConfig(const ValidatorConfig& config) {
    m_config = config;
    m_collisionDetector = CollisionDetector(config);
    m_graspEngine = GraspPhysicsEngine(config);
    m_extractionChecker = ExtractionChecker(config);
    m_springbackComp = SpringbackCompensator(config);
}

void SequenceValidator::generateFeedback(
    Phase4Output& output,
    const StepValidation& stepResult,
    int stepIndex
) const {
    int bendId = stepResult.bendId;

    // Collision feedback
    if (stepResult.collision.hasCollision) {
        output.errors.push_back(
            "Step " + std::to_string(stepIndex) +
            ": " + stepResult.collision.description);

        if (stepResult.collision.collidingBendId >= 0) {
            output.suggestions.push_back(
                "Try reordering bend " + std::to_string(bendId) +
                " before bend " + std::to_string(stepResult.collision.collidingBendId));
        }

        output.suggestions.push_back(
            "Consider adding a repositioning before bend " +
            std::to_string(bendId) + " to avoid collision");
    }

    // Grasp failure feedback
    if (!stepResult.grasp.isValid()) {
        output.errors.push_back(
            "Step " + std::to_string(stepIndex) +
            ": " + stepResult.grasp.describe());

        if (!stepResult.grasp.areaValid) {
            output.suggestions.push_back(
                "Grip area insufficient at bend " + std::to_string(bendId) +
                " (" + std::to_string(stepResult.grasp.gripArea) +
                " mm2) - add repositioning to improve grip");
        }

        if (!stepResult.grasp.comValid) {
            output.suggestions.push_back(
                "Center of mass too far from grip at bend " +
                std::to_string(bendId) +
                " - reorder smaller bends first to maintain balance");
        }

        if (!stepResult.grasp.torqueValid) {
            output.suggestions.push_back(
                "Torque exceeds limit at bend " + std::to_string(bendId) +
                " (" + std::to_string(stepResult.grasp.torque) +
                " Nm) - consider repositioning to reduce moment arm");
        }

        if (!stepResult.grasp.shearValid) {
            output.suggestions.push_back(
                "Shear force too high at bend " + std::to_string(bendId) +
                " - reduce bending speed or use stronger grip");
        }
    }

    // Extraction failure feedback
    if (!stepResult.extraction.canExtract) {
        output.errors.push_back(
            "Step " + std::to_string(stepIndex) +
            ": " + stepResult.extraction.description);

        output.suggestions.push_back(
            "Bend " + std::to_string(bendId) +
            " creates a tool trap - try reversing the bend order to " +
            "perform this bend earlier before surrounding flanges are bent");
    }
}

const phase1::BendFeature* SequenceValidator::findBend(
    const std::vector<phase1::BendFeature>& bends, int bendId
) {
    for (const auto& b : bends) {
        if (b.id == bendId) return &b;
    }
    return nullptr;
}

} // namespace phase4
} // namespace openpanelcam
