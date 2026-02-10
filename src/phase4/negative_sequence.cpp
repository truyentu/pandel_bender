#include "openpanelcam/phase4/negative_sequence.h"
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

NegativeSequenceAnalyzer::NegativeSequenceAnalyzer(const ValidatorConfig& config)
    : m_config(config)
    , m_detector(config) {}

NegativeSequenceResult NegativeSequenceAnalyzer::analyze(
    const std::vector<phase1::BendFeature>& bends)
{
    NegativeSequenceResult result;

    if (bends.empty()) {
        result.success = true;
        return result;
    }

    // Start with ALL bends in the folded state (fully bent workpiece)
    BentState foldedState;
    for (const auto& bend : bends) {
        foldedState.addBentFlange(bend);
    }

    std::set<int> processed;  // Bends already unfolded
    bool stop = false;
    int iteration = 0;

    // Algorithm 2: FindSequence(S, B, alpha)
    while (!stop) {
        iteration++;
        std::vector<int> gamma;  // Feasible bends this iteration

        for (const auto& bend : bends) {
            if (processed.count(bend.id)) continue;

            // Build state with only remaining (not-yet-unfolded) bends
            BentState currentState;
            for (const auto& other : bends) {
                if (other.id == bend.id) continue;  // Exclude the bend being tested
                if (processed.count(other.id)) continue;  // Exclude already unfolded
                currentState.addBentFlange(other);
            }

            if (isBendFeasible(bend, currentState)) {
                gamma.push_back(bend.id);
                processed.insert(bend.id);
            }
        }

        if (gamma.empty()) {
            stop = true;
            continue;
        }

        // Record the unfolded bends in reverse order
        for (int bendId : gamma) {
            result.reversedSequence.push_back(bendId);
        }
    }

    result.iterationsUsed = iteration;

    // Check if all bends were processed
    if (processed.size() == bends.size()) {
        result.success = true;
    } else {
        result.success = false;
        for (const auto& bend : bends) {
            if (!processed.count(bend.id)) {
                result.infeasibleBends.push_back(bend.id);
            }
        }
    }

    // Reverse the unfolding sequence to get forward bending order
    result.forwardSequence = result.reversedSequence;
    std::reverse(result.forwardSequence.begin(), result.forwardSequence.end());

    return result;
}

bool NegativeSequenceAnalyzer::isBendFeasible(
    const phase1::BendFeature& bend,
    const BentState& bentState) const
{
    // Generate swept volume with angular intervals
    SweptVolume swept = m_sweptGen.generate(bend);

    // Level 1: broad-phase AABB check
    CollisionResult broadResult = m_detector.checkStep(swept, bentState);
    if (broadResult.hasCollision) {
        return false;
    }

    // Level 2: fine-grained angular interval check
    CollisionResult intervalResult = m_detector.checkIntervals(swept, bentState);
    if (intervalResult.hasCollision) {
        return false;
    }

    return true;
}

NegativeSequenceResult NegativeSequenceAnalyzer::analyze(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<ToolGeometry>& tools)
{
    NegativeSequenceResult result;

    if (bends.empty()) {
        result.success = true;
        return result;
    }

    BentState foldedState;
    for (const auto& bend : bends) {
        foldedState.addBentFlange(bend);
    }

    std::set<int> processed;
    bool stop = false;
    int iteration = 0;

    while (!stop) {
        iteration++;
        std::vector<int> gamma;

        for (const auto& bend : bends) {
            if (processed.count(bend.id)) continue;

            BentState currentState;
            for (const auto& other : bends) {
                if (other.id == bend.id) continue;
                if (processed.count(other.id)) continue;
                currentState.addBentFlange(other);
            }

            if (isBendFeasible(bend, currentState, tools)) {
                gamma.push_back(bend.id);
                processed.insert(bend.id);
            }
        }

        if (gamma.empty()) {
            stop = true;
            continue;
        }

        for (int bendId : gamma) {
            result.reversedSequence.push_back(bendId);
        }
    }

    result.iterationsUsed = iteration;

    if (processed.size() == bends.size()) {
        result.success = true;
    } else {
        result.success = false;
        for (const auto& bend : bends) {
            if (!processed.count(bend.id)) {
                result.infeasibleBends.push_back(bend.id);
            }
        }
    }

    result.forwardSequence = result.reversedSequence;
    std::reverse(result.forwardSequence.begin(), result.forwardSequence.end());

    return result;
}

bool NegativeSequenceAnalyzer::isBendFeasible(
    const phase1::BendFeature& bend,
    const BentState& bentState,
    const std::vector<ToolGeometry>& tools) const
{
    // Part-part collision check (same as base version)
    if (!isBendFeasible(bend, bentState)) {
        return false;
    }

    // Tool-part collision check (paper: "clashes with punch tools")
    if (m_config.enableToolCollision && !tools.empty()) {
        SweptVolume swept = m_sweptGen.generate(bend);
        if (checkToolCollision(swept, tools)) {
            return false;  // Tool collision detected
        }
    }

    return true;
}

bool NegativeSequenceAnalyzer::checkToolCollision(
    const SweptVolume& swept,
    const std::vector<ToolGeometry>& tools) const
{
    AABB expandedSwept = swept.aabb.expand(m_config.collisionMargin);

    for (const auto& tool : tools) {
        if (!tool.valid) continue;

        // Check swept volume against punch
        if (expandedSwept.overlaps(tool.punchAABB)) {
            return true;
        }

        // Check swept volume against die
        if (expandedSwept.overlaps(tool.dieAABB)) {
            return true;
        }
    }

    return false;
}

} // namespace phase4
} // namespace openpanelcam
