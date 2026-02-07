#include "openpanelcam/phase3/successor_generator.h"
#include <cmath>

namespace openpanelcam {
namespace phase3 {

// ===== RepoTriggerDetector =====

RepoTriggerDetector::RepoTriggerDetector(double minGripArea)
    : m_minGripArea(minGripArea) {}

RepoReason RepoTriggerDetector::check(
    const SearchState& state,
    int bendId,
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<phase2::GraspConstraint>& graspConstraints
) const {
    // Check box closing first (highest priority)
    if (isBoxClosingBend(bendId, state, bends)) {
        return RepoReason::BOX_CLOSING;
    }

    // Check grip area exhaustion against grasp constraints
    if (isGripAreaExhausted(state, graspConstraints)) {
        return RepoReason::GRIP_AREA_EXHAUSTED;
    }

    return RepoReason::NONE;
}

bool RepoTriggerDetector::isBoxClosingBend(
    int bendId,
    const SearchState& state,
    const std::vector<phase1::BendFeature>& bends
) const {
    // A box-closing bend is one where opposite bends are already done,
    // creating a closed box shape that traps the gripper.
    // Heuristic: if bend count > 2 and remaining bends <= 1 after this one,
    // and total bends >= 4 (box requires at least 4 bends)
    int totalBends = static_cast<int>(bends.size());
    if (totalBends < 4) return false;

    int bentAfter = state.bentCount(); // already includes this bend
    int remaining = totalBends - bentAfter;

    // Box closing typically happens on the last 1-2 bends of a 4+ bend part
    // when opposing flanges create an enclosed area
    if (remaining == 0 && bentAfter >= 4) {
        // Check if opposing directions exist in bent bends
        bool hasPositiveX = false, hasNegativeX = false;
        bool hasPositiveY = false, hasNegativeY = false;

        for (const auto& bend : bends) {
            if (state.isBent(bend.id)) {
                if (bend.direction.x > 0.5) hasPositiveX = true;
                if (bend.direction.x < -0.5) hasNegativeX = true;
                if (bend.direction.y > 0.5) hasPositiveY = true;
                if (bend.direction.y < -0.5) hasNegativeY = true;
            }
        }

        // Box closing: opposing flanges in at least one axis
        if ((hasPositiveX && hasNegativeX) || (hasPositiveY && hasNegativeY)) {
            return true;
        }
    }

    return false;
}

bool RepoTriggerDetector::isGripAreaExhausted(
    const SearchState& state,
    const std::vector<phase2::GraspConstraint>& constraints
) const {
    // Find matching grasp constraint for current state
    for (const auto& constraint : constraints) {
        // Match by comparing bent bends
        uint32_t constraintMask = 0;
        for (int bentId : constraint.bentBends) {
            if (bentId >= 0 && bentId < 32) {
                constraintMask |= (1u << bentId);
            }
        }

        if (constraintMask == state.bentMask) {
            // Found matching constraint - check grip validity
            if (!constraint.hasValidGrip || constraint.validArea < m_minGripArea) {
                return true;
            }
            return false;
        }
    }

    // No matching constraint found - assume grip is valid
    return false;
}

// ===== SuccessorGenerator =====

SuccessorGenerator::SuccessorGenerator(
    const phase2::PrecedenceDAG& dag,
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<phase2::GraspConstraint>& graspConstraints
) : m_dag(dag), m_bends(bends), m_graspConstraints(graspConstraints) {}

bool SuccessorGenerator::canBend(int bendId, const SearchState& state) const {
    if (state.isBent(bendId)) return false;

    const auto* node = m_dag.getNodeByBendId(bendId);
    if (!node) return false;

    // All predecessors must be bent
    for (int predBendId : node->predecessors) {
        if (!state.isBent(predBendId)) {
            return false;
        }
    }

    return true;
}

std::vector<int> SuccessorGenerator::getBendableBends(const SearchState& state) const {
    std::vector<int> bendable;
    for (const auto& bend : m_bends) {
        if (canBend(bend.id, state)) {
            bendable.push_back(bend.id);
        }
    }
    return bendable;
}

SearchState SuccessorGenerator::applyBend(const SearchState& current, int bendId) const {
    SearchState next = current;
    next.markBent(bendId);

    for (const auto& bend : m_bends) {
        if (bend.id == bendId) {
            next.orientation = m_estimator.requiredOrientation(bend);
            next.abaConfig = m_estimator.requiredAbaConfig(bend);
            break;
        }
    }

    // Check repo trigger
    RepoReason reason = m_repoDetector.check(next, bendId, m_bends, m_graspConstraints);
    next.needsRepo = (reason != RepoReason::NONE);
    next.repoReason = reason;

    return next;
}

std::vector<SearchNode> SuccessorGenerator::generate(
    const SearchNode& current,
    int currentIdx,
    const MaskedTimeCost& costFn,
    const Heuristic& heuristic
) {
    std::vector<SearchNode> successors;

    auto bendable = getBendableBends(current.state);

    for (int bendId : bendable) {
        SearchNode successor;
        successor.state = applyBend(current.state, bendId);

        // Find bend feature
        const phase1::BendFeature* bend = nullptr;
        for (const auto& b : m_bends) {
            if (b.id == bendId) {
                bend = &b;
                break;
            }
        }
        if (!bend) continue;

        successor.g = current.g + costFn.stepCost(current.state, bendId, successor.state, *bend);
        successor.h = heuristic.estimate(successor.state, m_bends);
        successor.parentId = currentIdx;
        successor.lastBendId = bendId;
        successor.lastAction = ActionType::BEND;

        successors.push_back(successor);
    }

    return successors;
}

} // namespace phase3
} // namespace openpanelcam
