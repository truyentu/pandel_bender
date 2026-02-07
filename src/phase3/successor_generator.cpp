#include "openpanelcam/phase3/successor_generator.h"

namespace openpanelcam {
namespace phase3 {

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

    next.needsRepo = false;
    next.repoReason = RepoReason::NONE;

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
