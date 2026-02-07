#pragma once

#include "types.h"
#include "cost_function.h"
#include "heuristic.h"
#include "../phase2/precedence_dag.h"
#include "../phase2/types.h"
#include <vector>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Detects when repositioning is needed during bending
 *
 * Checks grasp constraints to determine if the current grip
 * is still valid after a bend operation.
 */
class RepoTriggerDetector {
public:
    explicit RepoTriggerDetector(double minGripArea = 100.0);

    /**
     * @brief Check if repo is needed after applying a bend
     * @param state Current search state (after bend applied)
     * @param bendId The bend that was just performed
     * @param bends All bend features
     * @param graspConstraints Grasp constraints from Phase 2
     * @return RepoReason::NONE if no repo needed, otherwise the reason
     */
    RepoReason check(const SearchState& state,
                     int bendId,
                     const std::vector<phase1::BendFeature>& bends,
                     const std::vector<phase2::GraspConstraint>& graspConstraints) const;

private:
    double m_minGripArea;

    bool isBoxClosingBend(int bendId, const SearchState& state,
                          const std::vector<phase1::BendFeature>& bends) const;
    bool isGripAreaExhausted(const SearchState& state,
                             const std::vector<phase2::GraspConstraint>& constraints) const;
};

/**
 * @brief Generates valid successor states for A* search
 *
 * Respects precedence constraints from Phase 2 DAG.
 * A bend is bendable only if all its predecessors are already bent.
 * Detects repo triggers and adds repo cost when needed.
 */
class SuccessorGenerator {
public:
    SuccessorGenerator(const phase2::PrecedenceDAG& dag,
                       const std::vector<phase1::BendFeature>& bends,
                       const std::vector<phase2::GraspConstraint>& graspConstraints);

    std::vector<SearchNode> generate(const SearchNode& current,
                                     int currentIdx,
                                     const MaskedTimeCost& costFn,
                                     const Heuristic& heuristic);

    bool canBend(int bendId, const SearchState& state) const;

    std::vector<int> getBendableBends(const SearchState& state) const;

private:
    const phase2::PrecedenceDAG& m_dag;
    const std::vector<phase1::BendFeature>& m_bends;
    const std::vector<phase2::GraspConstraint>& m_graspConstraints;
    BendTimeEstimator m_estimator;
    RepoTriggerDetector m_repoDetector;

    SearchState applyBend(const SearchState& current, int bendId) const;
};

} // namespace phase3
} // namespace openpanelcam
