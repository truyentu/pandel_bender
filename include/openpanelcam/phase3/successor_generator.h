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
 * @brief Generates valid successor states for A* search
 *
 * Respects precedence constraints from Phase 2 DAG.
 * A bend is bendable only if all its predecessors are already bent.
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

    SearchState applyBend(const SearchState& current, int bendId) const;
};

} // namespace phase3
} // namespace openpanelcam
