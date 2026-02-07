#pragma once

#include "types.h"
#include "cost_function.h"
#include "heuristic.h"
#include "zobrist_table.h"
#include "successor_generator.h"
#include "../phase2/constraint_solver.h"
#include <queue>
#include <unordered_map>
#include <memory>
#include <chrono>

namespace openpanelcam {
namespace phase3 {

struct SearchConfig {
    int maxNodes = 1000000;
    double timeoutSeconds = 30.0;
};

/**
 * @brief A* search engine for optimal bend sequencing
 */
class AStarSearch {
public:
    AStarSearch(const phase2::Phase2Output& phase2Output,
                const std::vector<phase1::BendFeature>& bends);

    Phase3Output search(const SearchConfig& config = SearchConfig());

    const SequencerStatistics& getStatistics() const { return m_stats; }

private:
    const phase2::Phase2Output& m_phase2;
    const std::vector<phase1::BendFeature>& m_bends;

    ZobristTable m_zobrist;
    MaskedTimeCost m_costFn;
    CombinedHeuristic m_heuristic;
    std::unique_ptr<SuccessorGenerator> m_successor;

    SequencerStatistics m_stats;

    Phase3Output reconstructPath(const std::vector<SearchNode>& allNodes,
                                  int goalNodeIndex);
    bool isGoal(const SearchState& state) const;
};

} // namespace phase3
} // namespace openpanelcam
