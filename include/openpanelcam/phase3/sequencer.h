#pragma once

#include "types.h"
#include "astar_search.h"
#include "../phase2/constraint_solver.h"

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Main entry point for Phase 3 bend sequencing
 *
 * Usage:
 *   Sequencer seq;
 *   Phase3Output output = seq.sequence(phase2Output, bends);
 *   if (output.success) {
 *       for (int bendId : output.bendSequence) { ... }
 *   }
 */
class Sequencer {
public:
    Sequencer();

    Phase3Output sequence(const phase2::Phase2Output& phase2Output,
                          const std::vector<phase1::BendFeature>& bends);

    void setConfig(const SearchConfig& config);

    const SequencerStatistics& getStatistics() const { return m_stats; }

private:
    SearchConfig m_config;
    SequencerStatistics m_stats;
};

} // namespace phase3
} // namespace openpanelcam
