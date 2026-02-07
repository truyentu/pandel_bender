#include "openpanelcam/phase3/sequencer.h"

namespace openpanelcam {
namespace phase3 {

Sequencer::Sequencer() {}

Phase3Output Sequencer::sequence(const phase2::Phase2Output& phase2Output,
                                  const std::vector<phase1::BendFeature>& bends) {
    AStarSearch search(phase2Output, bends);
    Phase3Output output = search.search(m_config);
    m_stats = output.stats;
    return output;
}

void Sequencer::setConfig(const SearchConfig& config) {
    m_config = config;
}

} // namespace phase3
} // namespace openpanelcam
