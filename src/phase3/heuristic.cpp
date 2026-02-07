#include "openpanelcam/phase3/heuristic.h"

namespace openpanelcam {
namespace phase3 {

// ===== H1: RotationalEntropyHeuristic =====

RotationalEntropyHeuristic::RotationalEntropyHeuristic(double weight)
    : m_weight(weight) {}

double RotationalEntropyHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.empty()) return 0.0;

    std::set<Orientation> orientations;
    for (const auto& bend : remaining) {
        orientations.insert(m_estimator.requiredOrientation(bend));
    }

    int distinctCount = static_cast<int>(orientations.size());
    if (distinctCount <= 1) return 0.0;

    return m_weight * (distinctCount - 1);
}

// ===== H2: ToolingVarianceHeuristic =====

ToolingVarianceHeuristic::ToolingVarianceHeuristic(double weight)
    : m_weight(weight) {}

double ToolingVarianceHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.size() < 2) return 0.0;

    std::vector<double> widths;
    widths.reserve(remaining.size());
    for (const auto& bend : remaining) {
        widths.push_back(bend.length + 10.0);
    }

    double mean = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();

    double sqSum = 0.0;
    for (double w : widths) {
        sqSum += (w - mean) * (w - mean);
    }
    double stddev = std::sqrt(sqSum / widths.size());

    return m_weight * stddev;
}

// ===== H3: GraspFragmentationHeuristic =====

GraspFragmentationHeuristic::GraspFragmentationHeuristic(double weight)
    : m_weight(weight) {}

double GraspFragmentationHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);

    if (remaining.size() <= 1) return 0.0;

    int fragmentationRisk = static_cast<int>(remaining.size()) / 3;
    return m_weight * fragmentationRisk;
}

// ===== CombinedHeuristic =====

CombinedHeuristic::CombinedHeuristic() {}

double CombinedHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    return m_h1.estimate(state, allBends)
         + m_h2.estimate(state, allBends)
         + m_h3.estimate(state, allBends);
}

} // namespace phase3
} // namespace openpanelcam
