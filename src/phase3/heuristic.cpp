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

// ===== H4: RemainingBendsHeuristic =====

RemainingBendsHeuristic::RemainingBendsHeuristic(double weight)
    : m_weight(weight) {}

double RemainingBendsHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    return m_weight * static_cast<double>(remaining.size());
}

// ===== CombinedHeuristic =====

CombinedHeuristic::CombinedHeuristic() {}

double CombinedHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    return m_h1.estimate(state, allBends)
         + m_h2.estimate(state, allBends)
         + m_h3.estimate(state, allBends)
         + m_h4.estimate(state, allBends);
}

// ===== H5: EnvelopeWidthHeuristic (paper h1) =====

EnvelopeWidthHeuristic::EnvelopeWidthHeuristic(double weight)
    : m_weight(weight) {}

double EnvelopeWidthHeuristic::computeEnvelopeWidth(
    const std::vector<phase1::BendFeature>& bends)
{
    if (bends.empty()) return 0.0;

    // Compute bounding range along X and Y axes
    double minX = bends[0].position.x, maxX = bends[0].position.x;
    double minY = bends[0].position.y, maxY = bends[0].position.y;

    for (const auto& bend : bends) {
        minX = std::min(minX, bend.position.x);
        maxX = std::max(maxX, bend.position.x);
        minY = std::min(minY, bend.position.y);
        maxY = std::max(maxY, bend.position.y);
    }

    double widthX = maxX - minX;
    double widthY = maxY - minY;

    // Return the MINIMUM width (envelope rectangle's narrow dimension)
    return std::min(widthX, widthY);
}

double EnvelopeWidthHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.empty()) return 0.0;

    return m_weight * computeEnvelopeWidth(remaining);
}

// ===== H6: TriangleAreaHeuristic (paper h2, inverse) =====

TriangleAreaHeuristic::TriangleAreaHeuristic(double weight)
    : m_weight(weight) {}

double TriangleAreaHeuristic::computeTriangleArea(const phase1::BendFeature& bend) {
    // Triangle formed by bend node and adjacent flange geometry
    // Approximation: 0.5 * bendLength * flangeExtent
    // flangeExtent approximated by distance from origin (position magnitude)
    double flangeExtent = std::sqrt(
        bend.position.x * bend.position.x +
        bend.position.y * bend.position.y +
        bend.position.z * bend.position.z
    );
    // Minimum extent to avoid division by zero downstream
    if (flangeExtent < 1.0) flangeExtent = 1.0;

    double bendLen = (bend.length > 0.0) ? bend.length : 1.0;
    return 0.5 * bendLen * flangeExtent;
}

double TriangleAreaHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.empty()) return 0.0;

    // Sum of triangle areas for remaining bends
    double totalArea = 0.0;
    for (const auto& bend : remaining) {
        totalArea += computeTriangleArea(bend);
    }

    // Paper formula: k2 / h2 -- inverse relationship
    // Larger total area = lower cost (shape-defining bends remaining = good)
    if (totalArea < 1e-6) return m_weight; // fallback for zero area

    return m_weight / totalArea;
}

// ===== PaperHeuristic: h(i) = k1*h1 + k2/h2 + h4 =====

PaperHeuristic::PaperHeuristic(double k1, double k2)
    : m_k1(k1), m_k2(k2) {}

double PaperHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.empty()) return 0.0;

    // h1: envelope width (paper's "minimum width of envelope rectangle")
    double h1 = EnvelopeWidthHeuristic::computeEnvelopeWidth(remaining);

    // h2: sum of triangle areas (paper's "bends' corresponding triangle areas")
    double h2 = 0.0;
    for (const auto& bend : remaining) {
        h2 += TriangleAreaHeuristic::computeTriangleArea(bend);
    }

    // Paper formula: h(i) = k1 * h1(i) + k2 / h2(i)
    double paperH = m_k1 * h1;
    if (h2 > 1e-6) {
        paperH += m_k2 / h2;
    } else {
        paperH += m_k2; // fallback
    }

    // Add admissible baseline for A* optimality
    double admissible = m_admissibleBase.estimate(state, allBends);

    return paperH + admissible;
}

} // namespace phase3
} // namespace openpanelcam
