#pragma once

#include "types.h"
#include "cost_function.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <set>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Base class for A* heuristics
 *
 * All heuristics must be admissible (never overestimate).
 */
class Heuristic {
public:
    virtual ~Heuristic() = default;

    virtual double estimate(const SearchState& state,
                           const std::vector<phase1::BendFeature>& allBends) const = 0;

    static std::vector<phase1::BendFeature> getRemainingBends(
        const SearchState& state,
        const std::vector<phase1::BendFeature>& allBends
    ) {
        std::vector<phase1::BendFeature> remaining;
        for (const auto& bend : allBends) {
            if (!state.isBent(bend.id)) {
                remaining.push_back(bend);
            }
        }
        return remaining;
    }
};

/**
 * @brief H1: Rotational Entropy Heuristic
 *
 * Penalizes sequences that leave bends requiring different orientations.
 * Formula: h1 = weight * (distinctOrientations - 1)
 */
class RotationalEntropyHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 0.8;

    explicit RotationalEntropyHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
    BendTimeEstimator m_estimator;
};

/**
 * @brief H2: Tooling Variance Heuristic
 *
 * Penalizes sequences with high variance in required ABA widths.
 * Formula: h2 = weight * stddev(remaining_aba_widths)
 */
class ToolingVarianceHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 0.003;

    explicit ToolingVarianceHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
};

/**
 * @brief H3: Grasp Fragmentation Heuristic
 *
 * Penalizes states where valid grip region is fragmented.
 * Simplified proxy: more remaining bends = higher fragmentation risk.
 * Formula: h3 = weight * (remainingBends / 3)
 */
class GraspFragmentationHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 2.0;

    explicit GraspFragmentationHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
};

/**
 * @brief H4: Remaining Bends Heuristic (Admissible baseline)
 *
 * Guaranteed admissible: h4 = weight * (N - bentCount)
 *
 * Since each bend costs at least baseBendTime (>= 1), the number of
 * remaining bends is always a lower bound on the true remaining cost.
 * This ensures A* optimality.
 *
 * From literature: h(b_i) = N - depth(b_i)
 */
class RemainingBendsHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 1.0;

    explicit RemainingBendsHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
};

/**
 * @brief H5: Envelope Width Heuristic (from paper h1)
 *
 * From paper: "Minimum Width of the envelope rectangle h1(i)"
 *
 * Estimates the minimum bounding width of remaining bend positions.
 * Minimizing this width increases batch processing efficiency by
 * reducing the "opening height" required during backhaul.
 *
 * Formula: h5 = k1 * envelopeWidth(remaining bends)
 * where envelopeWidth = max(pos) - min(pos) along the narrower axis
 */
class EnvelopeWidthHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 0.05;

    explicit EnvelopeWidthHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

    /// Compute envelope width for a set of bends (public for testing)
    static double computeEnvelopeWidth(const std::vector<phase1::BendFeature>& bends);

private:
    double m_weight;
};

/**
 * @brief H6: Triangle Area Heuristic (from paper h2, INVERSE)
 *
 * From paper: "Bends' corresponding triangle areas h2(i)"
 * "Bending angle which corresponds to the largest triangle area
 *  is always the shape defining bends."
 *
 * Paper formula uses k2/h2(i) -- INVERSE relationship.
 * Shape-defining bends (large area) should be unfolded FIRST
 * (= bent LAST), so remaining shape-defining bends = LOW cost.
 *
 * Triangle area approx: 0.5 * bendLength * flangeExtent
 */
class TriangleAreaHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 50.0;

    explicit TriangleAreaHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

    /// Compute triangle area for a bend (public for testing)
    static double computeTriangleArea(const phase1::BendFeature& bend);

private:
    double m_weight;
};

/**
 * @brief Paper-accurate heuristic: h(i) = k1*h1(i) + k2/h2(i) + h4
 *
 * Implements exactly the formula from "A Bending Sequence Planning
 * Algorithm Based on Multiple-Constraint Model":
 *   h(i) = k1 * envelopeWidth + k2 / sumTriangleArea
 *
 * Plus admissible baseline H4 for A* optimality guarantee.
 */
class PaperHeuristic : public Heuristic {
public:
    PaperHeuristic(double k1 = EnvelopeWidthHeuristic::DEFAULT_WEIGHT,
                   double k2 = TriangleAreaHeuristic::DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_k1;
    double m_k2;
    RemainingBendsHeuristic m_admissibleBase;
};

/**
 * @brief Combined heuristic: h = h1 + h2 + h3 + h4
 *
 * Sum of admissible heuristics is admissible.
 * H4 provides the admissible baseline guarantee.
 */
class CombinedHeuristic : public Heuristic {
public:
    CombinedHeuristic();

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    RotationalEntropyHeuristic m_h1;
    ToolingVarianceHeuristic m_h2;
    GraspFragmentationHeuristic m_h3;
    RemainingBendsHeuristic m_h4;
};

} // namespace phase3
} // namespace openpanelcam
