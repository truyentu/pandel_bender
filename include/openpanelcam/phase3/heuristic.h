#pragma once

#include "types.h"
#include "cost_function.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <set>
#include <cmath>
#include <numeric>

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
 * @brief Combined heuristic: h = h1 + h2 + h3
 *
 * Sum of admissible heuristics is admissible.
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
};

} // namespace phase3
} // namespace openpanelcam
