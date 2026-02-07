#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Configuration for cost function timing
 */
struct CostConfig {
    // Bend time parameters
    double baseBendTime = 2.0;        // Base time for 90 deg bend (seconds)
    double angleMultiplier = 0.01;    // Additional time per degree beyond 90
    double thicknessMultiplier = 0.5; // Additional time per mm thickness

    // Parallel operation times
    double rotationTime = 1.5;        // Time for 90 deg rotation
    double abaReconfigTime = 0.8;     // Time for ABA segment change

    // Repo times
    double repoReleaseTime = 2.0;
    double repoReorientTime = 1.5;
    double repoAcquireTime = 2.0;
    double repoSafetyMargin = 1.0;

    double totalRepoTime() const {
        return repoReleaseTime + repoReorientTime + repoAcquireTime + repoSafetyMargin;
    }
};

/**
 * @brief Estimates bend execution time based on geometry
 */
class BendTimeEstimator {
public:
    explicit BendTimeEstimator(const CostConfig& config = CostConfig());

    double estimate(const phase1::BendFeature& bend, double thickness = 1.5) const;

    Orientation requiredOrientation(const phase1::BendFeature& bend) const;

    uint16_t requiredAbaConfig(const phase1::BendFeature& bend) const;

private:
    CostConfig m_config;
};

/**
 * @brief Masked Time cost function for A* search
 *
 * Implements: StepCost = t_bend + max(t_rotation, t_aba) + t_repo
 *
 * Key insight: Rotation and ABA reconfiguration happen in PARALLEL,
 * so we use max() not sum().
 */
class MaskedTimeCost {
public:
    explicit MaskedTimeCost(const CostConfig& config = CostConfig());

    double stepCost(const SearchState& current,
                    int nextBendId,
                    const SearchState& nextState,
                    const phase1::BendFeature& bend) const;

    double bendTime(const phase1::BendFeature& bend) const;

    double rotationTime(Orientation from, Orientation to) const;

    double abaTime(uint16_t fromConfig, uint16_t toConfig) const;

    double repoTime(bool needsRepo) const;

    const CostConfig& config() const { return m_config; }

private:
    CostConfig m_config;
};

} // namespace phase3
} // namespace openpanelcam
