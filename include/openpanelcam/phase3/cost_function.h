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

    // --- Multiple-Constraint Model weights (K_i) ---
    // Binary constraint penalties from paper:
    //   g(i) = t_bend + max(t_rot, t_aba) + t_repo + Σ(C_s × K_i)
    //   C_s ∈ {0,1} — whether the event occurs
    double toolingChangeWeight = 1.0;      // K1: penalty when tool change needed
    double workpieceTurnoverWeight = 1.5;   // K2: penalty for turnover (180° rotation)
    double gravityLocationWeight = 0.5;     // K3: penalty for unfavorable CoG position

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
 * Implements the Multiple-Constraint Model from literature:
 *   StepCost = t_bend + max(t_rotation, t_aba) + t_repo + Σ(C_s × K_i)
 *
 * Key insight: Rotation and ABA reconfiguration happen in PARALLEL,
 * so we use max() not sum().
 *
 * Constraint weights K_i (binary penalties):
 *   K1: tooling change needed
 *   K2: workpiece turnover (180° rotation)
 *   K3: unfavorable gravity/center-of-gravity position
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

    /// Compute binary constraint penalty Σ(C_s × K_i)
    double constraintPenalty(const SearchState& current,
                             const SearchState& nextState) const;

    const CostConfig& config() const { return m_config; }

private:
    CostConfig m_config;
};

} // namespace phase3
} // namespace openpanelcam
