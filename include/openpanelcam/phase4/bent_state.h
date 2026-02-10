#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief A bent flange occupying space after bending
 */
struct BentFlange {
    int bendId = -1;
    AABB occupiedVolume;
    OBB occupiedOBB;
    double angle = 0.0;
};

/**
 * @brief Tracks the state of bent flanges as bending progresses
 *
 * Maintains a list of occupied volumes for collision checking.
 * Each time a bend is performed, the resulting flange position
 * is added to the state.
 */
class BentState {
public:
    /**
     * @brief Add a bent flange to the state
     * @param bend The bend feature that was just performed
     */
    void addBentFlange(const phase1::BendFeature& bend);

    /**
     * @brief Add a bent flange with k-factor correction
     *
     * Paper: simple rotation idealization causes false-positive collisions.
     * K-factor accounts for material deformation at bend, shrinking the
     * occupied AABB to avoid over-conservative volumes.
     *
     * @param bend The bend feature
     * @param kFactor K-factor correction (typically 0.3-0.5, 0 = no correction)
     */
    void addBentFlange(const phase1::BendFeature& bend, double kFactor);

    /**
     * @brief Get all bent flanges
     */
    const std::vector<BentFlange>& getBentFlanges() const { return m_flanges; }

    /**
     * @brief Get all occupied AABBs for collision checking
     */
    std::vector<AABB> getAllOccupiedVolumes() const;

    /**
     * @brief Get count of bent flanges
     */
    int count() const { return static_cast<int>(m_flanges.size()); }

    /**
     * @brief Reset to empty state
     */
    void reset();

private:
    std::vector<BentFlange> m_flanges;

    /**
     * @brief Estimate occupied AABB for a bent flange
     */
    AABB estimateOccupiedAABB(const phase1::BendFeature& bend) const;

    /**
     * @brief Estimate AABB with k-factor shrinkage
     */
    AABB estimateOccupiedAABB(const phase1::BendFeature& bend, double kFactor) const;
};

} // namespace phase4
} // namespace openpanelcam
