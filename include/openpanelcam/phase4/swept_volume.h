#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Simplified swept volume for a bend operation
 *
 * Represents the volume swept by a flange rotating around a bend line.
 * Uses AABB for fast broad-phase and OBB for tighter check.
 */
struct SweptVolume {
    AABB aabb;
    OBB obb;
    int bendId = -1;
    double sweepAngle = 0.0;
    std::vector<AABB> intervalAABBs;  // Angular interval AABBs for fine collision check
};

/**
 * @brief Generates swept volumes from bend geometry
 *
 * Estimates the volume swept by a flange during bending by computing
 * the bounding box of the arc traced by the flange tip.
 */
class SweptVolumeGenerator {
public:
    /**
     * @brief Generate swept volume for a bend
     * @param bend Bend feature with position, direction, angle, length
     * @return SweptVolume with AABB and OBB estimates
     */
    SweptVolume generate(const phase1::BendFeature& bend) const;

    /**
     * @brief Estimate AABB of swept arc
     *
     * The flange rotates around the bend line. The swept arc
     * extends from the initial flat position to the bent position.
     */
    AABB estimateSweptAABB(const phase1::BendFeature& bend) const;

    /**
     * @brief Generate AABBs for angular intervals during bend sweep
     *
     * Paper: 10 intervals if flange length < 100mm, 20 if >= 100mm.
     * Each interval covers (theta/I) degrees of the sweep.
     *
     * @param bend The bend feature
     * @return Vector of I+1 AABBs covering the sweep trajectory
     */
    std::vector<AABB> generateIntervalAABBs(const phase1::BendFeature& bend) const;

    /**
     * @brief Compute number of angular intervals per paper heuristic
     * @param flangeLength Length of the flange being bent
     * @return 10 if length < 100mm, 20 otherwise
     */
    static int computeIntervalCount(double flangeLength);

    /**
     * @brief Estimate AABB at a specific angle during the sweep
     * @param bend The bend feature
     * @param currentAngle The angle at this point in the sweep (degrees)
     * @return AABB at this intermediate position
     */
    AABB estimateAABBAtAngle(const phase1::BendFeature& bend, double currentAngle) const;
};

} // namespace phase4
} // namespace openpanelcam
