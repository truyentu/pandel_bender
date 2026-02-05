#pragma once

#include "openpanelcam/phase2/phase1_mock.h"
#include "openpanelcam/phase2/types.h"
#include <vector>

namespace openpanelcam {
namespace phase2 {

/**
 * @brief Grasp constraint generator
 *
 * Analyzes bent part states to determine valid grip regions and dead zones.
 * Implements:
 * 1. Dead zone calculation from standing flanges
 * 2. 2D projection of 3D geometry
 * 3. Valid region calculation (polygon subtraction)
 * 4. Maximum Inscribed Rectangle (MIR) algorithm
 * 5. Grip physics validation (COM, stability, friction)
 */
class GraspConstraintGenerator {
public:
    GraspConstraintGenerator();

    /**
     * @brief Analyze bent state to generate grasp constraint
     *
     * @param bends All bend features from Phase 1
     * @param bentBends List of bend IDs that are already bent
     * @return GraspConstraint for this bent state
     */
    GraspConstraint analyze(
        const std::vector<phase1::BendFeature>& bends,
        const std::vector<int>& bentBends
    );

    /**
     * @brief Get analysis statistics
     */
    struct Statistics {
        int totalStatesAnalyzed = 0;       // Number of states analyzed
        int totalDeadZonesGenerated = 0;   // Total dead zones created
        int validGripCount = 0;             // States with valid grip
        int invalidGripCount = 0;           // States with no valid grip

        // Performance metrics
        double analysisTimeMs = 0.0;        // Total analysis time
        double avgStateTimeMs = 0.0;        // Average time per state
    };

    const Statistics& getStatistics() const { return m_stats; }

private:
    Statistics m_stats;

    /**
     * @brief Calculate dead zones for current bent state
     *
     * For each bent bend, creates dead zone from standing flange.
     *
     * @param bends All bend features
     * @param bentBends List of bent bend IDs
     * @return Vector of dead zones
     */
    std::vector<DeadZone> calculateDeadZones(
        const std::vector<phase1::BendFeature>& bends,
        const std::vector<int>& bentBends
    );

    /**
     * @brief Project 3D flange to 2D base plane
     *
     * Projects the standing flange (after bending) onto Z=0 plane.
     *
     * @param bend The bend feature
     * @return Polygon2D representing flange footprint
     */
    Polygon2D projectFlangeToBasePlane(
        const phase1::BendFeature& bend
    );

    /**
     * @brief Calculate valid grip region
     *
     * Starts with sheet bounding box, subtracts all dead zones.
     *
     * @param sheetSize Sheet dimensions (width, height)
     * @param deadZones All dead zones to subtract
     * @return Polygon2D representing valid grip area
     */
    Polygon2D calculateValidRegion(
        const Point2D& sheetSize,
        const std::vector<DeadZone>& deadZones
    );

    /**
     * @brief Find Maximum Inscribed Rectangle (MIR)
     *
     * Finds largest axis-aligned rectangle that fits in valid region.
     *
     * Current implementation: Bounding box with inset
     * - Fast O(n) where n = vertices in valid region
     * - Optimal for axis-aligned rectangular valid regions
     * - Good approximation for convex polygons
     *
     * Future enhancement: Rotating calipers algorithm
     * - Would handle arbitrary convex polygons
     * - Could find oriented rectangles (not axis-aligned)
     * - Complexity: O(n log n)
     *
     * @param validRegion The valid grip polygon
     * @return Rectangle2D representing MIR (strictly contained)
     */
    Rectangle2D findMaxInscribedRect(
        const Polygon2D& validRegion
    );

    /**
     * @brief Validate grip physics
     *
     * Checks:
     * - Center of mass within grip region
     * - Sufficient friction (area > 100mm²)
     * - Stability (no excessive moment)
     *
     * @param gripRect The proposed grip rectangle
     * @param bends All bends
     * @param bentBends Bent bend IDs
     * @return true if grip is physically valid
     */
    bool validateGripPhysics(
        const Rectangle2D& gripRect,
        const std::vector<phase1::BendFeature>& bends,
        const std::vector<int>& bentBends
    );

    /**
     * @brief Calculate center of mass for bent part
     *
     * Simplified calculation assuming uniform material.
     *
     * @param bends All bends
     * @param bentBends Bent bend IDs
     * @return Point2D representing COM projection
     */
    Point2D calculateCenterOfMass(
        const std::vector<phase1::BendFeature>& bends,
        const std::vector<int>& bentBends
    );

    /**
     * @brief Expand polygon by safety margin
     *
     * Creates buffer zone around polygon by offsetting edges outward.
     * Simplified implementation uses bounding box expansion.
     *
     * @param poly Original polygon
     * @param margin Expansion distance (mm)
     * @return Expanded polygon
     */
    Polygon2D expandPolygon(
        const Polygon2D& poly,
        double margin
    );
};

} // namespace phase2
} // namespace openpanelcam
