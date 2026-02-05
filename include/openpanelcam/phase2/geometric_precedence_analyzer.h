#pragma once

#include "openpanelcam/phase2/phase1_mock.h"
#include "openpanelcam/phase2/types.h"
#include <vector>
#include <map>

namespace openpanelcam {
namespace phase2 {

/**
 * @brief Bend state simulator - tracks which bends are already bent
 *
 * Used to simulate progressive bending and check for geometric conflicts.
 */
class BentState {
public:
    BentState();

    /**
     * @brief Add a bend to the bent state
     * @param bendId The bend ID to mark as bent
     */
    void applyBend(int bendId);

    /**
     * @brief Check if a bend is already bent
     * @param bendId The bend ID to check
     * @return true if bend is in bent state
     */
    bool isBent(int bendId) const;

    /**
     * @brief Get list of all bent bends
     * @return Vector of bent bend IDs
     */
    const std::vector<int>& getBentBends() const { return m_bentBends; }

    /**
     * @brief Get number of bends in bent state
     */
    int count() const { return static_cast<int>(m_bentBends.size()); }

    /**
     * @brief Reset to initial flat state
     */
    void reset();

private:
    std::vector<int> m_bentBends;  // IDs of bends already bent
};

/**
 * @brief Geometric precedence analyzer
 *
 * Analyzes geometric conflicts between bends to determine precedence constraints.
 * Implements three types of analysis:
 * 1. Corner overlap detection (ray casting)
 * 2. Box closing detection (3-sided box trap)
 * 3. Sequential blocking (access analysis)
 */
class GeometricPrecedenceAnalyzer {
public:
    GeometricPrecedenceAnalyzer();

    /**
     * @brief Analyze all bends for geometric precedence constraints
     * @param bends Vector of bend features from Phase 1
     * @return Vector of precedence edges (constraints)
     */
    std::vector<PrecedenceEdge> analyze(
        const std::vector<phase1::BendFeature>& bends
    );

    /**
     * @brief Check for corner overlap between two bends
     *
     * Simulates bending bi first, then checks if bj's corners would
     * intersect bi's bent flange during bending motion.
     *
     * @param bi First bend to check
     * @param bj Second bend to check
     * @param state Current bent state
     * @return true if bi must bend before bj
     */
    bool checkCornerOverlap(
        const phase1::BendFeature& bi,
        const phase1::BendFeature& bj,
        const BentState& state
    );

    /**
     * @brief Check if bending would create a box-closing scenario
     *
     * Detects if current bent state forms 3 sides of a box and
     * next bend would close the 4th side (tool trap).
     *
     * @param bend Bend to check
     * @param state Current bent state
     * @return true if this bend would close a box
     */
    bool isBoxClosing(
        const phase1::BendFeature& bend,
        const BentState& state
    );

    /**
     * @brief Check if one bend blocks access to another
     *
     * Simulates bending bj first, then checks if bi's bend line
     * is still accessible.
     *
     * @param bi Bend to access
     * @param bj Potential blocker
     * @param state Current bent state
     * @return true if bj blocks access to bi
     */
    bool isBlocked(
        const phase1::BendFeature& bi,
        const phase1::BendFeature& bj,
        const BentState& state
    );

    /**
     * @brief Get analysis statistics
     */
    struct Statistics {
        int totalPairsChecked = 0;
        int cornerOverlapCount = 0;
        int boxClosingCount = 0;
        int sequentialBlockCount = 0;
        int totalConstraints = 0;

        // Performance metrics (Task 15)
        double analysisTimeMs = 0.0;      // Total analysis time
        double avgPairTimeMs = 0.0;       // Average time per pair
        int maxConstraintsPerPair = 0;    // Max constraints from single pair
    };

    const Statistics& getStatistics() const { return m_stats; }

private:
    Statistics m_stats;

    /**
     * @brief Helper: Get 4 corner points of a flange
     *
     * Computes the 4 corners of the flange face based on bend line
     * position, direction, and length.
     *
     * @param bend The bend feature
     * @return Vector of 4 corner points (simplified as Point3D)
     */
    std::vector<Point3D> getFlangeCorners(const phase1::BendFeature& bend);

    /**
     * @brief Helper: Predict motion path during bending
     *
     * Computes the direction vector that a point would move
     * during the bending operation.
     *
     * @param bend The bend being performed
     * @param point Point on the flange
     * @return Motion direction vector (as Point3D)
     */
    Point3D predictMotionPath(
        const phase1::BendFeature& bend,
        const Point3D& point
    );

    /**
     * @brief Helper: Check if ray intersects with flange region
     *
     * Simplified 2D ray-rectangle intersection test.
     * Real implementation would use OCCT ray-face intersection.
     *
     * @param rayOrigin Starting point of ray
     * @param rayDirection Direction of ray
     * @param flangeBend The bend whose flange we're testing against
     * @return true if ray intersects the flange
     */
    bool rayIntersectsFlange(
        const Point3D& rayOrigin,
        const Point3D& rayDirection,
        const phase1::BendFeature& flangeBend
    );

    /**
     * @brief Helper: Check if bent flanges form 3 sides of a box
     *
     * Analyzes the spatial arrangement of bent flanges to detect
     * if they form a U-shape (3 walls of a box).
     *
     * @param bentBends List of already bent bend IDs
     * @param allBends All bend features for lookup
     * @return true if 3 sides of box detected
     */
    bool forms3SidedBox(
        const std::vector<int>& bentBends,
        const std::vector<phase1::BendFeature>& allBends
    );

    /**
     * @brief Helper: Project 2D polygon for box analysis
     *
     * Projects a flange onto the base plane (Z=0) for 2D analysis.
     * Simplified version returns bounding box.
     *
     * @param bend The bend feature
     * @return Polygon2D representing flange projection
     */
    Polygon2D projectTo2D(const phase1::BendFeature& bend);

    /**
     * @brief Helper: Check if next bend would close the 4th side
     *
     * Given a 3-sided box configuration, checks if the next bend
     * would complete the enclosure.
     *
     * @param nextBend The bend to test
     * @param bentBends Already bent bends forming 3 sides
     * @param allBends All bend features
     * @return true if would close box
     */
    bool wouldClose4thSide(
        const phase1::BendFeature& nextBend,
        const std::vector<int>& bentBends,
        const std::vector<phase1::BendFeature>& allBends
    );
};

} // namespace phase2
} // namespace openpanelcam
