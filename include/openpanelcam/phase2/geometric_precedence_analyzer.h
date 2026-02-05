#pragma once

#include "openpanelcam/phase2/types.h"
// TODO: Re-enable when Phase 1 is fixed
// #include "openpanelcam/phase1/types.h"
#include <vector>
#include <map>

namespace openpanelcam {

// Temporary mock for Phase 1 types until Phase 1 is fixed
namespace phase1 {
    struct BendFeature {
        int id = -1;
        double angle = 0.0;
        double length = 0.0;
    };
}

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
    };

    const Statistics& getStatistics() const { return m_stats; }

private:
    Statistics m_stats;
};

} // namespace phase2
} // namespace openpanelcam
