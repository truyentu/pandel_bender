#pragma once

#include "openpanelcam/phase2/phase1_mock.h"
#include "openpanelcam/phase2/types.h"
#include <vector>

namespace openpanelcam {
namespace phase2 {

/**
 * @brief ABA (Adaptive Bending Arm) constraint analyzer
 *
 * Analyzes whether each bend can be performed using ABA tooling.
 * ABA tooling uses multiple adjustable segments to accommodate
 * different bend lengths.
 *
 * Key analysis:
 * 1. Calculate required tool width for each bend
 * 2. Solve subset sum problem to find segment combination
 * 3. Check for box closing scenarios (tool trap)
 * 4. Determine feasibility
 *
 * ABA segment sizes (typical): {25, 50, 75, 100, 125, 150, 175, 200}mm
 */
class ABAConstraintAnalyzer {
public:
    ABAConstraintAnalyzer();

    /**
     * @brief Analyze all bends for ABA tool feasibility
     *
     * @param bends Vector of bend features from Phase 1
     * @return Vector of ABA constraints (one per bend)
     */
    std::vector<ABAConstraint> analyze(
        const std::vector<phase1::BendFeature>& bends
    );

    /**
     * @brief Get analysis statistics
     */
    struct Statistics {
        int totalBendsAnalyzed = 0;      // Number of bends analyzed
        int feasibleCount = 0;            // Bends with valid ABA solution
        int infeasibleCount = 0;          // Bends without solution
        int boxClosingCount = 0;          // Bends that would close box

        // Performance metrics
        double analysisTimeMs = 0.0;      // Total analysis time
        double avgBendTimeMs = 0.0;       // Average time per bend
    };

    const Statistics& getStatistics() const { return m_stats; }

private:
    Statistics m_stats;

    // Available ABA segment sizes (in mm)
    // Standard Salvagnini configuration
    std::vector<int> m_availableSegments = {
        25, 50, 75, 100, 125, 150, 175, 200
    };

    /**
     * @brief Calculate required tool width for a bend
     *
     * Required width = bend length + safety clearance
     *
     * @param bend The bend feature
     * @return Required tool width in mm
     */
    double calculateRequiredWidth(
        const phase1::BendFeature& bend
    );

    /**
     * @brief Solve subset sum problem for segment selection
     *
     * Finds combination of ABA segments that sum to required width.
     * Uses dynamic programming approach.
     *
     * Optimization priorities:
     * 1. Cover target width (must satisfy)
     * 2. Minimize segment count (fewer = faster)
     * 3. Minimize waste (closer to target)
     *
     * @param targetWidth Required tool width
     * @return Vector of segment sizes (empty if no solution)
     */
    std::vector<int> solveSubsetSum(
        double targetWidth
    );

    /**
     * @brief Greedy fallback for very large targets
     *
     * Used when target > 1000mm (DP table too large).
     *
     * @param targetWidth Required tool width
     * @return Vector of segment sizes
     */
    std::vector<int> solveGreedy(
        double targetWidth
    );

    /**
     * @brief Check if bend would create box closing scenario
     *
     * Box closing occurs when:
     * - Bend is part of 4-sided enclosure
     * - Tool would be trapped inside after bending
     *
     * @param bend The bend feature
     * @param allBends All bends in part
     * @return true if box closing detected
     */
    bool isBoxClosing(
        const phase1::BendFeature& bend,
        const std::vector<phase1::BendFeature>& allBends
    );

    /**
     * @brief Calculate safety clearance
     *
     * Clearance depends on:
     * - Bend angle (larger angle → more clearance)
     * - Material thickness
     * - Tool geometry
     *
     * @param bend The bend feature
     * @return Clearance in mm
     */
    double calculateClearance(
        const phase1::BendFeature& bend
    );
};

} // namespace phase2
} // namespace openpanelcam
