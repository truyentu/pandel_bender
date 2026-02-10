#pragma once

#include "types.h"
#include "swept_volume.h"
#include "bent_state.h"
#include "collision_detector.h"
#include "../phase2/phase1_mock.h"
#include <vector>
#include <set>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief Result of negative sequence analysis
 */
struct NegativeSequenceResult {
    bool success = false;
    std::vector<int> reversedSequence;   // Order of bends found by unfolding
    std::vector<int> forwardSequence;    // Reversed → actual bending order
    std::vector<int> infeasibleBends;    // Bends that could not be unfolded
    int iterationsUsed = 0;
};

/**
 * @brief Negative Sequence Analyzer (Algorithm 2 from paper)
 *
 * Implements the reverse traversal approach from
 * "Bending Simulation Framework for Rapid Feasibility Checks":
 *
 * Start with fully folded workpiece, iteratively find feasible bends
 * to unfold. Rejection of nodes is more likely early in this reverse
 * search, allowing efficient pruning.
 *
 * procedure FindSequence(S, B, alpha)
 *   repeat
 *     Gamma <- all feasible bends for current state
 *     if Gamma empty: stop
 *     S <- Unfold(Gamma, alpha)
 *     sequence <- Gamma
 *   until stop
 */
class NegativeSequenceAnalyzer {
public:
    explicit NegativeSequenceAnalyzer(const ValidatorConfig& config = ValidatorConfig());

    /**
     * @brief Find a feasible bending sequence using reverse analysis
     *
     * Algorithm 2 from paper: starts from fully folded state,
     * finds bends that can be unfolded without collision.
     *
     * @param bends All bend features
     * @return NegativeSequenceResult with forward sequence
     */
    NegativeSequenceResult analyze(const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Find sequence with tool collision checking
     * @param bends All bend features
     * @param tools Tool geometries for tool-vs-part collision
     * @return NegativeSequenceResult with forward sequence
     */
    NegativeSequenceResult analyze(const std::vector<phase1::BendFeature>& bends,
                                   const std::vector<ToolGeometry>& tools);

    /**
     * @brief Check if a single bend can be unfolded in current state
     *
     * IsBendFeasible from paper: checks for self-intersection
     * and tool clashes at angular intervals during unfold.
     *
     * @param bend The bend to check
     * @param bentState Current bent state (with remaining folded bends)
     * @return true if bend can be unfolded without collision
     */
    bool isBendFeasible(const phase1::BendFeature& bend,
                        const BentState& bentState) const;

    /**
     * @brief Check feasibility including tool collision
     * @param bend The bend to check
     * @param bentState Current bent state
     * @param tools Tool geometries to check against
     * @return true if no self-intersection AND no tool clash
     */
    bool isBendFeasible(const phase1::BendFeature& bend,
                        const BentState& bentState,
                        const std::vector<ToolGeometry>& tools) const;

private:
    ValidatorConfig m_config;
    CollisionDetector m_detector;
    SweptVolumeGenerator m_sweptGen;

    /**
     * @brief Check if swept volume collides with any tool
     */
    bool checkToolCollision(const SweptVolume& swept,
                            const std::vector<ToolGeometry>& tools) const;
};

} // namespace phase4
} // namespace openpanelcam
