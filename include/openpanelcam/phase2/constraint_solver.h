#pragma once

#include "openpanelcam/phase2/phase1_mock.h"
#include "openpanelcam/phase2/types.h"
#include "openpanelcam/phase2/precedence_dag.h"
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"
#include "openpanelcam/phase2/grasp_constraint_generator.h"
#include "openpanelcam/phase2/aba_constraint_analyzer.h"
#include <vector>

namespace openpanelcam {
namespace phase2 {

// Complete solver output
struct Phase2Output {
    PrecedenceDAG precedenceGraph;
    std::vector<GraspConstraint> graspConstraints;
    std::vector<ABAConstraint> abaConstraints;
    std::vector<int> bendSequence;
    bool success = false;
    std::string analysisSummary;

    // Error tracking
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    // Default constructor
    Phase2Output() = default;

    // Move constructor and assignment (PrecedenceDAG is moveable)
    Phase2Output(Phase2Output&&) = default;
    Phase2Output& operator=(Phase2Output&&) = default;

    // Delete copy constructor (PrecedenceDAG might not be copyable)
    Phase2Output(const Phase2Output&) = delete;
    Phase2Output& operator=(const Phase2Output&) = delete;
};

/**
 * @brief Main constraint solver for Phase 2
 *
 * Orchestrates all constraint analysis modules:
 * - GeometricPrecedenceAnalyzer: Determines bend order from geometry
 * - GraspConstraintGenerator: Validates grip regions for each state
 * - ABAConstraintAnalyzer: Checks tool feasibility for each bend
 *
 * Builds complete precedence graph and generates optimal bend sequence.
 *
 * Usage:
 *   ConstraintSolver solver;
 *   auto output = solver.solve(bends);
 *   if (output.success) {
 *       // Use output.bendSequence for manufacturing
 *   }
 */
class ConstraintSolver {
public:
    ConstraintSolver();
    ~ConstraintSolver() = default;

    /**
     * @brief Solve all constraints for given bends
     *
     * @param bends Vector of bend features from Phase 1
     * @return Phase2Output Complete constraint solution
     */
    Phase2Output solve(const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Get solver statistics
     */
    struct Statistics {
        // Timing
        double totalSolveTimeMs = 0.0;
        double geometricAnalysisTimeMs = 0.0;
        double graspAnalysisTimeMs = 0.0;
        double abaAnalysisTimeMs = 0.0;
        double graphBuildTimeMs = 0.0;
        double topologicalSortTimeMs = 0.0;

        // Counts
        int totalBends = 0;
        int geometricConstraints = 0;
        int graspStatesAnalyzed = 0;
        int abaConstraintsGenerated = 0;
        int totalEdges = 0;

        // Flags
        bool hasInfeasibleBends = false;
        bool hasBoxClosing = false;
        bool hasGraspIssues = false;
    };

    const Statistics& getStatistics() const { return m_stats; }

    /**
     * @brief Reset solver state
     */
    void reset();

private:
    // Module instances
    GeometricPrecedenceAnalyzer m_geometricAnalyzer;
    GraspConstraintGenerator m_graspGenerator;
    ABAConstraintAnalyzer m_abaAnalyzer;

    // Statistics
    Statistics m_stats;

    /**
     * @brief Build precedence graph from geometric analysis
     *
     * @param bends Input bends
     * @param graph Output graph to populate
     */
    void buildPrecedenceGraph(
        const std::vector<phase1::BendFeature>& bends,
        PrecedenceDAG& graph
    );

    /**
     * @brief Generate analysis summary
     *
     * @param output Output structure to summarize
     * @return Human-readable summary string
     */
    std::string generateSummary(const Phase2Output& output);

    /**
     * @brief Check for critical errors
     *
     * @param output Output structure to check
     * @return true if output has critical errors
     */
    bool hasCriticalErrors(const Phase2Output& output);
};

} // namespace phase2
} // namespace openpanelcam
