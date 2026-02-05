#include "openpanelcam/phase2/constraint_solver.h"
#include <chrono>
#include <sstream>
#include <algorithm>

namespace openpanelcam {
namespace phase2 {

ConstraintSolver::ConstraintSolver() {
    m_stats = Statistics();
}

Phase2Output ConstraintSolver::solve(const std::vector<phase1::BendFeature>& bends) {
    auto startTime = std::chrono::high_resolution_clock::now();

    Phase2Output output;
    output.success = false;

    // Reset statistics
    m_stats = Statistics();
    m_stats.totalBends = static_cast<int>(bends.size());

    // Handle empty input
    if (bends.empty()) {
        output.success = true;
        output.analysisSummary = "No bends to analyze";
        return output;
    }

    try {
        // Step 1: Geometric precedence analysis
        auto geomStart = std::chrono::high_resolution_clock::now();
        buildPrecedenceGraph(bends, output.precedenceGraph);
        auto geomEnd = std::chrono::high_resolution_clock::now();
        m_stats.geometricAnalysisTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
            geomEnd - geomStart
        ).count() / 1000.0;
        m_stats.totalEdges = output.precedenceGraph.edgeCount();

        // Step 2: Grasp constraint analysis
        auto graspStart = std::chrono::high_resolution_clock::now();

        // Generate grasp constraints for key states:
        // 1. Flat state (no bends)
        GraspConstraint flatState = m_graspGenerator.analyze(bends, std::vector<int>());
        output.graspConstraints.push_back(flatState);

        // 2. Each single bend state
        for (const auto& bend : bends) {
            GraspConstraint singleBendState = m_graspGenerator.analyze(
                bends, std::vector<int>{bend.id}
            );
            output.graspConstraints.push_back(singleBendState);
        }

        auto graspEnd = std::chrono::high_resolution_clock::now();
        m_stats.graspAnalysisTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
            graspEnd - graspStart
        ).count() / 1000.0;
        m_stats.graspStatesAnalyzed = static_cast<int>(output.graspConstraints.size());

        // Step 3: ABA constraint analysis
        auto abaStart = std::chrono::high_resolution_clock::now();
        output.abaConstraints = m_abaAnalyzer.analyze(bends);
        auto abaEnd = std::chrono::high_resolution_clock::now();
        m_stats.abaAnalysisTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
            abaEnd - abaStart
        ).count() / 1000.0;
        m_stats.abaConstraintsGenerated = static_cast<int>(output.abaConstraints.size());

        // Step 4: Finalize graph
        auto graphStart = std::chrono::high_resolution_clock::now();
        bool finalizeSuccess = output.precedenceGraph.finalize();
        auto graphEnd = std::chrono::high_resolution_clock::now();
        m_stats.graphBuildTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
            graphEnd - graphStart
        ).count() / 1000.0;

        if (!finalizeSuccess) {
            output.success = false;
            output.errors.push_back("Failed to finalize precedence graph - likely has cycles");
            output.analysisSummary = generateSummary(output);

            // Track timing even for failures
            auto endTime = std::chrono::high_resolution_clock::now();
            m_stats.totalSolveTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
                endTime - startTime
            ).count() / 1000.0;

            return output;
        }

        // Step 5: Topological sort
        auto sortStart = std::chrono::high_resolution_clock::now();
        output.bendSequence = output.precedenceGraph.topologicalSort();
        auto sortEnd = std::chrono::high_resolution_clock::now();
        m_stats.topologicalSortTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
            sortEnd - sortStart
        ).count() / 1000.0;

        // Step 6: Check for errors
        if (hasCriticalErrors(output)) {
            output.success = false;

            // Add specific error messages
            if (!output.precedenceGraph.isFinalized()) {
                output.errors.push_back("Graph not finalized");
            }
            if (!output.precedenceGraph.isAcyclic()) {
                output.errors.push_back("Graph has cycles - no valid bend sequence");
            }
            if (output.bendSequence.empty() && m_stats.totalBends > 0) {
                output.errors.push_back("No valid bend sequence found");
            }
        } else {
            output.success = true;
        }

        // Add warnings for ABA and grasp issues
        for (const auto& aba : output.abaConstraints) {
            if (!aba.feasible) {
                output.warnings.push_back(
                    "Bend " + std::to_string(aba.bendId) + " has no ABA solution"
                );
            }
            if (aba.isBoxClosing) {
                output.warnings.push_back(
                    "Bend " + std::to_string(aba.bendId) + " may cause box closing"
                );
            }
        }

        for (const auto& grasp : output.graspConstraints) {
            if (!grasp.hasValidGrip) {
                output.warnings.push_back(
                    "State " + std::to_string(grasp.stateId) + " has no valid grip region"
                );
            }
        }

        // Step 7: Generate summary
        output.analysisSummary = generateSummary(output);

    } catch (const std::exception& e) {
        output.success = false;
        output.errors.push_back(std::string("Solver exception: ") + e.what());
        output.analysisSummary = "Solver failed: " + std::string(e.what());
    }

    // Track total time
    auto endTime = std::chrono::high_resolution_clock::now();
    m_stats.totalSolveTimeMs = std::chrono::duration_cast<std::chrono::microseconds>(
        endTime - startTime
    ).count() / 1000.0;

    return output;
}

void ConstraintSolver::buildPrecedenceGraph(
    const std::vector<phase1::BendFeature>& bends,
    PrecedenceDAG& graph
) {
    // Add nodes for all bends
    for (const auto& bend : bends) {
        graph.addNode(bend.id);
    }

    // Get geometric precedence edges
    auto edges = m_geometricAnalyzer.analyze(bends);
    m_stats.geometricConstraints = static_cast<int>(edges.size());

    // Add edges to graph
    for (const auto& edge : edges) {
        graph.addEdge(edge.fromBend, edge.toBend, edge.type,
                      edge.confidence, edge.reasoning);
    }
}

std::string ConstraintSolver::generateSummary(const Phase2Output& output) {
    std::ostringstream oss;

    oss << "Phase 2 Constraint Analysis:\n";
    oss << "- Total bends: " << m_stats.totalBends << "\n";
    oss << "- Precedence edges: " << m_stats.totalEdges << "\n";
    oss << "- Grasp states: " << m_stats.graspStatesAnalyzed << "\n";
    oss << "- ABA constraints: " << m_stats.abaConstraintsGenerated << "\n";
    oss << "- Bend sequence length: " << output.bendSequence.size() << "\n";

    if (output.success) {
        oss << "Status: SUCCESS\n";
    } else {
        oss << "Status: FAILED\n";
        if (!output.errors.empty()) {
            oss << "Errors:\n";
            for (const auto& err : output.errors) {
                oss << "  - " << err << "\n";
            }
        }
    }

    if (!output.warnings.empty()) {
        oss << "Warnings:\n";
        for (const auto& warn : output.warnings) {
            oss << "  - " << warn << "\n";
        }
    }

    oss << "Timing:\n";
    oss << "  - Total: " << m_stats.totalSolveTimeMs << " ms\n";
    oss << "  - Geometric: " << m_stats.geometricAnalysisTimeMs << " ms\n";
    oss << "  - Grasp: " << m_stats.graspAnalysisTimeMs << " ms\n";
    oss << "  - ABA: " << m_stats.abaAnalysisTimeMs << " ms\n";

    return oss.str();
}

bool ConstraintSolver::hasCriticalErrors(const Phase2Output& output) {
    bool hasCritical = false;

    // Check for graph issues
    if (!output.precedenceGraph.isFinalized()) {
        hasCritical = true;
    }

    // Need to cast away const to call isAcyclic() (it modifies visited state)
    if (!const_cast<PrecedenceDAG&>(output.precedenceGraph).isAcyclic()) {
        hasCritical = true;
    }

    // Check for empty sequence
    if (output.bendSequence.empty() && m_stats.totalBends > 0) {
        hasCritical = true;
    }

    // Check for ABA infeasibility
    for (const auto& aba : output.abaConstraints) {
        if (!aba.feasible) {
            m_stats.hasInfeasibleBends = true;
        }
        if (aba.isBoxClosing) {
            m_stats.hasBoxClosing = true;
        }
    }

    // Check for grasp issues
    for (const auto& grasp : output.graspConstraints) {
        if (!grasp.hasValidGrip) {
            m_stats.hasGraspIssues = true;
        }
    }

    return hasCritical;
}

void ConstraintSolver::reset() {
    m_stats = Statistics();
}

} // namespace phase2
} // namespace openpanelcam
