#include "openpanelcam/phase2/aba_constraint_analyzer.h"
#include <chrono>
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase2 {

ABAConstraintAnalyzer::ABAConstraintAnalyzer() {
    m_stats = Statistics();
}

std::vector<ABAConstraint> ABAConstraintAnalyzer::analyze(
    const std::vector<phase1::BendFeature>& bends
) {
    auto startTime = std::chrono::high_resolution_clock::now();

    std::vector<ABAConstraint> constraints;

    // Handle empty case
    if (bends.empty()) {
        return constraints;
    }

    // Analyze each bend independently
    for (const auto& bend : bends) {
        ABAConstraint constraint;

        constraint.bendId = bend.id;
        constraint.bendLength = bend.length;

        // Step 1: Calculate required tool width
        constraint.requiredWidth = calculateRequiredWidth(bend);

        // Step 2: Calculate clearance
        constraint.clearance = calculateClearance(bend);

        // Step 3: Solve subset sum to find segment combination
        constraint.segmentSolution = solveSubsetSum(constraint.requiredWidth);

        // Step 4: Determine feasibility
        if (!constraint.segmentSolution.empty()) {
            constraint.feasible = true;
            constraint.totalSegments = static_cast<int>(constraint.segmentSolution.size());

            // Calculate total width from segments
            constraint.totalWidth = 0.0;
            for (int segment : constraint.segmentSolution) {
                constraint.totalWidth += segment;
            }

            constraint.reason = "ABA segments found";
            m_stats.feasibleCount++;
        } else {
            constraint.feasible = false;
            constraint.totalSegments = 0;
            constraint.totalWidth = 0.0;
            constraint.reason = "No segment combination covers required width";
            m_stats.infeasibleCount++;
        }

        // Step 5: Check for box closing
        constraint.isBoxClosing = isBoxClosing(bend, bends);
        if (constraint.isBoxClosing) {
            m_stats.boxClosingCount++;
            constraint.reason += " (Warning: Box closing scenario)";
        }

        constraints.push_back(constraint);
        m_stats.totalBendsAnalyzed++;
    }

    // Track timing
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        endTime - startTime
    );
    m_stats.analysisTimeMs = duration.count() / 1000.0;

    if (m_stats.totalBendsAnalyzed > 0) {
        m_stats.avgBendTimeMs = m_stats.analysisTimeMs / m_stats.totalBendsAnalyzed;
    }

    return constraints;
}

double ABAConstraintAnalyzer::calculateRequiredWidth(
    const phase1::BendFeature& bend
) {
    // Required width = bend length + clearance
    double clearance = calculateClearance(bend);
    return bend.length + clearance;
}

double ABAConstraintAnalyzer::calculateClearance(
    const phase1::BendFeature& bend
) {
    // Clearance calculation based on bend angle
    // Larger angles require more clearance for tool access

    // Base clearance
    double baseClearance = 10.0;  // mm

    // Angle factor (90° bend needs more clearance than 45°)
    double angleFactor = std::abs(bend.angle) / 90.0;

    // Total clearance
    double clearance = baseClearance * (0.5 + 0.5 * angleFactor);

    // Minimum clearance
    if (clearance < 5.0) {
        clearance = 5.0;
    }

    return clearance;
}

std::vector<int> ABAConstraintAnalyzer::solveSubsetSum(
    double targetWidth
) {
    // Subset sum problem: Find combination of segments that sum to target
    // This is a simplified greedy approach
    // Real implementation would use dynamic programming

    std::vector<int> solution;

    // Round target to integer for segment matching
    int target = static_cast<int>(std::ceil(targetWidth));

    // Greedy approach: Use largest segments first
    std::vector<int> segments = m_availableSegments;
    std::sort(segments.rbegin(), segments.rend());  // Sort descending

    int remaining = target;

    for (int segment : segments) {
        while (remaining >= segment) {
            solution.push_back(segment);
            remaining -= segment;
        }

        if (remaining == 0) {
            break;  // Perfect match found
        }
    }

    // Check if we covered the target
    int totalCovered = 0;
    for (int seg : solution) {
        totalCovered += seg;
    }

    if (totalCovered < target) {
        // Need to add smallest segment to reach target
        if (!segments.empty()) {
            solution.push_back(segments.back());  // Add smallest
        }
    }

    // Verify solution covers target
    totalCovered = 0;
    for (int seg : solution) {
        totalCovered += seg;
    }

    if (totalCovered >= target) {
        return solution;
    }

    // No solution found
    return std::vector<int>();
}

bool ABAConstraintAnalyzer::isBoxClosing(
    const phase1::BendFeature& bend,
    const std::vector<phase1::BendFeature>& allBends
) {
    // Simplified box closing detection
    // Real implementation would check spatial arrangement

    // Heuristic: If we have 4+ bends at 90°, potential box closing
    int count90 = 0;
    for (const auto& b : allBends) {
        if (std::abs(b.angle - 90.0) < 5.0) {
            count90++;
        }
    }

    // If this bend is one of 4 bends at 90°, might form box
    if (count90 >= 4 && std::abs(bend.angle - 90.0) < 5.0) {
        // Conservative: flag as potential box closing
        // Real implementation would check geometric arrangement
        return false;  // Conservative: don't flag without spatial analysis
    }

    return false;
}

} // namespace phase2
} // namespace openpanelcam
