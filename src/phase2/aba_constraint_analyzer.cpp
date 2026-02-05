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
    // Dynamic Programming Subset Sum Solver
    // ======================================
    //
    // Goal: Find optimal combination of ABA segments to cover target width
    //
    // Optimization criteria (priority order):
    // 1. Must cover target width (totalWidth >= target)
    // 2. Minimize number of segments (fewer segments = faster setup)
    // 3. Minimize waste (totalWidth - target)
    //
    // Algorithm: DP with backtracking
    // State: dp[w] = minimum segments to achieve width w
    // Transition: dp[w] = min(dp[w], dp[w - segment] + 1)
    //
    // Complexity: O(target × segments) where segments = 8

    // Round target to integer for DP table
    int target = static_cast<int>(std::ceil(targetWidth));

    // Limit table size for performance (max 1000mm)
    const int MAX_WIDTH = 1000;
    if (target > MAX_WIDTH) {
        // For very large targets, use greedy as fallback
        return solveGreedy(targetWidth);
    }

    // DP table: dp[w] = minimum segments to achieve exactly width w
    // Initialize with large value (impossible)
    const int INF = 999999;
    std::vector<int> dp(target + 100, INF);  // Extra space for overage
    std::vector<int> parent(target + 100, -1);  // For backtracking

    // Base case: 0 width requires 0 segments
    dp[0] = 0;

    // Fill DP table
    for (int w = 0; w <= target + 50; w++) {
        if (dp[w] == INF) continue;  // Unreachable state

        // Try adding each segment
        for (int segment : m_availableSegments) {
            int newWidth = w + segment;
            if (newWidth > target + 50) continue;  // Don't go too far over

            // Update if this path uses fewer segments
            if (dp[w] + 1 < dp[newWidth]) {
                dp[newWidth] = dp[w] + 1;
                parent[newWidth] = w;  // Store previous width for backtracking
            }
        }
    }

    // Find best solution: smallest width >= target with minimum segments
    int bestWidth = -1;
    int minSegments = INF;
    int minWaste = INF;

    for (int w = target; w <= target + 50; w++) {
        if (dp[w] == INF) continue;

        int segments = dp[w];
        int waste = w - target;

        // Prioritize: fewer segments, then less waste
        if (segments < minSegments ||
            (segments == minSegments && waste < minWaste)) {
            bestWidth = w;
            minSegments = segments;
            minWaste = waste;
        }
    }

    // No solution found
    if (bestWidth == -1) {
        return std::vector<int>();
    }

    // Backtrack to reconstruct solution
    std::vector<int> solution;
    int currentWidth = bestWidth;

    while (currentWidth > 0) {
        int prevWidth = parent[currentWidth];
        if (prevWidth == -1) break;

        int segment = currentWidth - prevWidth;
        solution.push_back(segment);

        currentWidth = prevWidth;
    }

    // Sort solution for consistency (largest first)
    std::sort(solution.rbegin(), solution.rend());

    return solution;
}

std::vector<int> ABAConstraintAnalyzer::solveGreedy(
    double targetWidth
) {
    // Greedy fallback for large targets
    // (Extracted from old implementation)

    std::vector<int> solution;

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
            break;
        }
    }

    // If still short, add smallest segment
    if (remaining > 0 && !segments.empty()) {
        solution.push_back(segments.back());
    }

    return solution;
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
