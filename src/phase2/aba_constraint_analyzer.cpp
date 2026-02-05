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
    // Required ABA Tool Width Calculation
    // ====================================
    //
    // Required width ensures ABA segments can safely perform the bend
    // without tool interference or material collision.
    //
    // Components:
    // 1. Bend length: Actual length of bend line
    // 2. Clearance: Safety margin (angle-dependent)
    //
    // Total = BendLength + Clearance
    //
    // Example:
    // - 100mm bend at 90° → 100 + 10 = 110mm required
    // - 200mm bend at 45° → 200 + 7.5 = 207.5mm required
    // - 150mm bend at 135° → 150 + 12.5 = 162.5mm required

    double clearance = calculateClearance(bend);
    double requiredWidth = bend.length + clearance;

    return requiredWidth;
}

double ABAConstraintAnalyzer::calculateClearance(
    const phase1::BendFeature& bend
) {
    // ABA Tool Clearance Calculation
    // ===============================
    //
    // Clearance is additional width needed beyond bend length for:
    // 1. Tool access and positioning
    // 2. Material spring-back compensation
    // 3. Flange interference avoidance
    // 4. Tool geometry constraints
    //
    // Factors affecting clearance:
    // - Bend angle: Larger angles need more clearance
    // - Material thickness: Thicker material needs more space
    // - Bend radius: Affects flange projection
    // - Tool geometry: Physical constraints of ABA segments
    //
    // Formula derivation:
    // - Base clearance: 10mm (minimum tool access)
    // - Angle factor: |angle| / 90° (normalized 0-2 range)
    // - Total: base × (0.5 + 0.5 × angleFactor)
    //
    // Result range: [5mm, 15mm]
    // - 0° bend: 5mm (minimum, flat state)
    // - 45° bend: 7.5mm (moderate)
    // - 90° bend: 10mm (standard right angle)
    // - 135° bend: 12.5mm (obtuse)
    // - 180° bend: 15mm (maximum, sharp fold)

    // Base clearance (minimum tool access requirement)
    const double BASE_CLEARANCE = 10.0;  // mm

    // Absolute angle for calculation (handle negative bends)
    double absAngle = std::abs(bend.angle);

    // Angle factor: normalized to 90° reference
    // 0° → 0.0, 45° → 0.5, 90° → 1.0, 180° → 2.0
    double angleFactor = absAngle / 90.0;

    // Apply clearance formula
    // Linear interpolation: 50% base at 0°, 100% base at 90°, 150% at 180°
    double clearance = BASE_CLEARANCE * (0.5 + 0.5 * angleFactor);

    // Enforce bounds
    const double MIN_CLEARANCE = 5.0;   // mm (absolute minimum)
    const double MAX_CLEARANCE = 15.0;  // mm (practical maximum)

    if (clearance < MIN_CLEARANCE) {
        clearance = MIN_CLEARANCE;
    }
    if (clearance > MAX_CLEARANCE) {
        clearance = MAX_CLEARANCE;
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
    // Box Closing Detection Algorithm
    // ================================
    //
    // Goal: Detect if bending this bend would trap ABA tool inside a closed box
    //
    // Box closing occurs when:
    // 1. Three bends form U-shape (3-sided box)
    // 2. Fourth bend would close the 4th side
    // 3. Tool cannot escape after bending
    //
    // Detection strategy:
    // - Count bends at ~90° (rectangular box requirement)
    // - Check spatial arrangement (requires full geometry)
    // - Conservative approach: Avoid false positives
    //
    // Current implementation: Conservative heuristic
    // - Real implementation needs 3D geometry analysis
    // - Would check:
    //   * Flange connectivity (shared edges)
    //   * Spatial closure (forms enclosure)
    //   * Tool escape paths
    //
    // Safety philosophy:
    // - False negatives acceptable (miss some box scenarios)
    // - False positives problematic (reject valid bends)
    // - Conservative: Only flag when certain
    //
    // Future enhancement:
    // - Use OCCT TopoDS_Shape for actual geometry
    // - Project to 2D, check polygon closure
    // - Verify tool can exit after bend

    // Count bends at approximately 90° (±5°)
    int count90 = 0;
    for (const auto& b : allBends) {
        if (std::abs(b.angle - 90.0) < 5.0) {
            count90++;
        }
    }

    // Conservative heuristic: Need exactly 4 bends at 90° for box
    // This avoids false positives
    if (count90 < 4) {
        return false;  // Not enough bends for rectangular box
    }

    // If this bend is one of 4 at 90°, might form box
    // But without spatial analysis, we can't be certain

    // Additional check: Is this bend at 90°?
    if (std::abs(bend.angle - 90.0) > 5.0) {
        return false;  // This bend not contributing to rectangular box
    }

    // Conservative decision: Don't flag without spatial analysis
    // Real implementation would:
    // 1. Check if 3 other bends form U-shape
    // 2. Check if this bend aligns with gap
    // 3. Verify tool would be trapped

    // For now, return false (safe default)
    // This avoids rejecting valid bends
    // Operator can manually check if needed

    return false;
}

} // namespace phase2
} // namespace openpanelcam
