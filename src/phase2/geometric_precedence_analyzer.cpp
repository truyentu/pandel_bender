#include "openpanelcam/phase2/geometric_precedence_analyzer.h"
#include <algorithm>

namespace openpanelcam {
namespace phase2 {

//==============================================================================
// BentState Implementation
//==============================================================================

BentState::BentState() {
}

void BentState::applyBend(int bendId) {
    // Check if already bent
    auto it = std::find(m_bentBends.begin(), m_bentBends.end(), bendId);
    if (it == m_bentBends.end()) {
        m_bentBends.push_back(bendId);
    }
}

bool BentState::isBent(int bendId) const {
    return std::find(m_bentBends.begin(), m_bentBends.end(), bendId) != m_bentBends.end();
}

void BentState::reset() {
    m_bentBends.clear();
}

//==============================================================================
// GeometricPrecedenceAnalyzer Implementation
//==============================================================================

GeometricPrecedenceAnalyzer::GeometricPrecedenceAnalyzer() {
    m_stats = Statistics();
}

std::vector<PrecedenceEdge> GeometricPrecedenceAnalyzer::analyze(
    const std::vector<phase1::BendFeature>& bends
) {
    std::vector<PrecedenceEdge> constraints;

    if (bends.empty()) {
        return constraints;
    }

    // Reset statistics
    m_stats = Statistics();

    // For each pair of bends (bi, bj)
    for (size_t i = 0; i < bends.size(); i++) {
        for (size_t j = 0; j < bends.size(); j++) {
            if (i == j) continue;  // Skip self

            m_stats.totalPairsChecked++;

            const auto& bi = bends[i];
            const auto& bj = bends[j];

            // Create bent state for testing
            BentState state;

            // Type 1: Check corner overlap
            // If bi bending would cause corners to overlap bj's flange
            // then bi must bend before bj
            if (checkCornerOverlap(bi, bj, state)) {
                PrecedenceEdge edge;
                edge.id = static_cast<int>(constraints.size());
                edge.fromBend = bi.id;
                edge.toBend = bj.id;
                edge.type = ConstraintType::GEOMETRIC;
                edge.confidence = 1.0;
                edge.reasoning = "Corner overlap detected";

                constraints.push_back(edge);
                m_stats.cornerOverlapCount++;
                m_stats.totalConstraints++;
            }

            // Type 2: Check if bending would close a box
            // Note: Box closing is checked per bend, not per pair
            // We'll handle this in a separate loop
        }
    }

    // Type 2: Box closing check (per bend)
    for (const auto& bend : bends) {
        // Create state with all other bends
        BentState state;
        for (const auto& otherBend : bends) {
            if (otherBend.id != bend.id) {
                state.applyBend(otherBend.id);
            }
        }

        if (isBoxClosing(bend, state)) {
            // This bend would close a box - mark as constraint
            // (In real implementation, this would create multiple constraints)
            m_stats.boxClosingCount++;
        }
    }

    // Type 3: Sequential blocking
    // Already partially covered in corner overlap check

    return constraints;
}

bool GeometricPrecedenceAnalyzer::checkCornerOverlap(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
) {
    // Simplified implementation for now
    // Real implementation would:
    // 1. Simulate bending bi
    // 2. Get corner points of bi's flange
    // 3. Cast rays along motion path
    // 4. Check intersection with bj's flange

    // For now, return false (no overlap)
    // This will be implemented in detail later with actual geometry
    return false;
}

bool GeometricPrecedenceAnalyzer::isBoxClosing(
    const phase1::BendFeature& bend,
    const BentState& state
) {
    // Simplified implementation
    // Real implementation would:
    // 1. Project all bent flanges to 2D
    // 2. Check if they form 3 sides of a box
    // 3. Check if next bend would close 4th side

    // For now, return false (no box closing)
    return false;
}

bool GeometricPrecedenceAnalyzer::isBlocked(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
) {
    // Simplified implementation
    // Real implementation would:
    // 1. Simulate bending bj
    // 2. Check if bi's bend line is still accessible

    // For now, return false (not blocked)
    return false;
}

} // namespace phase2
} // namespace openpanelcam
