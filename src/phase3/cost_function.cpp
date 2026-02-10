#include "openpanelcam/phase3/cost_function.h"

namespace openpanelcam {
namespace phase3 {

// ===== BendTimeEstimator =====

BendTimeEstimator::BendTimeEstimator(const CostConfig& config)
    : m_config(config) {}

double BendTimeEstimator::estimate(const phase1::BendFeature& bend, double thickness) const {
    double time = m_config.baseBendTime;

    // Angle factor: scale by ratio to 90 degrees
    double angleFactor = std::abs(bend.angle) / 90.0;
    time *= angleFactor;

    // Thickness factor
    time += thickness * m_config.thicknessMultiplier;

    // Length factor: longer bends take slightly more time
    double lengthFactor = 1.0 + (bend.length / 1000.0) * 0.1;
    time *= lengthFactor;

    return time;
}

Orientation BendTimeEstimator::requiredOrientation(const phase1::BendFeature& bend) const {
    double dx = bend.direction.x;
    double dy = bend.direction.y;

    if (std::abs(dx) > std::abs(dy)) {
        return (dx > 0) ? Orientation::DEG_0 : Orientation::DEG_180;
    } else {
        return (dy > 0) ? Orientation::DEG_90 : Orientation::DEG_270;
    }
}

uint16_t BendTimeEstimator::requiredAbaConfig(const phase1::BendFeature& bend) const {
    double requiredWidth = bend.length + 10.0;
    return static_cast<uint16_t>(requiredWidth);
}

// ===== MaskedTimeCost =====

MaskedTimeCost::MaskedTimeCost(const CostConfig& config)
    : m_config(config) {}

double MaskedTimeCost::stepCost(const SearchState& current,
                                int /*nextBendId*/,
                                const SearchState& nextState,
                                const phase1::BendFeature& bend) const {
    double t_bend = bendTime(bend);
    double t_rot = rotationTime(current.orientation, nextState.orientation);
    double t_aba = abaTime(current.abaConfig, nextState.abaConfig);
    double t_repo = repoTime(nextState.needsRepo);

    // CRITICAL: Parallel operations use max(), not sum()
    double t_parallel = std::max(t_rot, t_aba);

    // Multiple-Constraint Model: add binary constraint penalties
    double penalty = constraintPenalty(current, nextState);

    return t_bend + t_parallel + t_repo + penalty;
}

double MaskedTimeCost::bendTime(const phase1::BendFeature& bend) const {
    double time = m_config.baseBendTime;

    double extraAngle = std::abs(bend.angle) - 90.0;
    if (extraAngle > 0) {
        time += extraAngle * m_config.angleMultiplier;
    }

    return time;
}

double MaskedTimeCost::rotationTime(Orientation from, Orientation to) const {
    if (from == to) return 0.0;

    int fromVal = static_cast<int>(from);
    int toVal = static_cast<int>(to);
    int diff = std::abs(fromVal - toVal);
    int steps = std::min(diff, 4 - diff);

    return steps * m_config.rotationTime;
}

double MaskedTimeCost::abaTime(uint16_t fromConfig, uint16_t toConfig) const {
    if (fromConfig == toConfig) return 0.0;
    return m_config.abaReconfigTime;
}

double MaskedTimeCost::repoTime(bool needsRepo) const {
    if (!needsRepo) return 0.0;
    return m_config.totalRepoTime();
}

double MaskedTimeCost::constraintPenalty(const SearchState& current,
                                          const SearchState& nextState) const {
    double penalty = 0.0;

    // C1: Tooling change — ABA config changed
    bool toolingChanged = (current.abaConfig != nextState.abaConfig);
    if (toolingChanged) {
        penalty += m_config.toolingChangeWeight;
    }

    // C2: Workpiece turnover — 180° rotation
    int fromVal = static_cast<int>(current.orientation);
    int toVal = static_cast<int>(nextState.orientation);
    int diff = std::abs(fromVal - toVal);
    int steps = std::min(diff, 4 - diff);
    bool isTurnover = (steps == 2); // 180° = 2 steps
    if (isTurnover) {
        penalty += m_config.workpieceTurnoverWeight;
    }

    // C3: Gravity/CoG — approximate by checking if many bends done on one side
    // Heuristic: if more than half the bends are done, the part geometry
    // may have shifted CoG unfavorably (simplified binary check)
    int bentCount = nextState.bentCount();
    int prevBentCount = current.bentCount();
    bool crossedHalfway = (prevBentCount <= 16 && bentCount > 16);
    if (crossedHalfway) {
        penalty += m_config.gravityLocationWeight;
    }

    return penalty;
}

} // namespace phase3
} // namespace openpanelcam
