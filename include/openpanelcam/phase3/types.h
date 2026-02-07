#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Part orientation on the machine (0, 90, 180, 270 degrees)
 */
enum class Orientation : uint8_t {
    DEG_0 = 0,
    DEG_90 = 1,
    DEG_180 = 2,
    DEG_270 = 3
};

/**
 * @brief Type of action in the sequence
 */
enum class ActionType {
    BEND,           // Perform a bend
    ROTATE,         // Rotate part
    ABA_RECONFIG,   // Reconfigure ABA segments
    REPOSITION      // Reposition (re-grip) part
};

/**
 * @brief Reason for repositioning
 */
enum class RepoReason {
    NONE,
    GRIP_AREA_EXHAUSTED,
    BOX_CLOSING,
    COM_OUTSIDE_GRIP,
    NO_VALID_GRIP
};

/**
 * @brief Convert orientation to degrees
 */
inline double orientationToDegrees(Orientation o) {
    return static_cast<double>(static_cast<uint8_t>(o)) * 90.0;
}

/**
 * @brief Get next orientation (rotate 90 degrees clockwise)
 */
inline Orientation rotateClockwise(Orientation o) {
    return static_cast<Orientation>((static_cast<uint8_t>(o) + 1) % 4);
}

/**
 * @brief Get opposite orientation (rotate 180 degrees)
 */
inline Orientation rotateOpposite(Orientation o) {
    return static_cast<Orientation>((static_cast<uint8_t>(o) + 2) % 4);
}

/**
 * @brief Search state in A* algorithm
 *
 * Encodes the complete state of the bending process:
 * - Which bends are completed (bitmask, max 32 bends)
 * - Current part orientation
 * - Current ABA segment configuration
 * - Current grip center position
 * - Whether repo is needed
 */
struct SearchState {
    uint32_t bentMask = 0;
    Orientation orientation = Orientation::DEG_0;
    uint16_t abaConfig = 0;
    double gripCenterX = 0.0;
    double gripCenterY = 0.0;
    bool needsRepo = false;
    RepoReason repoReason = RepoReason::NONE;

    bool isBent(int bendId) const {
        if (bendId < 0 || bendId >= 32) return false;
        return (bentMask & (1u << bendId)) != 0;
    }

    void markBent(int bendId) {
        if (bendId >= 0 && bendId < 32) {
            bentMask |= (1u << bendId);
        }
    }

    int bentCount() const {
        int count = 0;
        uint32_t mask = bentMask;
        while (mask) {
            count += mask & 1;
            mask >>= 1;
        }
        return count;
    }

    bool isGoal(int totalBends) const {
        if (totalBends <= 0 || totalBends > 32) return false;
        uint32_t goalMask = (1u << totalBends) - 1;
        return bentMask == goalMask;
    }

    bool operator==(const SearchState& other) const {
        return bentMask == other.bentMask &&
               orientation == other.orientation &&
               abaConfig == other.abaConfig;
    }

    bool operator!=(const SearchState& other) const {
        return !(*this == other);
    }
};

} // namespace phase3
} // namespace openpanelcam
