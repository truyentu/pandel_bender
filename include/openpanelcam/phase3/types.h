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

} // namespace phase3
} // namespace openpanelcam
