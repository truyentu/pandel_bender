#pragma once

namespace openpanelcam {
namespace phase2 {

enum class ConstraintType {
    GEOMETRIC,
    BOX_CLOSING,
    SEQUENTIAL
};

enum class DeadZoneType {
    STANDING_FLANGE,
    SAFETY_MARGIN,
    ABA_INTERFERENCE
};

} // namespace phase2
} // namespace openpanelcam
