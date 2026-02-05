#pragma once

namespace openpanelcam {

// Temporary mock for Phase 1 types until Phase 1 is fixed
namespace phase1 {
    struct BendFeature {
        int id = -1;
        double angle = 0.0;
        double length = 0.0;

        // Simplified geometry representation for testing
        struct {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        } position;  // Bend line position

        struct {
            double x = 0.0;
            double y = 1.0;  // Default: bend along Y-axis
            double z = 0.0;
        } direction;  // Bend line direction

        struct {
            double x = 0.0;
            double y = 0.0;
            double z = 1.0;
        } normal;  // Bend normal (rotation axis)
    };
}

} // namespace openpanelcam
