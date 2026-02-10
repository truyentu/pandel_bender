#pragma once

#include <openpanelcam/core/types.h>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <string>
#include <vector>

namespace openpanelcam {

struct UnfoldConfig {
    double thickness = 2.0;          // mm
    double defaultKFactor = 0.44;    // DIN 6935 mid-range default
    std::string material = "mild_steel";
    bool useDIN6935 = true;          // Use DIN 6935 K-factor table
    bool useHemEmpirical = true;     // Use empirical values for hems
    double sewingTolerance = 1e-4;   // mm
};

struct BendZoneInfo {
    int bendId = -1;
    double innerRadius = 0.0;    // mm
    double bendAngle = 0.0;      // degrees
    double bendLength = 0.0;     // mm (along bend axis)
    double kFactor = 0.44;
    double bendAllowance = 0.0;  // mm (flat length replacing bend zone)
    double bendDeduction = 0.0;  // mm
};

struct UnfoldResult {
    bool success = false;
    std::string errorMessage;

    TopoDS_Shape flatPattern;        // 2D flat shape
    std::vector<BendZoneInfo> bendZones;
    double totalFlatLength = 0.0;    // mm
    double totalFlatWidth = 0.0;     // mm

    int facesUnfolded = 0;
    int bendsProcessed = 0;
    double unfoldTime = 0.0;         // ms

    std::vector<std::string> warnings;
};

} // namespace openpanelcam
