#pragma once

#include <vector>
#include <string>
#include <array>

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

// Simple 3D point representation to avoid OpenCASCADE dependency in Phase 2
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct PrecedenceNode {
    int id = -1;
    int bendId = -1;
    std::vector<int> predecessors;
    std::vector<int> successors;
    int level = 0;
    bool visited = false;
};

struct PrecedenceEdge {
    int id = -1;
    int fromBend = -1;
    int toBend = -1;
    ConstraintType type = ConstraintType::GEOMETRIC;
    double confidence = 1.0;
    std::string reasoning;
    Point3D conflictPoint;
};

} // namespace phase2
} // namespace openpanelcam
