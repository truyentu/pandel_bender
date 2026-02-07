#include "openpanelcam/phase4/types.h"
#include <cmath>

namespace openpanelcam {
namespace phase4 {

// Helper: dot product of two 3D vectors
static double dot3(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// Helper: cross product
static void cross3(const double* a, const double* b, double* result) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

// Helper: project OBB onto axis and get half-extent
static double projectOBB(const OBB& obb, const double* axis) {
    return std::abs(dot3(axis, &obb.axes[0])) * obb.halfExtentX +
           std::abs(dot3(axis, &obb.axes[3])) * obb.halfExtentY +
           std::abs(dot3(axis, &obb.axes[6])) * obb.halfExtentZ;
}

bool OBB::overlaps(const OBB& other) const {
    // Separating Axis Theorem with 15 axes
    double d[3] = {
        other.centerX - centerX,
        other.centerY - centerY,
        other.centerZ - centerZ
    };

    // Test 3 axes from this OBB
    for (int i = 0; i < 3; i++) {
        const double* axis = &axes[i * 3];
        double dist = std::abs(dot3(d, axis));
        double r1 = projectOBB(*this, axis);
        double r2 = projectOBB(other, axis);
        if (dist > r1 + r2) return false;
    }

    // Test 3 axes from other OBB
    for (int i = 0; i < 3; i++) {
        const double* axis = &other.axes[i * 3];
        double dist = std::abs(dot3(d, axis));
        double r1 = projectOBB(*this, axis);
        double r2 = projectOBB(other, axis);
        if (dist > r1 + r2) return false;
    }

    // Test 9 cross product axes
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double crossAxis[3];
            cross3(&axes[i * 3], &other.axes[j * 3], crossAxis);

            // Skip near-zero cross products (parallel axes)
            double len2 = crossAxis[0]*crossAxis[0] +
                          crossAxis[1]*crossAxis[1] +
                          crossAxis[2]*crossAxis[2];
            if (len2 < 1e-12) continue;

            double dist = std::abs(dot3(d, crossAxis));
            double r1 = projectOBB(*this, crossAxis);
            double r2 = projectOBB(other, crossAxis);
            if (dist > r1 + r2) return false;
        }
    }

    return true; // No separating axis found → overlap
}

} // namespace phase4
} // namespace openpanelcam
