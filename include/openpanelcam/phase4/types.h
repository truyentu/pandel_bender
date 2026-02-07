#pragma once

#include <vector>
#include <string>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief 3D axis-aligned bounding box for collision detection
 */
struct AABB {
    double minX = 0, minY = 0, minZ = 0;
    double maxX = 0, maxY = 0, maxZ = 0;

    AABB() = default;
    AABB(double minX_, double minY_, double minZ_,
         double maxX_, double maxY_, double maxZ_)
        : minX(minX_), minY(minY_), minZ(minZ_)
        , maxX(maxX_), maxY(maxY_), maxZ(maxZ_) {}

    bool overlaps(const AABB& other) const {
        return minX <= other.maxX && maxX >= other.minX &&
               minY <= other.maxY && maxY >= other.minY &&
               minZ <= other.maxZ && maxZ >= other.minZ;
    }

    double volume() const {
        double dx = maxX - minX;
        double dy = maxY - minY;
        double dz = maxZ - minZ;
        if (dx < 0 || dy < 0 || dz < 0) return 0.0;
        return dx * dy * dz;
    }

    AABB expand(double margin) const {
        return AABB(minX - margin, minY - margin, minZ - margin,
                    maxX + margin, maxY + margin, maxZ + margin);
    }

    static AABB merge(const AABB& a, const AABB& b) {
        return AABB(std::min(a.minX, b.minX), std::min(a.minY, b.minY),
                    std::min(a.minZ, b.minZ), std::max(a.maxX, b.maxX),
                    std::max(a.maxY, b.maxY), std::max(a.maxZ, b.maxZ));
    }
};

/**
 * @brief Oriented bounding box for tighter collision detection
 *
 * Uses Separating Axis Theorem (SAT) for overlap test.
 * axes[0..2] = X axis, axes[3..5] = Y axis, axes[6..8] = Z axis
 */
struct OBB {
    double centerX = 0, centerY = 0, centerZ = 0;
    double halfExtentX = 0, halfExtentY = 0, halfExtentZ = 0;
    double axes[9] = {1,0,0, 0,1,0, 0,0,1}; // 3 axis vectors (row-major)

    OBB() = default;

    bool overlaps(const OBB& other) const;
};

/**
 * @brief Result of collision check for one step
 */
enum class CollisionType {
    NONE,
    SWEPT_VS_FIXED,
    TOOL_VS_PART,
    PART_VS_MACHINE
};

struct CollisionResult {
    bool hasCollision = false;
    CollisionType type = CollisionType::NONE;
    int bendId = -1;
    int collidingBendId = -1;
    double penetrationDepth = 0.0;
    std::string description;
};

/**
 * @brief Grasp physics validation result
 */
struct GraspValidation {
    bool areaValid = true;
    bool comValid = true;
    bool torqueValid = true;
    bool shearValid = true;

    double gripArea = 0.0;
    double comDistance = 0.0;
    double torque = 0.0;
    double shearForce = 0.0;

    bool isValid() const {
        return areaValid && comValid && torqueValid && shearValid;
    }

    std::string describe() const {
        if (isValid()) return "Grasp: VALID";
        std::string desc = "Grasp: FAILED -";
        if (!areaValid) desc += " area(" + std::to_string(gripArea) + "mm2)";
        if (!comValid) desc += " COM(" + std::to_string(comDistance) + "mm)";
        if (!torqueValid) desc += " torque(" + std::to_string(torque) + "Nm)";
        if (!shearValid) desc += " shear(" + std::to_string(shearForce) + "N)";
        return desc;
    }
};

/**
 * @brief Tool extraction check result
 */
struct ExtractionResult {
    bool canExtract = true;
    int trappedAtBend = -1;
    std::string description;
};

/**
 * @brief Springback compensation data
 */
struct SpringbackData {
    int bendId = -1;
    double targetAngle = 0.0;
    double compensatedAngle = 0.0;
    double springbackAngle = 0.0;
};

/**
 * @brief Single step validation result
 */
struct StepValidation {
    int stepIndex = -1;
    int bendId = -1;

    CollisionResult collision;
    GraspValidation grasp;
    ExtractionResult extraction;
    SpringbackData springback;

    bool isValid() const {
        return !collision.hasCollision &&
               grasp.isValid() &&
               extraction.canExtract;
    }

    std::string describe() const {
        std::string desc = "Step " + std::to_string(stepIndex) +
                           " (bend " + std::to_string(bendId) + "): ";
        if (isValid()) {
            desc += "VALID";
        } else {
            desc += "INVALID";
            if (collision.hasCollision) desc += " [collision]";
            if (!grasp.isValid()) desc += " [grasp]";
            if (!extraction.canExtract) desc += " [extraction]";
        }
        return desc;
    }
};

/**
 * @brief Complete Phase 4 output
 */
struct Phase4Output {
    bool success = false;
    bool allStepsValid = false;

    std::vector<StepValidation> stepResults;
    std::vector<SpringbackData> springbackTable;

    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::vector<std::string> suggestions;

    double validationTimeMs = 0.0;
    int collisionsDetected = 0;
    int graspFailures = 0;
    int extractionFailures = 0;

    std::string generateSummary() const {
        std::string s = "Phase 4 Validation Result:\n";
        s += "  Status: " + std::string(success ? "PASS" : "FAIL") + "\n";
        s += "  Steps: " + std::to_string(stepResults.size()) + "\n";
        s += "  Collisions: " + std::to_string(collisionsDetected) + "\n";
        s += "  Grasp Failures: " + std::to_string(graspFailures) + "\n";
        s += "  Extraction Failures: " + std::to_string(extractionFailures) + "\n";
        s += "  Validation Time: " + std::to_string(validationTimeMs) + " ms\n";
        return s;
    }
};

/**
 * @brief Configuration for physics validator
 */
struct ValidatorConfig {
    double minGripArea = 100.0;
    double maxComDistance = 50.0;
    double maxTorque = 10.0;
    double maxShearForce = 50.0;
    double frictionCoeff = 0.3;

    double collisionMargin = 2.0;
    bool useOBB = true;

    double materialDensity = 7.85e-6;
    double materialThickness = 1.5;
    double yieldStrength = 200.0;

    bool enableSpringback = true;
    double springbackBaseDeg = 2.0;
    double springbackPerMm = 0.5;
};

} // namespace phase4
} // namespace openpanelcam
