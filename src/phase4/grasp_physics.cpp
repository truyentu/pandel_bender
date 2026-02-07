#include "openpanelcam/phase4/grasp_physics.h"
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

GraspPhysicsEngine::GraspPhysicsEngine(const ValidatorConfig& config)
    : m_config(config) {}

GraspValidation GraspPhysicsEngine::validate(
    const BentState& state,
    const phase1::BendFeature& nextBend
) const {
    GraspValidation result;

    // Compute each physics metric
    result.gripArea = computeGripArea(state, nextBend);
    result.comDistance = computeComDistance(state, nextBend);
    result.torque = computeTorque(state, nextBend);
    result.shearForce = computeShearForce(nextBend);

    // Validate against thresholds
    result.areaValid = (result.gripArea >= m_config.minGripArea);
    result.comValid = (result.comDistance <= m_config.maxComDistance);
    result.torqueValid = (result.torque <= m_config.maxTorque);
    result.shearValid = (result.shearForce <= m_config.maxShearForce);

    return result;
}

double GraspPhysicsEngine::computeGripArea(
    const BentState& state,
    const phase1::BendFeature& nextBend
) const {
    // Start with base grip area estimated from part geometry
    // As flanges bend up, they reduce the flat area available for grip
    //
    // Base area = sum of all flange areas (length * bendLineLength)
    // Each bent flange removes: length * cos(angle) from flat projection
    // (when angle > 0, the flange lifts off the grip plane)

    // Estimate base flat area: use a reasonable default
    // In practice, this would come from the flat pattern
    double baseArea = 500.0 * 300.0; // Typical flat part area (mm²)

    // Each bent flange reduces grip area proportionally
    double lostArea = 0.0;
    for (const auto& flange : state.getBentFlanges()) {
        // A bent flange with angle α loses area proportional to sin(α)
        // because the flange lifts off the grip surface
        double angleRad = std::abs(flange.angle) * 3.14159265358979 / 180.0;
        double flangeArea = nextBend.length * 50.0; // Approximate flange width
        lostArea += flangeArea * std::sin(angleRad);
    }

    // The next bend will also lose area
    double nextAngleRad = std::abs(nextBend.angle) * 3.14159265358979 / 180.0;
    double nextFlangeArea = nextBend.length * 50.0;
    lostArea += nextFlangeArea * std::sin(nextAngleRad);

    return std::max(baseArea - lostArea, 0.0);
}

double GraspPhysicsEngine::computeComDistance(
    const BentState& state,
    const phase1::BendFeature& nextBend
) const {
    // Center of mass shifts as flanges bend
    // Each bent flange contributes to COM shift based on:
    // - Its mass (proportional to length * thickness * density)
    // - Its position offset after bending

    if (state.count() == 0 && nextBend.id < 0) return 0.0;

    double totalMass = 0.0;
    double comX = 0.0;
    double comY = 0.0;

    // Grip center is assumed at origin (0,0) in the flat plane
    // Each bent flange shifts COM away from center

    for (const auto& flange : state.getBentFlanges()) {
        double mass = flange.occupiedVolume.volume() * m_config.materialDensity;
        if (mass < 1e-12) mass = 0.01; // Minimum mass

        // The COM of the bent flange is approximately at the center of its AABB
        double cx = (flange.occupiedVolume.minX + flange.occupiedVolume.maxX) / 2.0;
        double cy = (flange.occupiedVolume.minY + flange.occupiedVolume.maxY) / 2.0;

        comX += mass * cx;
        comY += mass * cy;
        totalMass += mass;
    }

    // Add contribution from the next bend
    double nextMass = estimateFlangeMass(nextBend);
    comX += nextMass * nextBend.position.x;
    comY += nextMass * nextBend.position.y;
    totalMass += nextMass;

    if (totalMass < 1e-12) return 0.0;

    comX /= totalMass;
    comY /= totalMass;

    return std::sqrt(comX * comX + comY * comY);
}

double GraspPhysicsEngine::computeTorque(
    const BentState& state,
    const phase1::BendFeature& nextBend
) const {
    // Torque = total_mass * g * COM_distance
    // g = 9.81 m/s² = 9810 mm/s²
    constexpr double g = 9.81; // m/s² (torque in N·m)

    double comDist = computeComDistance(state, nextBend);

    // Estimate total part mass
    double totalMass = 0.0;
    for (const auto& flange : state.getBentFlanges()) {
        double vol = flange.occupiedVolume.volume();
        totalMass += vol * m_config.materialDensity;
    }
    totalMass += estimateFlangeMass(nextBend);

    // Ensure minimum mass for realistic parts
    totalMass = std::max(totalMass, 0.01); // At least 10g

    // Torque in N·m: mass(kg) * g(m/s²) * distance(m)
    // comDist is in mm, convert to m
    return totalMass * g * (comDist / 1000.0);
}

double GraspPhysicsEngine::computeShearForce(
    const phase1::BendFeature& nextBend
) const {
    // Shear force during bending depends on:
    // - Material yield strength
    // - Bend length
    // - Material thickness
    //
    // Approximate bending force: F = (σ_y * L * t²) / (4 * W)
    // where W = die opening width (approximate as 8*t for V-bend)
    // Shear on gripper = F * friction_factor

    double t = m_config.materialThickness;
    double L = nextBend.length;
    double sigma = m_config.yieldStrength; // MPa = N/mm²
    double W = 8.0 * t; // V-die opening width

    // Bending force (N)
    double bendForce = (sigma * L * t * t) / (4.0 * W);

    // Shear force on gripper is a fraction of bending force
    // due to lateral reaction
    double shearFraction = 0.1; // ~10% of bending force as lateral shear
    return bendForce * shearFraction;
}

double GraspPhysicsEngine::estimateFlangeMass(
    const phase1::BendFeature& bend
) const {
    // Mass = length * width * thickness * density
    // Approximate width as 50mm (typical flange width)
    double width = 50.0;
    double volume = bend.length * width * m_config.materialThickness; // mm³
    return volume * m_config.materialDensity; // kg
}

double GraspPhysicsEngine::estimatePartArea(
    const std::vector<phase1::BendFeature>& bends
) const {
    if (bends.empty()) return 0.0;

    // Rough estimate: bounding box of all bend positions
    double minX = bends[0].position.x;
    double maxX = bends[0].position.x;
    double minY = bends[0].position.y;
    double maxY = bends[0].position.y;

    for (const auto& b : bends) {
        minX = std::min(minX, b.position.x - b.length / 2.0);
        maxX = std::max(maxX, b.position.x + b.length / 2.0);
        minY = std::min(minY, b.position.y);
        maxY = std::max(maxY, b.position.y);
    }

    double width = maxX - minX;
    double height = maxY - minY;
    return std::max(width, 100.0) * std::max(height, 100.0);
}

} // namespace phase4
} // namespace openpanelcam
