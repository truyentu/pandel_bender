#include "openpanelcam/phase4/bent_state.h"
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

void BentState::addBentFlange(const phase1::BendFeature& bend) {
    BentFlange flange;
    flange.bendId = bend.id;
    flange.angle = bend.angle;
    flange.occupiedVolume = estimateOccupiedAABB(bend);

    // OBB from AABB
    flange.occupiedOBB.centerX = (flange.occupiedVolume.minX + flange.occupiedVolume.maxX) / 2.0;
    flange.occupiedOBB.centerY = (flange.occupiedVolume.minY + flange.occupiedVolume.maxY) / 2.0;
    flange.occupiedOBB.centerZ = (flange.occupiedVolume.minZ + flange.occupiedVolume.maxZ) / 2.0;
    flange.occupiedOBB.halfExtentX = (flange.occupiedVolume.maxX - flange.occupiedVolume.minX) / 2.0;
    flange.occupiedOBB.halfExtentY = (flange.occupiedVolume.maxY - flange.occupiedVolume.minY) / 2.0;
    flange.occupiedOBB.halfExtentZ = (flange.occupiedVolume.maxZ - flange.occupiedVolume.minZ) / 2.0;

    m_flanges.push_back(flange);
}

std::vector<AABB> BentState::getAllOccupiedVolumes() const {
    std::vector<AABB> volumes;
    volumes.reserve(m_flanges.size());
    for (const auto& f : m_flanges) {
        volumes.push_back(f.occupiedVolume);
    }
    return volumes;
}

void BentState::reset() {
    m_flanges.clear();
}

AABB BentState::estimateOccupiedAABB(const phase1::BendFeature& bend) const {
    // Estimate the final position of the bent flange
    // The flange starts at bend.position and extends along the bent direction

    double px = bend.position.x;
    double py = bend.position.y;
    double pz = bend.position.z;

    double flangeExtent = bend.length;
    double angleRad = std::abs(bend.angle) * 3.14159265358979 / 180.0;

    // The bent flange endpoint after rotation
    // Normal defines the plane of bending
    double nx = bend.normal.x;
    double ny = bend.normal.y;
    double nz = bend.normal.z;

    // Direction along the bend line
    double dx = bend.direction.x;
    double dy = bend.direction.y;
    double dz = bend.direction.z;

    // After bending, the flange extends:
    // - horizontally by flangeExtent * cos(angle) in the flat direction
    // - vertically by flangeExtent * sin(angle) along normal

    double cosA = std::cos(angleRad);
    double sinA = std::sin(angleRad);

    // Half-length along bend direction
    double halfLen = flangeExtent / 2.0;

    // The flange after bending occupies a volume from the bend line
    // extending outward. The extent depends on the bend angle.
    double flatExtent = flangeExtent * std::max(std::abs(cosA), 0.1);
    double heightExtent = flangeExtent * std::abs(sinA);

    // Build AABB around the bent flange position
    double extX = std::abs(dx) * halfLen + flatExtent * 0.5 + heightExtent * std::abs(nx);
    double extY = std::abs(dy) * halfLen + flatExtent * 0.5 + heightExtent * std::abs(ny);
    double extZ = std::abs(dz) * halfLen + flatExtent * 0.5 + heightExtent * std::abs(nz);

    extX = std::max(extX, 5.0);
    extY = std::max(extY, 5.0);
    extZ = std::max(extZ, 5.0);

    return AABB(px - extX, py - extY, pz - extZ,
                px + extX, py + extY, pz + extZ);
}

void BentState::addBentFlange(const phase1::BendFeature& bend, double kFactor) {
    BentFlange flange;
    flange.bendId = bend.id;
    flange.angle = bend.angle;
    flange.occupiedVolume = estimateOccupiedAABB(bend, kFactor);

    // OBB from AABB
    flange.occupiedOBB.centerX = (flange.occupiedVolume.minX + flange.occupiedVolume.maxX) / 2.0;
    flange.occupiedOBB.centerY = (flange.occupiedVolume.minY + flange.occupiedVolume.maxY) / 2.0;
    flange.occupiedOBB.centerZ = (flange.occupiedVolume.minZ + flange.occupiedVolume.maxZ) / 2.0;
    flange.occupiedOBB.halfExtentX = (flange.occupiedVolume.maxX - flange.occupiedVolume.minX) / 2.0;
    flange.occupiedOBB.halfExtentY = (flange.occupiedVolume.maxY - flange.occupiedVolume.minY) / 2.0;
    flange.occupiedOBB.halfExtentZ = (flange.occupiedVolume.maxZ - flange.occupiedVolume.minZ) / 2.0;

    m_flanges.push_back(flange);
}

AABB BentState::estimateOccupiedAABB(const phase1::BendFeature& bend, double kFactor) const {
    // Get base AABB
    AABB base = estimateOccupiedAABB(bend);

    if (kFactor <= 0.0 || kFactor >= 1.0) {
        return base;
    }

    // Paper: k-factor correction shrinks AABB to account for material
    // deformation at the bend. The neutral axis shifts inward by
    // kFactor * thickness, reducing the effective flange extent.
    // This avoids false-positive collisions from pure rotation idealization.
    double shrinkage = kFactor * 1.5;  // 1.5mm default thickness

    return AABB(
        base.minX + shrinkage, base.minY + shrinkage, base.minZ + shrinkage,
        base.maxX - shrinkage, base.maxY - shrinkage, base.maxZ - shrinkage
    );
}

} // namespace phase4
} // namespace openpanelcam
