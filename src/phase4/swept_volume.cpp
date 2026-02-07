#include "openpanelcam/phase4/swept_volume.h"
#include <cmath>
#include <algorithm>

namespace openpanelcam {
namespace phase4 {

SweptVolume SweptVolumeGenerator::generate(const phase1::BendFeature& bend) const {
    SweptVolume sv;
    sv.bendId = bend.id;
    sv.sweepAngle = bend.angle;
    sv.aabb = estimateSweptAABB(bend);

    // OBB approximation: use AABB center and half-extents with bend direction rotation
    sv.obb.centerX = (sv.aabb.minX + sv.aabb.maxX) / 2.0;
    sv.obb.centerY = (sv.aabb.minY + sv.aabb.maxY) / 2.0;
    sv.obb.centerZ = (sv.aabb.minZ + sv.aabb.maxZ) / 2.0;
    sv.obb.halfExtentX = (sv.aabb.maxX - sv.aabb.minX) / 2.0;
    sv.obb.halfExtentY = (sv.aabb.maxY - sv.aabb.minY) / 2.0;
    sv.obb.halfExtentZ = (sv.aabb.maxZ - sv.aabb.minZ) / 2.0;
    // Default identity axes for now

    return sv;
}

AABB SweptVolumeGenerator::estimateSweptAABB(const phase1::BendFeature& bend) const {
    // The flange rotates around the bend line from 0° to bend.angle
    // Bend line is at bend.position, along bend.direction
    // Normal defines the rotation plane

    double px = bend.position.x;
    double py = bend.position.y;
    double pz = bend.position.z;

    // Flange length (distance from bend line to flange tip)
    // Use bend.length as the flange extent
    double flangeExtent = bend.length;

    // Normal direction determines the sweep plane
    double nx = bend.normal.x;
    double ny = bend.normal.y;
    double nz = bend.normal.z;

    // Direction of the bend line
    double dx = bend.direction.x;
    double dy = bend.direction.y;
    double dz = bend.direction.z;

    // The flange starts perpendicular to normal and sweeps through angleRad
    // Initial flange direction: cross(direction, normal) gives the flat flange dir
    // We compute bounding box conservatively

    // Conservative: the swept arc radius is flangeExtent
    // The arc spans from 0 to angleRad
    // Compute the bounding box of the arc in the sweep plane

    // Arc start point offset (flat position): along -normal * flangeExtent
    // Arc end point offset (bent position): rotated by angle around direction

    // For simplicity, compute AABB as the box containing:
    // 1. The bend line segment (length along direction)
    // 2. The arc of the flange tip

    double halfLen = flangeExtent / 2.0; // Half the bend line length

    // Arc bounding: the flange sweeps in the plane perpendicular to direction
    // Maximum extent along each axis from arc
    double arcRadius = flangeExtent;

    // The initial flange points along -nz (down from base)
    // After bending, it points in a rotated direction
    // AABB conservatively bounds all positions

    // For each axis, the extent is max of:
    // - bend line extent (along direction)
    // - arc extent (perpendicular to direction)

    double extentX = std::abs(dx) * halfLen + arcRadius * (std::abs(nx) + std::abs(1.0 - std::abs(dx)));
    double extentY = std::abs(dy) * halfLen + arcRadius * (std::abs(ny) + std::abs(1.0 - std::abs(dy)));
    double extentZ = std::abs(dz) * halfLen + arcRadius * (std::abs(nz) + std::abs(1.0 - std::abs(dz)));

    // Clamp minimum extents
    extentX = std::max(extentX, 5.0);
    extentY = std::max(extentY, 5.0);
    extentZ = std::max(extentZ, 5.0);

    return AABB(px - extentX, py - extentY, pz - extentZ,
                px + extentX, py + extentY, pz + extentZ);
}

} // namespace phase4
} // namespace openpanelcam
