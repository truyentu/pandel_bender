/**
 * @file bend_classifier.cpp
 * @brief Bend classifier implementation
 */

#include <openpanelcam/phase1/bend_classifier.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/constants.h>
#include <openpanelcam/core/geometry_utils.h>

#include <cmath>
#include <algorithm>

namespace openpanelcam {

BendClassifier::BendClassifier()
    : m_thickness(2.0)
    , m_hemAngleThreshold(constants::HEM_ANGLE_THRESHOLD)
    , m_kFactor(0.5)
    , m_hasSolid(false)
{
}

void BendClassifier::setThickness(double thickness) {
    m_thickness = thickness;
}

void BendClassifier::setHemAngleThreshold(double angle) {
    m_hemAngleThreshold = angle;
}

void BendClassifier::setSolid(const TopoDS_Solid& solid) {
    m_solid = solid;
    m_hasSolid = !solid.IsNull();
}

void BendClassifier::setKFactor(double kFactor) {
    m_kFactor = kFactor;
}

std::vector<BendFeature> BendClassifier::classify(
    const FaceAdjacencyGraph& fag,
    int baseFaceId
) {
    std::vector<BendFeature> bends;

    if (fag.bendCount() == 0) {
        LOG_WARNING("No bends found in FAG");
        return bends;
    }

    LOG_INFO("Classifying {} bends", fag.bendCount());

    // Get all bend edges
    std::vector<int> bendEdgeIds = fag.getBendEdges();

    int bendId = 0;
    for (int edgeId : bendEdgeIds) {
        const FAG_Edge& edge = fag.getEdge(edgeId);

        // Determine which node is base vs flange
        auto [baseNodeId, flangeNodeId] = determineBaseAndFlange(edge, baseFaceId, fag);

        if (baseNodeId < 0 || flangeNodeId < 0) {
            LOG_WARNING("Could not determine base/flange for edge {}", edgeId);
            continue;
        }

        const FAG_Node& baseNode = fag.getNode(baseNodeId);
        const FAG_Node& flangeNode = fag.getNode(flangeNodeId);

        // Classify bend direction
        BendClassificationResult classification = classifyBend(edge, baseNode, flangeNode);

        // Extract complete BendFeature
        BendFeature bend = extractBendFeature(
            bendId,
            edge,
            baseNode,
            flangeNode,
            classification
        );

        bends.push_back(bend);

        LOG_INFO("Bend {}: {} {:.1f}deg (radius: {:.2f} mm, SDF: {:.2f})",
                 bend.id,
                 bend.direction == BendDirection::BEND_UP ? "UP" :
                 bend.direction == BendDirection::BEND_DOWN ? "DOWN" : "HEM",
                 bend.targetAngle,
                 bend.internalRadius,
                 bend.signedDistance);

        bendId++;
    }

    LOG_INFO("Classified {} bends successfully", bends.size());

    return bends;
}

BendClassificationResult BendClassifier::classifyBend(
    const FAG_Edge& edge,
    const FAG_Node& baseNode,
    const FAG_Node& flangeNode
) {
    BendClassificationResult result;

    // Get centroids
    gp_Pnt P_base = baseNode.centroid;
    gp_Pnt P_flange = flangeNode.centroid;

    // Get base normal (material-outward)
    gp_Dir N_base = baseNode.normal;

    // Compute signed distance (SDF method)
    gp_Vec vec(P_base, P_flange);
    double D = vec.Dot(N_base);

    result.signedDistance = D;

    // Classify based on sign and magnitude
    double absDist = std::abs(D);

    // Check if coplanar (might be HEM)
    if (absDist < m_thickness * 2.0) {
        // Check angle
        if (edge.bendAngle < m_hemAngleThreshold) {
            result.direction = BendDirection::HEM;
            result.confidence = 0.9;
            result.reasoning = "Small angle and coplanar";
        } else {
            // Coplanar but not small angle - use sign
            if (D > 0) {
                result.direction = BendDirection::BEND_UP;
            } else {
                result.direction = BendDirection::BEND_DOWN;
            }
            result.confidence = 0.7;
            result.reasoning = "Coplanar but angle > HEM threshold";
        }
    } else {
        // Clear separation - use SDF sign
        if (D > 0) {
            result.direction = BendDirection::BEND_UP;
            result.confidence = 0.95;
            result.reasoning = "Flange above base (SDF > 0)";
        } else {
            result.direction = BendDirection::BEND_DOWN;
            result.confidence = 0.95;
            result.reasoning = "Flange below base (SDF < 0)";
        }
    }

    return result;
}

BendFeature BendClassifier::extractBendFeature(
    int bendId,
    const FAG_Edge& edge,
    const FAG_Node& baseNode,
    const FAG_Node& flangeNode,
    const BendClassificationResult& classification
) {
    BendFeature bend;

    // Identity
    bend.id = bendId;
    bend.fagEdgeId = edge.id;
    bend.name = "Bend_" + std::to_string(bendId);

    // Participating faces
    bend.baseFaceId = baseNode.id;
    bend.baseFace = baseNode.face;

    bend.flangeFaceId = flangeNode.id;
    bend.flangeFace = flangeNode.face;

    bend.bendFace = edge.bendFace;
    bend.bendFaceId = -1;  // Not tracked in current FAG

    // Base geometry
    bend.baseCentroid = baseNode.centroid;
    bend.baseNormal = baseNode.normal;

    // Flange geometry
    bend.flangeCentroid = flangeNode.centroid;
    bend.flangeNormal = flangeNode.normal;

    // Compute flange dimensions
    computeFlangeDimensions(bend, flangeNode);

    // Bend line
    bend.bendLineStart = edge.bendAxisStart;
    bend.bendLineEnd = edge.bendAxisEnd;
    bend.bendLineLength = edge.bendLength;

    gp_Vec bendVec(bend.bendLineStart, bend.bendLineEnd);
    if (bendVec.Magnitude() > constants::LINEAR_TOLERANCE) {
        bend.bendLine = gp_Lin(bend.bendLineStart, gp_Dir(bendVec));
    }

    // Bend parameters
    bend.targetAngle = edge.bendAngle;
    bend.signedAngle = (classification.direction == BendDirection::BEND_UP) ?
                       edge.bendAngle : -edge.bendAngle;

    bend.internalRadius = edge.bendRadius;
    bend.kFactor = m_kFactor;

    // Classification
    bend.direction = classification.direction;
    bend.signedDistance = classification.signedDistance;

    // Determine convexity based on direction
    if (classification.direction == BendDirection::BEND_UP) {
        bend.convexity = BendConvexity::CONCAVE;
    } else if (classification.direction == BendDirection::BEND_DOWN) {
        bend.convexity = BendConvexity::CONVEX;
    } else {
        bend.convexity = BendConvexity::UNKNOWN;
    }

    // Initialize constraints (will be populated by Phase 2)
    bend.mustBendBefore.clear();
    bend.mustBendAfter.clear();

    // Initialize sequence info
    bend.sequencePosition = -1;

    return bend;
}

std::pair<int, int> BendClassifier::determineBaseAndFlange(
    const FAG_Edge& edge,
    int baseFaceId,
    const FaceAdjacencyGraph& fag
) {
    int node1 = edge.node1;
    int node2 = edge.node2;

    // Check flange levels
    const FAG_Node& n1 = fag.getNode(node1);
    const FAG_Node& n2 = fag.getNode(node2);

    // If one is the base face itself
    if (node1 == baseFaceId) {
        return {node1, node2};
    }
    if (node2 == baseFaceId) {
        return {node2, node1};
    }

    // Use flange levels (lower level = closer to base)
    if (n1.flangeLevel >= 0 && n2.flangeLevel >= 0) {
        if (n1.flangeLevel < n2.flangeLevel) {
            return {node1, node2};
        } else {
            return {node2, node1};
        }
    }

    // Fallback: use node1 as base
    LOG_WARNING("Could not determine base/flange by level, using node1 as base");
    return {node1, node2};
}

double BendClassifier::computeBendAllowance(
    double radius,
    double angle,
    double thickness,
    double kFactor
) {
    // Formula: BA = (pi/180) * angle * (radius + k*thickness)
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    return DEG_TO_RAD * angle * (radius + kFactor * thickness);
}

void BendClassifier::computeFlangeDimensions(
    BendFeature& bend,
    const FAG_Node& flangeNode
) {
    // Simple approach: use face area and bounding box
    bend.flangeArea = flangeNode.area;

    // Compute approximate dimensions from bounding box
    if (!flangeNode.boundingBox.IsVoid()) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        flangeNode.boundingBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);

        double dx = xmax - xmin;
        double dy = ymax - ymin;
        double dz = zmax - zmin;

        // Sort dimensions
        std::vector<double> dims = {dx, dy, dz};
        std::sort(dims.begin(), dims.end());

        // Smallest = thickness (ignore)
        // Medium = flange width
        // Largest = flange length
        bend.flangeWidth = dims[1];
        bend.flangeLength = dims[2];
    } else {
        bend.flangeWidth = 0.0;
        bend.flangeLength = 0.0;
    }
}

} // namespace openpanelcam
