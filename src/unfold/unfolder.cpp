#include <openpanelcam/unfold/unfolder.h>
#include <openpanelcam/phase1/fag.h>
#include <openpanelcam/core/logger.h>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <gp_Ax1.hxx>

#include <chrono>
#include <cmath>

namespace openpanelcam {

static constexpr double PI = 3.14159265358979323846;

SheetMetalUnfolder::SheetMetalUnfolder(const UnfoldConfig& config)
    : m_config(config)
{
    m_baCalc.setUseDIN6935(config.useDIN6935);
    m_baCalc.setUseHemEmpirical(config.useHemEmpirical);
}

double SheetMetalUnfolder::resolveKFactor(double radius) const {
    if (m_config.useDIN6935) {
        return m_din6935.getKFactor(radius, m_config.thickness);
    }
    if (!m_config.material.empty() && m_materialTable.hasMaterial(m_config.material)) {
        double rt = (m_config.thickness > 0) ? radius / m_config.thickness : 1.0;
        return m_materialTable.getKFactor(m_config.material, rt);
    }
    return m_config.defaultKFactor;
}

UnfoldResult SheetMetalUnfolder::unfold(
    const FaceAdjacencyGraph& fag, int baseFaceId)
{
    UnfoldResult result;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Validate input
    if (fag.nodeCount() == 0) {
        result.success = false;
        result.errorMessage = "FAG is empty";
        return result;
    }
    if (baseFaceId < 0 || baseFaceId >= fag.nodeCount()) {
        result.success = false;
        result.errorMessage = "Invalid base face ID: " + std::to_string(baseFaceId);
        return result;
    }

    // Step 1: Build spanning tree
    UnfoldTree tree = m_treeBuilder.build(fag, baseFaceId);

    if (tree.nodes.empty()) {
        result.success = false;
        result.errorMessage = "Failed to build unfold tree";
        return result;
    }

    result.facesUnfolded = static_cast<int>(tree.nodes.size());

    // Step 2: Compute bend zones (BA, BD, K-factor for each bend)
    computeBendZones(tree, fag, result);

    // Step 3: Compute accumulated transforms
    computeTransforms(tree, fag, result.bendZones);

    // Step 4: Apply transforms to create flat pattern
    // Only works with real OCCT shapes (not null test faces)
    bool hasRealShapes = false;
    for (const auto& node : tree.nodes) {
        const auto& fagNode = fag.getNode(node.faceId);
        if (!fagNode.face.IsNull()) {
            hasRealShapes = true;
            break;
        }
    }

    if (hasRealShapes) {
        BRepBuilderAPI_Sewing sewing(m_config.sewingTolerance);

        for (const auto& treeNode : tree.nodes) {
            const auto& fagNode = fag.getNode(treeNode.faceId);
            if (fagNode.face.IsNull()) continue;

            // Only add planar faces (skip cylindrical bend faces)
            if (fagNode.type != FaceType::PLANAR) continue;

            BRepBuilderAPI_Transform xform(
                fagNode.face, treeNode.accumulatedTransform, Standard_True);
            sewing.Add(xform.Shape());
        }

        sewing.Perform();
        result.flatPattern = sewing.SewedShape();
    }

    result.success = true;

    auto endTime = std::chrono::high_resolution_clock::now();
    result.unfoldTime = std::chrono::duration<double, std::milli>(
        endTime - startTime).count();

    return result;
}

void SheetMetalUnfolder::computeBendZones(
    const UnfoldTree& tree,
    const FaceAdjacencyGraph& fag,
    UnfoldResult& result)
{
    for (const auto& node : tree.nodes) {
        if (node.connectingEdgeId < 0) continue; // root node

        const auto& edge = fag.getEdge(node.connectingEdgeId);
        if (!edge.isBend) continue;

        BendZoneInfo bz;
        bz.bendId = edge.id;
        bz.innerRadius = edge.bendRadius;
        bz.bendAngle = edge.bendAngle;
        bz.bendLength = edge.bendLength;

        // Resolve K-factor
        bz.kFactor = resolveKFactor(bz.innerRadius);

        // Compute bend allowance
        auto ba = m_baCalc.compute(
            bz.innerRadius, bz.bendAngle, m_config.thickness, bz.kFactor);
        bz.bendAllowance = ba.bendAllowance;
        bz.bendDeduction = ba.bendDeduction;

        result.bendZones.push_back(bz);
        result.bendsProcessed++;
    }
}

void SheetMetalUnfolder::computeTransforms(
    UnfoldTree& tree,
    const FaceAdjacencyGraph& fag,
    const std::vector<BendZoneInfo>& bendZones)
{
    // Process nodes in BFS order (they are already in BFS order from builder)
    for (size_t i = 0; i < tree.nodes.size(); ++i) {
        auto& node = tree.nodes[i];

        if (node.parentIndex < 0) {
            // Root: identity transform
            continue;
        }

        if (node.connectingEdgeId < 0) continue;

        const auto& edge = fag.getEdge(node.connectingEdgeId);
        if (!edge.isBend) continue;

        // Compute local unfold rotation
        double unfoldAngleRad = -(PI - edge.bendAngle * PI / 180.0);

        gp_Trsf localUnfold;

        // Use bend axis if available
        if (!edge.sharedEdge.IsNull()) {
            BRepAdaptor_Curve edgeCurve(edge.sharedEdge);
            gp_Pnt startPt = edgeCurve.Value(edgeCurve.FirstParameter());
            gp_Pnt endPt = edgeCurve.Value(edgeCurve.LastParameter());

            if (startPt.Distance(endPt) > 1e-6) {
                gp_Ax1 axis(startPt, gp_Dir(gp_Vec(startPt, endPt)));
                localUnfold.SetRotation(axis, unfoldAngleRad);
            }
        } else {
            // Fallback: use stored bend axis
            localUnfold.SetRotation(edge.bendAxis, unfoldAngleRad);
        }

        // Find parent tree node and compose transforms
        gp_Trsf parentTransform;
        for (const auto& parentNode : tree.nodes) {
            if (parentNode.faceId == node.parentIndex) {
                parentTransform = parentNode.accumulatedTransform;
                break;
            }
        }

        node.accumulatedTransform = parentTransform.Multiplied(localUnfold);
    }
}

} // namespace openpanelcam
