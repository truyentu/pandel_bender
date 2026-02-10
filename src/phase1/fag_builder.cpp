/**
 * @file fag_builder.cpp
 * @brief FAG Builder implementation
 */

#include <openpanelcam/phase1/fag_builder.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/constants.h>
#include <openpanelcam/core/geometry_utils.h>

// OCCT Topology
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>

#include <sstream>

namespace openpanelcam {

FAGBuilder::FAGBuilder()
    : m_minBendRadius(constants::MIN_BEND_RADIUS)
    , m_minFaceArea(constants::AREA_TOLERANCE)
    , m_sdfValidation(false)
    , m_ignoredFaceCount(0)
    , m_totalFaces(0)
    , m_totalEdges(0)
    , m_bendEdges(0)
    , m_sharpEdges(0)
{
}

void FAGBuilder::setMinBendRadius(double radius) {
    m_minBendRadius = radius;
}

void FAGBuilder::setMinFaceArea(double area) {
    m_minFaceArea = area;
}

void FAGBuilder::setSDFValidation(bool enable) {
    m_sdfValidation = enable;
}

int FAGBuilder::getIgnoredFaceCount() const {
    return m_ignoredFaceCount;
}

std::vector<std::string> FAGBuilder::getWarnings() const {
    return m_warnings;
}

void FAGBuilder::addWarning(const std::string& message) {
    m_warnings.push_back(message);
    LOG_WARNING("FAGBuilder: {}", message);
}

void FAGBuilder::clearStatistics() {
    m_ignoredFaceCount = 0;
    m_totalFaces = 0;
    m_totalEdges = 0;
    m_bendEdges = 0;
    m_sharpEdges = 0;
    m_warnings.clear();
}

FaceAdjacencyGraph FAGBuilder::build(const TopoDS_Shape& shape) {
    clearStatistics();

    LOG_INFO("Building FAG from shape");

    FaceAdjacencyGraph fag;

    if (shape.IsNull()) {
        addWarning("Input shape is null");
        THROW_PHASE1(ErrorCode::GEOMETRY_INVALID,
                    "Cannot build FAG from null shape",
                    "Shape is null");
        return fag;
    }

    try {
        // STEP 1: Build edge→face mapping (O(n) algorithm from S1)
        // This is more efficient than nested loops over faces
        LOG_DEBUG("Step 1: Building edge→face mapping");

        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

        LOG_DEBUG("Found {} edges in shape", edgeToFaces.Extent());

        // STEP 2: Extract all faces and create nodes
        LOG_DEBUG("Step 2: Creating FAG nodes from faces");

        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

        m_totalFaces = faceMap.Extent();
        LOG_DEBUG("Found {} faces", m_totalFaces);

        // Reserve space for efficiency
        fag.reserve(m_totalFaces, edgeToFaces.Extent());

        // Map OCCT face to FAG node ID
        TopTools_DataMapOfShapeInteger faceToNodeId;

        for (int i = 1; i <= faceMap.Extent(); i++) {
            TopoDS_Face face = TopoDS::Face(faceMap(i));

            // Check face area (ignore tiny faces)
            double area = SurfaceAnalyzer::computeArea(face);
            if (area < m_minFaceArea) {
                LOG_DEBUG("Ignoring face {} with area {:.6f} mm² (below threshold)",
                         i, area);
                m_ignoredFaceCount++;
                continue;
            }

            // Add node to FAG
            int nodeId = fag.addNode(face);
            faceToNodeId.Bind(face, nodeId);

            LOG_DEBUG("Face {} → Node {} (area: {:.2f} mm²)", i, nodeId, area);
        }

        LOG_INFO("Created {} nodes ({} faces ignored)",
                 fag.nodeCount(), m_ignoredFaceCount);

        // STEP 3: Process edges and create FAG edges
        LOG_DEBUG("Step 3: Processing edges");

        m_totalEdges = edgeToFaces.Extent();

        for (int i = 1; i <= edgeToFaces.Extent(); i++) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeToFaces.FindKey(i));
            const TopTools_ListOfShape& faces = edgeToFaces.FindFromIndex(i);

            // Count adjacent faces
            int faceCount = faces.Extent();

            if (faceCount < 2) {
                // Boundary edge (only 1 face) - skip for now
                continue;
            }

            if (faceCount > 2) {
                addWarning("Edge has " + std::to_string(faceCount) +
                          " adjacent faces (expected 2)");
                continue;
            }

            // Get the two adjacent faces
            TopTools_ListIteratorOfListOfShape it(faces);
            TopoDS_Face face1 = TopoDS::Face(it.Value());
            it.Next();
            TopoDS_Face face2 = TopoDS::Face(it.Value());

            // Check if both faces are in FAG (not ignored)
            if (!faceToNodeId.IsBound(face1) || !faceToNodeId.IsBound(face2)) {
                // One or both faces were ignored
                continue;
            }

            int nodeId1 = faceToNodeId.Find(face1);
            int nodeId2 = faceToNodeId.Find(face2);

            // Classify edge
            EdgeClassification classification = classifyEdge(edge, face1, face2);

            // Create FAG edge
            if (classification.isBend) {
                // Need to find the cylindrical face between face1 and face2
                // For now, we'll use a placeholder approach
                // TODO: More sophisticated cylindrical face detection

                // Try to find a cylindrical face
                TopoDS_Face bendFace;

                // Simple heuristic: check if edge is circular
                if (EdgeAnalyzer::isCircular(edge)) {
                    // The bend might be represented by the edge itself
                    // In proper CAD models, there should be a cylindrical face
                    // For now, we'll create a bend edge without the cylindrical face
                    // and compute properties later
                }

                int edgeId = fag.addBendEdge(nodeId1, nodeId2, bendFace);
                m_bendEdges++;

                LOG_DEBUG("Created bend edge {} between nodes {} and {} "
                         "(radius: {:.2f} mm, angle: {:.1f}°)",
                         edgeId, nodeId1, nodeId2,
                         classification.radius, classification.angle);

            } else {
                int edgeId = fag.addEdge(nodeId1, nodeId2, edge);
                m_sharpEdges++;

                LOG_DEBUG("Created sharp edge {} between nodes {} and {} "
                         "(angle: {:.1f}°)",
                         edgeId, nodeId1, nodeId2, classification.angle);
            }
        }

        LOG_INFO("Created {} edges ({} bends, {} sharp)",
                 fag.edgeCount(), m_bendEdges, m_sharpEdges);

        // STEP 4: Finalize FAG (compute all geometric properties)
        LOG_DEBUG("Step 4: Finalizing FAG");
        fag.finalize();

        LOG_INFO("FAG build complete: {} nodes, {} edges, {} bends",
                 fag.nodeCount(), fag.edgeCount(), fag.bendCount());

        return fag;

    } catch (const Standard_Failure& e) {
        std::string msg = "OCCT exception during FAG build: " +
                         std::string(e.GetMessageString());
        LOG_ERROR("{}", msg);
        THROW_OCCT("FAG build failed", msg);
        return fag;
    }
}

EdgeClassification FAGBuilder::classifyEdge(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2
) {
    EdgeClassification result;

    if (edge.IsNull() || face1.IsNull() || face2.IsNull()) {
        result.confidence = 0.0;
        result.reasoning = "Null geometry";
        return result;
    }

    // Analyze edge geometry
    BRepAdaptor_Curve adaptor(edge);
    GeomAbs_CurveType curveType = adaptor.GetType();

    // Get face types
    bool isPlanar1 = SurfaceAnalyzer::isPlanar(face1);
    bool isPlanar2 = SurfaceAnalyzer::isPlanar(face2);

    // Compute dihedral angle
    DihedralAngleResult angleResult =
        AngleCalculator::calculateDihedralAngle(face1, face2, edge);

    result.angle = angleResult.openingAngle;

    // CLASSIFICATION LOGIC (from S1, S2)

    // Case 1: Circular edge (potential bend)
    if (curveType == GeomAbs_Circle) {
        auto circle = EdgeAnalyzer::getCircle(edge);
        if (circle) {
            result.radius = circle->Radius();

            // Check if radius is significant (not a chamfer)
            if (result.radius > m_minBendRadius) {
                // Both faces should be planar for a valid bend
                if (isPlanar1 && isPlanar2) {
                    result.isBend = true;
                    result.confidence = 0.95;
                    result.reasoning = "Circular edge with radius " +
                                      std::to_string(result.radius) + " mm";
                } else {
                    result.isBend = false;
                    result.confidence = 0.7;
                    result.reasoning = "Circular edge but not between planar faces";
                }
            } else {
                result.isBend = false;
                result.confidence = 0.8;
                result.reasoning = "Radius below MIN_BEND_RADIUS threshold";
            }
        }
    }
    // Case 2: Linear edge (sharp corner)
    else if (curveType == GeomAbs_Line) {
        result.isBend = false;
        result.radius = 0.0;
        result.confidence = 1.0;
        result.reasoning = "Linear edge - sharp corner";
    }
    // Case 3: Other curve types
    else {
        result.isBend = false;
        result.radius = 0.0;
        result.confidence = 0.5;
        result.reasoning = "Non-linear, non-circular edge";
    }

    return result;
}

bool FAGBuilder::isBoundaryEdge(int edgeFaceCount) const {
    return edgeFaceCount == 1;
}

std::string FAGBuilder::getStatistics() const {
    std::ostringstream oss;

    oss << "FAG Build Statistics:\n";
    oss << "  Total faces: " << m_totalFaces << "\n";
    oss << "  Ignored faces: " << m_ignoredFaceCount
        << " (area < " << m_minFaceArea << " mm²)\n";
    oss << "  Total edges: " << m_totalEdges << "\n";
    oss << "  Bend edges: " << m_bendEdges << "\n";
    oss << "  Sharp edges: " << m_sharpEdges << "\n";
    oss << "  Warnings: " << m_warnings.size() << "\n";

    if (!m_warnings.empty()) {
        oss << "  Warning list:\n";
        for (const auto& warning : m_warnings) {
            oss << "    - " << warning << "\n";
        }
    }

    return oss.str();
}

} // namespace openpanelcam

