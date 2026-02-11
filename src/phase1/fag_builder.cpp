/**
 * @file fag_builder.cpp
 * @brief FAG Builder implementation
 *
 * Algorithm (from research papers S1, S2):
 *
 * Sheet metal B-Rep structure:
 *   Planar faces = flanges/base (the flat parts)
 *   Cylindrical faces = bends (the curved connections)
 *   Each physical bend has 2 cylindrical faces (inner + outer)
 *   Each cylindrical face connects 2 planar faces via CIRCULAR edges
 *
 * Strategy:
 *   1. Classify all faces as PLANAR or CYLINDRICAL
 *   2. Create FAG nodes ONLY for planar faces
 *   3. Group inner/outer cylindrical faces per physical bend
 *   4. For each bend group, find 2 adjacent planar faces via circular edges
 *   5. For direct planar-to-planar connections → create sharp edges
 *   6. Finalize: compute geometric properties
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
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>

#include <sstream>
#include <set>
#include <map>

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
    , m_cylindricalFaceCount(0)
    , m_planarFaceCount(0)
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

int FAGBuilder::getCylindricalFaceCount() const {
    return m_cylindricalFaceCount;
}

int FAGBuilder::getPlanarFaceCount() const {
    return m_planarFaceCount;
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
    m_cylindricalFaceCount = 0;
    m_planarFaceCount = 0;
    m_warnings.clear();
}

/**
 * Helper: find planar faces adjacent to a cylindrical face
 * that are base/flange faces (normal perpendicular to bend axis),
 * NOT side/thickness faces (normal parallel to bend axis).
 *
 * Sheet metal B-Rep from extrusion+fillet:
 *   - Base/flange faces: normal ⊥ bend axis → connected via linear edges along extrusion
 *   - Side/thickness faces: normal ∥ bend axis → connected via circular arc edges
 *
 * We want base/flange, so filter by normal direction.
 */
static std::set<int> findAdjacentFlangeNodes(
    const TopoDS_Face& cylFace,
    const gp_Dir& bendAxisDir,
    const TopTools_IndexedDataMapOfShapeListOfShape& edgeToFaces,
    const TopTools_DataMapOfShapeInteger& faceToNodeId
) {
    std::set<int> result;

    TopExp_Explorer edgeExp(cylFace, TopAbs_EDGE);
    for (; edgeExp.More(); edgeExp.Next()) {
        TopoDS_Edge edge = TopoDS::Edge(edgeExp.Current());

        // Find this edge in the map
        for (int ei = 1; ei <= edgeToFaces.Extent(); ei++) {
            if (!edgeToFaces.FindKey(ei).IsSame(edge)) continue;

            const TopTools_ListOfShape& adjFaces = edgeToFaces.FindFromIndex(ei);
            TopTools_ListIteratorOfListOfShape it(adjFaces);
            for (; it.More(); it.Next()) {
                TopoDS_Face adjFace = TopoDS::Face(it.Value());

                // Skip the cylindrical face itself
                if (adjFace.IsSame(cylFace)) continue;

                // Only planar nodes
                if (!faceToNodeId.IsBound(adjFace)) continue;

                // Check if this planar face's normal is perpendicular to bend axis
                // (base/flange faces have normals ⊥ to bend axis)
                BRepAdaptor_Surface adjSurf(adjFace);
                if (adjSurf.GetType() == GeomAbs_Plane) {
                    gp_Dir faceNormal = adjSurf.Plane().Axis().Direction();
                    double dot = std::abs(faceNormal.Dot(bendAxisDir));

                    // dot ≈ 0 means perpendicular (base/flange)
                    // dot ≈ 1 means parallel (side/thickness)
                    if (dot < 0.3) {  // perpendicular threshold
                        result.insert(faceToNodeId.Find(adjFace));
                    }
                }
            }
            break;
        }
    }

    return result;
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
        // =================================================================
        // STEP 1: Build edge→face mapping (O(n) from S1)
        // =================================================================
        LOG_DEBUG("Step 1: Building edge-face mapping");

        TopTools_IndexedDataMapOfShapeListOfShape edgeToFaces;
        TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeToFaces);

        LOG_DEBUG("Found {} edges in shape", edgeToFaces.Extent());

        // =================================================================
        // STEP 2: Classify all faces
        // =================================================================
        LOG_DEBUG("Step 2: Classifying faces");

        TopTools_IndexedMapOfShape faceMap;
        TopExp::MapShapes(shape, TopAbs_FACE, faceMap);

        m_totalFaces = faceMap.Extent();

        // Maps
        TopTools_DataMapOfShapeInteger faceToNodeId;

        // Cylindrical face info: store face + radius + axis
        struct CylFaceInfo {
            TopoDS_Face face;
            double radius;
            gp_Ax1 axis;
        };
        std::vector<CylFaceInfo> allCylFaces;

        int otherCount = 0;

        for (int i = 1; i <= faceMap.Extent(); i++) {
            TopoDS_Face face = TopoDS::Face(faceMap(i));

            double area = SurfaceAnalyzer::computeArea(face);
            if (area < m_minFaceArea) {
                m_ignoredFaceCount++;
                continue;
            }

            BRepAdaptor_Surface adaptor(face);
            GeomAbs_SurfaceType surfType = adaptor.GetType();

            if (surfType == GeomAbs_Plane) {
                int nodeId = fag.addNode(face);
                faceToNodeId.Bind(face, nodeId);
                m_planarFaceCount++;

                LOG_DEBUG("Face {} -> Node {} (PLANAR, area: {:.2f})", i, nodeId, area);

            } else if (surfType == GeomAbs_Cylinder) {
                gp_Cylinder cyl = adaptor.Cylinder();
                double radius = cyl.Radius();

                if (radius >= m_minBendRadius && radius <= constants::MAX_BEND_RADIUS) {
                    CylFaceInfo info;
                    info.face = face;
                    info.radius = radius;
                    info.axis = cyl.Axis();
                    allCylFaces.push_back(info);
                    m_cylindricalFaceCount++;

                    LOG_DEBUG("Face {} -> CYLINDRICAL (R={:.2f}, area: {:.2f})",
                             i, radius, area);
                } else {
                    otherCount++;
                }
            } else {
                otherCount++;
            }
        }

        fag.reserve(m_planarFaceCount, m_planarFaceCount + m_cylindricalFaceCount);

        LOG_INFO("Face classification: {} planar, {} cylindrical, {} other, {} ignored",
                 m_planarFaceCount, m_cylindricalFaceCount, otherCount, m_ignoredFaceCount);

        // =================================================================
        // STEP 3: Group inner/outer cylindrical faces per physical bend
        //
        // Inner and outer faces of same bend share the same axis direction
        // and axis location. Group them and keep the inner one (smaller R).
        // =================================================================
        LOG_DEBUG("Step 3: Grouping cylindrical faces into physical bends");

        struct BendGroup {
            std::vector<size_t> faceIndices;  // indices into allCylFaces
            double innerRadius;
            size_t innerFaceIdx;
        };

        std::vector<BendGroup> bendGroups;

        std::vector<bool> assigned(allCylFaces.size(), false);

        for (size_t i = 0; i < allCylFaces.size(); i++) {
            if (assigned[i]) continue;

            BendGroup group;
            group.faceIndices.push_back(i);
            group.innerRadius = allCylFaces[i].radius;
            group.innerFaceIdx = i;
            assigned[i] = true;

            const gp_Ax1& axis_i = allCylFaces[i].axis;

            // Find matching outer face (same axis, different radius)
            for (size_t j = i + 1; j < allCylFaces.size(); j++) {
                if (assigned[j]) continue;

                const gp_Ax1& axis_j = allCylFaces[j].axis;

                // Check if axes are parallel (same direction)
                bool parallelDir = GeometryUtils::areDirectionsParallel(
                    axis_i.Direction(), axis_j.Direction(), 0.05);

                if (!parallelDir) continue;

                // Check if axes are colinear (same line)
                gp_Vec v(axis_i.Location(), axis_j.Location());
                double dist = 0.0;
                if (v.Magnitude() > constants::LINEAR_TOLERANCE) {
                    // Distance from axis_j location to axis_i line
                    gp_Vec cross = v.Crossed(gp_Vec(axis_i.Direction()));
                    dist = cross.Magnitude();
                }

                if (dist < 1.0) {  // within 1mm = same axis line
                    group.faceIndices.push_back(j);
                    assigned[j] = true;

                    // Track inner face (smaller radius)
                    if (allCylFaces[j].radius < group.innerRadius) {
                        group.innerRadius = allCylFaces[j].radius;
                        group.innerFaceIdx = j;
                    }
                }
            }

            bendGroups.push_back(group);
        }

        LOG_INFO("Grouped {} cylindrical faces into {} physical bends",
                 allCylFaces.size(), bendGroups.size());

        // =================================================================
        // STEP 4: For each bend group, create bend edge
        // Use the inner cylindrical face as the bend surface
        // Find 2 adjacent planar faces via CIRCULAR edges only
        // =================================================================
        LOG_DEBUG("Step 4: Creating bend edges");

        std::set<std::pair<int, int>> bendConnections;

        int groupIdx = 0;
        for (const auto& group : bendGroups) {
            const CylFaceInfo& innerFace = allCylFaces[group.innerFaceIdx];

            LOG_DEBUG("Bend group {}: R={:.2f}, {} cyl faces in group",
                     groupIdx, innerFace.radius, group.faceIndices.size());

            // Find planar faces adjacent that are base/flange (normal ⊥ bend axis)
            std::set<int> adjacentNodes = findAdjacentFlangeNodes(
                innerFace.face, innerFace.axis.Direction(),
                edgeToFaces, faceToNodeId);

            LOG_DEBUG("  Inner face -> {} adjacent flange nodes", adjacentNodes.size());
            for (int nodeId : adjacentNodes) {
                LOG_DEBUG("    Node {}", nodeId);
            }

            // If inner face doesn't give 2 nodes, try other faces in group
            if (adjacentNodes.size() != 2) {
                for (size_t idx : group.faceIndices) {
                    if (idx == group.innerFaceIdx) continue;
                    auto moreNodes = findAdjacentFlangeNodes(
                        allCylFaces[idx].face, allCylFaces[idx].axis.Direction(),
                        edgeToFaces, faceToNodeId);
                    adjacentNodes.insert(moreNodes.begin(), moreNodes.end());
                }
            }

            if (adjacentNodes.size() == 2) {
                auto it = adjacentNodes.begin();
                int n1 = std::min(*it, *std::next(it));
                int n2 = std::max(*it, *std::next(it));

                auto key = std::make_pair(n1, n2);
                if (bendConnections.find(key) == bendConnections.end()) {
                    bendConnections.insert(key);

                    int edgeId = fag.addBendEdge(n1, n2, innerFace.face);
                    m_bendEdges++;

                    LOG_DEBUG("Bend edge {}: nodes {}<->{} (R={:.2f}, group has {} cyl faces)",
                             edgeId, n1, n2, innerFace.radius, group.faceIndices.size());
                }

            } else if (adjacentNodes.size() < 2) {
                addWarning("Bend group (R=" + std::to_string(innerFace.radius) +
                          ") has only " + std::to_string(adjacentNodes.size()) +
                          " adjacent planar face(s) via circular edges");

            } else {
                // >2 adjacent nodes: pick the best pair
                // Use the 2 nodes with the smallest flange-level difference
                // For now, take the first 2
                auto it = adjacentNodes.begin();
                int n1 = std::min(*it, *std::next(it));
                int n2 = std::max(*it, *std::next(it));

                auto key = std::make_pair(n1, n2);
                if (bendConnections.find(key) == bendConnections.end()) {
                    bendConnections.insert(key);
                    int edgeId = fag.addBendEdge(n1, n2, innerFace.face);
                    m_bendEdges++;

                    addWarning("Bend group has " + std::to_string(adjacentNodes.size()) +
                              " adjacent planar faces, using first pair");
                    LOG_DEBUG("Bend edge {}: nodes {}<->{} (R={:.2f}, {} candidates)",
                             edgeId, n1, n2, innerFace.radius, adjacentNodes.size());
                }
            }
        }

        LOG_INFO("Created {} bend edges from {} physical bends",
                 m_bendEdges, bendGroups.size());

        // =================================================================
        // STEP 5: Create sharp edges for direct planar-to-planar connections
        // =================================================================
        LOG_DEBUG("Step 5: Creating sharp edges");

        m_totalEdges = edgeToFaces.Extent();

        for (int i = 1; i <= edgeToFaces.Extent(); i++) {
            const TopoDS_Edge& edge = TopoDS::Edge(edgeToFaces.FindKey(i));
            const TopTools_ListOfShape& faces = edgeToFaces.FindFromIndex(i);

            if (faces.Extent() != 2) continue;

            TopTools_ListIteratorOfListOfShape it(faces);
            TopoDS_Face face1 = TopoDS::Face(it.Value());
            it.Next();
            TopoDS_Face face2 = TopoDS::Face(it.Value());

            // Both must be planar nodes
            if (!faceToNodeId.IsBound(face1) || !faceToNodeId.IsBound(face2)) continue;

            int nodeId1 = faceToNodeId.Find(face1);
            int nodeId2 = faceToNodeId.Find(face2);
            if (nodeId1 == nodeId2) continue;

            int n1 = std::min(nodeId1, nodeId2);
            int n2 = std::max(nodeId1, nodeId2);

            // Skip if already connected by a bend
            if (bendConnections.count(std::make_pair(n1, n2))) continue;

            // Skip if already in FAG
            if (fag.findEdgeBetween(n1, n2) >= 0) continue;

            int edgeId = fag.addEdge(n1, n2, edge);
            m_sharpEdges++;

            LOG_DEBUG("Sharp edge {}: nodes {}<->{}", edgeId, n1, n2);
        }

        LOG_INFO("Created {} edges ({} bends, {} sharp)",
                 fag.edgeCount(), m_bendEdges, m_sharpEdges);

        // =================================================================
        // STEP 6: Finalize FAG
        // =================================================================
        LOG_DEBUG("Step 6: Finalizing FAG");
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

    BRepAdaptor_Curve adaptor(edge);
    GeomAbs_CurveType curveType = adaptor.GetType();

    bool isPlanar1 = SurfaceAnalyzer::isPlanar(face1);
    bool isPlanar2 = SurfaceAnalyzer::isPlanar(face2);

    DihedralAngleResult angleResult =
        AngleCalculator::calculateDihedralAngle(face1, face2, edge);
    result.angle = angleResult.openingAngle;

    if (curveType == GeomAbs_Circle) {
        auto circle = EdgeAnalyzer::getCircle(edge);
        if (circle) {
            result.radius = circle->Radius();

            if (result.radius > m_minBendRadius) {
                bool isCyl1 = SurfaceAnalyzer::isCylindrical(face1);
                bool isCyl2 = SurfaceAnalyzer::isCylindrical(face2);

                if ((isPlanar1 && isCyl2) || (isCyl1 && isPlanar2)) {
                    result.isBend = true;
                    result.confidence = 0.95;
                    result.reasoning = "Circular edge at planar-cylindrical boundary";
                } else if (isPlanar1 && isPlanar2) {
                    result.isBend = true;
                    result.confidence = 0.9;
                    result.reasoning = "Circular edge between planar faces";
                } else {
                    result.isBend = false;
                    result.confidence = 0.6;
                    result.reasoning = "Circular edge in non-standard configuration";
                }
            } else {
                result.isBend = false;
                result.confidence = 0.8;
                result.reasoning = "Radius below threshold";
            }
        }
    } else if (curveType == GeomAbs_Line) {
        result.isBend = false;
        result.radius = 0.0;
        result.confidence = 1.0;
        result.reasoning = "Linear edge";
    } else {
        result.isBend = false;
        result.radius = 0.0;
        result.confidence = 0.5;
        result.reasoning = "Non-standard curve type";
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
    oss << "  Planar faces: " << m_planarFaceCount << "\n";
    oss << "  Cylindrical faces: " << m_cylindricalFaceCount << "\n";
    oss << "  Ignored faces: " << m_ignoredFaceCount << "\n";
    oss << "  Bend edges: " << m_bendEdges << "\n";
    oss << "  Sharp edges: " << m_sharpEdges << "\n";

    if (!m_warnings.empty()) {
        oss << "  Warnings (" << m_warnings.size() << "):\n";
        for (const auto& w : m_warnings) {
            oss << "    - " << w << "\n";
        }
    }

    return oss.str();
}

} // namespace openpanelcam
