/**
 * @file fag.cpp
 * @brief Face-Adjacency Graph implementation
 */

#include <openpanelcam/phase1/fag.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/geometry_utils.h>

#include <BRepBndLib.hxx>

#include <queue>
#include <algorithm>

namespace openpanelcam {

//=============================================================================
// FaceAdjacencyGraph Implementation
//=============================================================================

FaceAdjacencyGraph::FaceAdjacencyGraph()
    : m_baseFaceId(-1)
    , m_finalized(false)
{
}

void FaceAdjacencyGraph::clear() {
    m_nodes.clear();
    m_edges.clear();
    m_nodePairToEdge.clear();
    m_baseFaceId = -1;
    m_finalized = false;
}

void FaceAdjacencyGraph::reserve(size_t nodeCount, size_t edgeCount) {
    m_nodes.reserve(nodeCount);
    m_edges.reserve(edgeCount);
}

int FaceAdjacencyGraph::addNode(const TopoDS_Face& face) {
    if (face.IsNull()) {
        LOG_WARNING("Attempting to add null face to FAG");
        return -1;
    }

    // Create new node
    FAG_Node node;
    node.id = static_cast<int>(m_nodes.size());
    node.face = face;
    node.name = "Face_" + std::to_string(node.id);

    // Add to graph
    m_nodes.push_back(node);

    LOG_DEBUG("Added node {} to FAG", node.id);

    return node.id;
}

int FaceAdjacencyGraph::addEdge(int node1, int node2, const TopoDS_Edge& sharedEdge) {
    // Validate node IDs
    if (node1 < 0 || node1 >= static_cast<int>(m_nodes.size()) ||
        node2 < 0 || node2 >= static_cast<int>(m_nodes.size())) {
        LOG_ERROR("Invalid node IDs for edge: {} <-> {}", node1, node2);
        return -1;
    }

    if (node1 == node2) {
        LOG_WARNING("Self-loop detected: node {}", node1);
        return -1;
    }

    // Create edge
    FAG_Edge edge;
    edge.id = static_cast<int>(m_edges.size());
    edge.node1 = std::min(node1, node2);  // Normalize order
    edge.node2 = std::max(node1, node2);
    edge.isBend = false;
    edge.sharedEdge = sharedEdge;

    // Compute edge length
    if (!sharedEdge.IsNull()) {
        edge.edgeLength = EdgeAnalyzer::computeLength(sharedEdge);
        edge.edgeMidpoint = EdgeAnalyzer::getMidpoint(sharedEdge);
    }

    // Add to graph
    m_edges.push_back(edge);

    // Update lookup maps
    auto key = std::make_pair(edge.node1, edge.node2);
    m_nodePairToEdge[key] = edge.id;

    // Update node adjacency
    m_nodes[node1].adjacentEdges.push_back(edge.id);
    m_nodes[node2].adjacentEdges.push_back(edge.id);

    LOG_DEBUG("Added sharp edge {} between nodes {} and {}", edge.id, node1, node2);

    return edge.id;
}

int FaceAdjacencyGraph::addBendEdge(int node1, int node2, const TopoDS_Face& bendFace) {
    // Validate node IDs
    if (node1 < 0 || node1 >= static_cast<int>(m_nodes.size()) ||
        node2 < 0 || node2 >= static_cast<int>(m_nodes.size())) {
        LOG_ERROR("Invalid node IDs for bend edge: {} <-> {}", node1, node2);
        return -1;
    }

    if (node1 == node2) {
        LOG_WARNING("Self-loop detected: node {}", node1);
        return -1;
    }

    // Create bend edge
    FAG_Edge edge;
    edge.id = static_cast<int>(m_edges.size());
    edge.node1 = std::min(node1, node2);  // Normalize order
    edge.node2 = std::max(node1, node2);
    edge.isBend = true;
    edge.bendFace = bendFace;

    // Add to graph
    m_edges.push_back(edge);

    // Update lookup maps
    auto key = std::make_pair(edge.node1, edge.node2);
    m_nodePairToEdge[key] = edge.id;

    // Update node adjacency
    m_nodes[node1].adjacentEdges.push_back(edge.id);
    m_nodes[node2].adjacentEdges.push_back(edge.id);

    LOG_DEBUG("Added bend edge {} between nodes {} and {}", edge.id, node1, node2);

    return edge.id;
}

void FaceAdjacencyGraph::finalize() {
    LOG_INFO("Finalizing FAG: {} nodes, {} edges", m_nodes.size(), m_edges.size());

    // Compute geometric properties for all nodes
    for (auto& node : m_nodes) {
        computeNodeGeometry(node);
    }

    // Compute properties for all bend edges
    for (auto& edge : m_edges) {
        if (edge.isBend) {
            computeBendProperties(edge);
        }
    }

    m_finalized = true;
    LOG_INFO("FAG finalization complete");
}

void FaceAdjacencyGraph::computeNodeGeometry(FAG_Node& node) {
    if (node.face.IsNull()) {
        LOG_WARNING("Cannot compute geometry for null face (node {})", node.id);
        return;
    }

    // Analyze surface
    SurfaceProperties props = SurfaceAnalyzer::analyze(node.face);

    node.type = props.type;
    node.centroid = props.centroid;
    node.area = props.area;
    node.boundingBox = props.boundingBox;

    if (node.type == FaceType::PLANAR) {
        node.plane = props.plane;
        node.normal = props.normal;
        node.isBendSurface = false;
    } else if (node.type == FaceType::CYLINDRICAL) {
        node.cylinderAxis = props.axis;
        node.cylinderRadius = props.radius;
        node.isBendSurface = true;
    }

    LOG_DEBUG("Node {}: type={}, area={:.2f} mm²",
              node.id,
              static_cast<int>(node.type),
              node.area);
}

void FaceAdjacencyGraph::computeBendProperties(FAG_Edge& edge) {
    if (!edge.isBend || edge.bendFace.IsNull()) {
        return;
    }

    // Extract bend axis from cylindrical face
    edge.bendAxis = SurfaceAnalyzer::extractBendAxis(edge.bendFace);

    // Get cylinder properties
    auto cylinder = SurfaceAnalyzer::getCylinder(edge.bendFace);
    if (cylinder) {
        edge.bendRadius = cylinder->Radius();
    }

    // Get UV bounds to compute bend line endpoints
    double uMin, uMax, vMin, vMax;
    SurfaceAnalyzer::getUVBounds(edge.bendFace, uMin, uMax, vMin, vMax);

    Handle(Geom_Surface) surf = BRep_Tool::Surface(edge.bendFace);
    edge.bendAxisStart = surf->Value(0, vMin);
    edge.bendAxisEnd = surf->Value(0, vMax);
    edge.bendLength = edge.bendAxisStart.Distance(edge.bendAxisEnd);

    // Compute dihedral angle if both faces are planar
    const FAG_Node& n1 = m_nodes[edge.node1];
    const FAG_Node& n2 = m_nodes[edge.node2];

    if (n1.type == FaceType::PLANAR && n2.type == FaceType::PLANAR) {
        // Angle between normals
        double cosAngle = n1.normal.Dot(n2.normal);
        cosAngle = GeometryUtils::clamp(cosAngle, -1.0, 1.0);
        double angle = std::acos(cosAngle);
        edge.bendAngle = angle * 180.0 / M_PI;

        // Classify bend type
        if (edge.bendAngle < constants::HEM_ANGLE_THRESHOLD) {
            edge.bendType = FAG_Edge::BendType::HEM;
        } else if (std::abs(edge.bendAngle - 90.0) < constants::RIGHT_ANGLE_TOLERANCE) {
            edge.bendType = FAG_Edge::BendType::RIGHT;
        } else if (edge.bendAngle < 90.0) {
            edge.bendType = FAG_Edge::BendType::ACUTE;
        } else {
            edge.bendType = FAG_Edge::BendType::OBTUSE;
        }

        LOG_DEBUG("Bend edge {}: angle={:.1f}°, radius={:.2f} mm, type={}",
                  edge.id,
                  edge.bendAngle,
                  edge.bendRadius,
                  static_cast<int>(edge.bendType));
    }
}

const FAG_Node& FaceAdjacencyGraph::getNode(int id) const {
    if (id < 0 || id >= static_cast<int>(m_nodes.size())) {
        THROW_PHASE1(ErrorCode::FAG_INVALID_NODE_ID,
                    "Invalid node ID",
                    "Node ID " + std::to_string(id) + " out of range");
    }
    return m_nodes[id];
}

FAG_Node& FaceAdjacencyGraph::getNode(int id) {
    if (id < 0 || id >= static_cast<int>(m_nodes.size())) {
        THROW_PHASE1(ErrorCode::FAG_INVALID_NODE_ID,
                    "Invalid node ID",
                    "Node ID " + std::to_string(id) + " out of range");
    }
    return m_nodes[id];
}

const FAG_Edge& FaceAdjacencyGraph::getEdge(int id) const {
    if (id < 0 || id >= static_cast<int>(m_edges.size())) {
        THROW_PHASE1(ErrorCode::FAG_INVALID_EDGE_ID,
                    "Invalid edge ID",
                    "Edge ID " + std::to_string(id) + " out of range");
    }
    return m_edges[id];
}

FAG_Edge& FaceAdjacencyGraph::getEdge(int id) {
    if (id < 0 || id >= static_cast<int>(m_edges.size())) {
        THROW_PHASE1(ErrorCode::FAG_INVALID_EDGE_ID,
                    "Invalid edge ID",
                    "Edge ID " + std::to_string(id) + " out of range");
    }
    return m_edges[id];
}

std::vector<int> FaceAdjacencyGraph::getEdgesForNode(int nodeId) const {
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) {
        return {};
    }
    return m_nodes[nodeId].adjacentEdges;
}

std::vector<int> FaceAdjacencyGraph::getAdjacentNodes(int nodeId) const {
    std::vector<int> adjacent;

    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) {
        return adjacent;
    }

    for (int edgeId : m_nodes[nodeId].adjacentEdges) {
        const FAG_Edge& edge = m_edges[edgeId];
        int otherNode = edge.otherNode(nodeId);
        adjacent.push_back(otherNode);
    }

    return adjacent;
}

std::vector<int> FaceAdjacencyGraph::getBendAdjacentNodes(int nodeId) const {
    std::vector<int> adjacent;

    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) {
        return adjacent;
    }

    for (int edgeId : m_nodes[nodeId].adjacentEdges) {
        const FAG_Edge& edge = m_edges[edgeId];
        if (edge.isBend) {
            int otherNode = edge.otherNode(nodeId);
            adjacent.push_back(otherNode);
        }
    }

    return adjacent;
}

int FaceAdjacencyGraph::findEdgeBetween(int node1, int node2) const {
    int n1 = std::min(node1, node2);
    int n2 = std::max(node1, node2);

    auto key = std::make_pair(n1, n2);
    auto it = m_nodePairToEdge.find(key);

    if (it != m_nodePairToEdge.end()) {
        return it->second;
    }

    return -1;
}

std::vector<int> FaceAdjacencyGraph::getBendEdges() const {
    std::vector<int> bendEdges;

    for (int i = 0; i < static_cast<int>(m_edges.size()); i++) {
        if (m_edges[i].isBend) {
            bendEdges.push_back(i);
        }
    }

    return bendEdges;
}

size_t FaceAdjacencyGraph::bendCount() const {
    size_t count = 0;
    for (const auto& edge : m_edges) {
        if (edge.isBend) {
            count++;
        }
    }
    return count;
}

void FaceAdjacencyGraph::setBaseFace(int nodeId) {
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) {
        LOG_ERROR("Invalid base face ID: {}", nodeId);
        return;
    }

    m_baseFaceId = nodeId;
    m_nodes[nodeId].isBaseFace = true;
    m_nodes[nodeId].flangeLevel = 0;

    LOG_INFO("Set base face: node {}", nodeId);

    // Compute flange levels via BFS
    computeFlangeLevels();
}

void FaceAdjacencyGraph::computeFlangeLevels() {
    if (m_baseFaceId < 0) {
        return;
    }

    // Reset all levels
    for (auto& node : m_nodes) {
        node.flangeLevel = -1;
        node.isFlange = false;
    }

    // BFS from base face
    std::queue<int> queue;
    queue.push(m_baseFaceId);
    m_nodes[m_baseFaceId].flangeLevel = 0;

    while (!queue.empty()) {
        int currentId = queue.front();
        queue.pop();

        int currentLevel = m_nodes[currentId].flangeLevel;

        // Visit all bend-adjacent nodes
        for (int neighborId : getBendAdjacentNodes(currentId)) {
            if (m_nodes[neighborId].flangeLevel < 0) {
                // Not visited yet
                m_nodes[neighborId].flangeLevel = currentLevel + 1;
                m_nodes[neighborId].isFlange = true;
                queue.push(neighborId);

                LOG_DEBUG("Node {} -> flange level {}", neighborId, currentLevel + 1);
            }
        }
    }
}

bool FaceAdjacencyGraph::isConnected() const {
    if (m_nodes.empty()) {
        return true;
    }

    // BFS to check connectivity
    std::vector<bool> visited(m_nodes.size(), false);
    std::queue<int> queue;

    queue.push(0);
    visited[0] = true;
    int visitedCount = 1;

    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();

        for (int neighbor : getAdjacentNodes(current)) {
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                visitedCount++;
                queue.push(neighbor);
            }
        }
    }

    return visitedCount == static_cast<int>(m_nodes.size());
}

Bnd_Box FaceAdjacencyGraph::getPartBoundingBox() const {
    Bnd_Box box;

    for (const auto& node : m_nodes) {
        if (!node.face.IsNull()) {
            BRepBndLib::Add(node.face, box);
        }
    }

    return box;
}

ValidationResult FaceAdjacencyGraph::validate() const {
    ValidationResult result;
    result.isValid = true;

    // Check if finalized
    if (!m_finalized) {
        result.warnings.push_back("FAG not finalized");
    }

    // Check node count
    if (m_nodes.empty()) {
        result.isValid = false;
        result.errorMessage = "FAG has no nodes";
        return result;
    }

    // Check for null faces
    for (const auto& node : m_nodes) {
        if (node.face.IsNull()) {
            result.warnings.push_back("Node " + std::to_string(node.id) + " has null face");
        }
    }

    // Check connectivity
    if (!isConnected()) {
        result.warnings.push_back("FAG is not fully connected");
    }

    // Check bend count
    if (bendCount() == 0) {
        result.warnings.push_back("No bends found in FAG");
    }

    return result;
}

bool FaceAdjacencyGraph::isValid() const {
    return !m_nodes.empty() && m_finalized;
}

} // namespace openpanelcam
