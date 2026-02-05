#include "openpanelcam/phase2/precedence_dag.h"
#include <algorithm>
#include <queue>

namespace openpanelcam {
namespace phase2 {

PrecedenceDAG::PrecedenceDAG()
    : m_finalized(false) {
}

int PrecedenceDAG::addNode(int bendId) {
    PrecedenceNode node;
    node.id = static_cast<int>(m_nodes.size());
    node.bendId = bendId;
    node.level = 0;
    node.visited = false;

    m_nodes.push_back(node);
    return node.id;
}

int PrecedenceDAG::addEdge(int fromBend, int toBend, ConstraintType type,
                            double confidence, const std::string& reasoning) {
    PrecedenceEdge edge;
    edge.id = static_cast<int>(m_edges.size());
    edge.fromBend = fromBend;
    edge.toBend = toBend;
    edge.type = type;
    edge.confidence = confidence;
    edge.reasoning = reasoning;

    m_edges.push_back(edge);

    // Update node adjacency lists
    for (auto& node : m_nodes) {
        if (node.bendId == fromBend) {
            node.successors.push_back(toBend);
        }
        if (node.bendId == toBend) {
            node.predecessors.push_back(fromBend);
        }
    }

    return edge.id;
}

const PrecedenceNode* PrecedenceDAG::getNode(int nodeId) const {
    if (nodeId < 0 || nodeId >= static_cast<int>(m_nodes.size())) {
        return nullptr;
    }
    return &m_nodes[nodeId];
}

const PrecedenceEdge* PrecedenceDAG::getEdge(int edgeId) const {
    if (edgeId < 0 || edgeId >= static_cast<int>(m_edges.size())) {
        return nullptr;
    }
    return &m_edges[edgeId];
}

int PrecedenceDAG::nodeCount() const {
    return static_cast<int>(m_nodes.size());
}

int PrecedenceDAG::edgeCount() const {
    return static_cast<int>(m_edges.size());
}

bool PrecedenceDAG::isFinalized() const {
    return m_finalized;
}

void PrecedenceDAG::clear() {
    m_nodes.clear();
    m_edges.clear();
    m_finalized = false;
}

bool PrecedenceDAG::finalize() {
    m_finalized = true;
    return true;
}

bool PrecedenceDAG::isAcyclic() {
    // Will implement in Task 8
    return true;
}

std::vector<int> PrecedenceDAG::topologicalSort() {
    // Will implement in Task 9
    return std::vector<int>();
}

bool PrecedenceDAG::hasCycleDFS(int nodeId, std::vector<int>& state) {
    // Will implement in Task 8
    return false;
}

} // namespace phase2
} // namespace openpanelcam
