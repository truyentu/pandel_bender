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
    if (m_finalized) {
        return true;  // Already finalized
    }

    // Reset all node levels
    for (auto& node : m_nodes) {
        node.level = 0;
        node.visited = false;
    }

    // Calculate levels using BFS approach
    // Level = longest path from any root node to this node

    // First, find all root nodes (no predecessors)
    std::vector<int> roots;
    for (const auto& node : m_nodes) {
        if (node.predecessors.empty()) {
            roots.push_back(node.id);
        }
    }

    // If no roots found, all nodes are in cycles or graph is empty
    if (roots.empty() && !m_nodes.empty()) {
        return false;  // Cannot finalize - likely has cycles
    }

    // BFS from all roots to calculate levels
    std::queue<int> queue;
    for (int rootId : roots) {
        queue.push(rootId);
    }

    while (!queue.empty()) {
        int currentId = queue.front();
        queue.pop();

        PrecedenceNode& currentNode = m_nodes[currentId];

        // Process all successors
        for (int successorBendId : currentNode.successors) {
            // Find successor node by bendId
            for (auto& node : m_nodes) {
                if (node.bendId == successorBendId) {
                    // Update level to max of (current level, predecessor level + 1)
                    int newLevel = currentNode.level + 1;
                    if (newLevel > node.level) {
                        node.level = newLevel;
                        queue.push(node.id);
                    }
                    break;
                }
            }
        }
    }

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
