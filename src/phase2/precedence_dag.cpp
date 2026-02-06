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
    // Use a visit counter to prevent infinite loops in case of cycles
    std::vector<int> visitCount(m_nodes.size(), 0);
    int maxVisits = static_cast<int>(m_nodes.size()) * static_cast<int>(m_nodes.size()) + 1;
    int totalVisits = 0;

    std::queue<int> queue;
    for (int rootId : roots) {
        queue.push(rootId);
    }

    while (!queue.empty() && totalVisits < maxVisits) {
        int currentId = queue.front();
        queue.pop();

        PrecedenceNode& currentNode = m_nodes[currentId];
        visitCount[currentId]++;
        totalVisits++;

        // Process all successors
        for (int successorBendId : currentNode.successors) {
            // Find successor node by bendId
            for (auto& node : m_nodes) {
                if (node.bendId == successorBendId) {
                    // Update level to max of (current level, predecessor level + 1)
                    int newLevel = currentNode.level + 1;
                    if (newLevel > node.level) {
                        node.level = newLevel;
                        // Only queue if not visited too many times (prevents cycle infinite loop)
                        if (visitCount[node.id] < static_cast<int>(m_nodes.size())) {
                            queue.push(node.id);
                        }
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
    // Use DFS-based cycle detection with 3-color algorithm
    // State: 0 = unvisited (white), 1 = visiting (gray), 2 = visited (black)

    std::vector<int> state(m_nodes.size(), 0);  // Initialize all as unvisited

    // Try DFS from each unvisited node
    for (size_t i = 0; i < m_nodes.size(); i++) {
        if (state[i] == 0) {  // Unvisited
            if (hasCycleDFS(static_cast<int>(i), state)) {
                return false;  // Cycle detected
            }
        }
    }

    return true;  // No cycles found
}

std::vector<int> PrecedenceDAG::topologicalSort() {
    // Kahn's algorithm for topological sorting
    // Returns bendIds in valid topological order, or empty vector if graph has cycles

    std::vector<int> result;

    // First check if graph is acyclic
    if (!isAcyclic()) {
        return result;  // Return empty - cannot sort cyclic graph
    }

    if (m_nodes.empty()) {
        return result;  // Empty graph
    }

    // Calculate in-degree for each node (count of predecessors)
    std::vector<int> inDegree(m_nodes.size(), 0);
    for (const auto& node : m_nodes) {
        inDegree[node.id] = static_cast<int>(node.predecessors.size());
    }

    // Queue of nodes with in-degree 0 (ready to process)
    std::queue<int> queue;
    for (size_t i = 0; i < m_nodes.size(); i++) {
        if (inDegree[i] == 0) {
            queue.push(static_cast<int>(i));
        }
    }

    // Process nodes in topological order
    while (!queue.empty()) {
        int currentId = queue.front();
        queue.pop();

        const PrecedenceNode& currentNode = m_nodes[currentId];
        result.push_back(currentNode.bendId);  // Add bendId to result

        // Decrease in-degree of all successors
        for (int successorBendId : currentNode.successors) {
            // Find successor node by bendId
            for (size_t i = 0; i < m_nodes.size(); i++) {
                if (m_nodes[i].bendId == successorBendId) {
                    inDegree[i]--;

                    // If in-degree becomes 0, add to queue
                    if (inDegree[i] == 0) {
                        queue.push(static_cast<int>(i));
                    }
                    break;
                }
            }
        }
    }

    // If result doesn't contain all nodes, graph had a cycle
    // (This shouldn't happen since we checked isAcyclic first)
    if (result.size() != m_nodes.size()) {
        result.clear();
        return result;
    }

    return result;
}

bool PrecedenceDAG::hasCycleDFS(int nodeId, std::vector<int>& state) {
    // Mark current node as visiting (gray)
    state[nodeId] = 1;

    const PrecedenceNode& node = m_nodes[nodeId];

    // Visit all successors
    for (int successorBendId : node.successors) {
        // Find successor node by bendId
        int successorNodeId = -1;
        for (size_t i = 0; i < m_nodes.size(); i++) {
            if (m_nodes[i].bendId == successorBendId) {
                successorNodeId = static_cast<int>(i);
                break;
            }
        }

        if (successorNodeId == -1) {
            continue;  // Successor not found (shouldn't happen)
        }

        if (state[successorNodeId] == 1) {
            // Visiting node again - cycle detected!
            return true;
        }

        if (state[successorNodeId] == 0) {
            // Unvisited - recurse
            if (hasCycleDFS(successorNodeId, state)) {
                return true;
            }
        }

        // If state == 2 (visited), no need to check again
    }

    // Mark current node as visited (black)
    state[nodeId] = 2;

    return false;  // No cycle found from this node
}

std::vector<int> PrecedenceDAG::findEdgesInCycles() {
    std::vector<int> edgesInCycles;

    // For each edge, check if removing it would affect cycle detection
    // An edge is in a cycle if there's a path from toBend back to fromBend
    for (size_t edgeIdx = 0; edgeIdx < m_edges.size(); edgeIdx++) {
        const PrecedenceEdge& edge = m_edges[edgeIdx];

        // Find the target node
        int targetNodeId = -1;
        int sourceNodeId = -1;
        for (size_t i = 0; i < m_nodes.size(); i++) {
            if (m_nodes[i].bendId == edge.toBend) {
                targetNodeId = static_cast<int>(i);
            }
            if (m_nodes[i].bendId == edge.fromBend) {
                sourceNodeId = static_cast<int>(i);
            }
        }

        if (targetNodeId == -1 || sourceNodeId == -1) {
            continue;
        }

        // BFS to check if there's a path from toBend to fromBend
        std::vector<bool> visited(m_nodes.size(), false);
        std::queue<int> queue;
        queue.push(targetNodeId);
        visited[targetNodeId] = true;

        bool foundPath = false;
        while (!queue.empty() && !foundPath) {
            int currentId = queue.front();
            queue.pop();

            const PrecedenceNode& currentNode = m_nodes[currentId];

            for (int successorBendId : currentNode.successors) {
                // Find successor node
                for (size_t i = 0; i < m_nodes.size(); i++) {
                    if (m_nodes[i].bendId == successorBendId) {
                        if (static_cast<int>(i) == sourceNodeId) {
                            // Found path back to source - this edge is in a cycle
                            foundPath = true;
                            break;
                        }
                        if (!visited[i]) {
                            visited[i] = true;
                            queue.push(static_cast<int>(i));
                        }
                        break;
                    }
                }
                if (foundPath) break;
            }
        }

        if (foundPath) {
            edgesInCycles.push_back(static_cast<int>(edgeIdx));
        }
    }

    return edgesInCycles;
}

void PrecedenceDAG::removeEdge(int edgeIndex) {
    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(m_edges.size())) {
        return;
    }

    // Remove the edge from the vector
    m_edges.erase(m_edges.begin() + edgeIndex);

    // Update edge IDs for remaining edges
    for (size_t i = 0; i < m_edges.size(); i++) {
        m_edges[i].id = static_cast<int>(i);
    }
}

void PrecedenceDAG::rebuildAdjacencyLists() {
    // Clear all adjacency lists
    for (auto& node : m_nodes) {
        node.successors.clear();
        node.predecessors.clear();
    }

    // Rebuild from edges
    for (const auto& edge : m_edges) {
        for (auto& node : m_nodes) {
            if (node.bendId == edge.fromBend) {
                node.successors.push_back(edge.toBend);
            }
            if (node.bendId == edge.toBend) {
                node.predecessors.push_back(edge.fromBend);
            }
        }
    }

    m_finalized = false;
}

std::vector<PrecedenceEdge> PrecedenceDAG::resolveCycles() {
    std::vector<PrecedenceEdge> removedEdges;

    // Handle empty graph
    if (m_nodes.empty()) {
        return removedEdges;
    }

    // Iteratively remove lowest confidence edges until acyclic
    // Add a safety limit to prevent infinite loops
    int maxIterations = static_cast<int>(m_edges.size()) + 1;
    int iteration = 0;

    while (!isAcyclic() && iteration < maxIterations) {
        iteration++;

        // Find all edges that are part of cycles
        std::vector<int> edgesInCycles = findEdgesInCycles();

        if (edgesInCycles.empty()) {
            // No edges in cycles found but graph is cyclic - shouldn't happen
            break;
        }

        // Find the edge with lowest confidence among those in cycles
        int lowestConfidenceIdx = edgesInCycles[0];
        double lowestConfidence = m_edges[lowestConfidenceIdx].confidence;

        for (int edgeIdx : edgesInCycles) {
            if (m_edges[edgeIdx].confidence < lowestConfidence) {
                lowestConfidence = m_edges[edgeIdx].confidence;
                lowestConfidenceIdx = edgeIdx;
            }
        }

        // Store the removed edge before removing it
        removedEdges.push_back(m_edges[lowestConfidenceIdx]);

        // Remove the edge
        removeEdge(lowestConfidenceIdx);

        // Rebuild adjacency lists
        rebuildAdjacencyLists();
    }

    // Now that graph is acyclic (or we gave up), finalize it
    if (isAcyclic()) {
        finalize();
    }

    return removedEdges;
}

} // namespace phase2
} // namespace openpanelcam
