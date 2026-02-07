#pragma once

#include "types.h"
#include <vector>
#include <string>

namespace openpanelcam {
namespace phase2 {

/**
 * @brief Precedence DAG (Directed Acyclic Graph) for bend sequencing
 *
 * Manages nodes (bends) and edges (precedence constraints) to determine
 * valid bend orderings. Supports cycle detection and topological sorting.
 */
class PrecedenceDAG {
public:
    /**
     * @brief Construct empty DAG
     */
    PrecedenceDAG();

    /**
     * @brief Add a node for a bend
     * @param bendId The bend ID from Phase 1
     * @return Node ID in this DAG
     */
    int addNode(int bendId);

    /**
     * @brief Add a precedence edge (fromBend must happen before toBend)
     * @param fromBend Source bend ID
     * @param toBend Target bend ID
     * @param type Constraint type
     * @param confidence Confidence level (0-1)
     * @param reasoning Human-readable explanation
     * @return Edge ID in this DAG
     */
    int addEdge(int fromBend, int toBend, ConstraintType type,
                double confidence, const std::string& reasoning);

    /**
     * @brief Get node by ID (const version)
     * @param nodeId Node ID
     * @return Pointer to node, or nullptr if not found
     */
    const PrecedenceNode* getNode(int nodeId) const;

    /**
     * @brief Get node by bend ID
     * @param bendId Bend ID from Phase 1
     * @return Pointer to node, or nullptr if not found
     */
    const PrecedenceNode* getNodeByBendId(int bendId) const;

    /**
     * @brief Get edge by ID (const version)
     * @param edgeId Edge ID
     * @return Pointer to edge, or nullptr if not found
     */
    const PrecedenceEdge* getEdge(int edgeId) const;

    /**
     * @brief Get number of nodes
     */
    int nodeCount() const;

    /**
     * @brief Get number of edges
     */
    int edgeCount() const;

    /**
     * @brief Check if DAG has been finalized
     */
    bool isFinalized() const;

    /**
     * @brief Clear all nodes and edges
     */
    void clear();

    /**
     * @brief Finalize DAG (prepares for cycle detection and toposort)
     * @return true if successful
     */
    bool finalize();

    /**
     * @brief Detect cycles in the DAG using DFS
     * @return true if acyclic (no cycles), false if cycles found
     */
    bool isAcyclic();

    /**
     * @brief Resolve cycles by removing lowest confidence edges
     *
     * Iteratively removes the lowest confidence edge from each detected
     * cycle until the graph becomes acyclic. The algorithm:
     * 1. While graph has cycles
     * 2. Find all edges involved in cycles
     * 3. Find lowest confidence edge
     * 4. Remove that edge
     * 5. Rebuild adjacency lists
     * 6. Repeat until acyclic
     *
     * @return Vector of removed edges
     */
    std::vector<PrecedenceEdge> resolveCycles();

    /**
     * @brief Get topological ordering using Kahn's algorithm
     * @return Vector of node IDs in valid topological order (empty if cyclic)
     */
    std::vector<int> topologicalSort();

private:
    std::vector<PrecedenceNode> m_nodes;
    std::vector<PrecedenceEdge> m_edges;
    bool m_finalized;

    // Helper for cycle detection (DFS)
    bool hasCycleDFS(int nodeId, std::vector<int>& state);

    // Helper for finding edges in cycles
    std::vector<int> findEdgesInCycles();

    // Helper for removing an edge by index
    void removeEdge(int edgeIndex);

    // Helper for rebuilding adjacency lists after edge removal
    void rebuildAdjacencyLists();
};

} // namespace phase2
} // namespace openpanelcam
