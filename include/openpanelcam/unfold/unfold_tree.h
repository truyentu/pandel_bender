#pragma once

#include <openpanelcam/unfold/types.h>
#include <gp_Trsf.hxx>
#include <vector>
#include <set>

namespace openpanelcam {

class FaceAdjacencyGraph;

/**
 * @brief Strategy for building the unfold spanning tree from FAG
 */
enum class TreeBuildStrategy {
    BFS,          ///< Simple BFS traversal (original, order-dependent)
    MST_PRIM      ///< Minimum Spanning Tree via Prim's algorithm (recommended)
};

struct UnfoldTreeNode {
    int faceId = -1;
    int parentIndex = -1;        // Index in tree.nodes (-1 for root)
    int connectingEdgeId = -1;   // FAG edge ID connecting to parent
    double bendAngle = 0.0;      // degrees
    double bendRadius = 0.0;     // mm
    gp_Trsf accumulatedTransform; // Cumulative transform from root
    std::vector<int> childIndices; // Indices in tree.nodes
};

struct UnfoldTree {
    std::vector<UnfoldTreeNode> nodes;
    int rootIndex = -1;
    TreeBuildStrategy strategyUsed = TreeBuildStrategy::BFS;
    double totalEdgeWeight = 0.0; ///< Sum of edge weights in the spanning tree
};

class UnfoldTreeBuilder {
public:
    UnfoldTreeBuilder();

    /// Build unfold tree with specified strategy (default: MST_PRIM)
    UnfoldTree build(const FaceAdjacencyGraph& fag, int baseFaceId,
                     TreeBuildStrategy strategy = TreeBuildStrategy::MST_PRIM);

    /**
     * @brief Compute edge weight for MST selection
     *
     * Weight formula from literature:
     *   w(edge) = bendLength * angleFactor
     *   angleFactor = 1.0 + |bendAngle - 90| / 180.0
     *
     * Lower weight = preferred edge (shorter bend lines and angles
     * closer to 90° produce better unfold results).
     */
    static double computeEdgeWeight(double bendLength, double bendAngle);

private:
    void buildBFS(const FaceAdjacencyGraph& fag, int baseFaceId, UnfoldTree& tree);
    void buildMSTPrim(const FaceAdjacencyGraph& fag, int baseFaceId, UnfoldTree& tree);
};

} // namespace openpanelcam
