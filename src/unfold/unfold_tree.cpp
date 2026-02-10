#include <openpanelcam/unfold/unfold_tree.h>
#include <openpanelcam/phase1/fag.h>
#include <queue>
#include <cmath>
#include <limits>
#include <functional>

namespace openpanelcam {

UnfoldTreeBuilder::UnfoldTreeBuilder() = default;

UnfoldTree UnfoldTreeBuilder::build(const FaceAdjacencyGraph& fag, int baseFaceId,
                                     TreeBuildStrategy strategy) {
    UnfoldTree tree;
    tree.strategyUsed = strategy;

    switch (strategy) {
        case TreeBuildStrategy::MST_PRIM:
            buildMSTPrim(fag, baseFaceId, tree);
            break;
        case TreeBuildStrategy::BFS:
        default:
            buildBFS(fag, baseFaceId, tree);
            break;
    }

    return tree;
}

double UnfoldTreeBuilder::computeEdgeWeight(double bendLength, double bendAngle) {
    // angleFactor: edges closer to 90° are preferred (lower weight)
    double angleFactor = 1.0 + std::abs(bendAngle - 90.0) / 180.0;

    // Guard against zero/negative bendLength
    double length = (bendLength > 0.0) ? bendLength : 1.0;

    return length * angleFactor;
}

// =============================================================================
// BFS (original algorithm, kept for backward compatibility)
// =============================================================================

void UnfoldTreeBuilder::buildBFS(
    const FaceAdjacencyGraph& fag, int baseFaceId, UnfoldTree& tree)
{
    std::set<int> visited;
    std::queue<int> bfsQueue; // queue of tree node indices

    // Create root node
    UnfoldTreeNode root;
    root.faceId = baseFaceId;
    root.parentIndex = -1;
    // Identity transform (root stays in place)

    tree.nodes.push_back(root);
    tree.rootIndex = 0;
    visited.insert(baseFaceId);
    bfsQueue.push(0);

    while (!bfsQueue.empty()) {
        int currentTreeIdx = bfsQueue.front();
        bfsQueue.pop();

        int currentFaceId = tree.nodes[currentTreeIdx].faceId;

        // Get all edges for this node
        auto edgeIds = fag.getEdgesForNode(currentFaceId);

        for (int edgeId : edgeIds) {
            const auto& edge = fag.getEdge(edgeId);

            // Only traverse bend edges
            if (!edge.isBend) continue;

            int neighborFaceId = edge.otherNode(currentFaceId);
            if (visited.count(neighborFaceId)) continue;
            visited.insert(neighborFaceId);

            // Create child node
            UnfoldTreeNode child;
            child.faceId = neighborFaceId;
            child.parentIndex = currentFaceId;
            child.connectingEdgeId = edgeId;
            child.bendAngle = edge.bendAngle;
            child.bendRadius = edge.bendRadius;

            int childTreeIdx = static_cast<int>(tree.nodes.size());
            tree.nodes.push_back(child);
            tree.nodes[currentTreeIdx].childIndices.push_back(childTreeIdx);
            bfsQueue.push(childTreeIdx);
        }
    }
}

// =============================================================================
// MST Prim's Algorithm
//
// Builds a Minimum Spanning Tree rooted at baseFaceId.
// Edge weight: w = bendLength * (1 + |angle - 90| / 180)
//
// Shorter bend lines and angles closer to 90° get lower weights,
// producing unfold trees that minimize total cut length and favor
// standard right-angle bends.
// =============================================================================

void UnfoldTreeBuilder::buildMSTPrim(
    const FaceAdjacencyGraph& fag, int baseFaceId, UnfoldTree& tree)
{
    // Priority queue entry: (weight, faceId, parentFaceId, edgeId)
    struct PrimEntry {
        double weight;
        int faceId;
        int parentFaceId;
        int edgeId;

        bool operator>(const PrimEntry& other) const {
            return weight > other.weight;
        }
    };

    std::set<int> inTree;
    std::priority_queue<PrimEntry, std::vector<PrimEntry>, std::greater<PrimEntry>> pq;

    // Map faceId -> tree node index for parent lookup
    std::unordered_map<int, int> faceToTreeIdx;

    // Create root node
    UnfoldTreeNode root;
    root.faceId = baseFaceId;
    root.parentIndex = -1;

    tree.nodes.push_back(root);
    tree.rootIndex = 0;
    tree.totalEdgeWeight = 0.0;
    inTree.insert(baseFaceId);
    faceToTreeIdx[baseFaceId] = 0;

    // Seed the priority queue with all bend edges from root
    auto rootEdges = fag.getEdgesForNode(baseFaceId);
    for (int edgeId : rootEdges) {
        const auto& edge = fag.getEdge(edgeId);
        if (!edge.isBend) continue;

        int neighbor = edge.otherNode(baseFaceId);
        double w = computeEdgeWeight(edge.bendLength, edge.bendAngle);
        pq.push({w, neighbor, baseFaceId, edgeId});
    }

    // Prim's main loop
    while (!pq.empty()) {
        auto [weight, faceId, parentFaceId, edgeId] = pq.top();
        pq.pop();

        // Skip if already in tree
        if (inTree.count(faceId)) continue;
        inTree.insert(faceId);

        // Find parent tree index
        int parentTreeIdx = faceToTreeIdx[parentFaceId];

        // Get edge data
        const auto& edge = fag.getEdge(edgeId);

        // Create child node
        UnfoldTreeNode child;
        child.faceId = faceId;
        child.parentIndex = parentFaceId;
        child.connectingEdgeId = edgeId;
        child.bendAngle = edge.bendAngle;
        child.bendRadius = edge.bendRadius;

        int childTreeIdx = static_cast<int>(tree.nodes.size());
        tree.nodes.push_back(child);
        tree.nodes[parentTreeIdx].childIndices.push_back(childTreeIdx);
        faceToTreeIdx[faceId] = childTreeIdx;

        tree.totalEdgeWeight += weight;

        // Add all bend edges from newly added face
        auto neighborEdges = fag.getEdgesForNode(faceId);
        for (int nEdgeId : neighborEdges) {
            const auto& nEdge = fag.getEdge(nEdgeId);
            if (!nEdge.isBend) continue;

            int nextFace = nEdge.otherNode(faceId);
            if (inTree.count(nextFace)) continue;

            double w = computeEdgeWeight(nEdge.bendLength, nEdge.bendAngle);
            pq.push({w, nextFace, faceId, nEdgeId});
        }
    }
}

} // namespace openpanelcam
