#include <openpanelcam/unfold/unfold_tree.h>
#include <openpanelcam/phase1/fag.h>
#include <queue>

namespace openpanelcam {

UnfoldTreeBuilder::UnfoldTreeBuilder() = default;

UnfoldTree UnfoldTreeBuilder::build(const FaceAdjacencyGraph& fag, int baseFaceId) {
    UnfoldTree tree;
    buildBFS(fag, baseFaceId, tree);
    return tree;
}

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

} // namespace openpanelcam
