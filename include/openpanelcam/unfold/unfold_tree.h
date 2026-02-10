#pragma once

#include <openpanelcam/unfold/types.h>
#include <gp_Trsf.hxx>
#include <vector>
#include <set>

namespace openpanelcam {

class FaceAdjacencyGraph;

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
};

class UnfoldTreeBuilder {
public:
    UnfoldTreeBuilder();

    UnfoldTree build(const FaceAdjacencyGraph& fag, int baseFaceId);

private:
    void buildBFS(const FaceAdjacencyGraph& fag, int baseFaceId, UnfoldTree& tree);
};

} // namespace openpanelcam
