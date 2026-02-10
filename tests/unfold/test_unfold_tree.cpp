#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <openpanelcam/unfold/unfold_tree.h>
#include <openpanelcam/phase1/fag.h>
#include <set>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Pln.hxx>

using namespace openpanelcam;
using Catch::Approx;

// Helper: create a non-null planar face for testing
static TopoDS_Face makeTestFace(double zOffset = 0.0) {
    gp_Pln plane(gp_Pnt(0, 0, zOffset), gp_Dir(0, 0, 1));
    return BRepBuilderAPI_MakeFace(plane, -50, 50, -50, 50).Face();
}

// Helper: create a minimal FAG for testing
static FaceAdjacencyGraph createTestFAG_LBracket() {
    // L-bracket: 2 planar faces + 1 bend
    FaceAdjacencyGraph fag;

    int n0 = fag.addNode(makeTestFace(0.0));  // base
    int n1 = fag.addNode(makeTestFace(10.0)); // flange

    int e0 = fag.addBendEdge(n0, n1, makeTestFace(5.0));

    auto& edge = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge.bendAngle = 90.0;
    edge.bendRadius = 2.0;
    edge.isBend = true;

    fag.setBaseFace(n0);
    return fag;
}

static FaceAdjacencyGraph createTestFAG_UChannel() {
    // U-channel: 3 planar faces + 2 bends
    FaceAdjacencyGraph fag;

    int n0 = fag.addNode(makeTestFace(0.0));  // base
    int n1 = fag.addNode(makeTestFace(10.0)); // left flange
    int n2 = fag.addNode(makeTestFace(20.0)); // right flange

    int e0 = fag.addBendEdge(n0, n1, makeTestFace(5.0));
    int e1 = fag.addBendEdge(n0, n2, makeTestFace(15.0));

    auto& edge0 = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge0.bendAngle = 90.0;
    edge0.bendRadius = 2.0;
    edge0.isBend = true;

    auto& edge1 = const_cast<FAG_Edge&>(fag.getEdge(e1));
    edge1.bendAngle = 90.0;
    edge1.bendRadius = 2.0;
    edge1.isBend = true;

    fag.setBaseFace(n0);
    return fag;
}

// =============================================================================
// Existing BFS tests (backward compatibility)
// =============================================================================

TEST_CASE("UnfoldTree: L-bracket has 2 nodes", "[unfold][tree]") {
    auto fag = createTestFAG_LBracket();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId());

    CHECK(tree.nodes.size() == 2);
    CHECK(tree.rootIndex == 0);
}

TEST_CASE("UnfoldTree: root node has identity transform", "[unfold][tree]") {
    auto fag = createTestFAG_LBracket();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId());

    const auto& root = tree.nodes[tree.rootIndex];
    CHECK(root.parentIndex == -1);
    CHECK(root.faceId == fag.getBaseFaceId());
}

TEST_CASE("UnfoldTree: child connected via bend edge", "[unfold][tree]") {
    auto fag = createTestFAG_LBracket();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId());

    // Find non-root node
    const UnfoldTreeNode* child = nullptr;
    for (const auto& node : tree.nodes) {
        if (node.parentIndex >= 0) {
            child = &node;
            break;
        }
    }

    REQUIRE(child != nullptr);
    CHECK(child->bendAngle == Approx(90.0));
    CHECK(child->bendRadius == Approx(2.0));
    CHECK(child->connectingEdgeId >= 0);
}

TEST_CASE("UnfoldTree: U-channel has 3 nodes", "[unfold][tree]") {
    auto fag = createTestFAG_UChannel();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId());

    CHECK(tree.nodes.size() == 3);

    // Root should have 2 children
    const auto& root = tree.nodes[tree.rootIndex];
    int childCount = 0;
    for (const auto& node : tree.nodes) {
        if (node.parentIndex == root.faceId) childCount++;
    }
    CHECK(childCount == 2);
}

TEST_CASE("UnfoldTree: all faces visited exactly once", "[unfold][tree]") {
    auto fag = createTestFAG_UChannel();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId());

    // No duplicate face IDs
    std::set<int> faceIds;
    for (const auto& node : tree.nodes) {
        CHECK(faceIds.find(node.faceId) == faceIds.end());
        faceIds.insert(node.faceId);
    }
}

// =============================================================================
// MST Prim tests
// =============================================================================

TEST_CASE("UnfoldTree MST: L-bracket same result as BFS", "[unfold][tree][mst]") {
    auto fag = createTestFAG_LBracket();
    UnfoldTreeBuilder builder;

    auto treeBFS = builder.build(fag, fag.getBaseFaceId(), TreeBuildStrategy::BFS);
    auto treeMST = builder.build(fag, fag.getBaseFaceId(), TreeBuildStrategy::MST_PRIM);

    // Same structure for trivial graph
    CHECK(treeBFS.nodes.size() == treeMST.nodes.size());
    CHECK(treeMST.rootIndex == 0);
    CHECK(treeMST.strategyUsed == TreeBuildStrategy::MST_PRIM);
    CHECK(treeBFS.strategyUsed == TreeBuildStrategy::BFS);
}

TEST_CASE("UnfoldTree MST: U-channel visits all faces", "[unfold][tree][mst]") {
    auto fag = createTestFAG_UChannel();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId(), TreeBuildStrategy::MST_PRIM);

    CHECK(tree.nodes.size() == 3);

    std::set<int> faceIds;
    for (const auto& node : tree.nodes) {
        faceIds.insert(node.faceId);
    }
    CHECK(faceIds.size() == 3);
}

TEST_CASE("UnfoldTree MST: tracks total edge weight", "[unfold][tree][mst]") {
    auto fag = createTestFAG_UChannel();
    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, fag.getBaseFaceId(), TreeBuildStrategy::MST_PRIM);

    // With bendLength=0 (not set in test FAG), weight uses fallback length=1
    // Each edge: w = 1.0 * (1.0 + |90-90|/180) = 1.0
    // 2 edges → totalWeight = 2.0
    CHECK(tree.totalEdgeWeight == Approx(2.0));
}

TEST_CASE("UnfoldTree MST: prefers shorter bend lines", "[unfold][tree][mst]") {
    // Create diamond graph: base -> A (short) and base -> B (long)
    // where A -> C and B -> C both exist, MST should pick shorter path to C
    FaceAdjacencyGraph fag;
    int base = fag.addNode(makeTestFace(0.0));
    int a = fag.addNode(makeTestFace(10.0));
    int b = fag.addNode(makeTestFace(20.0));
    int c = fag.addNode(makeTestFace(30.0));

    // base -> a: short bend (length=10)
    int e_ba = fag.addBendEdge(base, a, makeTestFace(100.0));
    auto& edge_ba = const_cast<FAG_Edge&>(fag.getEdge(e_ba));
    edge_ba.bendAngle = 90.0; edge_ba.bendRadius = 2.0;
    edge_ba.bendLength = 10.0; edge_ba.isBend = true;

    // base -> b: long bend (length=100)
    int e_bb = fag.addBendEdge(base, b, makeTestFace(101.0));
    auto& edge_bb = const_cast<FAG_Edge&>(fag.getEdge(e_bb));
    edge_bb.bendAngle = 90.0; edge_bb.bendRadius = 2.0;
    edge_bb.bendLength = 100.0; edge_bb.isBend = true;

    // a -> c: short bend (length=15)
    int e_ac = fag.addBendEdge(a, c, makeTestFace(102.0));
    auto& edge_ac = const_cast<FAG_Edge&>(fag.getEdge(e_ac));
    edge_ac.bendAngle = 90.0; edge_ac.bendRadius = 2.0;
    edge_ac.bendLength = 15.0; edge_ac.isBend = true;

    // b -> c: long bend (length=200)
    int e_bc = fag.addBendEdge(b, c, makeTestFace(103.0));
    auto& edge_bc = const_cast<FAG_Edge&>(fag.getEdge(e_bc));
    edge_bc.bendAngle = 90.0; edge_bc.bendRadius = 2.0;
    edge_bc.bendLength = 200.0; edge_bc.isBend = true;

    fag.setBaseFace(base);

    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, base, TreeBuildStrategy::MST_PRIM);

    // All 4 faces should be in tree
    CHECK(tree.nodes.size() == 4);

    // Find how C was reached - should be via A (edge a->c, length=15)
    // not via B (edge b->c, length=200)
    const UnfoldTreeNode* nodeC = nullptr;
    for (const auto& node : tree.nodes) {
        if (node.faceId == c) {
            nodeC = &node;
            break;
        }
    }
    REQUIRE(nodeC != nullptr);
    // C's parent should be A (faceId = a = 1), not B (faceId = b = 2)
    CHECK(nodeC->parentIndex == a);
    CHECK(nodeC->connectingEdgeId == e_ac);
}

TEST_CASE("UnfoldTree MST: prefers 90-degree angles", "[unfold][tree][mst]") {
    // Two paths to same face: one via 90° bend, one via 45° bend
    // MST should prefer the 90° path (lower weight)
    FaceAdjacencyGraph fag;
    int base = fag.addNode(makeTestFace(0.0));
    int a = fag.addNode(makeTestFace(10.0));
    int target = fag.addNode(makeTestFace(20.0));

    // base -> a: 90° bend, length=50
    int e0 = fag.addBendEdge(base, a, makeTestFace(100.0));
    auto& edge0 = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge0.bendAngle = 90.0; edge0.bendRadius = 2.0;
    edge0.bendLength = 50.0; edge0.isBend = true;

    // base -> target: 45° bend, length=50
    int e1 = fag.addBendEdge(base, target, makeTestFace(101.0));
    auto& edge1 = const_cast<FAG_Edge&>(fag.getEdge(e1));
    edge1.bendAngle = 45.0; edge1.bendRadius = 2.0;
    edge1.bendLength = 50.0; edge1.isBend = true;

    // a -> target: 90° bend, length=50
    int e2 = fag.addBendEdge(a, target, makeTestFace(102.0));
    auto& edge2 = const_cast<FAG_Edge&>(fag.getEdge(e2));
    edge2.bendAngle = 90.0; edge2.bendRadius = 2.0;
    edge2.bendLength = 50.0; edge2.isBend = true;

    fag.setBaseFace(base);

    // Weight computation:
    // base->a (90°): 50 * (1 + 0/180) = 50.0
    // base->target (45°): 50 * (1 + 45/180) = 50 * 1.25 = 62.5
    // a->target (90°): 50 * (1 + 0/180) = 50.0
    //
    // MST picks base->a (50) first, then a->target (50) vs base->target (62.5)
    // Total MST = 50 + 50 = 100, vs alternative 50 + 62.5 = 112.5

    UnfoldTreeBuilder builder;
    auto tree = builder.build(fag, base, TreeBuildStrategy::MST_PRIM);

    CHECK(tree.nodes.size() == 3);

    // Target should be reached via A (both are 90° = weight 50)
    // rather than directly from base (45° = weight 62.5)
    const UnfoldTreeNode* targetNode = nullptr;
    for (const auto& node : tree.nodes) {
        if (node.faceId == target) {
            targetNode = &node;
            break;
        }
    }
    REQUIRE(targetNode != nullptr);
    CHECK(targetNode->parentIndex == a);
}

TEST_CASE("UnfoldTree MST: default strategy is MST_PRIM", "[unfold][tree][mst]") {
    auto fag = createTestFAG_LBracket();
    UnfoldTreeBuilder builder;

    // build() without strategy parameter should default to MST_PRIM
    auto tree = builder.build(fag, fag.getBaseFaceId());
    CHECK(tree.strategyUsed == TreeBuildStrategy::MST_PRIM);
}

TEST_CASE("UnfoldTree MST: BFS backward compat via explicit strategy", "[unfold][tree][mst]") {
    auto fag = createTestFAG_UChannel();
    UnfoldTreeBuilder builder;

    auto tree = builder.build(fag, fag.getBaseFaceId(), TreeBuildStrategy::BFS);
    CHECK(tree.strategyUsed == TreeBuildStrategy::BFS);
    CHECK(tree.nodes.size() == 3);
}

TEST_CASE("computeEdgeWeight: 90-degree bend has factor 1.0", "[unfold][tree][mst]") {
    double w = UnfoldTreeBuilder::computeEdgeWeight(100.0, 90.0);
    CHECK(w == Approx(100.0));  // 100 * (1 + 0/180) = 100
}

TEST_CASE("computeEdgeWeight: 45-degree bend has higher weight", "[unfold][tree][mst]") {
    double w90 = UnfoldTreeBuilder::computeEdgeWeight(100.0, 90.0);
    double w45 = UnfoldTreeBuilder::computeEdgeWeight(100.0, 45.0);
    CHECK(w45 > w90);  // 45° deviation penalized
    CHECK(w45 == Approx(125.0));  // 100 * (1 + 45/180) = 125
}

TEST_CASE("computeEdgeWeight: longer bend = higher weight", "[unfold][tree][mst]") {
    double wShort = UnfoldTreeBuilder::computeEdgeWeight(10.0, 90.0);
    double wLong = UnfoldTreeBuilder::computeEdgeWeight(100.0, 90.0);
    CHECK(wLong > wShort);
}

TEST_CASE("computeEdgeWeight: zero length uses fallback", "[unfold][tree][mst]") {
    double w = UnfoldTreeBuilder::computeEdgeWeight(0.0, 90.0);
    CHECK(w == Approx(1.0));  // fallback length=1.0, factor=1.0
}

TEST_CASE("UnfoldTree MST: chain topology (A-B-C-D)", "[unfold][tree][mst]") {
    // Linear chain: only one path, MST = BFS
    FaceAdjacencyGraph fag;
    int n0 = fag.addNode(makeTestFace(0.0));
    int n1 = fag.addNode(makeTestFace(10.0));
    int n2 = fag.addNode(makeTestFace(20.0));
    int n3 = fag.addNode(makeTestFace(30.0));

    for (int i = 0; i < 3; ++i) {
        int e = fag.addBendEdge(i, i + 1, makeTestFace(100.0 + i));
        auto& edge = const_cast<FAG_Edge&>(fag.getEdge(e));
        edge.bendAngle = 90.0; edge.bendRadius = 2.0;
        edge.bendLength = 50.0; edge.isBend = true;
    }

    fag.setBaseFace(n0);

    UnfoldTreeBuilder builder;
    auto treeMST = builder.build(fag, n0, TreeBuildStrategy::MST_PRIM);
    auto treeBFS = builder.build(fag, n0, TreeBuildStrategy::BFS);

    CHECK(treeMST.nodes.size() == 4);
    CHECK(treeBFS.nodes.size() == 4);

    // Both should produce same structure for linear chain
    for (size_t i = 0; i < treeMST.nodes.size(); ++i) {
        CHECK(treeMST.nodes[i].faceId == treeBFS.nodes[i].faceId);
    }
}
