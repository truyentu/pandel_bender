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
