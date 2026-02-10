#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <openpanelcam/unfold/unfolder.h>
#include <openpanelcam/unfold/types.h>
#include <openpanelcam/phase1/fag.h>

#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Pln.hxx>

using namespace openpanelcam;
using Catch::Approx;

// Helper: create a non-null planar face for testing
static TopoDS_Face makeTestFace(double zOffset = 0.0) {
    gp_Pln plane(gp_Pnt(0, 0, zOffset), gp_Dir(0, 0, 1));
    return BRepBuilderAPI_MakeFace(plane, -50, 50, -50, 50).Face();
}

TEST_CASE("Unfolder: construct with default config", "[unfold][pipeline]") {
    UnfoldConfig config;
    SheetMetalUnfolder unfolder(config);

    CHECK(config.thickness == Approx(2.0));
    CHECK(config.defaultKFactor == Approx(0.44));
    CHECK(config.useDIN6935 == true);
}

TEST_CASE("Unfolder: fails gracefully with empty FAG", "[unfold][pipeline]") {
    UnfoldConfig config;
    SheetMetalUnfolder unfolder(config);

    FaceAdjacencyGraph emptyFag;
    auto result = unfolder.unfold(emptyFag, -1);

    CHECK(result.success == false);
    CHECK(!result.errorMessage.empty());
}

TEST_CASE("Unfolder: compute bend zones for L-bracket", "[unfold][pipeline]") {
    UnfoldConfig config;
    config.thickness = 2.0;
    config.defaultKFactor = 0.44;
    SheetMetalUnfolder unfolder(config);

    // Create test FAG (L-bracket)
    FaceAdjacencyGraph fag;
    int n0 = fag.addNode(makeTestFace(0.0));
    int n1 = fag.addNode(makeTestFace(10.0));
    int e0 = fag.addBendEdge(n0, n1, makeTestFace(5.0));

    auto& edge = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge.bendAngle = 90.0;
    edge.bendRadius = 2.0;
    edge.isBend = true;

    fag.setBaseFace(n0);

    auto result = unfolder.unfold(fag, n0);

    // Should process 1 bend zone
    CHECK(result.bendsProcessed == 1);
    CHECK(result.bendZones.size() == 1);

    // Check bend allowance values
    const auto& bz = result.bendZones[0];
    CHECK(bz.bendAngle == Approx(90.0));
    CHECK(bz.bendAllowance > 0.0);
    CHECK(bz.bendDeduction > 0.0);
    CHECK(bz.kFactor == Approx(0.44).margin(0.02));
}

TEST_CASE("Unfolder: multiple bends (U-channel)", "[unfold][pipeline]") {
    UnfoldConfig config;
    config.thickness = 1.5;
    SheetMetalUnfolder unfolder(config);

    FaceAdjacencyGraph fag;
    int n0 = fag.addNode(makeTestFace(0.0));
    int n1 = fag.addNode(makeTestFace(10.0));
    int n2 = fag.addNode(makeTestFace(20.0));

    int e0 = fag.addBendEdge(n0, n1, makeTestFace(5.0));
    int e1 = fag.addBendEdge(n0, n2, makeTestFace(15.0));

    auto& edge0 = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge0.bendAngle = 90.0; edge0.bendRadius = 1.5; edge0.isBend = true;

    auto& edge1 = const_cast<FAG_Edge&>(fag.getEdge(e1));
    edge1.bendAngle = 90.0; edge1.bendRadius = 1.5; edge1.isBend = true;

    fag.setBaseFace(n0);

    auto result = unfolder.unfold(fag, n0);

    CHECK(result.bendsProcessed == 2);
    CHECK(result.bendZones.size() == 2);
    CHECK(result.facesUnfolded == 3);
}

TEST_CASE("Unfolder: DIN 6935 auto K-factor", "[unfold][pipeline]") {
    UnfoldConfig config;
    config.thickness = 2.0;
    config.useDIN6935 = true;
    SheetMetalUnfolder unfolder(config);

    FaceAdjacencyGraph fag;
    int n0 = fag.addNode(makeTestFace(0.0));
    int n1 = fag.addNode(makeTestFace(10.0));
    int e0 = fag.addBendEdge(n0, n1, makeTestFace(5.0));

    auto& edge = const_cast<FAG_Edge&>(fag.getEdge(e0));
    edge.bendAngle = 90.0;
    edge.bendRadius = 2.0; // r/s = 1.0 → K ≈ 0.42
    edge.isBend = true;

    fag.setBaseFace(n0);

    auto result = unfolder.unfold(fag, n0);

    CHECK(result.bendZones[0].kFactor == Approx(0.42).margin(0.01));
}

TEST_CASE("Unfolder: chain of 4 bends (box)", "[unfold][pipeline]") {
    UnfoldConfig config;
    SheetMetalUnfolder unfolder(config);

    FaceAdjacencyGraph fag;
    int n0 = fag.addNode(makeTestFace(0.0));   // base
    int n1 = fag.addNode(makeTestFace(10.0));  // side 1
    int n2 = fag.addNode(makeTestFace(20.0));  // side 2
    int n3 = fag.addNode(makeTestFace(30.0));  // side 3
    int n4 = fag.addNode(makeTestFace(40.0));  // side 4
    (void)n1; (void)n2; (void)n3; (void)n4;

    for (int i = 1; i <= 4; ++i) {
        int e = fag.addBendEdge(n0, i, makeTestFace(100.0 + i));
        auto& edge = const_cast<FAG_Edge&>(fag.getEdge(e));
        edge.bendAngle = 90.0; edge.bendRadius = 2.0; edge.isBend = true;
    }

    fag.setBaseFace(n0);

    auto result = unfolder.unfold(fag, n0);

    CHECK(result.bendsProcessed == 4);
    CHECK(result.facesUnfolded == 5);
    CHECK(result.success == true);
}
