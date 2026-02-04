#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/types.h"

using namespace openpanelcam::phase2;

TEST_CASE("ConstraintType enum values exist", "[phase2][types]") {
    ConstraintType ct1 = ConstraintType::GEOMETRIC;
    ConstraintType ct2 = ConstraintType::BOX_CLOSING;
    ConstraintType ct3 = ConstraintType::SEQUENTIAL;
    REQUIRE(true);
}

TEST_CASE("DeadZoneType enum values exist", "[phase2][types]") {
    DeadZoneType dz1 = DeadZoneType::STANDING_FLANGE;
    DeadZoneType dz2 = DeadZoneType::SAFETY_MARGIN;
    DeadZoneType dz3 = DeadZoneType::ABA_INTERFERENCE;
    REQUIRE(true);
}

TEST_CASE("PrecedenceNode construction and fields", "[phase2][types]") {
    PrecedenceNode node;
    node.id = 5;
    node.bendId = 10;
    node.level = 2;
    node.visited = false;
    node.predecessors = {1, 3};
    node.successors = {7, 9};

    REQUIRE(node.id == 5);
    REQUIRE(node.bendId == 10);
    REQUIRE(node.level == 2);
    REQUIRE(node.visited == false);
    REQUIRE(node.predecessors.size() == 2);
    REQUIRE(node.successors.size() == 2);
}

TEST_CASE("PrecedenceEdge construction and fields", "[phase2][types]") {
    PrecedenceEdge edge;
    edge.id = 1;
    edge.fromBend = 0;
    edge.toBend = 2;
    edge.type = ConstraintType::GEOMETRIC;
    edge.confidence = 0.95;
    edge.reasoning = "Corner overlap detected";
    edge.conflictPoint = Point3D(10.0, 20.0, 0.0);

    REQUIRE(edge.id == 1);
    REQUIRE(edge.fromBend == 0);
    REQUIRE(edge.toBend == 2);
    REQUIRE(edge.type == ConstraintType::GEOMETRIC);
    REQUIRE(edge.confidence == 0.95);
    REQUIRE(edge.reasoning == "Corner overlap detected");
    REQUIRE(edge.conflictPoint.x == 10.0);
}
