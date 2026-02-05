#define CATCH_CONFIG_MAIN
#include <catch2/catch_test_macros.hpp>
#include <cmath>
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

TEST_CASE("Polygon2D area calculation", "[phase2][types]") {
    Polygon2D poly;
    poly.vertices = {
        Point2D(0, 0),
        Point2D(10, 0),
        Point2D(10, 10),
        Point2D(0, 10)
    };

    double area = poly.area();
    REQUIRE(std::abs(area - 100.0) < 0.01);
}

TEST_CASE("Polygon2D contains point", "[phase2][types]") {
    Polygon2D poly;
    poly.vertices = {
        Point2D(0, 0),
        Point2D(10, 0),
        Point2D(10, 10),
        Point2D(0, 10)
    };

    REQUIRE(poly.contains(Point2D(5, 5)) == true);
    REQUIRE(poly.contains(Point2D(15, 15)) == false);
}

TEST_CASE("Polygon2D centroid calculation", "[phase2][types]") {
    Polygon2D poly;
    poly.vertices = {
        Point2D(0, 0),
        Point2D(10, 0),
        Point2D(10, 10),
        Point2D(0, 10)
    };

    Point2D center = poly.centroid();
    REQUIRE(std::abs(center.x - 5.0) < 0.01);
    REQUIRE(std::abs(center.y - 5.0) < 0.01);
}

TEST_CASE("Rectangle2D construction and area", "[phase2][types]") {
    Rectangle2D rect(Point2D(0, 0), Point2D(10, 20));

    REQUIRE(std::abs(rect.width - 10.0) < 0.01);
    REQUIRE(std::abs(rect.height - 20.0) < 0.01);
    REQUIRE(std::abs(rect.area - 200.0) < 0.01);
}

TEST_CASE("Rectangle2D contains point", "[phase2][types]") {
    Rectangle2D rect(Point2D(0, 0), Point2D(10, 20));

    REQUIRE(rect.contains(Point2D(5, 10)) == true);
    REQUIRE(rect.contains(Point2D(15, 10)) == false);
}

TEST_CASE("Rectangle2D center", "[phase2][types]") {
    Rectangle2D rect(Point2D(0, 0), Point2D(10, 20));

    REQUIRE(std::abs(rect.center.x - 5.0) < 0.01);
    REQUIRE(std::abs(rect.center.y - 10.0) < 0.01);
}
