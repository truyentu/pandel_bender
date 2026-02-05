#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/precedence_dag.h"

using namespace openpanelcam::phase2;

TEST_CASE("PrecedenceDAG construction", "[phase2][dag]") {
    PrecedenceDAG dag;

    REQUIRE(dag.nodeCount() == 0);
    REQUIRE(dag.edgeCount() == 0);
    REQUIRE(dag.isFinalized() == false);
}

TEST_CASE("PrecedenceDAG add nodes", "[phase2][dag]") {
    PrecedenceDAG dag;

    int id1 = dag.addNode(0);  // Add node for bend 0
    int id2 = dag.addNode(1);  // Add node for bend 1
    int id3 = dag.addNode(2);  // Add node for bend 2

    REQUIRE(id1 == 0);
    REQUIRE(id2 == 1);
    REQUIRE(id3 == 2);
    REQUIRE(dag.nodeCount() == 3);
}

TEST_CASE("PrecedenceDAG add edges", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);

    int edgeId1 = dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "Test constraint 1");
    int edgeId2 = dag.addEdge(1, 2, ConstraintType::BOX_CLOSING, 0.95, "Test constraint 2");

    REQUIRE(edgeId1 >= 0);
    REQUIRE(edgeId2 >= 0);
    REQUIRE(dag.edgeCount() == 2);
}

TEST_CASE("PrecedenceDAG get node", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(5);  // Add node for bend 5

    const PrecedenceNode* node = dag.getNode(0);
    REQUIRE(node != nullptr);
    REQUIRE(node->id == 0);
    REQUIRE(node->bendId == 5);
}

TEST_CASE("PrecedenceDAG get edge", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addNode(1);
    int edgeId = dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 0.9, "Test edge");

    const PrecedenceEdge* edge = dag.getEdge(edgeId);
    REQUIRE(edge != nullptr);
    REQUIRE(edge->fromBend == 0);
    REQUIRE(edge->toBend == 1);
    REQUIRE(edge->type == ConstraintType::GEOMETRIC);
    REQUIRE(edge->confidence == 0.9);
    REQUIRE(edge->reasoning == "Test edge");
}

TEST_CASE("PrecedenceDAG clear", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addNode(1);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "Test");

    dag.clear();

    REQUIRE(dag.nodeCount() == 0);
    REQUIRE(dag.edgeCount() == 0);
    REQUIRE(dag.isFinalized() == false);
}
