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

TEST_CASE("PrecedenceDAG finalize", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "1->2");

    REQUIRE(dag.isFinalized() == false);

    bool result = dag.finalize();

    REQUIRE(result == true);
    REQUIRE(dag.isFinalized() == true);

    // After finalize, nodes should have updated levels
    const PrecedenceNode* node0 = dag.getNode(0);
    const PrecedenceNode* node1 = dag.getNode(1);
    const PrecedenceNode* node2 = dag.getNode(2);

    REQUIRE(node0->level >= 0);
    REQUIRE(node1->level >= 0);
    REQUIRE(node2->level >= 0);

    // Level should increase along dependency chain
    REQUIRE(node0->level < node1->level);
    REQUIRE(node1->level < node2->level);
}

TEST_CASE("PrecedenceDAG finalize with branching", "[phase2][dag]") {
    PrecedenceDAG dag;

    // Create diamond pattern:
    //     0
    //    / \
    //   1   2
    //    \ /
    //     3
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addNode(3);

    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(0, 2, ConstraintType::GEOMETRIC, 1.0, "0->2");
    dag.addEdge(1, 3, ConstraintType::GEOMETRIC, 1.0, "1->3");
    dag.addEdge(2, 3, ConstraintType::GEOMETRIC, 1.0, "2->3");

    bool result = dag.finalize();
    REQUIRE(result == true);

    const PrecedenceNode* node0 = dag.getNode(0);
    const PrecedenceNode* node1 = dag.getNode(1);
    const PrecedenceNode* node2 = dag.getNode(2);
    const PrecedenceNode* node3 = dag.getNode(3);

    // Node 0 should be at level 0 (root)
    REQUIRE(node0->level == 0);

    // Nodes 1 and 2 should be at level 1
    REQUIRE(node1->level == 1);
    REQUIRE(node2->level == 1);

    // Node 3 should be at level 2 (max of predecessors + 1)
    REQUIRE(node3->level == 2);
}

TEST_CASE("PrecedenceDAG isAcyclic - simple acyclic graph", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "1->2");

    dag.finalize();

    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("PrecedenceDAG isAcyclic - simple cycle", "[phase2][dag]") {
    PrecedenceDAG dag;

    // Create a simple cycle: 0 -> 1 -> 2 -> 0
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "1->2");
    dag.addEdge(2, 0, ConstraintType::GEOMETRIC, 1.0, "2->0");  // Creates cycle!

    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);
}

TEST_CASE("PrecedenceDAG isAcyclic - self cycle", "[phase2][dag]") {
    PrecedenceDAG dag;

    dag.addNode(0);
    dag.addEdge(0, 0, ConstraintType::GEOMETRIC, 1.0, "0->0");  // Self-loop

    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);
}

TEST_CASE("PrecedenceDAG isAcyclic - diamond pattern (acyclic)", "[phase2][dag]") {
    PrecedenceDAG dag;

    // Diamond is acyclic
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addNode(3);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(0, 2, ConstraintType::GEOMETRIC, 1.0, "0->2");
    dag.addEdge(1, 3, ConstraintType::GEOMETRIC, 1.0, "1->3");
    dag.addEdge(2, 3, ConstraintType::GEOMETRIC, 1.0, "2->3");

    dag.finalize();

    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("PrecedenceDAG isAcyclic - complex cycle", "[phase2][dag]") {
    PrecedenceDAG dag;

    // Create complex graph with hidden cycle
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addNode(3);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "0->1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "1->2");
    dag.addEdge(2, 3, ConstraintType::GEOMETRIC, 1.0, "2->3");
    dag.addEdge(3, 1, ConstraintType::GEOMETRIC, 1.0, "3->1");  // Creates cycle 1->2->3->1

    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);
}
