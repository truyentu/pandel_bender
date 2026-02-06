#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/precedence_dag.h"

using namespace openpanelcam::phase2;

TEST_CASE("resolveCycles removes lowest confidence edge", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);

    // Create cycle: 0 -> 1 (high confidence) and 1 -> 0 (low confidence)
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "Strong constraint");
    dag.addEdge(1, 0, ConstraintType::SEQUENTIAL, 0.5, "Weak constraint");
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);

    // Resolve cycle by removing lowest confidence edge
    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.size() == 1);
    REQUIRE(removedEdges[0].confidence == 0.5);
    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("resolveCycles handles multiple cycles", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);

    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 0.9, "Constraint 1");
    dag.addEdge(1, 0, ConstraintType::SEQUENTIAL, 0.3, "Weak 1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 0.8, "Constraint 2");
    dag.addEdge(2, 1, ConstraintType::BOX_CLOSING, 0.4, "Weak 2");

    dag.finalize();

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.size() >= 2);
    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("resolveCycles preserves acyclic graph", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);

    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "No cycle");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "No cycle");
    dag.finalize();

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.empty());
    REQUIRE(dag.isAcyclic() == true);
    REQUIRE(dag.edgeCount() == 2);
}

TEST_CASE("resolveCycles handles triangle cycle", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);

    // Create a 3-node cycle: 0 -> 1 -> 2 -> 0
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 0.9, "Edge 0->1");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 0.8, "Edge 1->2");
    dag.addEdge(2, 0, ConstraintType::SEQUENTIAL, 0.5, "Edge 2->0 (weakest)");
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.size() == 1);
    REQUIRE(removedEdges[0].confidence == 0.5);  // Weakest edge removed
    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("resolveCycles handles complex interconnected cycles", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addNode(3);

    // Create a diamond pattern with cycles
    // 0 <-> 1, 0 -> 2 -> 3 -> 0
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 0.95, "Edge 0->1");
    dag.addEdge(1, 0, ConstraintType::SEQUENTIAL, 0.25, "Edge 1->0 (weak)");
    dag.addEdge(0, 2, ConstraintType::GEOMETRIC, 0.85, "Edge 0->2");
    dag.addEdge(2, 3, ConstraintType::GEOMETRIC, 0.75, "Edge 2->3");
    dag.addEdge(3, 0, ConstraintType::BOX_CLOSING, 0.35, "Edge 3->0 (weak)");
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.size() >= 2);
    REQUIRE(dag.isAcyclic() == true);

    // Verify all removed edges have lower confidence than retained edges
    for (const auto& removed : removedEdges) {
        REQUIRE(removed.confidence <= 0.5);  // Only weak edges should be removed
    }
}

TEST_CASE("resolveCycles handles empty graph", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.finalize();

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.empty());
    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("resolveCycles handles single node", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.finalize();

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.empty());
    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("resolveCycles handles equal confidence edges", "[phase2][dag][cycles]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);

    // Both edges have same confidence
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 0.5, "Edge A");
    dag.addEdge(1, 0, ConstraintType::GEOMETRIC, 0.5, "Edge B");
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);

    auto removedEdges = dag.resolveCycles();

    REQUIRE(removedEdges.size() == 1);  // One of them must be removed
    REQUIRE(dag.isAcyclic() == true);
}
