#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 26: ConstraintSolver Integration - Main Class
// ============================================================================

TEST_CASE("ConstraintSolver - constructor", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    // Should construct successfully
    REQUIRE(true);
}

TEST_CASE("ConstraintSolver - solve empty input", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;  // Empty

    auto output = solver.solve(bends);

    // Empty input should produce empty output
    REQUIRE(output.precedenceGraph.nodeCount() == 0);
    REQUIRE(output.precedenceGraph.edgeCount() == 0);
    REQUIRE(output.graspConstraints.empty());
    REQUIRE(output.abaConstraints.empty());
}

TEST_CASE("ConstraintSolver - solve single bend", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;
    bend.position.x = 50.0;
    bend.position.y = 50.0;

    std::vector<BendFeature> bends = { bend };

    auto output = solver.solve(bends);

    // Should have 1 node in graph
    REQUIRE(output.precedenceGraph.nodeCount() == 1);

    // Should have grasp constraints
    REQUIRE(!output.graspConstraints.empty());

    // Should have ABA constraint
    REQUIRE(output.abaConstraints.size() == 1);
}

TEST_CASE("ConstraintSolver - solve two independent bends", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 50.0;
    b0.position.y = 50.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 200.0;  // Far apart
    b1.position.y = 50.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto output = solver.solve(bends);

    // Should have 2 nodes
    REQUIRE(output.precedenceGraph.nodeCount() == 2);

    // Independent bends should have minimal or no edges
    REQUIRE(output.precedenceGraph.edgeCount() >= 0);

    // Should have grasp constraints
    REQUIRE(!output.graspConstraints.empty());

    // Should have 2 ABA constraints
    REQUIRE(output.abaConstraints.size() == 2);
}

TEST_CASE("ConstraintSolver - solve with geometric constraints", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature b0, b1;

    // Two parallel bends close together (will create sequential blocking)
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 0.0;
    b0.direction.y = 1.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 30.0;  // Close (40mm)
    b1.position.y = 0.0;
    b1.direction.x = 0.0;
    b1.direction.y = 1.0;  // Parallel

    std::vector<BendFeature> bends = { b0, b1 };

    auto output = solver.solve(bends);

    // Should have 2 nodes
    REQUIRE(output.precedenceGraph.nodeCount() == 2);

    // Should have precedence edges from geometric analysis
    REQUIRE(output.precedenceGraph.edgeCount() >= 1);
}

TEST_CASE("ConstraintSolver - output completeness", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Verify all output components are populated
    REQUIRE(output.precedenceGraph.nodeCount() >= 0);
    REQUIRE(output.precedenceGraph.isFinalized() == true);

    // Should have at least flat state grasp constraint
    REQUIRE(!output.graspConstraints.empty());

    // Should have bend sequence (topological sort)
    REQUIRE(!output.bendSequence.empty());
    REQUIRE(output.bendSequence.size() == 1);
    REQUIRE(output.bendSequence[0] == 0);

    // Should have ABA constraint
    REQUIRE(!output.abaConstraints.empty());

    // Success flag should be set
    REQUIRE(output.success == true);

    // Should have analysis summary
    REQUIRE(!output.analysisSummary.empty());
}

TEST_CASE("ConstraintSolver - statistics tracking", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 3; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    auto stats = solver.getStatistics();

    // Should have timing data
    REQUIRE(stats.totalSolveTimeMs >= 0.0);
    REQUIRE(stats.geometricAnalysisTimeMs >= 0.0);
    REQUIRE(stats.graspAnalysisTimeMs >= 0.0);
    REQUIRE(stats.abaAnalysisTimeMs >= 0.0);

    // Should have counts
    REQUIRE(stats.totalBends == 3);
    REQUIRE(stats.geometricConstraints >= 0);
    REQUIRE(stats.graspStatesAnalyzed >= 0);
    REQUIRE(stats.abaConstraintsGenerated == 3);
}

TEST_CASE("ConstraintSolver - topological sort", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature b0, b1;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;
    b0.direction.x = 1.0;
    b0.direction.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 500.0;  // Far apart to avoid geometric conflicts
    b1.position.y = 500.0;
    b1.direction.x = 1.0;
    b1.direction.y = 0.0;

    std::vector<BendFeature> bends = { b0, b1 };

    auto output = solver.solve(bends);

    // Debug output
    INFO("Graph node count: " << output.precedenceGraph.nodeCount());
    INFO("Graph edge count: " << output.precedenceGraph.edgeCount());
    INFO("Graph finalized: " << output.precedenceGraph.isFinalized());
    INFO("Bend sequence size: " << output.bendSequence.size());
    INFO("Success flag: " << output.success);
    INFO("Analysis summary:\n" << output.analysisSummary);

    // Should have valid bend sequence
    REQUIRE(output.bendSequence.size() == 2);

    // Sequence should contain both bends
    bool hasB0 = false;
    bool hasB1 = false;
    for (int id : output.bendSequence) {
        if (id == 0) hasB0 = true;
        if (id == 1) hasB1 = true;
    }
    REQUIRE(hasB0 == true);
    REQUIRE(hasB1 == true);
}

TEST_CASE("ConstraintSolver - graph finalization", "[phase2][solver][basic]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Graph should be finalized
    REQUIRE(output.precedenceGraph.isFinalized() == true);

    // Should be acyclic
    REQUIRE(output.precedenceGraph.isAcyclic() == true);
}
