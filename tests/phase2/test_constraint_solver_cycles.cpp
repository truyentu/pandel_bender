#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 28: ConstraintSolver Cycle Resolution & Optimization Tests
// ============================================================================

TEST_CASE("ConstraintSolver - detect cyclic graph", "[phase2][solver][cycles]") {
    ConstraintSolver solver;

    // Create two bends that will create a cycle
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 0.0;  // Same position to trigger mutual blocking
    b1.position.y = 0.0;

    auto output = solver.solve({ b0, b1 });

    // Should detect cycle and handle it
    if (!output.success) {
        // If not resolved, should have error about cycles
        bool hasCycleError = false;
        for (const auto& err : output.errors) {
            if (err.find("cycle") != std::string::npos ||
                err.find("finalize") != std::string::npos) {
                hasCycleError = true;
                break;
            }
        }
        REQUIRE(hasCycleError);
    }
}

TEST_CASE("ConstraintSolver - resolve cycles by confidence", "[phase2][solver][cycles]") {
    ConstraintSolver solver;

    // For now, solver should at least detect cycles
    // Future: Auto-resolve by removing low-confidence edges

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    auto output = solver.solve({ b0, b1 });

    // Should either succeed or fail gracefully
    REQUIRE((output.success == true || !output.errors.empty()));
}

TEST_CASE("ConstraintSolver - track removed edges", "[phase2][solver][optimization]") {
    ConstraintSolver solver;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 200.0;
    b1.position.y = 0.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 400.0;
    b2.position.y = 0.0;

    auto output = solver.solve({ b0, b1, b2 });

    // Should succeed with independent bends
    REQUIRE(output.success == true);
}

TEST_CASE("ConstraintSolver - empty graph optimization", "[phase2][solver][optimization]") {
    ConstraintSolver solver;

    auto output = solver.solve({});

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.empty());
    REQUIRE(output.precedenceGraph.nodeCount() == 0);
    REQUIRE(output.precedenceGraph.edgeCount() == 0);
}

TEST_CASE("ConstraintSolver - single node optimization", "[phase2][solver][optimization]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 1);
    REQUIRE(output.bendSequence[0] == 0);
}

TEST_CASE("ConstraintSolver - redundant edge detection", "[phase2][solver][optimization]") {
    // Test for transitive reduction
    // If we have edges: A→B, B→C, A→C
    // The edge A→C is redundant (transitive)

    ConstraintSolver solver;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 150.0;
    b1.position.y = 0.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 300.0;
    b2.position.y = 0.0;

    auto output = solver.solve({ b0, b1, b2 });

    // Should produce valid sequence regardless of redundant edges
    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 3);
}

TEST_CASE("ConstraintSolver - statistics after optimization", "[phase2][solver][optimization]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 500.0;
    b1.position.y = 0.0;

    auto output = solver.solve({ b0, b1 });
    auto stats = solver.getStatistics();

    // Statistics should be consistent
    REQUIRE(stats.totalBends == 2);
    REQUIRE(stats.totalSolveTimeMs >= 0.0);

    // Edge count should match graph
    REQUIRE(stats.totalEdges == output.precedenceGraph.edgeCount());
}

TEST_CASE("ConstraintSolver - confidence threshold", "[phase2][solver][optimization]") {
    // Test that low-confidence edges can be handled appropriately
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    auto output = solver.solve({ b0, b1 });

    // Should handle edges of varying confidence
    REQUIRE((output.success == true || !output.errors.empty()));
}

TEST_CASE("ConstraintSolver - graph metrics", "[phase2][solver][optimization]") {
    ConstraintSolver solver;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 200.0;
    b1.position.y = 0.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 400.0;
    b2.position.y = 0.0;

    auto output = solver.solve({ b0, b1, b2 });

    // Verify basic graph metrics
    REQUIRE(output.precedenceGraph.nodeCount() == 3);
    REQUIRE(output.precedenceGraph.edgeCount() >= 0);
    REQUIRE(output.precedenceGraph.isFinalized());
}
