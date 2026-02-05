#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 30: ConstraintSolver End-to-End Integration Tests
// ============================================================================

TEST_CASE("E2E - Simple L-bracket (2 bends)", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    // Create L-bracket: two perpendicular bends
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
    b1.length = 80.0;
    b1.position.x = 0.0;
    b1.position.y = 100.0;
    b1.direction.x = 0.0;
    b1.direction.y = 1.0;

    auto output = solver.solve({ b0, b1 });

    // Should succeed
    REQUIRE(output.success == true);

    // Should have valid sequence
    REQUIRE(output.bendSequence.size() == 2);

    // Should have grasp constraints
    REQUIRE(output.graspConstraints.size() >= 3); // flat + after each bend

    // Should have ABA constraints
    REQUIRE(output.abaConstraints.size() == 2);

    // All ABA should be feasible for simple bends
    for (const auto& aba : output.abaConstraints) {
        REQUIRE(aba.feasible == true);
    }
}

TEST_CASE("E2E - U-channel (3 bends)", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    // Create U-channel: left wall, bottom, right wall
    BendFeature b0, b1, b2;

    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 150.0;
    b1.position.x = 100.0;
    b1.position.y = 0.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 250.0;
    b2.position.y = 0.0;

    auto output = solver.solve({ b0, b1, b2 });

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 3);
    REQUIRE(output.abaConstraints.size() == 3);
}

TEST_CASE("E2E - Complex multi-bend part", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    // Create complex part with 5 bends
    std::vector<BendFeature> bends;

    for (int i = 0; i < 5; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0 + i * 10.0;
        bend.position.x = i * 200.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 5);
    REQUIRE(output.abaConstraints.size() == 5);
    REQUIRE(output.precedenceGraph.nodeCount() == 5);
}

TEST_CASE("E2E - Performance benchmark (10 bends)", "[phase2][solver][e2e][benchmark]") {
    ConstraintSolver solver;

    // Create part with 10 bends
    std::vector<BendFeature> bends;

    for (int i = 0; i < 10; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 150.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);
    auto stats = solver.getStatistics();

    REQUIRE(output.success == true);

    // Performance requirements (Debug mode)
    // Should complete in reasonable time
    REQUIRE(stats.totalSolveTimeMs < 1000.0); // < 1 second

    // Individual phases should be fast
    REQUIRE(stats.geometricAnalysisTimeMs < 500.0);
    REQUIRE(stats.graspAnalysisTimeMs < 500.0);
    REQUIRE(stats.abaAnalysisTimeMs < 500.0);
}

TEST_CASE("E2E - Error recovery", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    // Test with bends that might create cycles
    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 10.0;  // Very close
    b1.position.y = 10.0;

    auto output = solver.solve({ b0, b1 });

    // Should either succeed or fail gracefully
    if (!output.success) {
        // Should have error messages
        REQUIRE(!output.errors.empty());
        REQUIRE(!output.analysisSummary.empty());
    }
}

TEST_CASE("E2E - Statistics validation", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 120.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 80.0;

    auto output = solver.solve({ b0, b1, b2 });
    auto stats = solver.getStatistics();

    // Validate all statistics are populated
    REQUIRE(stats.totalBends == 3);
    REQUIRE(stats.totalSolveTimeMs > 0.0);
    REQUIRE(stats.geometricAnalysisTimeMs >= 0.0);
    REQUIRE(stats.graspAnalysisTimeMs >= 0.0);
    REQUIRE(stats.abaAnalysisTimeMs >= 0.0);
    REQUIRE(stats.abaConstraintsGenerated == 3);
}

TEST_CASE("E2E - Complete output validation", "[phase2][solver][e2e]") {
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
    b1.position.x = 300.0;
    b1.position.y = 0.0;

    auto output = solver.solve({ b0, b1 });

    // Validate complete output structure
    REQUIRE(output.success == true);

    // Precedence graph
    REQUIRE(output.precedenceGraph.isFinalized());
    REQUIRE(output.precedenceGraph.nodeCount() == 2);

    // Constraints
    REQUIRE(!output.graspConstraints.empty());
    REQUIRE(output.abaConstraints.size() == 2);

    // Sequence
    REQUIRE(output.bendSequence.size() == 2);

    // Analysis
    REQUIRE(!output.analysisSummary.empty());
}

TEST_CASE("E2E - Multiple solver instances", "[phase2][solver][e2e]") {
    // Test that multiple solver instances can coexist
    ConstraintSolver solver1;
    ConstraintSolver solver2;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output1 = solver1.solve({ bend });
    auto output2 = solver2.solve({ bend });

    // Both should succeed independently
    REQUIRE(output1.success == true);
    REQUIRE(output2.success == true);
}

TEST_CASE("E2E - Solver reset and reuse", "[phase2][solver][e2e]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    // First solve
    auto output1 = solver.solve({ bend });
    REQUIRE(output1.success == true);

    // Reset
    solver.reset();

    // Second solve (should work independently)
    auto output2 = solver.solve({ bend });
    REQUIRE(output2.success == true);
}

TEST_CASE("E2E - Large part (20 bends)", "[phase2][solver][e2e][benchmark]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;

    for (int i = 0; i < 20; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 100.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);
    auto stats = solver.getStatistics();

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 20);

    // Should still be reasonably fast
    REQUIRE(stats.totalSolveTimeMs < 5000.0); // < 5 seconds for 20 bends
}
