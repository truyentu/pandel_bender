#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include <set>

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 27: ConstraintSolver Enhanced Validation Tests
// ============================================================================

TEST_CASE("ConstraintSolver - validate output structure", "[phase2][solver][validation]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Validate output structure
    REQUIRE(output.precedenceGraph.isFinalized());
    REQUIRE(output.precedenceGraph.isAcyclic());
    REQUIRE(!output.bendSequence.empty());
    REQUIRE(output.success == true);
}

TEST_CASE("ConstraintSolver - detect invalid state", "[phase2][solver][validation]") {
    ConstraintSolver solver;

    // Empty bends but expect to handle gracefully
    auto output = solver.solve({});

    REQUIRE(output.success == true);  // Empty input is valid
    REQUIRE(output.bendSequence.empty());
    REQUIRE(output.analysisSummary.find("No bends") != std::string::npos);
}

TEST_CASE("ConstraintSolver - grasp constraint completeness", "[phase2][solver][validation]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    auto output = solver.solve({ b0, b1 });

    // Should have grasp constraints for:
    // - Flat state (0 bends)
    // - After bend 0
    // - After bend 1
    // Minimum = 3 states
    REQUIRE(output.graspConstraints.size() >= 3);
}

TEST_CASE("ConstraintSolver - ABA constraint coverage", "[phase2][solver][validation]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 150.0;

    auto output = solver.solve({ b0, b1 });

    // Should have ABA constraint for each bend
    REQUIRE(output.abaConstraints.size() == 2);

    // Each constraint should have valid bend ID
    for (const auto& aba : output.abaConstraints) {
        REQUIRE((aba.bendId == 0 || aba.bendId == 1));
        REQUIRE(aba.requiredWidth > 0);
    }
}

TEST_CASE("ConstraintSolver - sequence ordering validation", "[phase2][solver][validation]") {
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

    // Sequence should contain all bends
    REQUIRE(output.bendSequence.size() == 2);

    // No bend should appear twice
    std::set<int> uniqueBends(output.bendSequence.begin(), output.bendSequence.end());
    REQUIRE(uniqueBends.size() == output.bendSequence.size());
}

TEST_CASE("ConstraintSolver - error accumulation", "[phase2][solver][validation]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Successful solve should have no errors
    REQUIRE(output.errors.empty());

    // But may have warnings
    REQUIRE(output.warnings.size() >= 0);
}

TEST_CASE("ConstraintSolver - statistics consistency", "[phase2][solver][validation]") {
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

    // Statistics should be consistent
    REQUIRE(stats.totalBends == 3);
    REQUIRE(stats.abaConstraintsGenerated == 3);

    // Total time should be sum of parts
    double sum = stats.geometricAnalysisTimeMs +
                 stats.graspAnalysisTimeMs +
                 stats.abaAnalysisTimeMs +
                 stats.graphBuildTimeMs +
                 stats.topologicalSortTimeMs;

    REQUIRE(stats.totalSolveTimeMs >= sum);
}
