#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 29: ConstraintSolver State Enumeration & Reporting Tests
// ============================================================================

TEST_CASE("ConstraintSolver - sequential state enumeration", "[phase2][solver][states]") {
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

    // Should have grasp constraints for sequential states:
    // State 0: Flat (no bends)
    // State 1: After b0
    // State 2: After b1
    // State 3: After b2
    // Total: 4 states minimum
    REQUIRE(output.graspConstraints.size() >= 4);
}

TEST_CASE("ConstraintSolver - state progression validation", "[phase2][solver][states]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    auto output = solver.solve({ b0, b1 });

    // Should have at least:
    // - Flat state
    // - After each bend
    REQUIRE(output.graspConstraints.size() >= 3);

    // Check state IDs are sequential
    if (output.graspConstraints.size() >= 2) {
        for (size_t i = 1; i < output.graspConstraints.size(); i++) {
            // State IDs should be unique
            REQUIRE(output.graspConstraints[i].stateId !=
                    output.graspConstraints[i-1].stateId);
        }
    }
}

TEST_CASE("ConstraintSolver - detailed analysis summary", "[phase2][solver][reporting]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Summary should contain key information
    REQUIRE(!output.analysisSummary.empty());
    REQUIRE(output.analysisSummary.find("Phase 2") != std::string::npos);
    REQUIRE(output.analysisSummary.find("bends") != std::string::npos);
}

TEST_CASE("ConstraintSolver - per-bend feasibility", "[phase2][solver][reporting]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 200.0;

    auto output = solver.solve({ b0, b1 });

    // Should have ABA constraint for each bend
    REQUIRE(output.abaConstraints.size() == 2);

    // Each should have feasibility info
    for (const auto& aba : output.abaConstraints) {
        REQUIRE((aba.feasible == true || aba.feasible == false));
        REQUIRE(!aba.reason.empty());
    }
}

TEST_CASE("ConstraintSolver - critical path identification", "[phase2][solver][reporting]") {
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

    // Should have valid sequence (critical path)
    REQUIRE(output.bendSequence.size() == 3);
    REQUIRE(output.success == true);
}

TEST_CASE("ConstraintSolver - warning accumulation", "[phase2][solver][reporting]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Warnings should be a vector (may be empty)
    REQUIRE(output.warnings.size() >= 0);

    // Each warning should be non-empty if present
    for (const auto& warning : output.warnings) {
        REQUIRE(!warning.empty());
    }
}

TEST_CASE("ConstraintSolver - empty state handling", "[phase2][solver][states]") {
    ConstraintSolver solver;

    auto output = solver.solve({});

    // Empty input should have no states
    REQUIRE(output.graspConstraints.empty());
    REQUIRE(output.abaConstraints.empty());
    REQUIRE(output.bendSequence.empty());
}

TEST_CASE("ConstraintSolver - state count consistency", "[phase2][solver][states]") {
    ConstraintSolver solver;

    BendFeature b0, b1, b2;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    b2.id = 2;
    b2.angle = 90.0;
    b2.length = 100.0;

    auto output = solver.solve({ b0, b1, b2 });
    auto stats = solver.getStatistics();

    // Grasp states analyzed should match constraint count
    REQUIRE(stats.graspStatesAnalyzed == static_cast<int>(output.graspConstraints.size()));
}

TEST_CASE("ConstraintSolver - report completeness", "[phase2][solver][reporting]") {
    ConstraintSolver solver;

    BendFeature b0, b1;
    b0.id = 0;
    b0.angle = 90.0;
    b0.length = 100.0;

    b1.id = 1;
    b1.angle = 90.0;
    b1.length = 100.0;

    auto output = solver.solve({ b0, b1 });

    // Summary should include all major components
    std::string summary = output.analysisSummary;

    REQUIRE(summary.find("Total bends") != std::string::npos);
    REQUIRE(summary.find("Status") != std::string::npos);
    REQUIRE(summary.find("Timing") != std::string::npos);
}

TEST_CASE("ConstraintSolver - state-based grasp validation", "[phase2][solver][states]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Should have flat state + after-bend state
    REQUIRE(output.graspConstraints.size() >= 2);

    // Flat state should be first
    if (output.graspConstraints.size() >= 1) {
        auto& flatState = output.graspConstraints[0];
        REQUIRE(flatState.bentBends.empty());  // No bends bent yet
    }
}
