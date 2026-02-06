#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ============================================================================
// Task 32: State Space Optimization - Progressive State Enumeration
// ============================================================================

TEST_CASE("Optimized state enumeration follows topological order", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 5; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 100.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    // Should have exactly n+1 states (flat + each progressive state)
    REQUIRE(output.graspConstraints.size() == 6);  // 5 bends + 1 flat

    // States should follow sequence order
    REQUIRE(output.graspConstraints[0].bentBends.empty());  // Flat state

    // Each subsequent state adds one bend
    for (size_t i = 1; i < output.graspConstraints.size(); i++) {
        REQUIRE(output.graspConstraints[i].bentBends.size() == i);
    }
}

TEST_CASE("State optimization handles 10 bends efficiently", "[phase2][solver][performance][optimization]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 10; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 100.0;
        bend.position.y = i * 50.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    // Linear states, not exponential (2^10 = 1024)
    REQUIRE(output.graspConstraints.size() == 11);  // 10 bends + 1 flat

    // Performance check - should be well under 1 second
    REQUIRE(solver.getStatistics().totalSolveTimeMs < 1000.0);
}

TEST_CASE("Progressive enumeration uses bend ID order", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    // Create bends with non-sequential IDs
    std::vector<BendFeature> bends;

    BendFeature b0, b1, b2;
    b0.id = 10;  // Non-sequential ID
    b0.angle = 90.0;
    b0.length = 100.0;
    b0.position.x = 0.0;
    b0.position.y = 0.0;

    b1.id = 20;
    b1.angle = 90.0;
    b1.length = 100.0;
    b1.position.x = 200.0;
    b1.position.y = 0.0;

    b2.id = 30;
    b2.angle = 90.0;
    b2.length = 100.0;
    b2.position.x = 400.0;
    b2.position.y = 0.0;

    bends.push_back(b0);
    bends.push_back(b1);
    bends.push_back(b2);

    auto output = solver.solve(bends);

    // Should have 4 states (flat + 3 progressive)
    REQUIRE(output.graspConstraints.size() == 4);

    // Check progressive accumulation
    REQUIRE(output.graspConstraints[0].bentBends.empty());
    REQUIRE(output.graspConstraints[1].bentBends.size() == 1);
    REQUIRE(output.graspConstraints[2].bentBends.size() == 2);
    REQUIRE(output.graspConstraints[3].bentBends.size() == 3);

    // Verify bend IDs are accumulated correctly
    REQUIRE(output.graspConstraints[1].bentBends[0] == 10);

    // State 2 should have bends 10 and 20
    auto& state2 = output.graspConstraints[2].bentBends;
    REQUIRE(std::find(state2.begin(), state2.end(), 10) != state2.end());
    REQUIRE(std::find(state2.begin(), state2.end(), 20) != state2.end());

    // State 3 should have all three bends
    auto& state3 = output.graspConstraints[3].bentBends;
    REQUIRE(std::find(state3.begin(), state3.end(), 10) != state3.end());
    REQUIRE(std::find(state3.begin(), state3.end(), 20) != state3.end());
    REQUIRE(std::find(state3.begin(), state3.end(), 30) != state3.end());
}

TEST_CASE("Single bend progressive enumeration", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;
    bend.length = 100.0;

    auto output = solver.solve({ bend });

    // Should have 2 states: flat + after bend 0
    REQUIRE(output.graspConstraints.size() == 2);

    // Flat state
    REQUIRE(output.graspConstraints[0].bentBends.empty());

    // After bend 0
    REQUIRE(output.graspConstraints[1].bentBends.size() == 1);
    REQUIRE(output.graspConstraints[1].bentBends[0] == 0);
}

TEST_CASE("Progressive states have unique state IDs", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 4; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    // Verify all state IDs are unique
    std::vector<int> stateIds;
    for (const auto& grasp : output.graspConstraints) {
        REQUIRE(std::find(stateIds.begin(), stateIds.end(), grasp.stateId) == stateIds.end());
        stateIds.push_back(grasp.stateId);
    }
}

TEST_CASE("Performance comparison: linear vs exponential", "[phase2][solver][performance][optimization]") {
    ConstraintSolver solver;

    // With 15 bends, exponential would be 2^15 = 32768 states
    // Linear should be exactly 16 states
    std::vector<BendFeature> bends;
    for (int i = 0; i < 15; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 50.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    // Must be linear: n+1 states
    REQUIRE(output.graspConstraints.size() == 16);

    // Statistics should reflect linear count
    auto stats = solver.getStatistics();
    REQUIRE(stats.graspStatesAnalyzed == 16);

    // Should complete quickly
    REQUIRE(stats.totalSolveTimeMs < 2000.0);
}

TEST_CASE("Progressive enumeration maintains state consistency", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    std::vector<BendFeature> bends;
    for (int i = 0; i < 3; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 100.0;
        bend.position.y = 0.0;
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);
    REQUIRE(output.graspConstraints.size() == 4);

    // Verify each state is a superset of the previous
    for (size_t i = 1; i < output.graspConstraints.size(); i++) {
        const auto& current = output.graspConstraints[i].bentBends;
        const auto& previous = output.graspConstraints[i-1].bentBends;

        // All previous bends should be in current
        for (int bendId : previous) {
            REQUIRE(std::find(current.begin(), current.end(), bendId) != current.end());
        }

        // Current should have exactly one more bend than previous
        REQUIRE(current.size() == previous.size() + 1);
    }
}

TEST_CASE("Empty input still works with progressive enumeration", "[phase2][solver][states][optimization]") {
    ConstraintSolver solver;

    auto output = solver.solve({});

    REQUIRE(output.graspConstraints.empty());
    REQUIRE(output.success == true);
}

TEST_CASE("Large scale optimization test", "[phase2][solver][performance][optimization][.large]") {
    ConstraintSolver solver;

    // 20 bends - exponential would be over 1 million states
    std::vector<BendFeature> bends;
    for (int i = 0; i < 20; i++) {
        BendFeature bend;
        bend.id = i;
        bend.angle = 90.0;
        bend.length = 100.0;
        bend.position.x = i * 50.0;
        bend.position.y = (i % 2) * 25.0;  // Alternating Y positions
        bends.push_back(bend);
    }

    auto output = solver.solve(bends);

    // Linear: 21 states instead of 1,048,576
    REQUIRE(output.graspConstraints.size() == 21);

    // Must complete in reasonable time
    REQUIRE(solver.getStatistics().totalSolveTimeMs < 5000.0);
}
