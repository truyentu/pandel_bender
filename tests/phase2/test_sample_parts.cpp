#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include "fixtures/sample_parts.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase2::fixtures;
using Catch::Approx;

// ============================================================================
// Task 35: Sample Data Fixtures - Test Cases
// ============================================================================

// ----------------------------------------------------------------------------
// L-Bracket Tests
// ----------------------------------------------------------------------------

TEST_CASE("L-bracket - fixture creation", "[phase2][samples][l-bracket]") {
    auto bends = createLBracket();

    REQUIRE(bends.size() == 2);
    REQUIRE(bends[0].id == 0);
    REQUIRE(bends[1].id == 1);
    REQUIRE(bends[0].angle == Approx(90.0));
    REQUIRE(bends[1].angle == Approx(90.0));
}

TEST_CASE("L-bracket - solves successfully", "[phase2][samples][l-bracket]") {
    ConstraintSolver solver;
    auto bends = createLBracket();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 2);
    REQUIRE(output.abaConstraints.size() == 2);
}

TEST_CASE("L-bracket - all bends feasible", "[phase2][samples][l-bracket]") {
    ConstraintSolver solver;
    auto bends = createLBracket();

    auto output = solver.solve(bends);

    for (const auto& aba : output.abaConstraints) {
        REQUIRE(aba.feasible == true);
    }
}

TEST_CASE("L-bracket - valid precedence graph", "[phase2][samples][l-bracket]") {
    ConstraintSolver solver;
    auto bends = createLBracket();

    auto output = solver.solve(bends);

    REQUIRE(output.precedenceGraph.nodeCount() == 2);
    REQUIRE(output.precedenceGraph.isFinalized() == true);
    REQUIRE(output.precedenceGraph.isAcyclic() == true);
}

// ----------------------------------------------------------------------------
// U-Channel Tests
// ----------------------------------------------------------------------------

TEST_CASE("U-channel - fixture creation", "[phase2][samples][u-channel]") {
    auto bends = createUChannel();

    REQUIRE(bends.size() == 3);
    REQUIRE(bends[0].id == 0);  // Left wall
    REQUIRE(bends[1].id == 1);  // Bottom
    REQUIRE(bends[2].id == 2);  // Right wall

    // All 90-degree bends
    for (const auto& bend : bends) {
        REQUIRE(bend.angle == Approx(90.0));
    }
}

TEST_CASE("U-channel - solves successfully", "[phase2][samples][u-channel]") {
    ConstraintSolver solver;
    auto bends = createUChannel();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 3);
}

TEST_CASE("U-channel - correct bend count in sequence", "[phase2][samples][u-channel]") {
    ConstraintSolver solver;
    auto bends = createUChannel();

    auto output = solver.solve(bends);

    // All bends should be in sequence
    REQUIRE(output.bendSequence.size() == 3);

    // Each bend ID should appear exactly once
    std::vector<bool> found(3, false);
    for (int id : output.bendSequence) {
        REQUIRE(id >= 0);
        REQUIRE(id < 3);
        found[id] = true;
    }

    for (bool f : found) {
        REQUIRE(f == true);
    }
}

// ----------------------------------------------------------------------------
// Box Tests
// ----------------------------------------------------------------------------

TEST_CASE("Box - fixture creation", "[phase2][samples][box]") {
    auto bends = createBox();

    REQUIRE(bends.size() == 4);

    // All 90-degree bends
    for (const auto& bend : bends) {
        REQUIRE(bend.angle == Approx(90.0));
        REQUIRE(bend.length == Approx(100.0));
    }
}

TEST_CASE("Box - detects potential issues", "[phase2][samples][box]") {
    ConstraintSolver solver;
    auto bends = createBox();

    auto output = solver.solve(bends);

    // Box should solve (may have constraints)
    REQUIRE(output.bendSequence.size() == 4);
}

TEST_CASE("Box - all 4 bends in sequence", "[phase2][samples][box]") {
    ConstraintSolver solver;
    auto bends = createBox();

    auto output = solver.solve(bends);

    // Each bend should appear exactly once
    std::vector<int> counts(4, 0);
    for (int id : output.bendSequence) {
        REQUIRE(id >= 0);
        REQUIRE(id < 4);
        counts[id]++;
    }

    for (int count : counts) {
        REQUIRE(count == 1);
    }
}

TEST_CASE("Box - has valid precedence graph", "[phase2][samples][box]") {
    ConstraintSolver solver;
    auto bends = createBox();

    auto output = solver.solve(bends);

    REQUIRE(output.precedenceGraph.nodeCount() == 4);
    REQUIRE(output.precedenceGraph.isFinalized() == true);
    REQUIRE(output.precedenceGraph.isAcyclic() == true);
}

// ----------------------------------------------------------------------------
// Complex Panel Tests
// ----------------------------------------------------------------------------

TEST_CASE("Complex panel - fixture creation", "[phase2][samples][complex]") {
    auto bends = createComplexPanel();

    REQUIRE(bends.size() == 7);

    // Check mixed angles
    REQUIRE(bends[0].angle == Approx(90.0));
    REQUIRE(bends[1].angle == Approx(45.0));
    REQUIRE(bends[2].angle == Approx(90.0));
    REQUIRE(bends[3].angle == Approx(135.0));
    REQUIRE(bends[4].angle == Approx(90.0));
    REQUIRE(bends[5].angle == Approx(60.0));
    REQUIRE(bends[6].angle == Approx(90.0));
}

TEST_CASE("Complex panel - solves with mixed angles", "[phase2][samples][complex]") {
    ConstraintSolver solver;
    auto bends = createComplexPanel();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 7);

    for (const auto& aba : output.abaConstraints) {
        REQUIRE(aba.feasible == true);
    }
}

TEST_CASE("Complex panel - varying lengths", "[phase2][samples][complex]") {
    auto bends = createComplexPanel();

    double lengths[] = {150.0, 80.0, 120.0, 100.0, 200.0, 90.0, 110.0};
    for (size_t i = 0; i < bends.size(); i++) {
        REQUIRE(bends[i].length == Approx(lengths[i]));
    }
}

TEST_CASE("Complex panel - all bends have unique IDs", "[phase2][samples][complex]") {
    auto bends = createComplexPanel();

    std::vector<bool> found(7, false);
    for (const auto& bend : bends) {
        REQUIRE(bend.id >= 0);
        REQUIRE(bend.id < 7);
        REQUIRE(found[bend.id] == false);
        found[bend.id] = true;
    }
}

// ----------------------------------------------------------------------------
// Single Bend Tests
// ----------------------------------------------------------------------------

TEST_CASE("Single bend - fixture creation", "[phase2][samples][single]") {
    auto bends = createSingleBend();

    REQUIRE(bends.size() == 1);
    REQUIRE(bends[0].id == 0);
    REQUIRE(bends[0].angle == Approx(90.0));
    REQUIRE(bends[0].length == Approx(100.0));
}

TEST_CASE("Single bend - solves trivially", "[phase2][samples][single]") {
    ConstraintSolver solver;
    auto bends = createSingleBend();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 1);
    REQUIRE(output.bendSequence[0] == 0);
}

// ----------------------------------------------------------------------------
// Z-Profile Tests
// ----------------------------------------------------------------------------

TEST_CASE("Z-profile - fixture creation", "[phase2][samples][z-profile]") {
    auto bends = createZProfile();

    REQUIRE(bends.size() == 2);
    REQUIRE(bends[0].angle == Approx(90.0));
    REQUIRE(bends[1].angle == Approx(-90.0));  // Opposite direction
}

TEST_CASE("Z-profile - solves with opposing bends", "[phase2][samples][z-profile]") {
    ConstraintSolver solver;
    auto bends = createZProfile();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 2);
}

// ----------------------------------------------------------------------------
// Hat Profile Tests
// ----------------------------------------------------------------------------

TEST_CASE("Hat profile - fixture creation", "[phase2][samples][hat]") {
    auto bends = createHatProfile();

    REQUIRE(bends.size() == 4);

    // All 90-degree bends
    for (const auto& bend : bends) {
        REQUIRE(bend.angle == Approx(90.0));
    }
}

TEST_CASE("Hat profile - solves successfully", "[phase2][samples][hat]") {
    ConstraintSolver solver;
    auto bends = createHatProfile();

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 4);
}

TEST_CASE("Hat profile - valid precedence graph", "[phase2][samples][hat]") {
    ConstraintSolver solver;
    auto bends = createHatProfile();

    auto output = solver.solve(bends);

    REQUIRE(output.precedenceGraph.nodeCount() == 4);
    REQUIRE(output.precedenceGraph.isAcyclic() == true);
}

// ----------------------------------------------------------------------------
// Cross-Profile Comparison Tests
// ----------------------------------------------------------------------------

TEST_CASE("All fixtures - produce acyclic graphs", "[phase2][samples][integration]") {
    ConstraintSolver solver;

    SECTION("L-bracket") {
        auto output = solver.solve(createLBracket());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("U-channel") {
        auto output = solver.solve(createUChannel());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("Box") {
        auto output = solver.solve(createBox());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("Complex panel") {
        auto output = solver.solve(createComplexPanel());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("Single bend") {
        auto output = solver.solve(createSingleBend());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("Z-profile") {
        auto output = solver.solve(createZProfile());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }

    SECTION("Hat profile") {
        auto output = solver.solve(createHatProfile());
        REQUIRE(output.precedenceGraph.isAcyclic() == true);
    }
}

TEST_CASE("All fixtures - have complete bend sequences", "[phase2][samples][integration]") {
    ConstraintSolver solver;

    auto checkComplete = [&solver](const std::vector<openpanelcam::phase1::BendFeature>& bends) {
        auto output = solver.solve(bends);
        REQUIRE(output.bendSequence.size() == bends.size());

        // Verify all bend IDs are present
        std::vector<bool> found(bends.size(), false);
        for (int id : output.bendSequence) {
            REQUIRE(id >= 0);
            REQUIRE(static_cast<size_t>(id) < bends.size());
            found[id] = true;
        }
        for (bool f : found) {
            REQUIRE(f == true);
        }
    };

    SECTION("L-bracket") { checkComplete(createLBracket()); }
    SECTION("U-channel") { checkComplete(createUChannel()); }
    SECTION("Box") { checkComplete(createBox()); }
    SECTION("Complex panel") { checkComplete(createComplexPanel()); }
    SECTION("Single bend") { checkComplete(createSingleBend()); }
    SECTION("Z-profile") { checkComplete(createZProfile()); }
    SECTION("Hat profile") { checkComplete(createHatProfile()); }
}

TEST_CASE("All fixtures - have matching ABA constraints", "[phase2][samples][integration]") {
    ConstraintSolver solver;

    auto checkABA = [&solver](const std::vector<openpanelcam::phase1::BendFeature>& bends) {
        auto output = solver.solve(bends);
        REQUIRE(output.abaConstraints.size() == bends.size());
    };

    SECTION("L-bracket") { checkABA(createLBracket()); }
    SECTION("U-channel") { checkABA(createUChannel()); }
    SECTION("Box") { checkABA(createBox()); }
    SECTION("Complex panel") { checkABA(createComplexPanel()); }
    SECTION("Single bend") { checkABA(createSingleBend()); }
    SECTION("Z-profile") { checkABA(createZProfile()); }
    SECTION("Hat profile") { checkABA(createHatProfile()); }
}
