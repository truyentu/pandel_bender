#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include "openpanelcam/phase2/phase1_adapter.h"
#include <set>

using namespace openpanelcam::phase2;

TEST_CASE("Phase1Adapter creates valid mock bends", "[phase2][integration]") {
    auto bends = Phase1Adapter::createMockBends(5);

    REQUIRE(bends.size() == 5);

    for (size_t i = 0; i < bends.size(); i++) {
        REQUIRE(bends[i].id == static_cast<int>(i));
        REQUIRE(bends[i].angle == 90.0);
        REQUIRE(bends[i].length > 0);
    }
}

TEST_CASE("ConstraintSolver accepts Phase1Adapter output", "[phase2][integration]") {
    ConstraintSolver solver;
    auto bends = Phase1Adapter::createMockBends(3);

    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 3);
}

TEST_CASE("Full pipeline mock integration", "[phase2][integration][e2e]") {
    auto bends = Phase1Adapter::createMockBends(7);

    ConstraintSolver solver;
    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(!output.bendSequence.empty());
    REQUIRE(output.precedenceGraph.isFinalized());
    REQUIRE(output.precedenceGraph.isAcyclic());

    std::set<int> sequencedBends(
        output.bendSequence.begin(),
        output.bendSequence.end()
    );
    REQUIRE(sequencedBends.size() == bends.size());
}

TEST_CASE("Phase1Adapter handles zero bends", "[phase2][integration]") {
    auto bends = Phase1Adapter::createMockBends(0);
    REQUIRE(bends.empty());
}

TEST_CASE("Phase1Adapter generates unique IDs", "[phase2][integration]") {
    auto bends = Phase1Adapter::createMockBends(10);

    std::set<int> ids;
    for (const auto& b : bends) {
        ids.insert(b.id);
    }
    REQUIRE(ids.size() == 10);
}

TEST_CASE("Phase1Adapter createMockBendsWithAngles works", "[phase2][integration]") {
    std::vector<double> angles = {45.0, 90.0, 135.0};
    auto bends = Phase1Adapter::createMockBendsWithAngles(6, angles);

    REQUIRE(bends.size() == 6);
    REQUIRE(bends[0].angle == 45.0);
    REQUIRE(bends[1].angle == 90.0);
    REQUIRE(bends[2].angle == 135.0);
    REQUIRE(bends[3].angle == 45.0);  // Cycles back
    REQUIRE(bends[4].angle == 90.0);
    REQUIRE(bends[5].angle == 135.0);
}

TEST_CASE("Phase1Adapter createSingleBend works", "[phase2][integration]") {
    auto bend = Phase1Adapter::createSingleBend(42, 120.0, 250.0, 100.0, 50.0, 0.0);

    REQUIRE(bend.id == 42);
    REQUIRE(bend.angle == 120.0);
    REQUIRE(bend.length == 250.0);
    REQUIRE(bend.position.x == 100.0);
    REQUIRE(bend.position.y == 50.0);
    REQUIRE(bend.position.z == 0.0);
}

TEST_CASE("Phase1Adapter createBoxClosingScenario generates 4 bends", "[phase2][integration]") {
    auto bends = Phase1Adapter::createBoxClosingScenario();

    REQUIRE(bends.size() == 4);

    for (const auto& bend : bends) {
        REQUIRE(bend.angle == 90.0);
        REQUIRE(bend.length == 200.0);
    }

    // Verify unique IDs
    std::set<int> ids;
    for (const auto& b : bends) {
        ids.insert(b.id);
    }
    REQUIRE(ids.size() == 4);
}

TEST_CASE("ConstraintSolver handles box closing scenario", "[phase2][integration]") {
    auto bends = Phase1Adapter::createBoxClosingScenario();

    ConstraintSolver solver;
    auto output = solver.solve(bends);

    // Box closing is a complex scenario - solver should either succeed or report errors
    REQUIRE((output.success == true || !output.errors.empty()));

    if (output.success) {
        REQUIRE(output.bendSequence.size() == 4);

        // Verify all bends are sequenced
        std::set<int> sequencedBends(
            output.bendSequence.begin(),
            output.bendSequence.end()
        );
        REQUIRE(sequencedBends.size() == 4);
    }
}

TEST_CASE("Large bend count integration test", "[phase2][integration][performance]") {
    auto bends = Phase1Adapter::createMockBends(50);

    ConstraintSolver solver;
    auto output = solver.solve(bends);

    REQUIRE(output.success == true);
    REQUIRE(output.bendSequence.size() == 50);

    // Check statistics are populated
    auto stats = solver.getStatistics();
    REQUIRE(stats.totalBends == 50);
}

TEST_CASE("Mock bends have correct geometric properties", "[phase2][integration]") {
    auto bends = Phase1Adapter::createMockBends(4);

    // Check positions are spaced correctly
    REQUIRE(bends[0].position.x == 0.0);
    REQUIRE(bends[1].position.x == 150.0);
    REQUIRE(bends[2].position.x == 300.0);
    REQUIRE(bends[3].position.x == 450.0);

    // Check alternating Y positions
    REQUIRE(bends[0].position.y == 0.0);
    REQUIRE(bends[1].position.y == 100.0);
    REQUIRE(bends[2].position.y == 0.0);
    REQUIRE(bends[3].position.y == 100.0);

    // Check alternating directions
    REQUIRE(bends[0].direction.x == 1.0);
    REQUIRE(bends[0].direction.y == 0.0);
    REQUIRE(bends[1].direction.x == 0.0);
    REQUIRE(bends[1].direction.y == 1.0);
}

TEST_CASE("Empty angles vector uses default angle", "[phase2][integration]") {
    std::vector<double> emptyAngles;
    auto bends = Phase1Adapter::createMockBendsWithAngles(3, emptyAngles);

    REQUIRE(bends.size() == 3);
    for (const auto& bend : bends) {
        REQUIRE(bend.angle == 90.0);
    }
}
