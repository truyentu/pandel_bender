#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/astar_search.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// Helper: create Phase2Output with simple independent bends
static Phase2Output makePhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) {
        p2.precedenceGraph.addNode(i);
    }
    p2.precedenceGraph.finalize();
    p2.success = true;
    for (int i = 0; i < n; i++) {
        p2.bendSequence.push_back(i);
    }
    return p2;
}

// Helper: create Phase2Output with chain 0->1->2->...
static Phase2Output makeChainPhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) {
        p2.precedenceGraph.addNode(i);
    }
    for (int i = 0; i < n - 1; i++) {
        p2.precedenceGraph.addEdge(i, i + 1, ConstraintType::GEOMETRIC, 1.0, "chain");
    }
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static std::vector<BendFeature> makeBends(int n) {
    std::vector<BendFeature> bends(n);
    for (int i = 0; i < n; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = 100.0;
        bends[i].direction = {1.0, 0.0, 0.0};
    }
    return bends;
}

TEST_CASE("AStarSearch empty input", "[phase3][astar]") {
    auto p2 = makePhase2(0);
    std::vector<BendFeature> bends;

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.empty());
}

TEST_CASE("AStarSearch single bend", "[phase3][astar]") {
    auto p2 = makePhase2(1);
    auto bends = makeBends(1);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.success == true);
    REQUIRE(result.optimal == true);
    REQUIRE(result.bendSequence.size() == 1);
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.totalCycleTime > 0.0);
}

TEST_CASE("AStarSearch two independent bends", "[phase3][astar]") {
    auto p2 = makePhase2(2);
    auto bends = makeBends(2);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 2);

    // Both bends must appear
    bool has0 = false, has1 = false;
    for (int id : result.bendSequence) {
        if (id == 0) has0 = true;
        if (id == 1) has1 = true;
    }
    REQUIRE(has0);
    REQUIRE(has1);
}

TEST_CASE("AStarSearch chain respects order", "[phase3][astar]") {
    auto p2 = makeChainPhase2(3);
    auto bends = makeBends(3);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 3);
    // Must be [0, 1, 2] due to chain constraints
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.bendSequence[1] == 1);
    REQUIRE(result.bendSequence[2] == 2);
}

TEST_CASE("AStarSearch statistics populated", "[phase3][astar]") {
    auto p2 = makePhase2(3);
    auto bends = makeBends(3);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.stats.nodesExpanded > 0);
    REQUIRE(result.stats.nodesGenerated > 0);
    REQUIRE(result.stats.searchTimeMs >= 0.0);
    REQUIRE(result.stats.solutionDepth == 3);
}

TEST_CASE("AStarSearch actions match sequence", "[phase3][astar]") {
    auto p2 = makePhase2(3);
    auto bends = makeBends(3);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.actions.size() == result.bendSequence.size());
    for (size_t i = 0; i < result.actions.size(); i++) {
        REQUIRE(result.actions[i].type == ActionType::BEND);
        REQUIRE(result.actions[i].bendId == result.bendSequence[i]);
        REQUIRE(result.actions[i].duration > 0.0);
    }
}

TEST_CASE("AStarSearch five bends within time", "[phase3][astar]") {
    auto p2 = makePhase2(5);
    auto bends = makeBends(5);

    AStarSearch search(p2, bends);

    SearchConfig config;
    config.timeoutSeconds = 5.0;
    auto result = search.search(config);

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 5);
    REQUIRE(result.stats.searchTimeMs < 5000.0);
}

TEST_CASE("AStarSearch diamond DAG", "[phase3][astar]") {
    // DAG:  0 -> 1
    //       0 -> 2
    //       1 -> 3
    //       2 -> 3
    Phase2Output p2;
    for (int i = 0; i < 4; i++) p2.precedenceGraph.addNode(i);
    p2.precedenceGraph.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "");
    p2.precedenceGraph.addEdge(0, 2, ConstraintType::GEOMETRIC, 1.0, "");
    p2.precedenceGraph.addEdge(1, 3, ConstraintType::GEOMETRIC, 1.0, "");
    p2.precedenceGraph.addEdge(2, 3, ConstraintType::GEOMETRIC, 1.0, "");
    p2.precedenceGraph.finalize();
    p2.success = true;

    auto bends = makeBends(4);

    AStarSearch search(p2, bends);
    auto result = search.search();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 4);

    // Bend 0 must be first, bend 3 must be last
    REQUIRE(result.bendSequence[0] == 0);
    REQUIRE(result.bendSequence[3] == 3);
}

TEST_CASE("AStarSearch max nodes limit", "[phase3][astar]") {
    auto p2 = makePhase2(8);
    auto bends = makeBends(8);

    AStarSearch search(p2, bends);

    SearchConfig config;
    config.maxNodes = 5;  // Very low limit
    auto result = search.search(config);

    // Should either succeed quickly or hit limit
    if (!result.success) {
        REQUIRE(result.errorMessage.find("Max nodes") != std::string::npos);
    }
}
