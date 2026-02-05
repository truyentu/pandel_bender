#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

TEST_CASE("BentState construction", "[phase2][geometric]") {
    BentState state;

    REQUIRE(state.count() == 0);
    REQUIRE(state.getBentBends().empty());
}

TEST_CASE("BentState apply and check bends", "[phase2][geometric]") {
    BentState state;

    state.applyBend(0);
    state.applyBend(1);
    state.applyBend(2);

    REQUIRE(state.count() == 3);
    REQUIRE(state.isBent(0) == true);
    REQUIRE(state.isBent(1) == true);
    REQUIRE(state.isBent(2) == true);
    REQUIRE(state.isBent(3) == false);
}

TEST_CASE("BentState reset", "[phase2][geometric]") {
    BentState state;

    state.applyBend(0);
    state.applyBend(1);
    REQUIRE(state.count() == 2);

    state.reset();
    REQUIRE(state.count() == 0);
    REQUIRE(state.isBent(0) == false);
    REQUIRE(state.isBent(1) == false);
}

TEST_CASE("GeometricPrecedenceAnalyzer construction", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;

    auto stats = analyzer.getStatistics();
    REQUIRE(stats.totalPairsChecked == 0);
    REQUIRE(stats.totalConstraints == 0);
}

TEST_CASE("GeometricPrecedenceAnalyzer analyze empty bends", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;
    std::vector<BendFeature> bends;  // Empty

    auto constraints = analyzer.analyze(bends);

    REQUIRE(constraints.empty());
}

TEST_CASE("GeometricPrecedenceAnalyzer analyze single bend", "[phase2][geometric]") {
    GeometricPrecedenceAnalyzer analyzer;

    // Create a dummy bend
    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;

    std::vector<BendFeature> bends = { bend };

    auto constraints = analyzer.analyze(bends);

    // Single bend -> no constraints
    REQUIRE(constraints.empty());
}

TEST_CASE("BentState get bent bends list", "[phase2][geometric]") {
    BentState state;

    state.applyBend(5);
    state.applyBend(7);

    const auto& bentList = state.getBentBends();
    REQUIRE(bentList.size() == 2);
    REQUIRE(bentList[0] == 5);
    REQUIRE(bentList[1] == 7);
}
