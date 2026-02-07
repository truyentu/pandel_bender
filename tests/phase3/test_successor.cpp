#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/successor_generator.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

// Helper: create simple DAG with N independent bends (no edges)
static PrecedenceDAG makeSimpleDAG(int n) {
    PrecedenceDAG dag;
    for (int i = 0; i < n; i++) {
        dag.addNode(i);
    }
    dag.finalize();
    return dag;
}

// Helper: create bends
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

TEST_CASE("SuccessorGenerator canBend - no predecessors", "[phase3][successor]") {
    auto dag = makeSimpleDAG(3);
    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;

    SuccessorGenerator gen(dag, bends, grasp);

    SearchState state;
    REQUIRE(gen.canBend(0, state) == true);
    REQUIRE(gen.canBend(1, state) == true);
    REQUIRE(gen.canBend(2, state) == true);
}

TEST_CASE("SuccessorGenerator canBend - already bent", "[phase3][successor]") {
    auto dag = makeSimpleDAG(3);
    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;

    SuccessorGenerator gen(dag, bends, grasp);

    SearchState state;
    state.markBent(0);

    REQUIRE(gen.canBend(0, state) == false);
    REQUIRE(gen.canBend(1, state) == true);
}

TEST_CASE("SuccessorGenerator canBend - with predecessors", "[phase3][successor]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    // 0 -> 1 -> 2
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "test");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "test");
    dag.finalize();

    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;
    SuccessorGenerator gen(dag, bends, grasp);

    SearchState state;
    // Only bend 0 is bendable initially
    REQUIRE(gen.canBend(0, state) == true);
    REQUIRE(gen.canBend(1, state) == false);  // needs 0
    REQUIRE(gen.canBend(2, state) == false);  // needs 1

    state.markBent(0);
    REQUIRE(gen.canBend(1, state) == true);   // 0 is done
    REQUIRE(gen.canBend(2, state) == false);  // still needs 1

    state.markBent(1);
    REQUIRE(gen.canBend(2, state) == true);   // both 0,1 done
}

TEST_CASE("SuccessorGenerator getBendableBends", "[phase3][successor]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 2, ConstraintType::GEOMETRIC, 1.0, "test");
    dag.finalize();

    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;
    SuccessorGenerator gen(dag, bends, grasp);

    SearchState state;
    auto bendable = gen.getBendableBends(state);
    // 0 and 1 are bendable (2 needs 0)
    REQUIRE(bendable.size() == 2);

    state.markBent(0);
    bendable = gen.getBendableBends(state);
    // 1 and 2 are now bendable
    REQUIRE(bendable.size() == 2);
}

TEST_CASE("SuccessorGenerator generate produces valid successors", "[phase3][successor]") {
    auto dag = makeSimpleDAG(3);
    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;
    SuccessorGenerator gen(dag, bends, grasp);

    MaskedTimeCost costFn;
    CombinedHeuristic heuristic;

    SearchNode initial = SearchNode::createInitial();
    auto successors = gen.generate(initial, 0, costFn, heuristic);

    // 3 independent bends -> 3 successors
    REQUIRE(successors.size() == 3);

    for (const auto& succ : successors) {
        REQUIRE(succ.g > 0.0);
        REQUIRE(succ.parentId == 0);
        REQUIRE(succ.lastBendId >= 0);
        REQUIRE(succ.lastBendId < 3);
        REQUIRE(succ.state.bentCount() == 1);
    }
}

TEST_CASE("SuccessorGenerator generate respects chain", "[phase3][successor]") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "chain");
    dag.addEdge(1, 2, ConstraintType::GEOMETRIC, 1.0, "chain");
    dag.finalize();

    auto bends = makeBends(3);
    std::vector<GraspConstraint> grasp;
    SuccessorGenerator gen(dag, bends, grasp);

    MaskedTimeCost costFn;
    CombinedHeuristic heuristic;

    SearchNode initial = SearchNode::createInitial();
    auto successors = gen.generate(initial, 0, costFn, heuristic);

    // Only bend 0 is available
    REQUIRE(successors.size() == 1);
    REQUIRE(successors[0].lastBendId == 0);
}

TEST_CASE("SuccessorGenerator generate at goal produces nothing", "[phase3][successor]") {
    auto dag = makeSimpleDAG(2);
    auto bends = makeBends(2);
    std::vector<GraspConstraint> grasp;
    SuccessorGenerator gen(dag, bends, grasp);

    MaskedTimeCost costFn;
    CombinedHeuristic heuristic;

    SearchNode goal = SearchNode::createInitial();
    goal.state.markBent(0);
    goal.state.markBent(1);

    auto successors = gen.generate(goal, 0, costFn, heuristic);
    REQUIRE(successors.empty());
}
