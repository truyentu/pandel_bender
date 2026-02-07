#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/sequencer.h"
#include "openpanelcam/phase2/constraint_solver.h"
#include <chrono>

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;
using Catch::Approx;

static Phase2Output makePhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) p2.precedenceGraph.addNode(i);
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static Phase2Output makeChainPhase2(int n) {
    Phase2Output p2;
    for (int i = 0; i < n; i++) p2.precedenceGraph.addNode(i);
    for (int i = 0; i < n - 1; i++)
        p2.precedenceGraph.addEdge(i, i + 1, ConstraintType::GEOMETRIC, 1.0, "chain");
    p2.precedenceGraph.finalize();
    p2.success = true;
    return p2;
}

static std::vector<BendFeature> makeBends(int n) {
    std::vector<BendFeature> bends(n);
    for (int i = 0; i < n; i++) {
        bends[i].id = i;
        bends[i].angle = 90.0;
        bends[i].length = 100.0 + i * 10.0; // Varying lengths
        // Distribute directions across orientations
        switch (i % 4) {
            case 0: bends[i].direction = {1, 0, 0}; break;
            case 1: bends[i].direction = {0, 1, 0}; break;
            case 2: bends[i].direction = {-1, 0, 0}; break;
            case 3: bends[i].direction = {0, -1, 0}; break;
        }
    }
    return bends;
}

// ===== Performance Benchmarks =====

TEST_CASE("Benchmark: 3 bends (L-bracket)", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makePhase2(3);
    auto bends = makeBends(3);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = seq.sequence(p2, bends);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 3);
    REQUIRE(ms < 100.0); // Should be nearly instant
    INFO("3 bends: " << ms << " ms, nodes expanded: " << result.stats.nodesExpanded);
}

TEST_CASE("Benchmark: 5 bends (typical part)", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makePhase2(5);
    auto bends = makeBends(5);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = seq.sequence(p2, bends);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 5);
    REQUIRE(ms < 1000.0); // < 1 second
    INFO("5 bends: " << ms << " ms, nodes expanded: " << result.stats.nodesExpanded);
}

TEST_CASE("Benchmark: 8 bends (complex part)", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makePhase2(8);
    auto bends = makeBends(8);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = seq.sequence(p2, bends);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 8);
    REQUIRE(ms < 5000.0); // < 5 seconds
    INFO("8 bends: " << ms << " ms, nodes expanded: " << result.stats.nodesExpanded);
}

TEST_CASE("Benchmark: 10 bends (advanced part)", "[phase3][benchmark]") {
    Sequencer seq;

    SearchConfig config;
    config.maxNodes = 1000000;
    config.timeoutSeconds = 10.0;
    seq.setConfig(config);

    auto p2 = makePhase2(10);
    auto bends = makeBends(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = seq.sequence(p2, bends);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 10);
    REQUIRE(ms < 10000.0); // < 10 seconds
    INFO("10 bends: " << ms << " ms, nodes expanded: " << result.stats.nodesExpanded);
}

TEST_CASE("Benchmark: 10 bends chain (fully constrained)", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makeChainPhase2(10);
    auto bends = makeBends(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = seq.sequence(p2, bends);
    auto end = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(result.bendSequence.size() == 10);
    // Chain is trivial - only one path exists
    REQUIRE(ms < 100.0); // Should be nearly instant
    INFO("10 bends chain: " << ms << " ms, nodes expanded: " << result.stats.nodesExpanded);

    // Verify chain order
    for (int i = 0; i < 10; i++) {
        REQUIRE(result.bendSequence[i] == i);
    }
}

TEST_CASE("Benchmark: deduplication effectiveness", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makePhase2(6);
    auto bends = makeBends(6);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);

    // Deduplication should skip many duplicates
    // 6! = 720 permutations but many share same state
    double dedupRate = 0.0;
    if (result.stats.nodesGenerated > 0) {
        dedupRate = static_cast<double>(result.stats.duplicatesSkipped) /
                    result.stats.nodesGenerated * 100.0;
    }
    INFO("Dedup rate: " << dedupRate << "%, expanded: " << result.stats.nodesExpanded
         << ", generated: " << result.stats.nodesGenerated
         << ", skipped: " << result.stats.duplicatesSkipped);

    // Should have non-trivial deduplication
    REQUIRE(result.stats.duplicatesSkipped > 0);
}

TEST_CASE("Benchmark: heuristic effectiveness", "[phase3][benchmark]") {
    Sequencer seq;
    auto p2 = makePhase2(5);
    auto bends = makeBends(5);

    auto result = seq.sequence(p2, bends);

    REQUIRE(result.success == true);

    // With good heuristic, should expand much less than 5! = 120 nodes
    INFO("Nodes expanded: " << result.stats.nodesExpanded << " (total permutations: 120)");
    // Heuristic should reduce search significantly
    REQUIRE(result.stats.nodesExpanded < 120);
}
