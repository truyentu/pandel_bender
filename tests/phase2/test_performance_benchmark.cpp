#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include "openpanelcam/phase2/phase1_adapter.h"

using namespace openpanelcam::phase2;

// ============================================================================
// Task 38: Performance Benchmarks with Catch2
// ============================================================================

TEST_CASE("Benchmark: 5 bends", "[phase2][benchmark]") {
    ConstraintSolver solver;
    auto bends = Phase1Adapter::createMockBends(5);

    BENCHMARK("solve 5 bends") {
        solver.reset();
        return solver.solve(bends);
    };
}

TEST_CASE("Benchmark: 10 bends", "[phase2][benchmark]") {
    ConstraintSolver solver;
    auto bends = Phase1Adapter::createMockBends(10);

    BENCHMARK("solve 10 bends") {
        solver.reset();
        return solver.solve(bends);
    };
}

TEST_CASE("Benchmark: 20 bends", "[phase2][benchmark]") {
    ConstraintSolver solver;
    auto bends = Phase1Adapter::createMockBends(20);

    BENCHMARK("solve 20 bends") {
        solver.reset();
        return solver.solve(bends);
    };
}

TEST_CASE("Benchmark: Component breakdown", "[phase2][benchmark][detailed]") {
    ConstraintSolver solver;
    auto bends = Phase1Adapter::createMockBends(10);

    auto output = solver.solve(bends);
    auto stats = solver.getStatistics();

    INFO("Total time: " << stats.totalSolveTimeMs << " ms");
    INFO("Geometric: " << stats.geometricAnalysisTimeMs << " ms");
    INFO("Grasp: " << stats.graspAnalysisTimeMs << " ms");
    INFO("ABA: " << stats.abaAnalysisTimeMs << " ms");
    INFO("Graph: " << stats.graphBuildTimeMs << " ms");
    INFO("Sort: " << stats.topologicalSortTimeMs << " ms");

    // Performance requirements (Debug mode)
    REQUIRE(stats.totalSolveTimeMs < 1000.0);

    REQUIRE(stats.geometricAnalysisTimeMs < 500.0);
    REQUIRE(stats.graspAnalysisTimeMs < 300.0);
    REQUIRE(stats.abaAnalysisTimeMs < 200.0);
}

TEST_CASE("Performance: Scaling with bend count", "[phase2][benchmark][scaling]") {
    ConstraintSolver solver;

    std::vector<int> bendCounts = {5, 10, 15, 20};
    std::vector<double> times;

    for (int count : bendCounts) {
        auto bends = Phase1Adapter::createMockBends(count);
        solver.reset();
        auto output = solver.solve(bends);
        times.push_back(solver.getStatistics().totalSolveTimeMs);

        INFO("Bends: " << count << ", Time: " << times.back() << " ms");
        REQUIRE(output.success == true);
    }

    // Verify O(n^2) scaling - ratio should be roughly (n2/n1)^2
    // 20 bends should be ~4x slower than 10 bends (not 16x for O(n^2))
    // Allow generous margin for Debug mode overhead
    REQUIRE(times[3] < times[1] * 10);  // 20 bends < 10x of 10 bends time
}
