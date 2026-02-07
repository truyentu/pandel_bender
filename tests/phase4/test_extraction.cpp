#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/extraction_checker.h"

using namespace openpanelcam::phase4;
using namespace openpanelcam::phase1;
using Catch::Approx;

static BendFeature makeBend(int id, double angle, double length,
                             double px, double py, double pz,
                             double dx, double dy, double dz) {
    BendFeature b;
    b.id = id;
    b.angle = angle;
    b.length = length;
    b.position = {px, py, pz};
    b.direction = {dx, dy, dz};
    b.normal = {0, 0, 1};
    return b;
}

// ===== ExtractionChecker Tests =====

TEST_CASE("ExtractionChecker passes for empty state", "[phase4][extraction]") {
    ExtractionChecker checker;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    state.addBentFlange(bend);

    auto result = checker.check(bend, state);
    REQUIRE(result.canExtract == true);
}

TEST_CASE("ExtractionChecker passes for distant flanges", "[phase4][extraction]") {
    ExtractionChecker checker;
    BentState state;

    auto bend0 = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto bend1 = makeBend(1, 90.0, 100.0, 2000, 2000, 0, 0, 1, 0);
    state.addBentFlange(bend0);
    state.addBentFlange(bend1);

    auto result = checker.check(bend1, state);
    REQUIRE(result.canExtract == true);
}

TEST_CASE("ExtractionChecker detects trap from overhead flange", "[phase4][extraction]") {
    ExtractionChecker checker;
    BentState state;

    // Bend 0 at origin
    auto bend0 = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    state.addBentFlange(bend0);

    // Bend 1 positioned so its bent flange creates an obstacle above bend 0's tool
    // This simulates a box-like situation where extraction is blocked
    auto bendAbove = makeBend(1, 90.0, 200.0, 0, 0, 50, 1, 0, 0);
    state.addBentFlange(bendAbove);

    // Check extraction for bend 0 - the flange from bend 1 is above
    auto result = checker.check(bend0, state);

    // The overhead flange should block vertical extraction
    REQUIRE(result.canExtract == false);
    REQUIRE(result.trappedAtBend == 0);
    REQUIRE(!result.description.empty());
}

TEST_CASE("ExtractionChecker canExtractVertically with clear path", "[phase4][extraction]") {
    ExtractionChecker checker;

    AABB tool(-50, -15, -25, 50, 15, 25);
    std::vector<AABB> obstacles = {
        AABB(500, 500, 0, 600, 600, 100) // Far away
    };

    REQUIRE(checker.canExtractVertically(tool, obstacles) == true);
}

TEST_CASE("ExtractionChecker canExtractVertically with blocking obstacle", "[phase4][extraction]") {
    ExtractionChecker checker;

    AABB tool(-50, -10, -25, 50, 10, 25);
    std::vector<AABB> obstacles = {
        AABB(-30, -5, 30, 30, 5, 200) // Above the tool, overlapping XY
    };

    REQUIRE(checker.canExtractVertically(tool, obstacles) == false);
}

TEST_CASE("ExtractionChecker canExtractVertically obstacle below is OK", "[phase4][extraction]") {
    ExtractionChecker checker;

    AABB tool(-50, -10, 0, 50, 10, 50);
    std::vector<AABB> obstacles = {
        AABB(-30, -5, -200, 30, 5, -100) // Below the tool
    };

    REQUIRE(checker.canExtractVertically(tool, obstacles) == true);
}

// ===== SpringbackCompensator Tests =====

TEST_CASE("SpringbackCompensator default construction", "[phase4][springback]") {
    SpringbackCompensator comp;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto data = comp.compensate(bend);

    REQUIRE(data.bendId == 0);
    REQUIRE(data.targetAngle == 90.0);
    REQUIRE(data.springbackAngle > 0.0);
    REQUIRE(data.compensatedAngle > 90.0); // Overbend
}

TEST_CASE("SpringbackCompensator compensated angle > target", "[phase4][springback]") {
    SpringbackCompensator comp;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto data = comp.compensate(bend);

    REQUIRE(data.compensatedAngle > data.targetAngle);
    REQUIRE(data.compensatedAngle == Approx(data.targetAngle + data.springbackAngle));
}

TEST_CASE("SpringbackCompensator disabled returns same angle", "[phase4][springback]") {
    ValidatorConfig config;
    config.enableSpringback = false;
    SpringbackCompensator comp(config);

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto data = comp.compensate(bend);

    REQUIRE(data.compensatedAngle == 90.0);
    REQUIRE(data.springbackAngle == 0.0);
}

TEST_CASE("SpringbackCompensator larger angle = more springback", "[phase4][springback]") {
    SpringbackCompensator comp;
    auto bend45 = makeBend(0, 45.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto bend90 = makeBend(1, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto bend135 = makeBend(2, 135.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto data45 = comp.compensate(bend45);
    auto data90 = comp.compensate(bend90);
    auto data135 = comp.compensate(bend135);

    REQUIRE(data135.springbackAngle > data90.springbackAngle);
    REQUIRE(data90.springbackAngle > data45.springbackAngle);
}

TEST_CASE("SpringbackCompensator thicker material = more springback", "[phase4][springback]") {
    ValidatorConfig thinConfig;
    thinConfig.materialThickness = 0.5;
    SpringbackCompensator thinComp(thinConfig);

    ValidatorConfig thickConfig;
    thickConfig.materialThickness = 3.0;
    SpringbackCompensator thickComp(thickConfig);

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto dataThin = thinComp.compensate(bend);
    auto dataThick = thickComp.compensate(bend);

    REQUIRE(dataThick.springbackAngle > dataThin.springbackAngle);
}

TEST_CASE("SpringbackCompensator negative angle", "[phase4][springback]") {
    SpringbackCompensator comp;
    auto bend = makeBend(0, -90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto data = comp.compensate(bend);

    REQUIRE(data.targetAngle == -90.0);
    REQUIRE(data.compensatedAngle < -90.0); // More negative (overbend)
    REQUIRE(data.springbackAngle > 0.0);
}

TEST_CASE("SpringbackCompensator compensateAll", "[phase4][springback]") {
    SpringbackCompensator comp;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 45.0, 150.0, 200, 0, 0, 0, 1, 0),
        makeBend(2, 135.0, 80.0, 0, 200, 0, 1, 0, 0),
    };

    auto results = comp.compensateAll(bends);

    REQUIRE(results.size() == 3);
    REQUIRE(results[0].bendId == 0);
    REQUIRE(results[1].bendId == 1);
    REQUIRE(results[2].bendId == 2);

    for (const auto& r : results) {
        REQUIRE(r.compensatedAngle > r.targetAngle);
        REQUIRE(r.springbackAngle > 0.0);
    }
}

TEST_CASE("SpringbackCompensator springback formula", "[phase4][springback]") {
    ValidatorConfig config;
    config.springbackBaseDeg = 2.0;
    config.springbackPerMm = 0.5;
    config.materialThickness = 1.5;
    SpringbackCompensator comp(config);

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto data = comp.compensate(bend);

    // Expected: baseDeg + perMm * thickness = 2.0 + 0.5 * 1.5 = 2.75°
    // Scaled by angle/90 = 90/90 = 1.0
    // springbackAngle = 2.75°
    REQUIRE(data.springbackAngle == Approx(2.75));
    REQUIRE(data.compensatedAngle == Approx(92.75));
}
