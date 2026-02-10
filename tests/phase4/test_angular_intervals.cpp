#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/swept_volume.h"
#include "openpanelcam/phase4/collision_detector.h"
#include "openpanelcam/phase4/bent_state.h"

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

// ===== computeIntervalCount Tests =====

TEST_CASE("computeIntervalCount: short flange = 10 intervals", "[phase4][intervals]") {
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(50.0) == 10);
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(99.9) == 10);
}

TEST_CASE("computeIntervalCount: long flange = 20 intervals", "[phase4][intervals]") {
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(100.0) == 20);
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(500.0) == 20);
}

TEST_CASE("computeIntervalCount: boundary at 100mm", "[phase4][intervals]") {
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(99.99) == 10);
    REQUIRE(SweptVolumeGenerator::computeIntervalCount(100.0) == 20);
}

// ===== generateIntervalAABBs Tests =====

TEST_CASE("generateIntervalAABBs: produces correct count for short flange", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);

    auto intervals = gen.generateIntervalAABBs(bend);

    // 10 intervals + 1 = 11 AABBs
    REQUIRE(intervals.size() == 11);
}

TEST_CASE("generateIntervalAABBs: produces correct count for long flange", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 200.0, 0, 0, 0, 1, 0, 0);

    auto intervals = gen.generateIntervalAABBs(bend);

    // 20 intervals + 1 = 21 AABBs
    REQUIRE(intervals.size() == 21);
}

TEST_CASE("generateIntervalAABBs: all AABBs have positive volume", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto intervals = gen.generateIntervalAABBs(bend);

    for (size_t i = 0; i < intervals.size(); i++) {
        REQUIRE(intervals[i].volume() > 0.0);
    }
}

TEST_CASE("generateIntervalAABBs: all AABBs contain bend position", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 50, 30, 0, 1, 0, 0);

    auto intervals = gen.generateIntervalAABBs(bend);

    for (size_t i = 0; i < intervals.size(); i++) {
        REQUIRE(intervals[i].minX <= 50.0);
        REQUIRE(intervals[i].maxX >= 50.0);
        REQUIRE(intervals[i].minY <= 30.0);
        REQUIRE(intervals[i].maxY >= 30.0);
    }
}

// ===== estimateAABBAtAngle Tests =====

TEST_CASE("estimateAABBAtAngle: at zero angle has positive volume", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    AABB atZero = gen.estimateAABBAtAngle(bend, 0.0);
    REQUIRE(atZero.volume() > 0.0);
}

TEST_CASE("estimateAABBAtAngle: at full angle has positive volume", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    AABB atFull = gen.estimateAABBAtAngle(bend, 90.0);
    REQUIRE(atFull.volume() > 0.0);
}

// ===== SweptVolume now includes intervalAABBs =====

TEST_CASE("SweptVolume::generate populates intervalAABBs", "[phase4][intervals]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 150.0, 0, 0, 0, 1, 0, 0);

    SweptVolume sv = gen.generate(bend);

    // 150mm >= 100mm -> 20 intervals -> 21 AABBs
    REQUIRE(sv.intervalAABBs.size() == 21);
    REQUIRE(sv.aabb.volume() > 0.0);
}

// ===== CollisionDetector::checkIntervals Tests =====

TEST_CASE("checkIntervals: no collision when empty state", "[phase4][intervals]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto result = detector.checkIntervals(sv, state);
    REQUIRE(result.hasCollision == false);
}

TEST_CASE("checkIntervals: detects collision at intervals", "[phase4][intervals]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    // Place a flange at the origin
    state.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    // Bend sweeping through the same area
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto result = detector.checkIntervals(sv, state);
    REQUIRE(result.hasCollision == true);
    REQUIRE(result.bendId == 0);
    REQUIRE(result.collidingBendId == 1);
}

TEST_CASE("checkIntervals: no collision for distant flanges", "[phase4][intervals]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    state.addBentFlange(makeBend(1, 90.0, 50.0, 5000, 5000, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto result = detector.checkIntervals(sv, state);
    REQUIRE(result.hasCollision == false);
}

TEST_CASE("checkIntervals: empty intervalAABBs returns no collision", "[phase4][intervals]") {
    CollisionDetector detector;
    BentState state;
    state.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    SweptVolume sv;
    sv.bendId = 0;
    // intervalAABBs is empty

    auto result = detector.checkIntervals(sv, state);
    REQUIRE(result.hasCollision == false);
}

TEST_CASE("checkIntervals: description mentions interval index", "[phase4][intervals]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    state.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto result = detector.checkIntervals(sv, state);
    if (result.hasCollision) {
        REQUIRE(result.description.find("interval") != std::string::npos);
    }
}
