#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/cost_function.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ===== MaskedTimeCost Tests =====

TEST_CASE("MaskedTimeCost uses default config", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.config().baseBendTime == 2.0);
    REQUIRE(cost.config().rotationTime == 1.5);
    REQUIRE(cost.config().abaReconfigTime == 0.8);
}

TEST_CASE("bendTime returns base time for 90 degree", "[phase3][cost]") {
    MaskedTimeCost cost;

    BendFeature bend;
    bend.angle = 90.0;

    REQUIRE(cost.bendTime(bend) == Approx(2.0));
}

TEST_CASE("bendTime increases for larger angles", "[phase3][cost]") {
    MaskedTimeCost cost;

    BendFeature bend90, bend120;
    bend90.angle = 90.0;
    bend120.angle = 120.0;

    REQUIRE(cost.bendTime(bend120) > cost.bendTime(bend90));
}

TEST_CASE("rotationTime is 0 for same orientation", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.rotationTime(Orientation::DEG_0, Orientation::DEG_0) == 0.0);
    REQUIRE(cost.rotationTime(Orientation::DEG_90, Orientation::DEG_90) == 0.0);
}

TEST_CASE("rotationTime for 90 degree rotation", "[phase3][cost]") {
    MaskedTimeCost cost;

    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_90);
    REQUIRE(time == Approx(1.5));
}

TEST_CASE("rotationTime takes shortest path", "[phase3][cost]") {
    MaskedTimeCost cost;

    // 0 -> 270 should be 1 step (shortest), not 3 steps
    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_270);
    REQUIRE(time == Approx(1.5));
}

TEST_CASE("rotationTime for 180 degree rotation", "[phase3][cost]") {
    MaskedTimeCost cost;

    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_180);
    REQUIRE(time == Approx(3.0));  // 2 steps
}

TEST_CASE("abaTime is 0 for same config", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.abaTime(100, 100) == 0.0);
}

TEST_CASE("abaTime is fixed for any change", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.abaTime(100, 200) == Approx(0.8));
    REQUIRE(cost.abaTime(0, 1) == Approx(0.8));
}

TEST_CASE("repoTime is 0 when not needed", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.repoTime(false) == 0.0);
}

TEST_CASE("repoTime includes all phases", "[phase3][cost]") {
    MaskedTimeCost cost;

    // 2.0 + 1.5 + 2.0 + 1.0 = 6.5 seconds
    REQUIRE(cost.repoTime(true) == Approx(6.5));
}

TEST_CASE("stepCost uses max for parallel operations", "[phase3][cost]") {
    MaskedTimeCost cost;

    SearchState current;
    current.orientation = Orientation::DEG_0;
    current.abaConfig = 100;

    SearchState next;
    next.orientation = Orientation::DEG_90;  // Rotation needed (1.5s)
    next.abaConfig = 200;                     // ABA change needed (0.8s)
    next.needsRepo = false;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;  // 2.0s bend time

    double totalCost = cost.stepCost(current, 0, next, bend);

    // Expected: 2.0 (bend) + max(1.5, 0.8) (parallel) + 0 (repo) = 3.5
    REQUIRE(totalCost == Approx(3.5));
}

TEST_CASE("stepCost with repo adds full repo time", "[phase3][cost]") {
    MaskedTimeCost cost;

    SearchState current, next;
    next.needsRepo = true;

    BendFeature bend;
    bend.angle = 90.0;

    double totalCost = cost.stepCost(current, 0, next, bend);

    // Expected: 2.0 (bend) + 0 (no parallel) + 6.5 (repo) = 8.5
    REQUIRE(totalCost == Approx(8.5));
}

// ===== BendTimeEstimator Tests =====

TEST_CASE("BendTimeEstimator basic estimate", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend;
    bend.angle = 90.0;
    bend.length = 100.0;

    double time = estimator.estimate(bend);
    REQUIRE(time > 0.0);
    REQUIRE(time < 10.0);
}

TEST_CASE("BendTimeEstimator longer bend takes more time", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature short_bend, long_bend;
    short_bend.angle = 90.0;
    short_bend.length = 50.0;
    long_bend.angle = 90.0;
    long_bend.length = 500.0;

    REQUIRE(estimator.estimate(long_bend) > estimator.estimate(short_bend));
}

TEST_CASE("BendTimeEstimator larger angle takes more time", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend90, bend135;
    bend90.angle = 90.0;
    bend90.length = 100.0;
    bend135.angle = 135.0;
    bend135.length = 100.0;

    REQUIRE(estimator.estimate(bend135) > estimator.estimate(bend90));
}

TEST_CASE("BendTimeEstimator requiredOrientation", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bendX, bendY, bendNegX, bendNegY;
    bendX.direction = {1.0, 0.0, 0.0};
    bendY.direction = {0.0, 1.0, 0.0};
    bendNegX.direction = {-1.0, 0.0, 0.0};
    bendNegY.direction = {0.0, -1.0, 0.0};

    REQUIRE(estimator.requiredOrientation(bendX) == Orientation::DEG_0);
    REQUIRE(estimator.requiredOrientation(bendY) == Orientation::DEG_90);
    REQUIRE(estimator.requiredOrientation(bendNegX) == Orientation::DEG_180);
    REQUIRE(estimator.requiredOrientation(bendNegY) == Orientation::DEG_270);
}

TEST_CASE("BendTimeEstimator requiredAbaConfig", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend;
    bend.length = 100.0;

    uint16_t config = estimator.requiredAbaConfig(bend);
    REQUIRE(config == 110);  // 100 + 10 clearance
}

TEST_CASE("BendTimeEstimator with custom config", "[phase3][cost][estimator]") {
    CostConfig cfg;
    cfg.baseBendTime = 3.0;
    BendTimeEstimator estimator(cfg);

    BendFeature bend;
    bend.angle = 90.0;
    bend.length = 0.0;

    // With 0 length and default thickness 1.5: 3.0 * 1.0 + 1.5 * 0.5 = 3.75
    // lengthFactor = 1.0 + (0/1000) * 0.1 = 1.0
    // time = 3.0 * 1.0 * 1.0 + 0.75 = 3.75
    double time = estimator.estimate(bend);
    REQUIRE(time == Approx(3.75));
}
