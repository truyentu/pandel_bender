#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/grasp_physics.h"

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

// ===== GraspPhysicsEngine Constructor =====

TEST_CASE("GraspPhysicsEngine default construction", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    REQUIRE(engine.config().minGripArea == 100.0);
    REQUIRE(engine.config().maxComDistance == 50.0);
    REQUIRE(engine.config().maxTorque == 10.0);
    REQUIRE(engine.config().maxShearForce == 50.0);
}

TEST_CASE("GraspPhysicsEngine custom config", "[phase4][grasp]") {
    ValidatorConfig config;
    config.minGripArea = 200.0;
    config.maxComDistance = 30.0;
    GraspPhysicsEngine engine(config);
    REQUIRE(engine.config().minGripArea == 200.0);
    REQUIRE(engine.config().maxComDistance == 30.0);
}

// ===== Grip Area =====

TEST_CASE("GraspPhysicsEngine grip area positive for empty state", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double area = engine.computeGripArea(state, bend);
    REQUIRE(area > 0.0);
}

TEST_CASE("GraspPhysicsEngine grip area decreases with more bends", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state0; // Empty
    BentState state1;
    state1.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));

    auto nextBend = makeBend(1, 90.0, 100.0, 200, 0, 0, 0, 1, 0);

    double area0 = engine.computeGripArea(state0, nextBend);
    double area1 = engine.computeGripArea(state1, nextBend);

    // More bends = less grip area
    REQUIRE(area1 < area0);
}

TEST_CASE("GraspPhysicsEngine small angle preserves more area", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;

    auto bendSmall = makeBend(0, 30.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto bendLarge = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double areaSmall = engine.computeGripArea(state, bendSmall);
    double areaLarge = engine.computeGripArea(state, bendLarge);

    // Smaller angle loses less area
    REQUIRE(areaSmall > areaLarge);
}

// ===== Center of Mass =====

TEST_CASE("GraspPhysicsEngine COM distance for empty state", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double com = engine.computeComDistance(state, bend);
    REQUIRE(com >= 0.0);
}

TEST_CASE("GraspPhysicsEngine COM distance increases with offset bends", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState stateNear;
    stateNear.addBentFlange(makeBend(0, 90.0, 100.0, 10, 10, 0, 1, 0, 0));

    BentState stateFar;
    stateFar.addBentFlange(makeBend(0, 90.0, 100.0, 500, 500, 0, 1, 0, 0));

    auto nextBend = makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0);

    double comNear = engine.computeComDistance(stateNear, nextBend);
    double comFar = engine.computeComDistance(stateFar, nextBend);

    REQUIRE(comFar > comNear);
}

// ===== Torque =====

TEST_CASE("GraspPhysicsEngine torque is non-negative", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double torque = engine.computeTorque(state, bend);
    REQUIRE(torque >= 0.0);
}

TEST_CASE("GraspPhysicsEngine torque increases with more mass", "[phase4][grasp]") {
    ValidatorConfig thinConfig;
    thinConfig.materialThickness = 0.5;
    GraspPhysicsEngine thinEngine(thinConfig);

    ValidatorConfig thickConfig;
    thickConfig.materialThickness = 3.0;
    GraspPhysicsEngine thickEngine(thickConfig);

    BentState state;
    auto bend = makeBend(0, 90.0, 200.0, 100, 0, 0, 1, 0, 0);

    double torqueThin = thinEngine.computeTorque(state, bend);
    double torqueThick = thickEngine.computeTorque(state, bend);

    REQUIRE(torqueThick > torqueThin);
}

// ===== Shear Force =====

TEST_CASE("GraspPhysicsEngine shear force is positive", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double shear = engine.computeShearForce(bend);
    REQUIRE(shear > 0.0);
}

TEST_CASE("GraspPhysicsEngine shear increases with bend length", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    auto bendShort = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);
    auto bendLong = makeBend(1, 90.0, 500.0, 0, 0, 0, 1, 0, 0);

    double shearShort = engine.computeShearForce(bendShort);
    double shearLong = engine.computeShearForce(bendLong);

    REQUIRE(shearLong > shearShort);
}

TEST_CASE("GraspPhysicsEngine shear increases with thickness", "[phase4][grasp]") {
    ValidatorConfig thinConfig;
    thinConfig.materialThickness = 0.5;
    GraspPhysicsEngine thinEngine(thinConfig);

    ValidatorConfig thickConfig;
    thickConfig.materialThickness = 3.0;
    GraspPhysicsEngine thickEngine(thickConfig);

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    double shearThin = thinEngine.computeShearForce(bend);
    double shearThick = thickEngine.computeShearForce(bend);

    REQUIRE(shearThick > shearThin);
}

// ===== Full Validation =====

TEST_CASE("GraspPhysicsEngine validate passes for simple case", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    GraspValidation result = engine.validate(state, bend);

    REQUIRE(result.areaValid == true);
    REQUIRE(result.gripArea > 0.0);
    REQUIRE(result.shearForce > 0.0);
}

TEST_CASE("GraspPhysicsEngine validate fails with strict config", "[phase4][grasp]") {
    ValidatorConfig config;
    config.minGripArea = 1e9; // Impossibly large
    GraspPhysicsEngine engine(config);

    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    GraspValidation result = engine.validate(state, bend);

    REQUIRE(result.areaValid == false);
    REQUIRE(result.isValid() == false);
}

TEST_CASE("GraspPhysicsEngine validate reports describe text", "[phase4][grasp]") {
    ValidatorConfig config;
    config.minGripArea = 1e9;
    GraspPhysicsEngine engine(config);

    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    GraspValidation result = engine.validate(state, bend);
    std::string desc = result.describe();

    REQUIRE(desc.find("FAILED") != std::string::npos);
    REQUIRE(desc.find("area") != std::string::npos);
}

TEST_CASE("GraspPhysicsEngine validate all valid returns VALID describe", "[phase4][grasp]") {
    GraspPhysicsEngine engine;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    GraspValidation result = engine.validate(state, bend);

    if (result.isValid()) {
        REQUIRE(result.describe() == "Grasp: VALID");
    }
}
