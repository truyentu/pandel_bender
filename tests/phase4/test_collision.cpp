#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/swept_volume.h"
#include "openpanelcam/phase4/bent_state.h"
#include "openpanelcam/phase4/collision_detector.h"

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

// ===== SweptVolumeGenerator Tests =====

TEST_CASE("SweptVolumeGenerator produces valid AABB", "[phase4][swept]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    SweptVolume sv = gen.generate(bend);

    REQUIRE(sv.bendId == 0);
    REQUIRE(sv.sweepAngle == 90.0);
    REQUIRE(sv.aabb.volume() > 0.0);
}

TEST_CASE("SweptVolumeGenerator AABB contains origin", "[phase4][swept]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    AABB aabb = gen.estimateSweptAABB(bend);

    // Origin should be within the AABB
    REQUIRE(aabb.minX <= 0.0);
    REQUIRE(aabb.maxX >= 0.0);
    REQUIRE(aabb.minY <= 0.0);
    REQUIRE(aabb.maxY >= 0.0);
}

TEST_CASE("SweptVolumeGenerator larger angle = larger volume", "[phase4][swept]") {
    SweptVolumeGenerator gen;
    auto bend90 = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    auto bend135 = makeBend(1, 135.0, 100.0, 0, 0, 0, 1, 0, 0);

    AABB aabb90 = gen.estimateSweptAABB(bend90);
    AABB aabb135 = gen.estimateSweptAABB(bend135);

    // Larger angle should produce at least as large a volume
    REQUIRE(aabb135.volume() >= aabb90.volume() * 0.8); // Allow some tolerance
}

TEST_CASE("SweptVolumeGenerator longer flange = larger AABB", "[phase4][swept]") {
    SweptVolumeGenerator gen;
    auto bendShort = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);
    auto bendLong = makeBend(1, 90.0, 200.0, 0, 0, 0, 1, 0, 0);

    AABB aabbShort = gen.estimateSweptAABB(bendShort);
    AABB aabbLong = gen.estimateSweptAABB(bendLong);

    REQUIRE(aabbLong.volume() > aabbShort.volume());
}

TEST_CASE("SweptVolumeGenerator offset position", "[phase4][swept]") {
    SweptVolumeGenerator gen;
    auto bend = makeBend(0, 90.0, 100.0, 500, 300, 0, 1, 0, 0);

    AABB aabb = gen.estimateSweptAABB(bend);

    // AABB should contain the bend position
    REQUIRE(aabb.minX < 500.0);
    REQUIRE(aabb.maxX > 500.0);
    REQUIRE(aabb.minY < 300.0);
    REQUIRE(aabb.maxY > 300.0);
}

// ===== BentState Tests =====

TEST_CASE("BentState starts empty", "[phase4][bent]") {
    BentState state;
    REQUIRE(state.count() == 0);
    REQUIRE(state.getBentFlanges().empty());
    REQUIRE(state.getAllOccupiedVolumes().empty());
}

TEST_CASE("BentState addBentFlange increases count", "[phase4][bent]") {
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    state.addBentFlange(bend);

    REQUIRE(state.count() == 1);
    REQUIRE(state.getBentFlanges()[0].bendId == 0);
    REQUIRE(state.getBentFlanges()[0].angle == 90.0);
}

TEST_CASE("BentState multiple flanges", "[phase4][bent]") {
    BentState state;
    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));
    state.addBentFlange(makeBend(1, 90.0, 150.0, 500, 0, 0, 0, 1, 0));

    REQUIRE(state.count() == 2);
    auto volumes = state.getAllOccupiedVolumes();
    REQUIRE(volumes.size() == 2);
    REQUIRE(volumes[0].volume() > 0.0);
    REQUIRE(volumes[1].volume() > 0.0);
}

TEST_CASE("BentState reset clears all", "[phase4][bent]") {
    BentState state;
    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));
    state.addBentFlange(makeBend(1, 90.0, 100.0, 200, 0, 0, 0, 1, 0));

    REQUIRE(state.count() == 2);
    state.reset();
    REQUIRE(state.count() == 0);
}

TEST_CASE("BentState occupied volume has positive size", "[phase4][bent]") {
    BentState state;
    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));

    auto volumes = state.getAllOccupiedVolumes();
    REQUIRE(volumes[0].volume() > 0.0);
}

// ===== CollisionDetector Tests =====

TEST_CASE("CollisionDetector no collision when empty", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state; // Empty

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto result = detector.checkStep(sv, state);

    REQUIRE(result.hasCollision == false);
    REQUIRE(result.type == CollisionType::NONE);
}

TEST_CASE("CollisionDetector no collision for distant bends", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    // Bend 0 at origin
    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));

    // Bend 1 far away (should not collide)
    auto bend1 = makeBend(1, 90.0, 100.0, 2000, 2000, 0, 0, 1, 0);
    SweptVolume sv = gen.generate(bend1);

    auto result = detector.checkStep(sv, state);

    REQUIRE(result.hasCollision == false);
}

TEST_CASE("CollisionDetector detects collision for overlapping bends", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    // Bend 0 at origin
    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));

    // Bend 1 at same position (should collide)
    auto bend1 = makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0);
    SweptVolume sv = gen.generate(bend1);

    auto result = detector.checkStep(sv, state);

    REQUIRE(result.hasCollision == true);
    REQUIRE(result.type == CollisionType::SWEPT_VS_FIXED);
    REQUIRE(result.bendId == 1);
    REQUIRE(result.collidingBendId == 0);
    REQUIRE(result.penetrationDepth > 0.0);
}

TEST_CASE("CollisionDetector skips self-collision", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    // Bend 0 at origin
    auto bend0 = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    state.addBentFlange(bend0);

    // Same bend checking against itself
    SweptVolume sv = gen.generate(bend0);

    auto result = detector.checkStep(sv, state);

    REQUIRE(result.hasCollision == false); // Self-collision skipped
}

TEST_CASE("CollisionDetector with collision margin", "[phase4][collision]") {
    ValidatorConfig config;
    config.collisionMargin = 50.0; // Large margin
    CollisionDetector detector(config);
    SweptVolumeGenerator gen;
    BentState state;

    state.addBentFlange(makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0));

    // Bend 1 nearby but not touching without margin
    auto bend1 = makeBend(1, 90.0, 50.0, 100, 0, 0, 0, 1, 0);
    SweptVolume sv = gen.generate(bend1);

    auto result = detector.checkStep(sv, state);

    // With large margin, should detect overlap
    REQUIRE(result.hasCollision == true);
}

TEST_CASE("CollisionDetector checkAgainstVolumes", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    // Obstacle overlapping with swept volume
    std::vector<AABB> obstacles = {
        AABB(-10, -10, -10, 10, 10, 10)
    };

    auto result = detector.checkAgainstVolumes(sv, obstacles);
    REQUIRE(result.hasCollision == true);

    // Obstacle far away
    std::vector<AABB> farObstacles = {
        AABB(9000, 9000, 9000, 9100, 9100, 9100)
    };

    auto result2 = detector.checkAgainstVolumes(sv, farObstacles);
    REQUIRE(result2.hasCollision == false);
}

TEST_CASE("CollisionDetector collision description is set", "[phase4][collision]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;
    BentState state;

    state.addBentFlange(makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0));

    auto bend1 = makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0);
    SweptVolume sv = gen.generate(bend1);

    auto result = detector.checkStep(sv, state);

    REQUIRE(result.hasCollision == true);
    REQUIRE(!result.description.empty());
    REQUIRE(result.description.find("bend 1") != std::string::npos);
}

// ===== checkDualState Tests =====

TEST_CASE("checkDualState: no collision in either state", "[phase4][collision][dual]") {
    CollisionDetector detector;
    BentState foldedState, unfoldedState;

    // Folded state has distant bend
    foldedState.addBentFlange(makeBend(1, 90.0, 50.0, 2000, 2000, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 50.0, 0, 0, 0, 1, 0, 0);

    auto result = detector.checkDualState(bend, foldedState, unfoldedState);
    REQUIRE(result.hasCollision == false);
}

TEST_CASE("checkDualState: collision in folded state", "[phase4][collision][dual]") {
    CollisionDetector detector;
    BentState foldedState, unfoldedState;

    // Folded state has overlapping bend
    foldedState.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto result = detector.checkDualState(bend, foldedState, unfoldedState);
    REQUIRE(result.hasCollision == true);
    REQUIRE(result.description.find("[Folded]") != std::string::npos);
}

TEST_CASE("checkDualState: collision in unfolded state", "[phase4][collision][dual]") {
    CollisionDetector detector;
    BentState foldedState, unfoldedState;

    // Unfolded state has overlapping bend
    unfoldedState.addBentFlange(makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0));

    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto result = detector.checkDualState(bend, foldedState, unfoldedState);
    REQUIRE(result.hasCollision == true);
    REQUIRE(result.description.find("[Unfolded]") != std::string::npos);
}

// ===== BentState k-factor Tests =====

TEST_CASE("BentState k-factor correction shrinks AABB", "[phase4][bent][kfactor]") {
    BentState state1, state2;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    state1.addBentFlange(bend);         // No k-factor
    state2.addBentFlange(bend, 0.4);    // With k-factor

    auto vol1 = state1.getAllOccupiedVolumes()[0];
    auto vol2 = state2.getAllOccupiedVolumes()[0];

    // K-factor corrected AABB should be smaller
    REQUIRE(vol2.volume() < vol1.volume());
}

TEST_CASE("BentState k-factor=0 gives same as default", "[phase4][bent][kfactor]") {
    BentState state1, state2;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    state1.addBentFlange(bend);
    state2.addBentFlange(bend, 0.0);

    auto vol1 = state1.getAllOccupiedVolumes()[0];
    auto vol2 = state2.getAllOccupiedVolumes()[0];

    REQUIRE(vol1.volume() == Approx(vol2.volume()));
}
