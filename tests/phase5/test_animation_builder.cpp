#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/animation_builder.h"

using namespace openpanelcam::phase5;
using namespace openpanelcam::phase1;
using Catch::Approx;

static BendFeature makeBend(int id, double angle, double length,
                             double px, double py, double pz,
                             double dx, double dy, double dz) {
    BendFeature b;
    b.id = id; b.angle = angle; b.length = length;
    b.position = {px, py, pz};
    b.direction = {dx, dy, dz};
    b.normal = {0, 0, 1};
    return b;
}

// ===== Bone Hierarchy =====

TEST_CASE("AnimationBuilder empty bends produces root only", "[phase5][anim]") {
    AnimationBuilder builder;
    auto bones = builder.buildBoneHierarchy({});
    REQUIRE(bones.size() == 1);
    REQUIRE(bones[0].name == "base");
    REQUIRE(bones[0].parentIndex == -1);
}

TEST_CASE("AnimationBuilder 3 bends produces 4 bones", "[phase5][anim]") {
    AnimationBuilder builder;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 300, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 120.0, 0, 300, 0, 1, 0, 0)
    };

    auto bones = builder.buildBoneHierarchy(bends);

    REQUIRE(bones.size() == 4); // root + 3 flanges
    REQUIRE(bones[0].name == "base");
    REQUIRE(bones[1].name == "flange_0");
    REQUIRE(bones[2].name == "flange_1");
    REQUIRE(bones[3].name == "flange_2");

    // All flanges parented to base
    for (size_t i = 1; i < bones.size(); i++) {
        REQUIRE(bones[i].parentIndex == 0);
    }
}

TEST_CASE("AnimationBuilder bone positions match bend positions", "[phase5][anim]") {
    AnimationBuilder builder;
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 100, 200, 0, 1, 0, 0)
    };

    auto bones = builder.buildBoneHierarchy(bends);

    REQUIRE(bones[1].originX == Approx(100.0));
    REQUIRE(bones[1].originY == Approx(200.0));
    REQUIRE(bones[1].axisX == Approx(1.0));
    REQUIRE(bones[1].axisY == Approx(0.0));
}

// ===== Keyframes =====

TEST_CASE("AnimationBuilder empty program produces initial keyframe", "[phase5][anim]") {
    AnimationBuilder builder;
    MachineProgram prog;

    auto keyframes = builder.buildKeyframes(prog);

    REQUIRE(keyframes.size() == 1); // Just the initial
    REQUIRE(keyframes[0].time == 0.0);
    REQUIRE(keyframes[0].boneIndex == 0);
}

TEST_CASE("AnimationBuilder bend creates keyframe", "[phase5][anim]") {
    AnimationBuilder builder;
    MachineProgram prog;

    MachineInstruction bend;
    bend.stepId = 1;
    bend.type = InstructionType::BEND;
    bend.bendId = 0;
    bend.targetAngle = 90.0;
    bend.duration = 2.5;
    prog.instructions.push_back(bend);

    auto keyframes = builder.buildKeyframes(prog);

    REQUIRE(keyframes.size() == 2); // initial + bend
    REQUIRE(keyframes[1].time == Approx(2.5));
    REQUIRE(keyframes[1].boneIndex == 1); // bendId 0 + 1
    REQUIRE(keyframes[1].rotationAngle == 90.0);
}

TEST_CASE("AnimationBuilder cumulative time", "[phase5][anim]") {
    AnimationBuilder builder;
    MachineProgram prog;

    MachineInstruction b1;
    b1.type = InstructionType::BEND; b1.bendId = 0;
    b1.targetAngle = 90.0; b1.duration = 2.5;
    prog.instructions.push_back(b1);

    MachineInstruction rot;
    rot.type = InstructionType::ROTATE; rot.rotationAngle = 90;
    rot.duration = 1.5;
    prog.instructions.push_back(rot);

    MachineInstruction b2;
    b2.type = InstructionType::BEND; b2.bendId = 1;
    b2.targetAngle = 90.0; b2.duration = 2.5;
    prog.instructions.push_back(b2);

    auto keyframes = builder.buildKeyframes(prog);

    REQUIRE(keyframes.size() == 4); // initial + bend + rotate + bend
    REQUIRE(keyframes[1].time == Approx(2.5));
    REQUIRE(keyframes[2].time == Approx(4.0)); // 2.5 + 1.5
    REQUIRE(keyframes[3].time == Approx(6.5)); // 4.0 + 2.5
}

TEST_CASE("AnimationBuilder ABA setup skipped in keyframes", "[phase5][anim]") {
    AnimationBuilder builder;
    MachineProgram prog;

    MachineInstruction aba;
    aba.type = InstructionType::ABA_SETUP;
    aba.duration = 0.8;
    prog.instructions.push_back(aba);

    auto keyframes = builder.buildKeyframes(prog);

    REQUIRE(keyframes.size() == 1); // Only initial (ABA skipped)
}

// ===== Full build =====

TEST_CASE("AnimationBuilder full build", "[phase5][anim]") {
    AnimationBuilder builder;
    MachineProgram prog;
    prog.partName = "TEST";
    prog.totalCycleTime = 5.0;
    prog.bendCount = 2;

    MachineInstruction b1;
    b1.type = InstructionType::BEND; b1.bendId = 0;
    b1.targetAngle = 90.0; b1.duration = 2.5;
    prog.instructions.push_back(b1);

    MachineInstruction b2;
    b2.type = InstructionType::BEND; b2.bendId = 1;
    b2.targetAngle = 90.0; b2.duration = 2.5;
    prog.instructions.push_back(b2);

    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 300, 0, 0, 0, 1, 0)
    };

    auto viz = builder.build(prog, bends);

    REQUIRE(viz.partName == "TEST");
    REQUIRE(viz.cycleTime == 5.0);
    REQUIRE(viz.bendCount == 2);
    REQUIRE(viz.bones.size() == 3); // root + 2 flanges
    REQUIRE(viz.keyframes.size() == 3); // initial + 2 bends
}
