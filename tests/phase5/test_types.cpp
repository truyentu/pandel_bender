#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/types.h"

using namespace openpanelcam::phase5;
using Catch::Approx;

// ===== InstructionType =====

TEST_CASE("InstructionType enum values", "[phase5][types]") {
    REQUIRE(static_cast<int>(InstructionType::BEND) == 0);
    REQUIRE(static_cast<int>(InstructionType::ROTATE) == 1);
    REQUIRE(static_cast<int>(InstructionType::ABA_SETUP) == 2);
    REQUIRE(static_cast<int>(InstructionType::REPOSITION) == 3);
    REQUIRE(static_cast<int>(InstructionType::GRIP) == 4);
    REQUIRE(static_cast<int>(InstructionType::RELEASE) == 5);
}

// ===== MachineInstruction =====

TEST_CASE("MachineInstruction default construction", "[phase5][types]") {
    MachineInstruction instr;
    REQUIRE(instr.stepId == -1);
    REQUIRE(instr.type == InstructionType::BEND);
    REQUIRE(instr.bendId == -1);
    REQUIRE(instr.targetAngle == 0.0);
    REQUIRE(instr.compensatedAngle == 0.0);
    REQUIRE(instr.bendForce == 0.0);
    REQUIRE(instr.duration == 0.0);
}

TEST_CASE("MachineInstruction bend setup", "[phase5][types]") {
    MachineInstruction instr;
    instr.stepId = 1;
    instr.type = InstructionType::BEND;
    instr.bendId = 0;
    instr.targetAngle = 90.0;
    instr.compensatedAngle = 92.75;
    instr.springbackAngle = 2.75;
    instr.bendForce = 15.5;
    instr.bendLength = 200.0;
    instr.startX = 0; instr.startY = 50;
    instr.endX = 200; instr.endY = 50;
    instr.duration = 2.5;

    REQUIRE(instr.stepId == 1);
    REQUIRE(instr.compensatedAngle == Approx(92.75));
    REQUIRE(instr.bendForce == Approx(15.5));
}

TEST_CASE("MachineInstruction rotation setup", "[phase5][types]") {
    MachineInstruction instr;
    instr.type = InstructionType::ROTATE;
    instr.rotationAngle = 90;
    instr.duration = 1.5;

    REQUIRE(instr.rotationAngle == 90);
}

TEST_CASE("MachineInstruction ABA setup", "[phase5][types]") {
    MachineInstruction instr;
    instr.type = InstructionType::ABA_SETUP;
    instr.abaConfig = 0b1010;
    instr.segmentPositions = {0.0, 100.0, 250.0};
    instr.segmentWidths = {100.0, 150.0, 200.0};

    REQUIRE(instr.segmentPositions.size() == 3);
    REQUIRE(instr.segmentWidths.size() == 3);
}

// ===== MachineProgram =====

TEST_CASE("MachineProgram default construction", "[phase5][types]") {
    MachineProgram prog;
    REQUIRE(prog.partName == "UNNAMED");
    REQUIRE(prog.createdBy == "OpenPanelCAM");
    REQUIRE(prog.machineModel == "P4");
    REQUIRE(prog.materialThickness == 1.5);
    REQUIRE(prog.materialType == "AISI304");
    REQUIRE(prog.upperTool == "Standard");
    REQUIRE(prog.lowerTool == "ABA");
    REQUIRE(prog.adaptiveEnabled == true);
    REQUIRE(prog.angleTolerance == 0.5);
    REQUIRE(prog.forceLimit == 25.0);
    REQUIRE(prog.instructions.empty());
}

TEST_CASE("MachineProgram add instructions", "[phase5][types]") {
    MachineProgram prog;
    prog.partName = "BOX001";
    prog.totalCycleTime = 45.2;
    prog.bendCount = 4;

    MachineInstruction bend;
    bend.stepId = 1;
    bend.type = InstructionType::BEND;
    prog.instructions.push_back(bend);

    MachineInstruction rotate;
    rotate.stepId = 2;
    rotate.type = InstructionType::ROTATE;
    prog.instructions.push_back(rotate);

    REQUIRE(prog.instructions.size() == 2);
    REQUIRE(prog.instructions[0].type == InstructionType::BEND);
    REQUIRE(prog.instructions[1].type == InstructionType::ROTATE);
}

// ===== AnimationBone =====

TEST_CASE("AnimationBone default construction", "[phase5][types]") {
    AnimationBone bone;
    REQUIRE(bone.index == -1);
    REQUIRE(bone.parentIndex == -1);
    REQUIRE(bone.name.empty());
}

TEST_CASE("AnimationBone hierarchy", "[phase5][types]") {
    AnimationBone root;
    root.index = 0;
    root.name = "base";
    root.parentIndex = -1;

    AnimationBone child;
    child.index = 1;
    child.name = "flange_top";
    child.parentIndex = 0;
    child.originX = 0; child.originY = 100;
    child.axisX = 1; child.axisY = 0; child.axisZ = 0;

    REQUIRE(root.parentIndex == -1);
    REQUIRE(child.parentIndex == 0);
}

// ===== AnimationKeyframe =====

TEST_CASE("AnimationKeyframe default construction", "[phase5][types]") {
    AnimationKeyframe kf;
    REQUIRE(kf.time == 0.0);
    REQUIRE(kf.boneIndex == -1);
    REQUIRE(kf.rotationAngle == 0.0);
}

// ===== VisualizationData =====

TEST_CASE("VisualizationData default construction", "[phase5][types]") {
    VisualizationData viz;
    REQUIRE(viz.cameraY == Approx(-500.0));
    REQUIRE(viz.cameraZ == Approx(300.0));
    REQUIRE(viz.fov == 45.0);
    REQUIRE(viz.bones.empty());
    REQUIRE(viz.keyframes.empty());
}

// ===== Phase5Output =====

TEST_CASE("Phase5Output default construction", "[phase5][types]") {
    Phase5Output output;
    REQUIRE(output.success == false);
    REQUIRE(output.xmlOutput.empty());
    REQUIRE(output.jsonOutput.empty());
    REQUIRE(output.xmlValid == false);
    REQUIRE(output.jsonValid == false);
    REQUIRE(output.errors.empty());
}

// ===== PostProcessorConfig =====

TEST_CASE("PostProcessorConfig default values", "[phase5][types]") {
    PostProcessorConfig config;
    REQUIRE(config.maxBendAngle == 135.0);
    REQUIRE(config.minBendAngle == -135.0);
    REQUIRE(config.maxForce == 25.0);
    REQUIRE(config.maxPartLength == 2500.0);
    REQUIRE(config.generateXML == true);
    REQUIRE(config.generateJSON == true);
    REQUIRE(config.includeAdaptiveControl == true);
    REQUIRE(config.defaultMaterial == "AISI304");
    REQUIRE(config.animationFPS == 30.0);
}

TEST_CASE("PostProcessorConfig custom values", "[phase5][types]") {
    PostProcessorConfig config;
    config.maxBendAngle = 120.0;
    config.generateJSON = false;
    config.defaultMaterial = "Aluminum";

    REQUIRE(config.maxBendAngle == 120.0);
    REQUIRE(config.generateJSON == false);
    REQUIRE(config.defaultMaterial == "Aluminum");
}
