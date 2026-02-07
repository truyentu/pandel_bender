#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/output_validator.h"

using namespace openpanelcam::phase5;
using Catch::Approx;

static MachineProgram makeValidProgram() {
    MachineProgram prog;
    prog.partName = "BOX001";
    prog.totalCycleTime = 6.5;
    prog.bendCount = 2;

    MachineInstruction b1;
    b1.stepId = 1; b1.type = InstructionType::BEND;
    b1.bendId = 0; b1.targetAngle = 90.0;
    b1.compensatedAngle = 92.75; b1.springbackAngle = 2.75;
    b1.bendForce = 0.94; b1.bendLength = 200.0;
    b1.startX = 0; b1.startY = 50; b1.endX = 200; b1.endY = 50;
    b1.duration = 2.5;
    prog.instructions.push_back(b1);

    MachineInstruction rot;
    rot.stepId = 2; rot.type = InstructionType::ROTATE;
    rot.rotationAngle = 90; rot.duration = 1.5;
    prog.instructions.push_back(rot);

    MachineInstruction b2;
    b2.stepId = 3; b2.type = InstructionType::BEND;
    b2.bendId = 1; b2.targetAngle = 90.0;
    b2.compensatedAngle = 92.75; b2.springbackAngle = 2.75;
    b2.bendForce = 1.41; b2.bendLength = 300.0;
    b2.startX = 0; b2.startY = 100; b2.endX = 300; b2.endY = 100;
    b2.duration = 2.5;
    prog.instructions.push_back(b2);

    return prog;
}

// ===== Valid program =====

TEST_CASE("OutputValidator valid program passes", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();

    auto result = validator.validate(prog);

    REQUIRE(result.valid == true);
    REQUIRE(result.errors.empty());
}

// ===== Angle limits =====

TEST_CASE("OutputValidator catches angle exceeding max", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[0].targetAngle = 150.0;  // > 135

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
    REQUIRE(result.errors.size() >= 1);
    REQUIRE(result.errors[0].find("angle") != std::string::npos);
}

TEST_CASE("OutputValidator catches angle below min", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[0].targetAngle = -140.0;  // < -135

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
}

TEST_CASE("OutputValidator boundary angle valid", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[0].targetAngle = 135.0;  // exactly at limit

    auto result = validator.validate(prog);

    REQUIRE(validator.checkAngleLimits(prog.instructions[0]) == true);
}

TEST_CASE("OutputValidator negative angle within limits", "[phase5][validator]") {
    OutputValidator validator;
    MachineInstruction bend;
    bend.type = InstructionType::BEND;
    bend.targetAngle = -90.0;

    REQUIRE(validator.checkAngleLimits(bend) == true);
}

// ===== Force limits =====

TEST_CASE("OutputValidator catches excessive force", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[0].bendForce = 30.0;  // > 25 kN default

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
    REQUIRE(result.errors.size() >= 1);
    REQUIRE(result.errors[0].find("force") != std::string::npos);
}

TEST_CASE("OutputValidator force within limit", "[phase5][validator]") {
    OutputValidator validator;
    MachineInstruction bend;
    bend.type = InstructionType::BEND;
    bend.bendForce = 20.0;

    REQUIRE(validator.checkForceLimits(bend) == true);
}

TEST_CASE("OutputValidator custom force limit", "[phase5][validator]") {
    PostProcessorConfig config;
    config.maxForce = 50.0;
    OutputValidator validator(config);

    MachineInstruction bend;
    bend.type = InstructionType::BEND;
    bend.bendForce = 30.0;  // > default 25 but < custom 50

    REQUIRE(validator.checkForceLimits(bend) == true);
}

// ===== Part size =====

TEST_CASE("OutputValidator catches oversized part", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[0].endX = 3000.0;  // > 2500mm

    auto result = validator.validate(prog);

    REQUIRE(result.warnings.size() >= 1);
}

TEST_CASE("OutputValidator part size within limits", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();

    REQUIRE(validator.checkPartSize(prog) == true);
}

// ===== Cycle time consistency =====

TEST_CASE("OutputValidator catches cycle time mismatch", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.totalCycleTime = 100.0;  // Way off from sum of durations (6.5)

    auto result = validator.validate(prog);

    REQUIRE(result.warnings.size() >= 1);
    REQUIRE(result.warnings[0].find("cycle time") != std::string::npos);
}

TEST_CASE("OutputValidator cycle time consistent", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();

    REQUIRE(validator.checkCycleTimeConsistency(prog) == true);
}

// ===== Step ID uniqueness =====

TEST_CASE("OutputValidator catches duplicate step IDs", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();
    prog.instructions[2].stepId = 1;  // Duplicate with first

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
    REQUIRE(result.errors.size() >= 1);
    REQUIRE(result.errors[0].find("Duplicate") != std::string::npos);
}

TEST_CASE("OutputValidator unique step IDs pass", "[phase5][validator]") {
    OutputValidator validator;
    auto prog = makeValidProgram();

    REQUIRE(validator.checkStepIdUniqueness(prog) == true);
}

// ===== Empty program =====

TEST_CASE("OutputValidator empty program valid", "[phase5][validator]") {
    OutputValidator validator;
    MachineProgram prog;

    auto result = validator.validate(prog);

    REQUIRE(result.valid == true);
    REQUIRE(result.errors.empty());
}

// ===== Non-bend instructions skip checks =====

TEST_CASE("OutputValidator skips angle check for non-bend", "[phase5][validator]") {
    OutputValidator validator;
    MachineInstruction rot;
    rot.type = InstructionType::ROTATE;
    rot.rotationAngle = 180;

    REQUIRE(validator.checkAngleLimits(rot) == true);
    REQUIRE(validator.checkForceLimits(rot) == true);
}
