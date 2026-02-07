#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/instruction_builder.h"

using namespace openpanelcam::phase5;
using namespace openpanelcam::phase4;
using namespace openpanelcam::phase3;
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

static Phase3Output makeSequence(const std::vector<int>& bendIds) {
    Phase3Output seq;
    seq.bendSequence = bendIds;
    seq.success = true;
    for (int id : bendIds) {
        SequenceAction action;
        action.type = ActionType::BEND;
        action.bendId = id;
        seq.actions.push_back(action);
    }
    return seq;
}

static Phase4Output makeValidated(const std::vector<int>& bendIds) {
    Phase4Output p4;
    p4.success = true;
    for (int id : bendIds) {
        SpringbackData sb;
        sb.bendId = id;
        sb.targetAngle = 90.0;
        sb.compensatedAngle = 92.75;
        sb.springbackAngle = 2.75;
        p4.springbackTable.push_back(sb);
    }
    return p4;
}

// ===== Constructor =====

TEST_CASE("InstructionBuilder default construction", "[phase5][builder]") {
    InstructionBuilder builder;
    REQUIRE(builder.config().defaultMaterial == "AISI304");
    REQUIRE(builder.config().maxForce == 25.0);
}

// ===== Bend Force Calculation =====

TEST_CASE("InstructionBuilder computeBendForce", "[phase5][builder]") {
    InstructionBuilder builder;
    // F = (200 * 100 * 1.5^2) / (4 * 12) / 1000
    //   = (200 * 100 * 2.25) / 48 / 1000
    //   = 45000 / 48 / 1000
    //   = 0.9375 kN
    double force = builder.computeBendForce(100.0, 1.5, 200.0);
    REQUIRE(force == Approx(0.9375));
}

TEST_CASE("InstructionBuilder bend force increases with length", "[phase5][builder]") {
    InstructionBuilder builder;
    double forceShort = builder.computeBendForce(100.0, 1.5, 200.0);
    double forceLong = builder.computeBendForce(500.0, 1.5, 200.0);
    REQUIRE(forceLong > forceShort);
}

TEST_CASE("InstructionBuilder bend force increases with thickness", "[phase5][builder]") {
    InstructionBuilder builder;
    double forceThin = builder.computeBendForce(100.0, 0.5, 200.0);
    double forceThick = builder.computeBendForce(100.0, 3.0, 200.0);
    REQUIRE(forceThick > forceThin);
}

// ===== Build Bend Instruction =====

TEST_CASE("InstructionBuilder buildBendInstruction", "[phase5][builder]") {
    InstructionBuilder builder;
    auto bend = makeBend(0, 90.0, 200.0, 100, 50, 0, 1, 0, 0);
    SpringbackData sb;
    sb.bendId = 0;
    sb.targetAngle = 90.0;
    sb.compensatedAngle = 92.75;
    sb.springbackAngle = 2.75;

    auto instr = builder.buildBendInstruction(1, bend, sb);

    REQUIRE(instr.stepId == 1);
    REQUIRE(instr.type == InstructionType::BEND);
    REQUIRE(instr.bendId == 0);
    REQUIRE(instr.targetAngle == 90.0);
    REQUIRE(instr.compensatedAngle == Approx(92.75));
    REQUIRE(instr.springbackAngle == Approx(2.75));
    REQUIRE(instr.bendLength == 200.0);
    REQUIRE(instr.bendForce > 0.0);
    REQUIRE(instr.duration > 0.0);
    REQUIRE(!instr.description.empty());
}

TEST_CASE("InstructionBuilder bend line endpoints", "[phase5][builder]") {
    InstructionBuilder builder;
    auto bend = makeBend(0, 90.0, 200.0, 100, 50, 0, 1, 0, 0);
    SpringbackData sb;
    sb.bendId = 0; sb.targetAngle = 90.0;
    sb.compensatedAngle = 92.0; sb.springbackAngle = 2.0;

    auto instr = builder.buildBendInstruction(1, bend, sb);

    // Direction (1,0,0), length 200, position (100,50)
    // start = (100 - 1*100, 50 - 0*100) = (0, 50)
    // end = (100 + 1*100, 50 + 0*100) = (200, 50)
    REQUIRE(instr.startX == Approx(0.0));
    REQUIRE(instr.startY == Approx(50.0));
    REQUIRE(instr.endX == Approx(200.0));
    REQUIRE(instr.endY == Approx(50.0));
}

// ===== Build Rotation Instruction =====

TEST_CASE("InstructionBuilder buildRotationInstruction", "[phase5][builder]") {
    InstructionBuilder builder;
    SequenceAction action;
    action.type = ActionType::ROTATE;
    action.newOrientation = Orientation::DEG_90;

    auto instr = builder.buildRotationInstruction(2, action);

    REQUIRE(instr.stepId == 2);
    REQUIRE(instr.type == InstructionType::ROTATE);
    REQUIRE(instr.rotationAngle == 90);
    REQUIRE(instr.duration > 0.0);
}

// ===== Build ABA Instruction =====

TEST_CASE("InstructionBuilder buildABAInstruction", "[phase5][builder]") {
    InstructionBuilder builder;
    SequenceAction action;
    action.type = ActionType::ABA_RECONFIG;
    action.newAbaConfig = 0b1010;

    auto instr = builder.buildABAInstruction(3, action);

    REQUIRE(instr.stepId == 3);
    REQUIRE(instr.type == InstructionType::ABA_SETUP);
    REQUIRE(instr.abaConfig == 0b1010);
    REQUIRE(instr.duration > 0.0);
}

// ===== Build Reposition Instruction =====

TEST_CASE("InstructionBuilder buildRepositionInstruction", "[phase5][builder]") {
    InstructionBuilder builder;
    SequenceAction action;
    action.type = ActionType::REPOSITION;
    action.duration = 4.0;
    action.description = "Reposition for grip";

    auto instr = builder.buildRepositionInstruction(4, action);

    REQUIRE(instr.stepId == 4);
    REQUIRE(instr.type == InstructionType::REPOSITION);
    REQUIRE(instr.duration == 4.0);
    REQUIRE(instr.description == "Reposition for grip");
}

TEST_CASE("InstructionBuilder reposition default duration", "[phase5][builder]") {
    InstructionBuilder builder;
    SequenceAction action;
    action.type = ActionType::REPOSITION;
    action.duration = 0.0; // No duration specified

    auto instr = builder.buildRepositionInstruction(5, action);

    REQUIRE(instr.duration == 3.0); // Default
}

// ===== Full Build =====

TEST_CASE("InstructionBuilder build from 3-bend sequence", "[phase5][builder]") {
    InstructionBuilder builder;
    auto seq = makeSequence({0, 1, 2});
    auto p4 = makeValidated({0, 1, 2});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 300, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 120.0, 0, 300, 0, 1, 0, 0)
    };

    auto program = builder.build(p4, seq, bends);

    REQUIRE(program.instructions.size() == 3);
    REQUIRE(program.bendCount == 3);
    REQUIRE(program.repositionCount == 0);
    REQUIRE(program.totalCycleTime > 0.0);
    REQUIRE(program.createdBy == "OpenPanelCAM");
    REQUIRE(!program.timestamp.empty());

    // Each instruction should be a BEND
    for (const auto& instr : program.instructions) {
        REQUIRE(instr.type == InstructionType::BEND);
        REQUIRE(instr.compensatedAngle == Approx(92.75));
        REQUIRE(instr.bendForce > 0.0);
    }
}

TEST_CASE("InstructionBuilder build empty sequence", "[phase5][builder]") {
    InstructionBuilder builder;
    Phase3Output seq;
    seq.success = true;
    Phase4Output p4;
    p4.success = true;

    auto program = builder.build(p4, seq, {});

    REQUIRE(program.instructions.empty());
    REQUIRE(program.bendCount == 0);
    REQUIRE(program.totalCycleTime == 0.0);
}

TEST_CASE("InstructionBuilder build with mixed actions", "[phase5][builder]") {
    InstructionBuilder builder;
    Phase3Output seq;
    seq.success = true;
    seq.bendSequence = {0, 1};

    // BEND 0
    SequenceAction a1;
    a1.type = ActionType::BEND;
    a1.bendId = 0;
    seq.actions.push_back(a1);

    // ROTATE
    SequenceAction a2;
    a2.type = ActionType::ROTATE;
    a2.newOrientation = Orientation::DEG_90;
    seq.actions.push_back(a2);

    // BEND 1
    SequenceAction a3;
    a3.type = ActionType::BEND;
    a3.bendId = 1;
    seq.actions.push_back(a3);

    auto p4 = makeValidated({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 300, 0, 0, 0, 1, 0)
    };

    auto program = builder.build(p4, seq, bends);

    REQUIRE(program.instructions.size() == 3);
    REQUIRE(program.instructions[0].type == InstructionType::BEND);
    REQUIRE(program.instructions[1].type == InstructionType::ROTATE);
    REQUIRE(program.instructions[2].type == InstructionType::BEND);
    REQUIRE(program.bendCount == 2);
    REQUIRE(program.rotationCount == 1);
}

TEST_CASE("InstructionBuilder step IDs are sequential", "[phase5][builder]") {
    InstructionBuilder builder;
    auto seq = makeSequence({0, 1, 2});
    auto p4 = makeValidated({0, 1, 2});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 300, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 120.0, 0, 300, 0, 1, 0, 0)
    };

    auto program = builder.build(p4, seq, bends);

    for (size_t i = 0; i < program.instructions.size(); i++) {
        REQUIRE(program.instructions[i].stepId == static_cast<int>(i + 1));
    }
}
