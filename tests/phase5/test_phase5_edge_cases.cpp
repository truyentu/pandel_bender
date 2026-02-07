#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/post_processor.h"
#include "openpanelcam/phase5/output_validator.h"

using namespace openpanelcam::phase5;
using namespace openpanelcam::phase4;
using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;
using Catch::Approx;

// ===== Helpers =====

static Phase4Output makeP4(int numBends) {
    Phase4Output p4;
    p4.success = true;
    for (int i = 0; i < numBends; i++) {
        SpringbackData sb;
        sb.bendId = i;
        sb.targetAngle = 90.0;
        sb.compensatedAngle = 92.75;
        sb.springbackAngle = 2.75;
        p4.springbackTable.push_back(sb);
    }
    return p4;
}

static Phase3Output makeP3(int numBends) {
    Phase3Output p3;
    p3.success = true;
    for (int i = 0; i < numBends; i++) {
        p3.bendSequence.push_back(i);
        SequenceAction a;
        a.type = ActionType::BEND;
        a.bendId = i;
        p3.actions.push_back(a);
    }
    return p3;
}

static std::vector<BendFeature> makeBends(int numBends) {
    std::vector<BendFeature> bends;
    for (int i = 0; i < numBends; i++) {
        BendFeature b;
        b.id = i;
        b.angle = 90.0;
        b.length = 100.0 + i * 50.0;
        b.position = {static_cast<double>(i * 200), 0, 0};
        b.direction = {1, 0, 0};
        b.normal = {0, 0, 1};
        bends.push_back(b);
    }
    return bends;
}

// ===== Edge Cases: Empty =====

TEST_CASE("Edge: empty sequence produces valid output", "[phase5][edge]") {
    PostProcessor pp;
    Phase4Output p4; p4.success = true;
    Phase3Output p3; p3.success = true;
    std::vector<BendFeature> bends;

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.program.instructions.empty());
    REQUIRE(!result.xmlOutput.empty());
    REQUIRE(!result.jsonOutput.empty());
    REQUIRE(result.errors.empty());
}

// ===== Edge Cases: Single bend =====

TEST_CASE("Edge: single bend produces correct output", "[phase5][edge]") {
    PostProcessor pp;
    auto p4 = makeP4(1);
    auto p3 = makeP3(1);
    auto bends = makeBends(1);

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.program.instructions.size() == 1);
    REQUIRE(result.program.bendCount == 1);
    REQUIRE(result.xmlOutput.find("Bend") != std::string::npos);
    REQUIRE(result.jsonOutput.find("skeleton") != std::string::npos);
}

// ===== Edge Cases: 10 bends =====

TEST_CASE("Edge: 10 bends all present in output", "[phase5][edge]") {
    PostProcessor pp;
    auto p4 = makeP4(10);
    auto p3 = makeP3(10);
    auto bends = makeBends(10);

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.program.instructions.size() == 10);
    REQUIRE(result.program.bendCount == 10);
    REQUIRE(result.visualization.bones.size() == 11); // root + 10 flanges
}

// ===== Edge Cases: Out-of-range angles =====

TEST_CASE("Edge: out-of-range angle caught by validator", "[phase5][edge]") {
    OutputValidator validator;
    MachineProgram prog;
    prog.totalCycleTime = 2.5;

    MachineInstruction b;
    b.stepId = 1; b.type = InstructionType::BEND;
    b.targetAngle = 150.0;  // > 135
    b.bendForce = 1.0;
    b.duration = 2.5;
    prog.instructions.push_back(b);

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
    REQUIRE(!result.errors.empty());
}

// ===== Edge Cases: Excessive force =====

TEST_CASE("Edge: excessive force caught by validator", "[phase5][edge]") {
    OutputValidator validator;
    MachineProgram prog;
    prog.totalCycleTime = 2.5;

    MachineInstruction b;
    b.stepId = 1; b.type = InstructionType::BEND;
    b.targetAngle = 90.0;
    b.bendForce = 30.0;  // > 25 kN
    b.duration = 2.5;
    prog.instructions.push_back(b);

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
}

// ===== Edge Cases: Negative angles =====

TEST_CASE("Edge: negative angle within limits", "[phase5][edge]") {
    PostProcessor pp;
    auto p4 = makeP4(1);
    p4.springbackTable[0].targetAngle = -90.0;
    p4.springbackTable[0].compensatedAngle = -92.75;

    auto p3 = makeP3(1);
    auto bends = makeBends(1);
    bends[0].angle = -90.0;

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
}

// ===== Edge Cases: All step types =====

TEST_CASE("Edge: all step types present in program", "[phase5][edge]") {
    PostProcessor pp;
    auto p4 = makeP4(2);
    auto bends = makeBends(2);

    Phase3Output p3;
    p3.success = true;
    p3.bendSequence = {0, 1};

    SequenceAction a0;
    a0.type = ActionType::BEND; a0.bendId = 0;
    p3.actions.push_back(a0);

    SequenceAction a1;
    a1.type = ActionType::ROTATE;
    a1.newOrientation = Orientation::DEG_90;
    p3.actions.push_back(a1);

    SequenceAction a2;
    a2.type = ActionType::ABA_RECONFIG;
    a2.newAbaConfig = 0x00FF;
    p3.actions.push_back(a2);

    SequenceAction a3;
    a3.type = ActionType::BEND; a3.bendId = 1;
    p3.actions.push_back(a3);

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    // Should have: bend + rotate + aba + bend = 4 instructions
    REQUIRE(result.program.instructions.size() == 4);

    // Verify types present
    bool hasBend = false, hasRotate = false, hasAba = false;
    for (const auto& instr : result.program.instructions) {
        if (instr.type == InstructionType::BEND) hasBend = true;
        if (instr.type == InstructionType::ROTATE) hasRotate = true;
        if (instr.type == InstructionType::ABA_SETUP) hasAba = true;
    }
    REQUIRE(hasBend);
    REQUIRE(hasRotate);
    REQUIRE(hasAba);
}

// ===== Edge Cases: Multiple validation errors =====

TEST_CASE("Edge: multiple validation errors accumulated", "[phase5][edge]") {
    OutputValidator validator;
    MachineProgram prog;
    prog.totalCycleTime = 100.0;  // wrong

    MachineInstruction b1;
    b1.stepId = 1; b1.type = InstructionType::BEND;
    b1.targetAngle = 150.0;  // over limit
    b1.bendForce = 30.0;     // over limit
    b1.duration = 2.5;
    prog.instructions.push_back(b1);

    MachineInstruction b2;
    b2.stepId = 1; b2.type = InstructionType::BEND;  // duplicate ID
    b2.targetAngle = -140.0;  // under limit
    b2.bendForce = 1.0;
    b2.duration = 2.5;
    prog.instructions.push_back(b2);

    auto result = validator.validate(prog);

    REQUIRE(result.valid == false);
    REQUIRE(result.errors.size() >= 3);  // angle + force + angle + duplicate
}
