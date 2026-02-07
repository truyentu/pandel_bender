#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/xml_generator.h"

using namespace openpanelcam::phase5;
using Catch::Approx;

static MachineProgram makeSimpleProgram() {
    MachineProgram prog;
    prog.partName = "BOX001";
    prog.createdBy = "OpenPanelCAM";
    prog.timestamp = "2026-02-08T10:00:00";
    prog.machineModel = "P4";
    prog.materialThickness = 1.5;
    prog.materialType = "AISI304";
    prog.yieldStrength = 200.0;
    prog.totalCycleTime = 7.5;
    prog.bendCount = 2;
    prog.repositionCount = 0;
    prog.rotationCount = 1;

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

// ===== XML Generation =====

TEST_CASE("PBXMLGenerator produces non-empty XML", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();

    std::string xml = gen.generate(prog);

    REQUIRE(!xml.empty());
    REQUIRE(xml.find("Job") != std::string::npos);
}

TEST_CASE("PBXMLGenerator contains Header", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("Header") != std::string::npos);
    REQUIRE(xml.find("Material") != std::string::npos);
    REQUIRE(xml.find("AISI304") != std::string::npos);
    REQUIRE(xml.find("Tooling") != std::string::npos);
    REQUIRE(xml.find("JobInfo") != std::string::npos);
    REQUIRE(xml.find("BOX001") != std::string::npos);
}

TEST_CASE("PBXMLGenerator contains ProcessSequence", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("ProcessSequence") != std::string::npos);
    REQUIRE(xml.find("Step") != std::string::npos);
    REQUIRE(xml.find("Bend") != std::string::npos);
    REQUIRE(xml.find("Rotation") != std::string::npos);
}

TEST_CASE("PBXMLGenerator contains Angle target and compensated", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("Angle") != std::string::npos);
    REQUIRE(xml.find("target") != std::string::npos);
    REQUIRE(xml.find("compensated") != std::string::npos);
}

TEST_CASE("PBXMLGenerator contains Metadata", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("Metadata") != std::string::npos);
    REQUIRE(xml.find("TotalCycleTime") != std::string::npos);
    REQUIRE(xml.find("BendCount") != std::string::npos);
}

TEST_CASE("PBXMLGenerator contains AdaptiveControl", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("AdaptiveControl") != std::string::npos);
    REQUIRE(xml.find("AngleFeedback") != std::string::npos);
    REQUIRE(xml.find("ForceLimit") != std::string::npos);
    REQUIRE(xml.find("SpeedControl") != std::string::npos);
}

TEST_CASE("PBXMLGenerator adaptive control disabled", "[phase5][xml]") {
    PostProcessorConfig config;
    config.includeAdaptiveControl = false;
    PBXMLGenerator gen(config);
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("AdaptiveControl") == std::string::npos);
}

TEST_CASE("PBXMLGenerator empty program", "[phase5][xml]") {
    PBXMLGenerator gen;
    MachineProgram prog;
    prog.partName = "EMPTY";

    std::string xml = gen.generate(prog);

    REQUIRE(!xml.empty());
    REQUIRE(xml.find("EMPTY") != std::string::npos);
    REQUIRE(xml.find("ProcessSequence") != std::string::npos);
}

TEST_CASE("PBXMLGenerator step count matches", "[phase5][xml]") {
    PBXMLGenerator gen;
    auto prog = makeSimpleProgram();
    std::string xml = gen.generate(prog);

    // Count occurrences of "<Step"
    size_t count = 0;
    size_t pos = 0;
    while ((pos = xml.find("<Step", pos)) != std::string::npos) {
        count++;
        pos += 5;
    }
    REQUIRE(count == 3); // 2 bends + 1 rotation
}

TEST_CASE("PBXMLGenerator contains reposition step", "[phase5][xml]") {
    PBXMLGenerator gen;
    MachineProgram prog;
    MachineInstruction repo;
    repo.stepId = 1; repo.type = InstructionType::REPOSITION;
    repo.duration = 3.0; repo.description = "Re-grip";
    prog.instructions.push_back(repo);

    std::string xml = gen.generate(prog);

    REQUIRE(xml.find("Reposition") != std::string::npos);
}
