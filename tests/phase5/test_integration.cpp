#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include "openpanelcam/phase3/sequencer.h"
#include "openpanelcam/phase4/sequence_validator.h"
#include "openpanelcam/phase5/post_processor.h"

using namespace openpanelcam::phase1;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase3;
using namespace openpanelcam::phase4;
using namespace openpanelcam::phase5;
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

// ===== Full Pipeline: Phase 2 → 3 → 4 → 5 =====

TEST_CASE("Full pipeline: Phase 2 -> 3 -> 4 -> 5 (3 bends)", "[phase5][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 500, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 120.0, 0, 500, 0, -1, 0, 0)
    };

    // Phase 2: Solve constraints
    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);
    REQUIRE(p2.success == true);

    // Phase 3: Find optimal sequence
    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);
    REQUIRE(p3.success == true);
    REQUIRE(p3.bendSequence.size() == 3);

    // Phase 4: Validate the sequence
    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);
    REQUIRE(p4.springbackTable.size() == 3);

    // Phase 5: Post-process
    PostProcessor pp;
    Phase5Output p5 = pp.process(p4, p3, bends);

    REQUIRE(p5.success == true);
    REQUIRE(!p5.xmlOutput.empty());
    REQUIRE(!p5.jsonOutput.empty());
    REQUIRE(p5.xmlValid == true);
    REQUIRE(p5.jsonValid == true);
    REQUIRE(p5.errors.empty());

    // XML should contain bends
    REQUIRE(p5.xmlOutput.find("Bend") != std::string::npos);
    REQUIRE(p5.xmlOutput.find("Job") != std::string::npos);

    // JSON should contain skeleton
    REQUIRE(p5.jsonOutput.find("skeleton") != std::string::npos);
    REQUIRE(p5.jsonOutput.find("animation") != std::string::npos);

    // Program should have at least 3 bends
    int bendCount = 0;
    for (const auto& instr : p5.program.instructions) {
        if (instr.type == InstructionType::BEND) bendCount++;
    }
    REQUIRE(bendCount == 3);
}

TEST_CASE("Full pipeline: 2-bend L-bracket end-to-end", "[phase5][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, 0, 0, 0, 0, 1, 0),
        makeBend(1, 90.0, 200.0, 600, 0, 0, 0, 1, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);
    REQUIRE(p2.success == true);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);
    REQUIRE(p3.success == true);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    PostProcessor pp;
    Phase5Output p5 = pp.process(p4, p3, bends);

    REQUIRE(p5.success == true);
    REQUIRE(!p5.xmlOutput.empty());
    REQUIRE(!p5.jsonOutput.empty());

    // Visualization should have root + 2 flanges
    REQUIRE(p5.visualization.bones.size() == 3);
    REQUIRE(p5.visualization.bones[0].name == "base");
}

TEST_CASE("Full pipeline: output generation time is reasonable", "[phase5][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 150.0, 500, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 120.0, 0, 500, 0, -1, 0, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    PostProcessor pp;
    Phase5Output p5 = pp.process(p4, p3, bends);

    REQUIRE(p5.generationTimeMs < 50.0);
    REQUIRE(p5.generationTimeMs > 0.0);
}

TEST_CASE("Full pipeline: springback in XML output", "[phase5][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, 0, 0, 0, 1, 0, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    PostProcessor pp;
    Phase5Output p5 = pp.process(p4, p3, bends);

    REQUIRE(p5.success == true);

    // XML should contain springback-compensated angle
    REQUIRE(p5.xmlOutput.find("compensated") != std::string::npos);
    REQUIRE(p5.xmlOutput.find("Springback") != std::string::npos);
}
