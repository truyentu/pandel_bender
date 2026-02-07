#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase2/constraint_solver.h"
#include "openpanelcam/phase3/sequencer.h"
#include "openpanelcam/phase4/sequence_validator.h"

using namespace openpanelcam::phase1;
using namespace openpanelcam::phase2;
using namespace openpanelcam::phase3;
using namespace openpanelcam::phase4;
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

// ===== Full Pipeline: Phase 2 → Phase 3 → Phase 4 =====

TEST_CASE("Full pipeline: ConstraintSolver -> Sequencer -> Validator (3 bends)", "[phase4][integration]") {
    // Create test bends - L-bracket with 3 bends, well separated
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

    REQUIRE(p4.stepResults.size() == 3);
    REQUIRE(p4.springbackTable.size() == 3);
    REQUIRE(p4.validationTimeMs >= 0.0);

    // Check springback computed for all bends
    for (const auto& sb : p4.springbackTable) {
        REQUIRE(sb.compensatedAngle > sb.targetAngle);
        REQUIRE(sb.springbackAngle > 0.0);
    }
}

TEST_CASE("Full pipeline: 2-bend L-bracket all valid", "[phase4][integration]") {
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

    REQUIRE(p4.collisionsDetected == 0);
    REQUIRE(p4.stepResults.size() == 2);
    // Check each step individually - some may have grasp warnings
    // but collision detection should pass for distant bends
    for (const auto& step : p4.stepResults) {
        REQUIRE(step.collision.hasCollision == false);
    }
}

TEST_CASE("Full pipeline: summary output contains all info", "[phase4][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 45.0, 150.0, 400, 0, 0, 0, 1, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    std::string p3Summary = p3.generateSummary();
    std::string p4Summary = p4.generateSummary();

    REQUIRE(p3Summary.find("Phase 3") != std::string::npos);
    REQUIRE(p4Summary.find("Phase 4") != std::string::npos);
    REQUIRE(p4Summary.find("Steps: 2") != std::string::npos);
}

TEST_CASE("Full pipeline: 5-bend part validates", "[phase4][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 150.0, 0, 0, 0, 0, 1, 0),
        makeBend(1, 90.0, 150.0, 500, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 100.0, 0, 300, 0, 1, 0, 0),
        makeBend(3, 90.0, 100.0, 500, 300, 0, 1, 0, 0),
        makeBend(4, 45.0, 80.0, 250, 150, 0, 0, 1, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);
    REQUIRE(p2.success == true);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);
    REQUIRE(p3.success == true);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    REQUIRE(p4.stepResults.size() == 5);
    REQUIRE(p4.springbackTable.size() == 5);
    REQUIRE(p4.validationTimeMs < 100.0);
}

TEST_CASE("Full pipeline: step describe output", "[phase4][integration]") {
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    ConstraintSolver solver;
    Phase2Output p2 = solver.solve(bends);

    Sequencer seq;
    Phase3Output p3 = seq.sequence(p2, bends);

    SequenceValidator validator;
    Phase4Output p4 = validator.validate(p3, bends);

    REQUIRE(p4.stepResults.size() == 1);
    std::string desc = p4.stepResults[0].describe();
    REQUIRE(desc.find("Step 0") != std::string::npos);
}
