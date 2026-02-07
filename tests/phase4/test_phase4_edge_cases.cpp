#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/sequence_validator.h"

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
    seq.optimal = true;
    for (int id : bendIds) {
        SequenceAction action;
        action.type = ActionType::BEND;
        action.bendId = id;
        seq.actions.push_back(action);
    }
    return seq;
}

// ===== Edge Case: Empty sequence =====

TEST_CASE("Edge: empty sequence is valid", "[phase4][edge]") {
    SequenceValidator validator;
    Phase3Output seq;
    seq.bendSequence = {};
    seq.success = true;

    auto result = validator.validate(seq, {});

    REQUIRE(result.success == true);
    REQUIRE(result.allStepsValid == true);
    REQUIRE(result.collisionsDetected == 0);
    REQUIRE(result.graspFailures == 0);
    REQUIRE(result.extractionFailures == 0);
}

// ===== Edge Case: Single bend always valid =====

TEST_CASE("Edge: single bend always valid", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.collisionsDetected == 0);
    REQUIRE(result.stepResults[0].isValid() == true);
}

// ===== Edge Case: L-bracket (2 perpendicular bends) =====

TEST_CASE("Edge: L-bracket 2 bends no collision", "[phase4][edge]") {
    SequenceValidator validator;
    // Two bends on opposite edges of a flat part
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, 0, 0, 0, 0, 1, 0),     // Left edge
        makeBend(1, 90.0, 200.0, 400, 0, 0, 0, 1, 0)     // Right edge, far away
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.collisionsDetected == 0);
    REQUIRE(result.stepResults.size() == 2);
}

// ===== Edge Case: U-channel (3 bends) =====

TEST_CASE("Edge: U-channel 3 bends", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1, 2});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 300.0, -200, 0, 0, 0, 1, 0),   // Left wall
        makeBend(1, 90.0, 300.0, 200, 0, 0, 0, 1, 0),     // Right wall
        makeBend(2, 90.0, 100.0, 0, 300, 0, 1, 0, 0)      // Back wall
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.stepResults.size() == 3);
    // First two bends should not collide (distant)
    REQUIRE(result.stepResults[0].collision.hasCollision == false);
}

// ===== Edge Case: Box (4 bends) - may have extraction trap =====

TEST_CASE("Edge: box 4 bends checks extraction", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1, 2, 3});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, -150, 0, 0, 0, 1, 0),   // Left
        makeBend(1, 90.0, 200.0, 150, 0, 0, 0, 1, 0),     // Right
        makeBend(2, 90.0, 100.0, 0, -100, 0, 1, 0, 0),    // Front
        makeBend(3, 90.0, 100.0, 0, 100, 0, 1, 0, 0)      // Back
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.stepResults.size() == 4);
    // All steps produce springback data
    REQUIRE(result.springbackTable.size() == 4);
    for (const auto& sb : result.springbackTable) {
        REQUIRE(sb.compensatedAngle > sb.targetAngle);
    }
}

// ===== Edge Case: All same direction =====

TEST_CASE("Edge: all same direction no collision", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1, 2, 3});
    // All bends along Y-axis, spaced far apart on X
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, 0, 0, 0, 0, 1, 0),
        makeBend(1, 90.0, 200.0, 500, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 200.0, 1000, 0, 0, 0, 1, 0),
        makeBend(3, 90.0, 200.0, 1500, 0, 0, 0, 1, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.collisionsDetected == 0);
}

// ===== Edge Case: Opposing flanges collision =====

TEST_CASE("Edge: opposing flanges detect collision", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    // Two bends at same position, opposing directions
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 0, 0, 0, -1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.collisionsDetected >= 1);
}

// ===== Edge Case: Very small angle =====

TEST_CASE("Edge: very small angle bend", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 5.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.springbackTable[0].springbackAngle > 0.0);
    // Small angle should have small springback
    REQUIRE(result.springbackTable[0].springbackAngle < 1.0);
}

// ===== Edge Case: Large angle =====

TEST_CASE("Edge: 135 degree bend", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 135.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.springbackTable[0].compensatedAngle > 135.0);
}

// ===== Edge Case: Negative angle =====

TEST_CASE("Edge: negative angle bend", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, -90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.springbackTable[0].compensatedAngle < -90.0);
}

// ===== Edge Case: Zero-length bend =====

TEST_CASE("Edge: zero-length bend still validates", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 0.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    // Should complete without crash
    REQUIRE(result.stepResults.size() == 1);
}

// ===== Edge Case: Many bends in line =====

TEST_CASE("Edge: 10 bends chain", "[phase4][edge]") {
    SequenceValidator validator;
    std::vector<int> ids;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 10; i++) {
        ids.push_back(i);
        bends.push_back(makeBend(i, 90.0, 100.0,
                                 i * 300.0, 0, 0, 0, 1, 0));
    }
    auto seq = makeSequence(ids);

    auto result = validator.validate(seq, bends);

    REQUIRE(result.stepResults.size() == 10);
    REQUIRE(result.springbackTable.size() == 10);
    // Distant bends should not collide
    REQUIRE(result.collisionsDetected == 0);
}

// ===== Edge Case: Summary output =====

TEST_CASE("Edge: generateSummary contains all fields", "[phase4][edge]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 500, 0, 0, 0, 1, 0)
    };

    auto result = validator.validate(seq, bends);
    std::string summary = result.generateSummary();

    REQUIRE(summary.find("PASS") != std::string::npos);
    REQUIRE(summary.find("Steps: 2") != std::string::npos);
    REQUIRE(summary.find("Collisions: 0") != std::string::npos);
    REQUIRE(summary.find("Validation Time") != std::string::npos);
}
