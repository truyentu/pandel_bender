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

// ===== SequenceValidator Constructor =====

TEST_CASE("SequenceValidator default construction", "[phase4][validator]") {
    SequenceValidator validator;
    REQUIRE(validator.config().collisionMargin == 2.0);
    REQUIRE(validator.config().enableSpringback == true);
}

TEST_CASE("SequenceValidator custom config", "[phase4][validator]") {
    ValidatorConfig config;
    config.collisionMargin = 5.0;
    config.enableSpringback = false;
    SequenceValidator validator(config);
    REQUIRE(validator.config().collisionMargin == 5.0);
    REQUIRE(validator.config().enableSpringback == false);
}

// ===== Empty Sequence =====

TEST_CASE("SequenceValidator empty sequence passes", "[phase4][validator]") {
    SequenceValidator validator;
    Phase3Output seq;
    seq.bendSequence = {};
    seq.success = true;
    std::vector<BendFeature> bends;

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.allStepsValid == true);
    REQUIRE(result.stepResults.empty());
}

// ===== Failed Phase 3 Input =====

TEST_CASE("SequenceValidator rejects failed Phase 3 input", "[phase4][validator]") {
    SequenceValidator validator;
    Phase3Output seq;
    seq.success = false;
    seq.bendSequence = {0};
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == false);
    REQUIRE(!result.errors.empty());
}

// ===== Single Bend =====

TEST_CASE("SequenceValidator single bend passes", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.stepResults.size() == 1);
    REQUIRE(result.stepResults[0].bendId == 0);
    REQUIRE(result.stepResults[0].isValid() == true);
    REQUIRE(result.collisionsDetected == 0);
}

// ===== L-bracket (2 distant bends) =====

TEST_CASE("SequenceValidator L-bracket no collision", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 500, 0, 0, 0, 1, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.stepResults.size() == 2);
    REQUIRE(result.collisionsDetected == 0);
}

// ===== Collision Detection =====

TEST_CASE("SequenceValidator detects collision for overlapping bends", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0) // Same position
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.collisionsDetected == 1);
    REQUIRE(result.stepResults[1].collision.hasCollision == true);
}

// ===== Springback =====

TEST_CASE("SequenceValidator produces springback table", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 45.0, 150.0, 500, 0, 0, 0, 1, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.springbackTable.size() == 2);
    REQUIRE(result.springbackTable[0].targetAngle == 90.0);
    REQUIRE(result.springbackTable[0].compensatedAngle > 90.0);
    REQUIRE(result.springbackTable[1].targetAngle == 45.0);
    REQUIRE(result.springbackTable[1].compensatedAngle > 45.0);
}

TEST_CASE("SequenceValidator springback disabled", "[phase4][validator]") {
    ValidatorConfig config;
    config.enableSpringback = false;
    SequenceValidator validator(config);
    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.springbackTable[0].springbackAngle == 0.0);
    REQUIRE(result.springbackTable[0].compensatedAngle == 90.0);
}

// ===== Validation Time =====

TEST_CASE("SequenceValidator records validation time", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1, 2});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 300, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 100.0, 600, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.validationTimeMs >= 0.0);
    REQUIRE(result.validationTimeMs < 1000.0); // Should be fast
}

// ===== Missing Bend ID =====

TEST_CASE("SequenceValidator handles missing bend ID", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 99}); // 99 doesn't exist
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == false);
    REQUIRE(!result.errors.empty());
}

// ===== setConfig =====

TEST_CASE("SequenceValidator setConfig updates all subcomponents", "[phase4][validator]") {
    SequenceValidator validator;
    ValidatorConfig newConfig;
    newConfig.collisionMargin = 10.0;
    newConfig.enableSpringback = false;
    newConfig.minGripArea = 500.0;

    validator.setConfig(newConfig);

    REQUIRE(validator.config().collisionMargin == 10.0);
    REQUIRE(validator.config().enableSpringback == false);
    REQUIRE(validator.config().minGripArea == 500.0);
}

// ===== U-channel (3 bends chain) =====

TEST_CASE("SequenceValidator U-channel passes", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1, 2});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 200.0, 0, 0, 0, 0, 1, 0),
        makeBend(1, 90.0, 200.0, 300, 0, 0, 0, 1, 0),
        makeBend(2, 90.0, 100.0, 150, 200, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.stepResults.size() == 3);
    // All steps should produce valid results (no collision for distant bends)
    REQUIRE(result.stepResults[0].collision.hasCollision == false);
}

// ===== Feedback Tests =====

TEST_CASE("SequenceValidator generates collision feedback", "[phase4][feedback]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0) // Collision
    };

    auto result = validator.validate(seq, bends);

    if (result.collisionsDetected > 0) {
        REQUIRE(!result.errors.empty());
        REQUIRE(!result.suggestions.empty());

        // Check that suggestions mention reordering
        bool hasReorderSuggestion = false;
        for (const auto& s : result.suggestions) {
            if (s.find("reorder") != std::string::npos ||
                s.find("repositioning") != std::string::npos) {
                hasReorderSuggestion = true;
                break;
            }
        }
        REQUIRE(hasReorderSuggestion);
    }
}

TEST_CASE("SequenceValidator generates grasp feedback on strict config", "[phase4][feedback]") {
    ValidatorConfig config;
    config.minGripArea = 1e9; // Impossibly large
    SequenceValidator validator(config);

    auto seq = makeSequence({0});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0)
    };

    auto result = validator.validate(seq, bends);

    REQUIRE(result.success == false);
    REQUIRE(result.graspFailures > 0);
    REQUIRE(!result.errors.empty());
    REQUIRE(!result.suggestions.empty());

    // Check suggestions mention grip
    bool hasGripSuggestion = false;
    for (const auto& s : result.suggestions) {
        if (s.find("Grip") != std::string::npos ||
            s.find("grip") != std::string::npos) {
            hasGripSuggestion = true;
            break;
        }
    }
    REQUIRE(hasGripSuggestion);
}

TEST_CASE("SequenceValidator generateSummary works", "[phase4][feedback]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 500, 0, 0, 0, 1, 0)
    };

    auto result = validator.validate(seq, bends);
    std::string summary = result.generateSummary();

    REQUIRE(!summary.empty());
    REQUIRE(summary.find("Phase 4") != std::string::npos);
    REQUIRE(summary.find("Steps: 2") != std::string::npos);
}

// ===== validateStep direct =====

TEST_CASE("SequenceValidator validateStep returns full result", "[phase4][validator]") {
    SequenceValidator validator;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto step = validator.validateStep(0, bend, state);

    REQUIRE(step.stepIndex == 0);
    REQUIRE(step.bendId == 0);
    REQUIRE(step.collision.hasCollision == false);
    REQUIRE(step.springback.targetAngle == 90.0);
    REQUIRE(step.springback.compensatedAngle > 90.0);
}

TEST_CASE("SequenceValidator validateStep describe output", "[phase4][validator]") {
    SequenceValidator validator;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto step = validator.validateStep(0, bend, state);
    std::string desc = step.describe();

    REQUIRE(desc.find("Step 0") != std::string::npos);
    REQUIRE(desc.find("bend 0") != std::string::npos);
}

// ===== Multi-step validation tracking =====

TEST_CASE("SequenceValidator tracks failure counts", "[phase4][validator]") {
    SequenceValidator validator;
    auto seq = makeSequence({0, 1});
    std::vector<BendFeature> bends = {
        makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0),
        makeBend(1, 90.0, 100.0, 0, 0, 0, 0, 1, 0) // Same position → collision
    };

    auto result = validator.validate(seq, bends);

    // Failure counts should be consistent
    int actualCollisions = 0;
    int actualGraspFails = 0;
    int actualExtractFails = 0;
    for (const auto& sr : result.stepResults) {
        if (sr.collision.hasCollision) actualCollisions++;
        if (!sr.grasp.isValid()) actualGraspFails++;
        if (!sr.extraction.canExtract) actualExtractFails++;
    }
    REQUIRE(result.collisionsDetected == actualCollisions);
    REQUIRE(result.graspFailures == actualGraspFails);
    REQUIRE(result.extractionFailures == actualExtractFails);
}
