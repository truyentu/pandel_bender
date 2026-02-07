#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/post_processor.h"

using namespace openpanelcam::phase5;
using namespace openpanelcam::phase4;
using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;
using Catch::Approx;

// Helper: minimal valid Phase4Output
static Phase4Output makePhase4Output() {
    Phase4Output p4;
    p4.success = true;

    SpringbackData sb0;
    sb0.bendId = 0;
    sb0.targetAngle = 90.0;
    sb0.compensatedAngle = 92.75;
    sb0.springbackAngle = 2.75;
    p4.springbackTable.push_back(sb0);

    SpringbackData sb1;
    sb1.bendId = 1;
    sb1.targetAngle = 90.0;
    sb1.compensatedAngle = 92.75;
    sb1.springbackAngle = 2.75;
    p4.springbackTable.push_back(sb1);

    return p4;
}

// Helper: minimal Phase3Output
static Phase3Output makePhase3Output() {
    Phase3Output p3;
    p3.success = true;
    p3.bendSequence = {0, 1};

    SequenceAction a0;
    a0.type = ActionType::BEND;
    a0.bendId = 0;
    p3.actions.push_back(a0);

    SequenceAction a1;
    a1.type = ActionType::ROTATE;
    a1.newOrientation = Orientation::DEG_90;
    p3.actions.push_back(a1);

    SequenceAction a2;
    a2.type = ActionType::BEND;
    a2.bendId = 1;
    p3.actions.push_back(a2);

    return p3;
}

// Helper: minimal bends
static std::vector<BendFeature> makeBends() {
    std::vector<BendFeature> bends;

    BendFeature b0;
    b0.id = 0; b0.angle = 90.0; b0.length = 200.0;
    b0.position = {0, 50, 0};
    b0.direction = {1, 0, 0};
    b0.normal = {0, 0, 1};
    bends.push_back(b0);

    BendFeature b1;
    b1.id = 1; b1.angle = 90.0; b1.length = 300.0;
    b1.position = {0, 100, 0};
    b1.direction = {0, 1, 0};
    b1.normal = {0, 0, 1};
    bends.push_back(b1);

    return bends;
}

// ===== Basic pipeline =====

TEST_CASE("PostProcessor basic pipeline succeeds", "[phase5][pipeline]") {
    PostProcessor pp;
    auto p4 = makePhase4Output();
    auto p3 = makePhase3Output();
    auto bends = makeBends();

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.errors.empty());
    REQUIRE(result.generationTimeMs > 0);
}

TEST_CASE("PostProcessor generates XML", "[phase5][pipeline]") {
    PostProcessor pp;
    auto p4 = makePhase4Output();
    auto p3 = makePhase3Output();
    auto bends = makeBends();

    auto result = pp.process(p4, p3, bends);

    REQUIRE(!result.xmlOutput.empty());
    REQUIRE(result.xmlValid == true);
    REQUIRE(result.xmlOutput.find("Job") != std::string::npos);
    REQUIRE(result.xmlOutput.find("Bend") != std::string::npos);
}

TEST_CASE("PostProcessor generates JSON", "[phase5][pipeline]") {
    PostProcessor pp;
    auto p4 = makePhase4Output();
    auto p3 = makePhase3Output();
    auto bends = makeBends();

    auto result = pp.process(p4, p3, bends);

    REQUIRE(!result.jsonOutput.empty());
    REQUIRE(result.jsonValid == true);
    REQUIRE(result.jsonOutput.find("scene") != std::string::npos);
    REQUIRE(result.jsonOutput.find("skeleton") != std::string::npos);
}

TEST_CASE("PostProcessor program has correct instruction count", "[phase5][pipeline]") {
    PostProcessor pp;
    auto p4 = makePhase4Output();
    auto p3 = makePhase3Output();
    auto bends = makeBends();

    auto result = pp.process(p4, p3, bends);

    // 2 bends + 1 rotation = 3 instructions
    REQUIRE(result.program.instructions.size() == 3);
    REQUIRE(result.program.bendCount == 2);
}

TEST_CASE("PostProcessor visualization has bones and keyframes", "[phase5][pipeline]") {
    PostProcessor pp;
    auto p4 = makePhase4Output();
    auto p3 = makePhase3Output();
    auto bends = makeBends();

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.visualization.bones.size() == 3);  // root + 2 flanges
    REQUIRE(result.visualization.keyframes.size() >= 3);  // initial + at least 2
}

// ===== Config options =====

TEST_CASE("PostProcessor XML only", "[phase5][pipeline]") {
    PostProcessorConfig config;
    config.generateXML = true;
    config.generateJSON = false;
    PostProcessor pp(config);

    auto result = pp.process(makePhase4Output(), makePhase3Output(), makeBends());

    REQUIRE(!result.xmlOutput.empty());
    REQUIRE(result.jsonOutput.empty());
    REQUIRE(result.success == true);
}

TEST_CASE("PostProcessor JSON only", "[phase5][pipeline]") {
    PostProcessorConfig config;
    config.generateXML = false;
    config.generateJSON = true;
    PostProcessor pp(config);

    auto result = pp.process(makePhase4Output(), makePhase3Output(), makeBends());

    REQUIRE(result.xmlOutput.empty());
    REQUIRE(!result.jsonOutput.empty());
    REQUIRE(result.success == true);
}

TEST_CASE("PostProcessor setConfig works", "[phase5][pipeline]") {
    PostProcessor pp;
    PostProcessorConfig config;
    config.generateJSON = false;
    pp.setConfig(config);

    auto result = pp.process(makePhase4Output(), makePhase3Output(), makeBends());

    REQUIRE(result.jsonOutput.empty());
}

// ===== Empty input =====

TEST_CASE("PostProcessor empty sequence", "[phase5][pipeline]") {
    PostProcessor pp;
    Phase4Output p4; p4.success = true;
    Phase3Output p3; p3.success = true;
    std::vector<BendFeature> bends;

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.program.instructions.empty());
    REQUIRE(!result.xmlOutput.empty());
    REQUIRE(!result.jsonOutput.empty());
}
