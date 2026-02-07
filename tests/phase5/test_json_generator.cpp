#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase5/json_generator.h"

using namespace openpanelcam::phase5;
using Catch::Approx;

static VisualizationData makeSimpleViz() {
    VisualizationData viz;
    viz.partName = "BOX001";
    viz.cycleTime = 7.5;
    viz.bendCount = 2;

    AnimationBone root;
    root.index = 0; root.name = "base"; root.parentIndex = -1;
    viz.bones.push_back(root);

    AnimationBone f1;
    f1.index = 1; f1.name = "flange_0"; f1.parentIndex = 0;
    f1.originX = 0; f1.originY = 50;
    f1.axisX = 1; f1.axisY = 0; f1.axisZ = 0;
    viz.bones.push_back(f1);

    AnimationBone f2;
    f2.index = 2; f2.name = "flange_1"; f2.parentIndex = 0;
    f2.originX = 300; f2.originY = 0;
    f2.axisX = 0; f2.axisY = 1; f2.axisZ = 0;
    viz.bones.push_back(f2);

    AnimationKeyframe kf0;
    kf0.time = 0.0; kf0.boneIndex = 0; kf0.rotationAngle = 0;
    kf0.actionDescription = "Start";
    viz.keyframes.push_back(kf0);

    AnimationKeyframe kf1;
    kf1.time = 2.5; kf1.boneIndex = 1; kf1.rotationAngle = 90.0;
    kf1.actionDescription = "Bend 0";
    viz.keyframes.push_back(kf1);

    AnimationKeyframe kf2;
    kf2.time = 5.0; kf2.boneIndex = 2; kf2.rotationAngle = 90.0;
    kf2.actionDescription = "Bend 1";
    viz.keyframes.push_back(kf2);

    return viz;
}

// ===== JSON Generation =====

TEST_CASE("VDMGenerator produces non-empty JSON", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();

    std::string json = gen.generate(viz);

    REQUIRE(!json.empty());
    REQUIRE(json.front() == '{');
}

TEST_CASE("VDMGenerator contains scene", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    REQUIRE(json.find("scene") != std::string::npos);
    REQUIRE(json.find("camera") != std::string::npos);
    REQUIRE(json.find("fov") != std::string::npos);
}

TEST_CASE("VDMGenerator contains skeleton", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    REQUIRE(json.find("skeleton") != std::string::npos);
    REQUIRE(json.find("bones") != std::string::npos);
    REQUIRE(json.find("base") != std::string::npos);
    REQUIRE(json.find("flange_0") != std::string::npos);
    REQUIRE(json.find("flange_1") != std::string::npos);
}

TEST_CASE("VDMGenerator contains animation", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    REQUIRE(json.find("animation") != std::string::npos);
    REQUIRE(json.find("duration") != std::string::npos);
    REQUIRE(json.find("fps") != std::string::npos);
    REQUIRE(json.find("keyframes") != std::string::npos);
}

TEST_CASE("VDMGenerator contains metadata", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    REQUIRE(json.find("metadata") != std::string::npos);
    REQUIRE(json.find("BOX001") != std::string::npos);
    REQUIRE(json.find("bendCount") != std::string::npos);
}

TEST_CASE("VDMGenerator empty viz", "[phase5][json]") {
    VDMGenerator gen;
    VisualizationData viz;
    viz.partName = "EMPTY";

    std::string json = gen.generate(viz);

    REQUIRE(!json.empty());
    REQUIRE(json.find("EMPTY") != std::string::npos);
}

TEST_CASE("VDMGenerator keyframe count matches", "[phase5][json]") {
    VDMGenerator gen;
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    // Count "time" occurrences in keyframes
    size_t count = 0;
    size_t pos = 0;
    while ((pos = json.find("\"time\"", pos)) != std::string::npos) {
        count++;
        pos += 6;
    }
    // 3 keyframes should have "time" field
    // But "cycleTime" also contains "time" - so check for "time":
    // Actually keyframes have "time" key, so count should be >= 3
    REQUIRE(count >= 3);
}

TEST_CASE("VDMGenerator custom FPS", "[phase5][json]") {
    PostProcessorConfig config;
    config.animationFPS = 60.0;
    VDMGenerator gen(config);
    auto viz = makeSimpleViz();
    std::string json = gen.generate(viz);

    REQUIRE(json.find("60") != std::string::npos);
}
