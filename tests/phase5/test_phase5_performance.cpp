#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include "openpanelcam/phase5/post_processor.h"
#include "openpanelcam/phase5/xml_generator.h"
#include "openpanelcam/phase5/json_generator.h"
#include <chrono>

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

static MachineProgram makeProgram(int numBends) {
    MachineProgram prog;
    prog.partName = "PERF_TEST";
    prog.totalCycleTime = numBends * 2.5;
    prog.bendCount = numBends;
    for (int i = 0; i < numBends; i++) {
        MachineInstruction b;
        b.stepId = i + 1;
        b.type = InstructionType::BEND;
        b.bendId = i;
        b.targetAngle = 90.0;
        b.compensatedAngle = 92.75;
        b.springbackAngle = 2.75;
        b.bendForce = 0.94;
        b.bendLength = 100.0 + i * 50.0;
        b.startX = 0; b.startY = i * 100.0;
        b.endX = b.bendLength; b.endY = i * 100.0;
        b.duration = 2.5;
        prog.instructions.push_back(b);
    }
    return prog;
}

static VisualizationData makeViz(int numBends) {
    VisualizationData viz;
    viz.partName = "PERF_TEST";
    viz.cycleTime = numBends * 2.5;
    viz.bendCount = numBends;

    AnimationBone root;
    root.index = 0; root.name = "base"; root.parentIndex = -1;
    viz.bones.push_back(root);

    for (int i = 0; i < numBends; i++) {
        AnimationBone bone;
        bone.index = i + 1;
        bone.name = "flange_" + std::to_string(i);
        bone.parentIndex = 0;
        bone.originX = i * 200.0;
        bone.axisX = 1; bone.axisY = 0; bone.axisZ = 0;
        viz.bones.push_back(bone);

        AnimationKeyframe kf;
        kf.time = (i + 1) * 2.5;
        kf.boneIndex = i + 1;
        kf.rotationAngle = 90.0;
        kf.actionDescription = "Bend " + std::to_string(i);
        viz.keyframes.push_back(kf);
    }
    return viz;
}

// ===== Timing Tests (< 5ms targets) =====

TEST_CASE("Performance: XML generation < 5ms for 10 bends", "[phase5][perf]") {
    PBXMLGenerator gen;
    auto prog = makeProgram(10);

    auto start = std::chrono::high_resolution_clock::now();
    std::string xml = gen.generate(prog);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(!xml.empty());
    REQUIRE(ms < 5.0);
}

TEST_CASE("Performance: JSON generation < 5ms for 10 bends", "[phase5][perf]") {
    VDMGenerator gen;
    auto viz = makeViz(10);

    auto start = std::chrono::high_resolution_clock::now();
    std::string json = gen.generate(viz);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(!json.empty());
    REQUIRE(ms < 5.0);
}

TEST_CASE("Performance: full pipeline < 20ms for 10 bends", "[phase5][perf]") {
    PostProcessor pp;
    auto p4 = makeP4(10);
    auto p3 = makeP3(10);
    auto bends = makeBends(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = pp.process(p4, p3, bends);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.success == true);
    REQUIRE(ms < 20.0);
}

// ===== Scalability =====

TEST_CASE("Performance: 20 bends still within limits", "[phase5][perf]") {
    PostProcessor pp;
    auto p4 = makeP4(20);
    auto p3 = makeP3(20);
    auto bends = makeBends(20);

    auto result = pp.process(p4, p3, bends);

    REQUIRE(result.success == true);
    REQUIRE(result.program.instructions.size() == 20);
    REQUIRE(result.generationTimeMs < 50.0);
}

// ===== Catch2 Benchmarks =====

TEST_CASE("Benchmark: XML generation", "[phase5][!benchmark]") {
    PBXMLGenerator gen;
    auto prog = makeProgram(10);

    BENCHMARK("XML gen 10 bends") {
        return gen.generate(prog);
    };
}

TEST_CASE("Benchmark: JSON generation", "[phase5][!benchmark]") {
    VDMGenerator gen;
    auto viz = makeViz(10);

    BENCHMARK("JSON gen 10 bends") {
        return gen.generate(viz);
    };
}

TEST_CASE("Benchmark: full pipeline", "[phase5][!benchmark]") {
    PostProcessor pp;
    auto p4 = makeP4(10);
    auto p3 = makeP3(10);
    auto bends = makeBends(10);

    BENCHMARK("Pipeline 10 bends") {
        return pp.process(p4, p3, bends);
    };
}
