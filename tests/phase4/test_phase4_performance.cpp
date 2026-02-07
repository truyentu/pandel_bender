#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <chrono>
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

// ===== Performance: Single step validation =====

TEST_CASE("Perf: single step < 1ms", "[phase4][perf]") {
    SequenceValidator validator;
    BentState state;
    auto bend = makeBend(0, 90.0, 100.0, 0, 0, 0, 1, 0, 0);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 100; i++) {
        BentState s;
        validator.validateStep(0, bend, s);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double perStepMs = totalMs / 100.0;

    REQUIRE(perStepMs < 1.0); // < 1ms per step
}

// ===== Performance: 5-bend sequence =====

TEST_CASE("Perf: 5-bend sequence < 10ms", "[phase4][perf]") {
    SequenceValidator validator;
    std::vector<int> ids;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 5; i++) {
        ids.push_back(i);
        bends.push_back(makeBend(i, 90.0, 100.0 + i * 20,
                                 i * 300.0, 0, 0, 0, 1, 0));
    }
    auto seq = makeSequence(ids);

    auto start = std::chrono::high_resolution_clock::now();

    for (int iter = 0; iter < 10; iter++) {
        validator.validate(seq, bends);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double perRunMs = totalMs / 10.0;

    REQUIRE(perRunMs < 10.0); // < 10ms for 5-bend
}

// ===== Performance: 10-bend sequence < 100ms =====

TEST_CASE("Perf: 10-bend sequence < 100ms", "[phase4][perf]") {
    SequenceValidator validator;
    std::vector<int> ids;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 10; i++) {
        ids.push_back(i);
        bends.push_back(makeBend(i, 90.0, 100.0 + i * 10,
                                 i * 250.0, (i % 2) * 200.0, 0,
                                 (i % 2 == 0) ? 0.0 : 1.0,
                                 (i % 2 == 0) ? 1.0 : 0.0,
                                 0));
    }
    auto seq = makeSequence(ids);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = validator.validate(seq, bends);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.stepResults.size() == 10);
    REQUIRE(ms < 100.0); // < 100ms for 10 bends
}

// ===== Performance: 20-bend sequence < 500ms =====

TEST_CASE("Perf: 20-bend sequence < 500ms", "[phase4][perf]") {
    SequenceValidator validator;
    std::vector<int> ids;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 20; i++) {
        ids.push_back(i);
        double angle = 45.0 + (i % 4) * 30.0; // 45, 75, 105, 135
        bends.push_back(makeBend(i, angle, 80.0 + i * 10,
                                 i * 200.0, (i % 3) * 150.0, 0,
                                 (i % 2 == 0) ? 0.0 : 1.0,
                                 (i % 2 == 0) ? 1.0 : 0.0,
                                 0));
    }
    auto seq = makeSequence(ids);

    auto start = std::chrono::high_resolution_clock::now();
    auto result = validator.validate(seq, bends);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(result.stepResults.size() == 20);
    REQUIRE(result.springbackTable.size() == 20);
    REQUIRE(ms < 500.0); // < 500ms for 20 bends
}

// ===== Performance: Collision detection scalability =====

TEST_CASE("Perf: collision detection scales with N bends", "[phase4][perf]") {
    CollisionDetector detector;
    SweptVolumeGenerator gen;

    // Build a state with many bent flanges
    BentState state;
    for (int i = 0; i < 50; i++) {
        state.addBentFlange(makeBend(i, 90.0, 100.0,
                                     i * 300.0, 0, 0, 0, 1, 0));
    }

    auto bend = makeBend(50, 90.0, 100.0, 25000, 0, 0, 1, 0, 0);
    SweptVolume sv = gen.generate(bend);

    auto start = std::chrono::high_resolution_clock::now();

    for (int iter = 0; iter < 1000; iter++) {
        detector.checkStep(sv, state);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double totalMs = std::chrono::duration<double, std::milli>(end - start).count();
    double perCheckMs = totalMs / 1000.0;

    REQUIRE(perCheckMs < 1.0); // < 1ms even with 50 flanges
}

// ===== Performance: Springback batch =====

TEST_CASE("Perf: springback compensation for 100 bends", "[phase4][perf]") {
    SpringbackCompensator comp;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 100; i++) {
        bends.push_back(makeBend(i, 90.0, 100.0, i * 100.0, 0, 0, 1, 0, 0));
    }

    auto start = std::chrono::high_resolution_clock::now();
    auto results = comp.compensateAll(bends);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();

    REQUIRE(results.size() == 100);
    REQUIRE(ms < 10.0); // < 10ms for 100 bends
}

// ===== Performance: Validation time is recorded =====

TEST_CASE("Perf: validationTimeMs is recorded accurately", "[phase4][perf]") {
    SequenceValidator validator;
    std::vector<int> ids;
    std::vector<BendFeature> bends;
    for (int i = 0; i < 5; i++) {
        ids.push_back(i);
        bends.push_back(makeBend(i, 90.0, 100.0,
                                 i * 400.0, 0, 0, 0, 1, 0));
    }
    auto seq = makeSequence(ids);

    auto result = validator.validate(seq, bends);

    REQUIRE(result.validationTimeMs > 0.0);
    REQUIRE(result.validationTimeMs < 100.0);
}
