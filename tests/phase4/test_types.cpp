#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase4/types.h"

using namespace openpanelcam::phase4;
using Catch::Approx;

// ===== AABB Tests =====

TEST_CASE("AABB default initialization", "[phase4][types][aabb]") {
    AABB box;
    REQUIRE(box.minX == 0.0);
    REQUIRE(box.maxX == 0.0);
    REQUIRE(box.volume() == 0.0);
}

TEST_CASE("AABB volume calculation", "[phase4][types][aabb]") {
    AABB box(0, 0, 0, 10, 20, 30);
    REQUIRE(box.volume() == Approx(6000.0));
}

TEST_CASE("AABB volume returns 0 for degenerate", "[phase4][types][aabb]") {
    AABB box(10, 0, 0, 5, 20, 30); // minX > maxX
    REQUIRE(box.volume() == 0.0);
}

TEST_CASE("AABB overlaps - true case", "[phase4][types][aabb]") {
    AABB a(0, 0, 0, 10, 10, 10);
    AABB b(5, 5, 5, 15, 15, 15);
    REQUIRE(a.overlaps(b) == true);
    REQUIRE(b.overlaps(a) == true);
}

TEST_CASE("AABB overlaps - false case", "[phase4][types][aabb]") {
    AABB a(0, 0, 0, 10, 10, 10);
    AABB b(20, 20, 20, 30, 30, 30);
    REQUIRE(a.overlaps(b) == false);
}

TEST_CASE("AABB overlaps - touching edge", "[phase4][types][aabb]") {
    AABB a(0, 0, 0, 10, 10, 10);
    AABB b(10, 0, 0, 20, 10, 10); // Touching at x=10
    REQUIRE(a.overlaps(b) == true); // Touching counts as overlap
}

TEST_CASE("AABB overlaps - separated on one axis", "[phase4][types][aabb]") {
    AABB a(0, 0, 0, 10, 10, 10);
    AABB b(0, 0, 11, 10, 10, 20); // Separated on Z
    REQUIRE(a.overlaps(b) == false);
}

TEST_CASE("AABB expand adds margin", "[phase4][types][aabb]") {
    AABB box(10, 20, 30, 40, 50, 60);
    AABB expanded = box.expand(5.0);

    REQUIRE(expanded.minX == Approx(5.0));
    REQUIRE(expanded.minY == Approx(15.0));
    REQUIRE(expanded.minZ == Approx(25.0));
    REQUIRE(expanded.maxX == Approx(45.0));
    REQUIRE(expanded.maxY == Approx(55.0));
    REQUIRE(expanded.maxZ == Approx(65.0));
}

TEST_CASE("AABB merge unions two boxes", "[phase4][types][aabb]") {
    AABB a(0, 0, 0, 10, 10, 10);
    AABB b(5, 5, 5, 20, 20, 20);
    AABB merged = AABB::merge(a, b);

    REQUIRE(merged.minX == 0.0);
    REQUIRE(merged.minY == 0.0);
    REQUIRE(merged.minZ == 0.0);
    REQUIRE(merged.maxX == 20.0);
    REQUIRE(merged.maxY == 20.0);
    REQUIRE(merged.maxZ == 20.0);
}

TEST_CASE("AABB overlaps - contained box", "[phase4][types][aabb]") {
    AABB outer(0, 0, 0, 100, 100, 100);
    AABB inner(10, 10, 10, 20, 20, 20);
    REQUIRE(outer.overlaps(inner) == true);
    REQUIRE(inner.overlaps(outer) == true);
}

// ===== OBB Tests =====

TEST_CASE("OBB default is axis-aligned", "[phase4][types][obb]") {
    OBB obb;
    REQUIRE(obb.axes[0] == 1.0);
    REQUIRE(obb.axes[4] == 1.0);
    REQUIRE(obb.axes[8] == 1.0);
}

TEST_CASE("OBB overlaps - aligned boxes overlap", "[phase4][types][obb]") {
    OBB a;
    a.centerX = 0; a.centerY = 0; a.centerZ = 0;
    a.halfExtentX = 5; a.halfExtentY = 5; a.halfExtentZ = 5;

    OBB b;
    b.centerX = 8; b.centerY = 0; b.centerZ = 0;
    b.halfExtentX = 5; b.halfExtentY = 5; b.halfExtentZ = 5;

    REQUIRE(a.overlaps(b) == true); // 5+5 > 8
}

TEST_CASE("OBB overlaps - aligned boxes separated", "[phase4][types][obb]") {
    OBB a;
    a.centerX = 0; a.centerY = 0; a.centerZ = 0;
    a.halfExtentX = 5; a.halfExtentY = 5; a.halfExtentZ = 5;

    OBB b;
    b.centerX = 15; b.centerY = 0; b.centerZ = 0;
    b.halfExtentX = 5; b.halfExtentY = 5; b.halfExtentZ = 5;

    REQUIRE(a.overlaps(b) == false); // 5+5 < 15
}

TEST_CASE("OBB overlaps - rotated box", "[phase4][types][obb]") {
    OBB a;
    a.centerX = 0; a.centerY = 0; a.centerZ = 0;
    a.halfExtentX = 10; a.halfExtentY = 1; a.halfExtentZ = 1;
    // Identity rotation

    OBB b;
    b.centerX = 5; b.centerY = 5; b.centerZ = 0;
    b.halfExtentX = 10; b.halfExtentY = 1; b.halfExtentZ = 1;
    // Rotated 45 degrees around Z axis
    double angle = 3.14159265 / 4.0;
    double c = std::cos(angle), s = std::sin(angle);
    b.axes[0] = c;  b.axes[1] = s;  b.axes[2] = 0;
    b.axes[3] = -s; b.axes[4] = c;  b.axes[5] = 0;
    b.axes[6] = 0;  b.axes[7] = 0;  b.axes[8] = 1;

    // These should overlap (long thin boxes crossing)
    REQUIRE(a.overlaps(b) == true);
}

TEST_CASE("OBB overlaps - rotated box separated", "[phase4][types][obb]") {
    OBB a;
    a.centerX = 0; a.centerY = 0; a.centerZ = 0;
    a.halfExtentX = 5; a.halfExtentY = 5; a.halfExtentZ = 5;

    OBB b;
    b.centerX = 0; b.centerY = 0; b.centerZ = 20;
    b.halfExtentX = 5; b.halfExtentY = 5; b.halfExtentZ = 5;
    // Rotated 45 degrees around Z
    double angle = 3.14159265 / 4.0;
    double c = std::cos(angle), s = std::sin(angle);
    b.axes[0] = c;  b.axes[1] = s;  b.axes[2] = 0;
    b.axes[3] = -s; b.axes[4] = c;  b.axes[5] = 0;
    b.axes[6] = 0;  b.axes[7] = 0;  b.axes[8] = 1;

    REQUIRE(a.overlaps(b) == false); // Separated on Z
}

// ===== CollisionResult Tests =====

TEST_CASE("CollisionResult defaults to no collision", "[phase4][types]") {
    CollisionResult cr;
    REQUIRE(cr.hasCollision == false);
    REQUIRE(cr.type == CollisionType::NONE);
}

// ===== GraspValidation Tests =====

TEST_CASE("GraspValidation default is valid", "[phase4][types]") {
    GraspValidation gv;
    REQUIRE(gv.isValid() == true);
}

TEST_CASE("GraspValidation invalid when area fails", "[phase4][types]") {
    GraspValidation gv;
    gv.areaValid = false;
    gv.gripArea = 50.0;
    REQUIRE(gv.isValid() == false);
    REQUIRE(gv.describe().find("FAILED") != std::string::npos);
    REQUIRE(gv.describe().find("area") != std::string::npos);
}

TEST_CASE("GraspValidation describe shows all failures", "[phase4][types]") {
    GraspValidation gv;
    gv.areaValid = false;
    gv.comValid = false;
    gv.torqueValid = false;
    gv.shearValid = false;
    std::string desc = gv.describe();
    REQUIRE(desc.find("area") != std::string::npos);
    REQUIRE(desc.find("COM") != std::string::npos);
    REQUIRE(desc.find("torque") != std::string::npos);
    REQUIRE(desc.find("shear") != std::string::npos);
}

// ===== StepValidation Tests =====

TEST_CASE("StepValidation valid by default", "[phase4][types]") {
    StepValidation sv;
    REQUIRE(sv.isValid() == true);
}

TEST_CASE("StepValidation invalid on collision", "[phase4][types]") {
    StepValidation sv;
    sv.collision.hasCollision = true;
    REQUIRE(sv.isValid() == false);
    REQUIRE(sv.describe().find("collision") != std::string::npos);
}

TEST_CASE("StepValidation invalid on extraction failure", "[phase4][types]") {
    StepValidation sv;
    sv.extraction.canExtract = false;
    REQUIRE(sv.isValid() == false);
    REQUIRE(sv.describe().find("extraction") != std::string::npos);
}

// ===== Phase4Output Tests =====

TEST_CASE("Phase4Output default is failure", "[phase4][types]") {
    Phase4Output out;
    REQUIRE(out.success == false);
    REQUIRE(out.allStepsValid == false);
    REQUIRE(out.stepResults.empty());
}

TEST_CASE("Phase4Output generateSummary", "[phase4][types]") {
    Phase4Output out;
    out.success = true;
    out.collisionsDetected = 0;
    out.graspFailures = 0;
    out.validationTimeMs = 5.3;

    StepValidation sv;
    sv.stepIndex = 0;
    sv.bendId = 0;
    out.stepResults.push_back(sv);

    std::string summary = out.generateSummary();
    REQUIRE(summary.find("PASS") != std::string::npos);
    REQUIRE(summary.find("1") != std::string::npos); // 1 step
}

// ===== ValidatorConfig Tests =====

TEST_CASE("ValidatorConfig defaults", "[phase4][types]") {
    ValidatorConfig cfg;
    REQUIRE(cfg.minGripArea == 100.0);
    REQUIRE(cfg.maxTorque == 10.0);
    REQUIRE(cfg.collisionMargin == 2.0);
    REQUIRE(cfg.enableSpringback == true);
    REQUIRE(cfg.materialThickness == 1.5);
}

// ===== SpringbackData Tests =====

TEST_CASE("SpringbackData defaults", "[phase4][types]") {
    SpringbackData sd;
    REQUIRE(sd.bendId == -1);
    REQUIRE(sd.targetAngle == 0.0);
    REQUIRE(sd.compensatedAngle == 0.0);
    REQUIRE(sd.springbackAngle == 0.0);
}
