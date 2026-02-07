# Phase 4: Physics Validator Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Validate bend sequences with collision detection, grasp physics, tool extraction checks, and springback compensation. Phase 4 is the "Gatekeeper" — no sequence reaches the machine without passing validation.

**Architecture:** Build a validation pipeline that takes Phase 3 output (bend sequence) and validates each step physically. Use simplified geometry (no OpenCASCADE dependency) with analytical collision detection, physics-based grip validation, and empirical springback models.

**Key Decision:** Phase 4 does NOT depend on OpenCASCADE. We use the same mock BendFeature from Phase 2/3. This allows:
- Development without OCCT setup complexity
- Fast validation (~ms per step)
- Easy testing with mock data
- OCCT integration can be added later as Phase 4.5

**Tech Stack:** C++17, Eigen3 (optional), Phase 2/3 outputs, Catch2 tests

---

## Prerequisites

- Phase 2 complete (40/40 tasks) ✓
- Phase 3 complete (116 tests) ✓
- Phase 3 merged to master ✓
- Working directory: `E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller`

---

## Module Overview

```
Phase 4 Physics Validator
├── Core Types & Config (Tasks 1-3)
│   ├── ValidationResult
│   ├── CollisionResult
│   ├── GraspValidation
│   └── ValidatorConfig
│
├── Swept Volume (Tasks 4-6)
│   ├── SweptVolume (AABB representation)
│   ├── SweptVolumeGenerator
│   └── SweptVolumeCache
│
├── Collision Detection (Tasks 7-10)
│   ├── AABB overlap test
│   ├── OBB overlap test (SAT)
│   ├── CollisionDetector (hierarchical)
│   └── Bent flange tracking
│
├── Grasp Physics (Tasks 11-14)
│   ├── AreaValidator
│   ├── CenterOfMassChecker
│   ├── TorqueValidator
│   └── GraspPhysicsEngine
│
├── Tool Extraction (Tasks 15-17)
│   ├── ExtractionPathChecker
│   ├── TrapDetector
│   └── SpringbackCompensator
│
├── Validator Pipeline (Tasks 18-20)
│   ├── StepValidator (single step)
│   ├── SequenceValidator (full sequence)
│   └── Phase4Output + feedback
│
└── Testing & Docs (Tasks 21-24)
    ├── Unit tests per module
    ├── Integration tests
    ├── Performance benchmarks
    └── Documentation
```

---

## Task 1: Core Types and ValidationResult

**Goal:** Define all Phase 4 data structures.

**Files:**
- Create: `include/openpanelcam/phase4/types.h`
- Create: `tests/phase4/test_types.cpp`
- Create: `tests/phase4/CMakeLists.txt`

### Step 1.1: Create Phase 4 types header

```cpp
// include/openpanelcam/phase4/types.h
#pragma once

#include <vector>
#include <string>
#include <cstdint>

namespace openpanelcam {
namespace phase4 {

/**
 * @brief 3D axis-aligned bounding box for collision detection
 */
struct AABB {
    double minX = 0, minY = 0, minZ = 0;
    double maxX = 0, maxY = 0, maxZ = 0;

    bool overlaps(const AABB& other) const;
    double volume() const;
    AABB expand(double margin) const;
    static AABB merge(const AABB& a, const AABB& b);
};

/**
 * @brief Oriented bounding box for tighter collision
 */
struct OBB {
    double centerX = 0, centerY = 0, centerZ = 0;
    double halfExtentX = 0, halfExtentY = 0, halfExtentZ = 0;
    // Rotation matrix (3x3 stored as array)
    double axes[9] = {1,0,0, 0,1,0, 0,0,1};

    bool overlaps(const OBB& other) const; // SAT test
};

/**
 * @brief Result of collision check for one step
 */
enum class CollisionType {
    NONE,
    SWEPT_VS_FIXED,      // Moving flange hits stationary geometry
    TOOL_VS_PART,         // Tool collides with part
    PART_VS_MACHINE       // Part collides with machine frame
};

struct CollisionResult {
    bool hasCollision = false;
    CollisionType type = CollisionType::NONE;
    int bendId = -1;              // Which bend caused collision
    int collidingBendId = -1;     // Which bent flange was hit
    double penetrationDepth = 0.0;
    std::string description;
};

/**
 * @brief Grasp physics validation result
 */
struct GraspValidation {
    bool areaValid = true;
    bool comValid = true;
    bool torqueValid = true;
    bool shearValid = true;

    double gripArea = 0.0;
    double comDistance = 0.0;      // COM distance from grip center
    double torque = 0.0;          // N·m
    double shearForce = 0.0;      // N

    bool isValid() const { return areaValid && comValid && torqueValid && shearValid; }
    std::string describe() const;
};

/**
 * @brief Tool extraction check result
 */
struct ExtractionResult {
    bool canExtract = true;
    int trappedAtBend = -1;
    std::string description;
};

/**
 * @brief Springback compensation data
 */
struct SpringbackData {
    int bendId = -1;
    double targetAngle = 0.0;
    double compensatedAngle = 0.0;  // Actual machine angle
    double springbackAngle = 0.0;   // Compensation amount
};

/**
 * @brief Single step validation result
 */
struct StepValidation {
    int stepIndex = -1;
    int bendId = -1;

    CollisionResult collision;
    GraspValidation grasp;
    ExtractionResult extraction;
    SpringbackData springback;

    bool isValid() const {
        return !collision.hasCollision &&
               grasp.isValid() &&
               extraction.canExtract;
    }

    std::string describe() const;
};

/**
 * @brief Complete Phase 4 output
 */
struct Phase4Output {
    bool success = false;
    bool allStepsValid = false;

    std::vector<StepValidation> stepResults;
    std::vector<SpringbackData> springbackTable;

    // Feedback for failed steps
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::vector<std::string> suggestions;

    // Statistics
    double validationTimeMs = 0.0;
    int collisionsDetected = 0;
    int graspFailures = 0;
    int extractionFailures = 0;

    std::string generateSummary() const;
};

/**
 * @brief Configuration for physics validator
 */
struct ValidatorConfig {
    // Grip constraints
    double minGripArea = 100.0;       // mm²
    double maxComDistance = 50.0;      // mm from grip center
    double maxTorque = 10.0;          // N·m
    double maxShearForce = 50.0;      // N
    double frictionCoeff = 0.3;       // Vacuum grip friction

    // Collision margins
    double collisionMargin = 2.0;     // mm safety margin
    bool useOBB = true;               // Use OBB after AABB

    // Material properties
    double materialDensity = 7.85e-6; // kg/mm³ (steel)
    double materialThickness = 1.5;   // mm
    double yieldStrength = 200.0;     // MPa

    // Springback
    bool enableSpringback = true;
    double springbackBaseDeg = 2.0;   // Base springback for steel
    double springbackPerMm = 0.5;     // Additional per mm thickness
};

} // namespace phase4
} // namespace openpanelcam
```

### Step 1.2: Create test file with basic tests

```cpp
// tests/phase4/test_types.cpp
// Test AABB overlap, OBB basics, ValidationResult defaults, config
```

### Step 1.3: Create CMakeLists for Phase 4 tests

### Step 1.4: Run test, commit
```bash
git commit -m "feat(phase4): add core types (AABB, OBB, ValidationResult, Config)"
```

---

## Task 2: AABB Implementation and Tests

**Goal:** Implement AABB overlap detection.

**Files:**
- Modify: `include/openpanelcam/phase4/types.h` (add inline methods)
- Modify: `tests/phase4/test_types.cpp`

### Step 2.1: Implement AABB methods
- `overlaps()`: 6-axis separation test
- `volume()`: (max-min) product
- `expand()`: add margin to all sides
- `merge()`: union of two AABBs

### Step 2.2: Tests for all AABB operations

### Step 2.3: Commit
```bash
git commit -m "feat(phase4): implement AABB overlap detection"
```

---

## Task 3: OBB Implementation (SAT)

**Goal:** Implement OBB overlap using Separating Axis Theorem.

**Files:**
- Create: `src/phase4/collision_primitives.cpp`
- Modify: `tests/phase4/test_types.cpp`

### Step 3.1: Implement OBB::overlaps() with SAT
- 15 axes: 3 from each OBB face normal + 9 cross products
- Early exit on first separating axis

### Step 3.2: Tests

### Step 3.3: Commit
```bash
git commit -m "feat(phase4): implement OBB overlap with Separating Axis Theorem"
```

---

## Task 4: SweptVolume Generator

**Goal:** Generate simplified swept volumes for bend operations.

**Files:**
- Create: `include/openpanelcam/phase4/swept_volume.h`
- Create: `src/phase4/swept_volume.cpp`
- Create: `tests/phase4/test_swept_volume.cpp`

### Step 4.1: SweptVolume class
```cpp
struct SweptVolume {
    AABB aabb;           // Fast broad-phase box
    OBB obb;             // Tighter fit
    int bendId = -1;
    double sweepAngle = 0.0;
};

class SweptVolumeGenerator {
public:
    SweptVolume generate(const phase1::BendFeature& bend) const;
    AABB estimateSweptAABB(const phase1::BendFeature& bend) const;
};
```

### Step 4.2: Estimate swept volume from bend geometry
- Use bend angle, length, and position to compute AABB
- Flange rotates around bend line → swept arc
- AABB = bounding box of the arc

### Step 4.3: Tests, commit
```bash
git commit -m "feat(phase4): implement SweptVolumeGenerator with AABB estimation"
```

---

## Task 5: Bent Flange Tracker

**Goal:** Track the position of bent flanges as bending progresses.

**Files:**
- Create: `include/openpanelcam/phase4/bent_state.h`
- Create: `src/phase4/bent_state.cpp`
- Create: `tests/phase4/test_bent_state.cpp`

### Step 5.1: BentState class
```cpp
struct BentFlange {
    int bendId = -1;
    AABB occupiedVolume;
    double angle = 0.0;
};

class BentState {
public:
    void addBentFlange(const phase1::BendFeature& bend);
    const std::vector<BentFlange>& getBentFlanges() const;
    std::vector<AABB> getAllOccupiedVolumes() const;
    void reset();
};
```

### Step 5.2: Tests, commit
```bash
git commit -m "feat(phase4): implement BentState for tracking bent flanges"
```

---

## Task 6: CollisionDetector (Hierarchical)

**Goal:** Hierarchical collision detection: AABB → OBB → report.

**Files:**
- Create: `include/openpanelcam/phase4/collision_detector.h`
- Create: `src/phase4/collision_detector.cpp`
- Create: `tests/phase4/test_collision.cpp`

### Step 6.1: CollisionDetector class
```cpp
class CollisionDetector {
public:
    explicit CollisionDetector(const ValidatorConfig& config = ValidatorConfig());

    CollisionResult checkStep(const SweptVolume& swept,
                              const BentState& bentState) const;

    std::vector<CollisionResult> checkAll(
        const std::vector<SweptVolume>& swepts,
        const BentState& bentState) const;
};
```

### Step 6.2: Hierarchical check
1. AABB overlap test (fast rejection)
2. If AABB overlaps and config.useOBB → OBB test
3. Report collision with details

### Step 6.3: Tests with known collision/no-collision scenarios

### Step 6.4: Commit
```bash
git commit -m "feat(phase4): implement hierarchical CollisionDetector (AABB → OBB)"
```

---

## Task 7-8: Grasp Physics Engine

**Goal:** Validate grip stability: area, COM, torque, shear.

**Files:**
- Create: `include/openpanelcam/phase4/grasp_physics.h`
- Create: `src/phase4/grasp_physics.cpp`
- Create: `tests/phase4/test_grasp_physics.cpp`

### Step 7.1: GraspPhysicsEngine class
```cpp
class GraspPhysicsEngine {
public:
    explicit GraspPhysicsEngine(const ValidatorConfig& config);

    GraspValidation validate(const BentState& state,
                             const phase1::BendFeature& nextBend,
                             const std::vector<phase2::GraspConstraint>& constraints) const;

    double computeGripArea(const BentState& state) const;
    double computeComDistance(const BentState& state) const;
    double computeTorque(const BentState& state, double gripArea) const;
    double computeShear(const BentState& state) const;
};
```

### Step 7.2-8: Implement each check, tests, commit
```bash
git commit -m "feat(phase4): implement GraspPhysicsEngine (area, COM, torque, shear)"
```

---

## Task 9-10: Tool Extraction Checker

**Goal:** Check if tool can be extracted after bending.

**Files:**
- Create: `include/openpanelcam/phase4/extraction_checker.h`
- Create: `src/phase4/extraction_checker.cpp`
- Create: `tests/phase4/test_extraction.cpp`

### Step 9.1: ExtractionChecker class
```cpp
class ExtractionChecker {
public:
    ExtractionResult check(const phase1::BendFeature& bend,
                           const BentState& stateAfter) const;
    bool canExtractVertically(const AABB& toolPosition,
                              const std::vector<AABB>& obstacles) const;
};
```

### Step 9.2: Springback compensator
```cpp
class SpringbackCompensator {
public:
    explicit SpringbackCompensator(const ValidatorConfig& config);

    SpringbackData compensate(const phase1::BendFeature& bend) const;
    std::vector<SpringbackData> compensateAll(
        const std::vector<phase1::BendFeature>& bends) const;
};
```

### Step 9.3-10: Tests, commit
```bash
git commit -m "feat(phase4): implement ExtractionChecker and SpringbackCompensator"
```

---

## Task 11-12: SequenceValidator (Main Pipeline)

**Goal:** Validate entire bend sequence step by step.

**Files:**
- Create: `include/openpanelcam/phase4/sequence_validator.h`
- Create: `src/phase4/sequence_validator.cpp`
- Create: `tests/phase4/test_sequence_validator.cpp`

### Step 11.1: SequenceValidator class
```cpp
class SequenceValidator {
public:
    explicit SequenceValidator(const ValidatorConfig& config = ValidatorConfig());

    Phase4Output validate(const phase3::Phase3Output& sequence,
                          const phase2::Phase2Output& constraints,
                          const std::vector<phase1::BendFeature>& bends);

    void setConfig(const ValidatorConfig& config);
};
```

### Step 11.2: Pipeline logic
```
For each step in sequence:
  1. Generate swept volume for this bend
  2. Check collision with all bent flanges
  3. Validate grasp physics
  4. Check tool extraction
  5. Compute springback compensation
  6. Update bent state
  7. Record step result
```

### Step 11.3-12: Integration tests Phase 2 → 3 → 4

### Step 11.4: Commit
```bash
git commit -m "feat(phase4): implement SequenceValidator pipeline"
```

---

## Task 13-14: Feedback Generator

**Goal:** Generate actionable feedback when validation fails.

**Files:**
- Modify: `src/phase4/sequence_validator.cpp`
- Modify: `tests/phase4/test_sequence_validator.cpp`

### Step 13.1: Feedback logic
```cpp
// When collision detected:
suggestions.push_back("Reorder bend " + id + " before bend " + collidingId);

// When grasp fails:
suggestions.push_back("Add repositioning before bend " + id);

// When extraction fails:
suggestions.push_back("Bend " + id + " creates tool trap - reverse order");
```

### Step 13.2-14: Tests, commit
```bash
git commit -m "feat(phase4): add validation feedback and correction suggestions"
```

---

## Task 15-17: Edge Cases and Performance Tests

**Goal:** Test edge cases and benchmark performance.

**Files:**
- Create: `tests/phase4/test_edge_cases.cpp`
- Create: `tests/phase4/test_performance.cpp`

### Test cases:
- Empty sequence → pass
- Single bend → always valid
- L-bracket (2 bends) → no collision
- U-channel (3 chain) → valid
- Box (4 bends) → may have extraction trap
- 10 bends performance → < 100ms
- All same direction → no collision
- Opposing flanges → collision detected

### Performance target:
- < 1ms per step validation
- < 100ms for 10-step sequence
- < 500ms for 20-step sequence

```bash
git commit -m "test(phase4): add edge case and performance tests"
```

---

## Task 18-20: Integration Tests and Documentation

### Task 18: Full Pipeline Test (Phase 2 → 3 → 4)
```cpp
TEST_CASE("Full pipeline: ConstraintSolver → Sequencer → Validator") {
    // Phase 2
    ConstraintSolver solver;
    auto p2 = solver.solve(bends);

    // Phase 3
    Sequencer seq;
    auto p3 = seq.sequence(p2, bends);

    // Phase 4
    SequenceValidator validator;
    auto p4 = validator.validate(p3, p2, bends);

    REQUIRE(p4.success == true);
}
```

### Task 19: Update ROADMAP progress
### Task 20: Create `docs/Phase4_Architecture.md`

```bash
git commit -m "docs(phase4): add architecture documentation and update roadmap"
```

---

## Summary

### Total Tasks: 20

| Task Range | Component | Description |
|------------|-----------|-------------|
| 1-3 | Core Types | AABB, OBB, ValidationResult, ValidatorConfig |
| 4-5 | Swept Volume | Generator, BentState tracker |
| 6 | Collision | Hierarchical AABB → OBB detector |
| 7-8 | Grasp Physics | Area, COM, torque, shear validation |
| 9-10 | Tool Extract | Extraction check, springback compensation |
| 11-12 | Pipeline | SequenceValidator main class |
| 13-14 | Feedback | Suggestions for failed validations |
| 15-17 | Testing | Edge cases, performance benchmarks |
| 18-20 | Integration | Full pipeline test, documentation |

### Key Algorithms

1. **AABB Overlap** - 6-axis separation test, O(1)
2. **OBB/SAT** - 15-axis Separating Axis Theorem
3. **Swept Volume Estimation** - Arc bounding from bend geometry
4. **Torque/Shear Physics** - Force balance for grip stability
5. **Springback Compensation** - Empirical material model

### Performance Targets

- < 1ms per step validation
- < 100ms for typical 10-bend sequence
- AABB rejects 90%+ non-collisions
- Zero false negatives (never miss a real collision)

### Dependencies

- Phase 2 output (GraspConstraints, ABAConstraints)
- Phase 3 output (Phase3Output with bendSequence)
- NO OpenCASCADE dependency (uses mock BendFeature)
- Optional: Eigen3 for matrix math (OBB rotation)

---

**Plan complete and saved to `docs/plans/2026-02-07-phase4-physics-validator.md`**
