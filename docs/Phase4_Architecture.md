# Phase 4: Physics Validator - Architecture

## Overview

Phase 4 is the **Gatekeeper** of the OpenPanelCAM pipeline. No bend sequence reaches the machine without passing physical validation. It takes Phase 3 output (optimized bend sequence) and validates each step against real-world physics constraints.

## Architecture

```
SequenceValidator (main pipeline)
├── SweptVolumeGenerator
│   └── Estimates AABB of arc swept by flange during bending
├── CollisionDetector (hierarchical)
│   ├── Level 1: AABB overlap with safety margin
│   └── Level 2: OBB/SAT refinement (15-axis)
├── BentState
│   └── Tracks occupied volumes of all bent flanges
├── GraspPhysicsEngine
│   ├── Grip area validation
│   ├── Center of mass distance check
│   ├── Torque calculation (mass * g * COM_distance)
│   └── Shear force validation
├── ExtractionChecker
│   └── Vertical tool extraction path check
├── SpringbackCompensator
│   └── Empirical model: baseDeg + perMm * thickness
└── Feedback Generator
    └── Actionable suggestions for failed validations
```

## Pipeline Flow

For each bend in sequence:
1. **SweptVolume**: Generate AABB bounding the arc swept by the flange
2. **Collision**: Check swept volume against all previously bent flanges
3. **Grasp**: Validate grip area, COM distance, torque, shear force
4. **Extraction**: Verify tool can extract vertically after bending
5. **Springback**: Compute compensation angle for material springback
6. **Feedback**: Generate suggestions if any check fails
7. **Update State**: Add bent flange to BentState for next step

## Key Algorithms

### AABB Overlap (O(1))
6-axis separation test. Two boxes overlap iff they overlap on all 3 axes.

### OBB/SAT (15-axis)
Separating Axis Theorem with 15 potential separating axes:
- 3 face normals from OBB A
- 3 face normals from OBB B
- 9 cross products (A_i x B_j)

Early exit on first separating axis found.

### Swept Volume Estimation
Conservative AABB bounding the arc swept by a flange rotating around the bend line:
- Arc radius = flange length
- Extent per axis considers bend direction, normal, and position

### Grasp Physics
- **Grip area**: baseArea - sum(flangeArea * sin(angle)) per bent flange
- **Torque**: totalMass * g * COM_distance (N*m)
- **Shear**: (sigma_y * L * t^2) / (4 * W) * 0.1 (10% of V-bend force)

### Springback Compensation
Empirical model scaled by angle:
```
springback = (baseDeg + perMm * thickness) * (angle / 90)
compensatedAngle = targetAngle + springback
```

## Configuration (ValidatorConfig)

| Parameter | Default | Description |
|-----------|---------|-------------|
| minGripArea | 100 mm^2 | Minimum vacuum grip contact area |
| maxComDistance | 50 mm | Max COM distance from grip center |
| maxTorque | 10 N*m | Max torque about grip point |
| maxShearForce | 200 N | Max lateral shear force |
| frictionCoeff | 0.3 | Vacuum grip friction coefficient |
| collisionMargin | 2 mm | Safety margin for AABB expansion |
| useOBB | true | Enable OBB refinement after AABB |
| materialDensity | 7.85e-6 kg/mm^3 | Steel density |
| materialThickness | 1.5 mm | Sheet metal thickness |
| yieldStrength | 200 MPa | Material yield strength |
| enableSpringback | true | Enable springback compensation |
| springbackBaseDeg | 2.0 deg | Base springback angle |
| springbackPerMm | 0.5 deg/mm | Additional springback per mm thickness |

## API Reference

### SequenceValidator
```cpp
class SequenceValidator {
    explicit SequenceValidator(const ValidatorConfig& config = ValidatorConfig());
    Phase4Output validate(const Phase3Output& sequence,
                          const std::vector<BendFeature>& bends);
    StepValidation validateStep(int stepIndex,
                                const BendFeature& bend,
                                BentState& state);
    void setConfig(const ValidatorConfig& config);
};
```

### CollisionDetector
```cpp
class CollisionDetector {
    explicit CollisionDetector(const ValidatorConfig& config = ValidatorConfig());
    CollisionResult checkStep(const SweptVolume& swept,
                              const BentState& bentState) const;
    CollisionResult checkAgainstVolumes(const SweptVolume& swept,
                                        const std::vector<AABB>& obstacles) const;
};
```

### GraspPhysicsEngine
```cpp
class GraspPhysicsEngine {
    explicit GraspPhysicsEngine(const ValidatorConfig& config = ValidatorConfig());
    GraspValidation validate(const BentState& state,
                             const BendFeature& nextBend) const;
    double computeGripArea(...) const;
    double computeComDistance(...) const;
    double computeTorque(...) const;
    double computeShearForce(...) const;
};
```

### ExtractionChecker
```cpp
class ExtractionChecker {
    ExtractionResult check(const BendFeature& bend,
                           const BentState& stateAfter) const;
    bool canExtractVertically(const AABB& toolPosition,
                              const std::vector<AABB>& obstacles) const;
};
```

### SpringbackCompensator
```cpp
class SpringbackCompensator {
    SpringbackData compensate(const BendFeature& bend) const;
    std::vector<SpringbackData> compensateAll(
        const std::vector<BendFeature>& bends) const;
};
```

## Performance

| Scenario | Target | Actual |
|----------|--------|--------|
| Single step | < 1 ms | < 0.1 ms |
| 5-bend sequence | < 10 ms | < 1 ms |
| 10-bend sequence | < 100 ms | < 5 ms |
| 20-bend sequence | < 500 ms | < 10 ms |
| Collision check (50 flanges) | < 1 ms | < 0.01 ms |
| Springback (100 bends) | < 10 ms | < 1 ms |

## Test Coverage

| Module | Tests | Assertions |
|--------|-------|------------|
| Core Types (AABB, OBB, SAT) | 26 | 60 |
| SweptVolume + BentState + CollisionDetector | 17 | 41 |
| GraspPhysicsEngine | 16 | 23 |
| ExtractionChecker + SpringbackCompensator | 14 | 34 |
| SequenceValidator + Feedback | 19 | 55 |
| Edge Cases | 13 | 35 |
| Performance | 7 | 12 |
| Integration (Phase 2->3->4) | 5 | 28 |
| **Total Phase 4** | **117** | **288** |

## Dependencies

- Phase 2: ConstraintSolver, Phase2Output, GraspConstraint
- Phase 3: Sequencer, Phase3Output, SequenceAction
- Phase 1 Mock: BendFeature (simplified geometry)
- NO OpenCASCADE dependency
- C++17, Catch2 v3

## File Structure

```
include/openpanelcam/phase4/
├── types.h              # AABB, OBB, CollisionResult, ValidatorConfig, etc.
├── swept_volume.h       # SweptVolume, SweptVolumeGenerator
├── bent_state.h         # BentFlange, BentState
├── collision_detector.h # CollisionDetector
├── grasp_physics.h      # GraspPhysicsEngine
├── extraction_checker.h # ExtractionChecker, SpringbackCompensator
└── sequence_validator.h # SequenceValidator (main pipeline)

src/phase4/
├── collision_primitives.cpp  # OBB::overlaps() SAT implementation
├── swept_volume.cpp          # Swept AABB estimation
├── bent_state.cpp            # Occupied volume tracking
├── collision_detector.cpp    # Hierarchical collision check
├── grasp_physics.cpp         # Grip area, COM, torque, shear
├── extraction_checker.cpp    # Tool extraction + springback
└── sequence_validator.cpp    # Main validation pipeline + feedback

tests/phase4/
├── test_types.cpp              # AABB, OBB, SAT tests
├── test_collision.cpp          # Swept volume, bent state, collision
├── test_grasp_physics.cpp      # Grasp physics tests
├── test_extraction.cpp         # Extraction + springback tests
├── test_sequence_validator.cpp # Validator pipeline + feedback
├── test_phase4_edge_cases.cpp  # Edge case scenarios
├── test_phase4_performance.cpp # Performance benchmarks
└── test_integration.cpp        # Full Phase 2->3->4 pipeline
```
