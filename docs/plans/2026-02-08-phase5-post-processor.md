# Phase 5: Post-Processor Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Generate machine-readable output (PB-XML for Salvagnini P4 Panel Bender) and visualization data (JSON for HMI). Phase 5 is "The Translator" — converting validated sequences into machine instructions.

**Architecture:** Two output generators (XML + JSON) fed by a unified instruction builder that transforms Phase 4 validated output into machine-level commands. Includes validation layer to verify output correctness.

**Key Decision:** Use PugiXML for XML generation and nlohmann/json for JSON — both already in vcpkg/CMakeLists. No OpenCASCADE dependency. GLB/GLTF export deferred to future (Phase 5.5).

**Tech Stack:** C++17, PugiXML, nlohmann/json, Phase 2/3/4 outputs, Catch2 tests

---

## Prerequisites

- Phase 2 complete (40 tasks) ✓
- Phase 3 complete (116 tests) ✓
- Phase 4 complete (117 tests) ✓
- Phase 3+4 merged to master ✓
- nlohmann_json and pugixml in CMakeLists ✓
- Working directory: `E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller`

---

## Module Overview

```
Phase 5 Post-Processor
├── Core Types & Config (Tasks 1-2)
│   ├── MachineInstruction (Bend, Rotate, ABASetup, Reposition)
│   ├── MachineProgram (header + instructions + metadata)
│   ├── VisualizationData (bones, keyframes)
│   └── PostProcessorConfig
│
├── Instruction Builder (Tasks 3-5)
│   ├── InstructionBuilder (Phase4Output → MachineInstruction[])
│   ├── BendInstruction generation (angle, force, springback)
│   ├── RotationInstruction generation
│   ├── ABASetup instruction generation
│   └── Reposition instruction generation
│
├── XML Generator (Tasks 6-8)
│   ├── PBXMLGenerator (MachineProgram → XML string)
│   ├── Job header (material, tooling, job info)
│   ├── Process sequence (steps with parameters)
│   ├── Adaptive control parameters
│   └── Metadata (cycle time, bend count)
│
├── JSON/VDM Generator (Tasks 9-11)
│   ├── VDMGenerator (VisualizationData → JSON string)
│   ├── Bone hierarchy builder
│   ├── Keyframe animation data
│   └── Scene configuration (camera, lights)
│
├── Validation Layer (Tasks 12-14)
│   ├── OutputValidator (machine limits, sanity checks)
│   ├── Angle limits check (-135° to +135°)
│   ├── Force capacity check
│   ├── Cycle time consistency
│   └── Step ID uniqueness
│
├── PostProcessor Pipeline (Tasks 15-16)
│   ├── PostProcessor (main class)
│   ├── Full pipeline: Phase4Output → XML + JSON + validation
│   └── Phase5Output with all outputs
│
└── Testing & Docs (Tasks 17-20)
    ├── Edge case tests
    ├── Performance benchmarks
    ├── Integration tests (Phase 2→3→4→5)
    └── Documentation
```

---

## Task 1: Core Types and Data Structures

**Goal:** Define all Phase 5 data structures.

**Files:**
- Create: `include/openpanelcam/phase5/types.h`
- Create: `tests/phase5/test_types.cpp`
- Create: `tests/phase5/CMakeLists.txt`

### Step 1.1: Core types header

```cpp
// include/openpanelcam/phase5/types.h
#pragma once
#include <string>
#include <vector>
#include <cstdint>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Type of machine instruction
 */
enum class InstructionType {
    BEND,
    ROTATE,
    ABA_SETUP,
    REPOSITION,
    GRIP,
    RELEASE
};

/**
 * @brief Single machine instruction
 */
struct MachineInstruction {
    int stepId = -1;
    InstructionType type = InstructionType::BEND;

    // Bend parameters
    int bendId = -1;
    double targetAngle = 0.0;
    double compensatedAngle = 0.0;  // After springback
    double springbackAngle = 0.0;
    double bendForce = 0.0;         // kN
    double bendLength = 0.0;        // mm

    // Bend line position
    double startX = 0.0, startY = 0.0;
    double endX = 0.0, endY = 0.0;

    // Rotation parameters
    int rotationAngle = 0;          // 0, 90, 180, 270

    // ABA parameters
    uint16_t abaConfig = 0;
    std::vector<double> segmentPositions;
    std::vector<double> segmentWidths;

    // Motion profile
    double duration = 0.0;          // seconds
    std::string description;
};

/**
 * @brief Complete machine program
 */
struct MachineProgram {
    // Header
    std::string jobId;
    std::string partName = "UNNAMED";
    std::string createdBy = "OpenPanelCAM";
    std::string timestamp;
    std::string machineModel = "P4";

    // Material
    double materialThickness = 1.5;
    std::string materialType = "AISI304";
    double yieldStrength = 200.0;

    // Tooling
    std::string upperTool = "Standard";
    std::string lowerTool = "ABA";

    // Instructions
    std::vector<MachineInstruction> instructions;

    // Metadata
    double totalCycleTime = 0.0;
    int bendCount = 0;
    int repositionCount = 0;
    int rotationCount = 0;

    // Adaptive control
    bool adaptiveEnabled = true;
    double angleTolerance = 0.5;     // degrees
    double forceLimit = 25.0;        // kN
    bool speedAdaptive = true;
};

/**
 * @brief Bone for skeletal animation
 */
struct AnimationBone {
    int index = -1;
    std::string name;
    int parentIndex = -1;  // -1 for root
    double originX = 0, originY = 0, originZ = 0;
    double axisX = 0, axisY = 0, axisZ = 1;
};

/**
 * @brief Keyframe for animation
 */
struct AnimationKeyframe {
    double time = 0.0;       // seconds
    int boneIndex = -1;
    double rotationAngle = 0.0;
    double translationX = 0, translationY = 0, translationZ = 0;
    std::string actionDescription;
};

/**
 * @brief Visualization data for HMI
 */
struct VisualizationData {
    // Scene
    double cameraX = 0, cameraY = -500, cameraZ = 300;
    double lookAtX = 0, lookAtY = 0, lookAtZ = 0;
    double fov = 45.0;

    // Bones
    std::vector<AnimationBone> bones;

    // Keyframes
    std::vector<AnimationKeyframe> keyframes;

    // Metadata
    std::string partName;
    double cycleTime = 0.0;
    int bendCount = 0;
};

/**
 * @brief Complete Phase 5 output
 */
struct Phase5Output {
    bool success = false;

    MachineProgram program;
    VisualizationData visualization;

    std::string xmlOutput;       // Generated PB-XML
    std::string jsonOutput;      // Generated VDM JSON

    // Validation
    bool xmlValid = false;
    bool jsonValid = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    double generationTimeMs = 0.0;
};

/**
 * @brief Configuration for post-processor
 */
struct PostProcessorConfig {
    // Machine limits
    double maxBendAngle = 135.0;    // degrees
    double minBendAngle = -135.0;   // degrees
    double maxForce = 25.0;         // kN
    double maxPartLength = 2500.0;  // mm
    double maxPartWidth = 1250.0;   // mm

    // Output options
    bool generateXML = true;
    bool generateJSON = true;
    bool includeAdaptiveControl = true;
    bool includeComments = true;

    // Material defaults
    double defaultThickness = 1.5;
    std::string defaultMaterial = "AISI304";

    // Animation
    double animationFPS = 30.0;
};

} // namespace phase5
} // namespace openpanelcam
```

### Step 1.2: Basic tests for type construction and defaults

### Step 1.3: CMakeLists for Phase 5 tests

### Step 1.4: Run test, commit
```bash
git commit -m "feat(phase5): add core types (MachineInstruction, MachineProgram, VisualizationData)"
```

---

## Task 2: Phase 5 Library Setup

**Goal:** Set up Phase 5 CMake library with PugiXML and nlohmann_json dependencies.

**Files:**
- Modify: `src/phase5/CMakeLists.txt`
- Modify: `CMakeLists.txt` (enable phase5)

### Step 2.1: Update src/phase5/CMakeLists.txt from INTERFACE to STATIC

### Step 2.2: Link PugiXML and nlohmann_json

### Step 2.3: Verify build, commit
```bash
git commit -m "feat(phase5): setup Phase 5 library with PugiXML and nlohmann_json"
```

---

## Task 3-4: Instruction Builder

**Goal:** Transform Phase 4 validated output into machine instructions.

**Files:**
- Create: `include/openpanelcam/phase5/instruction_builder.h`
- Create: `src/phase5/instruction_builder.cpp`
- Create: `tests/phase5/test_instruction_builder.cpp`

### Step 3.1: InstructionBuilder class
```cpp
class InstructionBuilder {
public:
    explicit InstructionBuilder(const PostProcessorConfig& config = PostProcessorConfig());

    MachineProgram build(const phase4::Phase4Output& validated,
                         const phase3::Phase3Output& sequence,
                         const std::vector<phase1::BendFeature>& bends);

    MachineInstruction buildBendInstruction(int stepId,
                                             const phase1::BendFeature& bend,
                                             const phase4::SpringbackData& springback);

    MachineInstruction buildRotationInstruction(int stepId,
                                                 const phase3::SequenceAction& action);

    MachineInstruction buildABAInstruction(int stepId,
                                            const phase3::SequenceAction& action);

    MachineInstruction buildRepositionInstruction(int stepId,
                                                    const phase3::SequenceAction& action);
};
```

### Step 3.2: Build logic
- Iterate Phase 3 actions
- For BEND: use Phase 4 springback data, compute force
- For ROTATE: extract angle from action
- For ABA_CHANGE: extract config from action
- For REPOSITION: add repo instruction

### Step 3.3: Bend force calculation
```
F = (σ_y × L × t²) / (4 × W)  // kN
W = 8 × t (V-die opening)
```

### Step 4: Tests and commit
```bash
git commit -m "feat(phase5): implement InstructionBuilder (Phase4 → MachineInstructions)"
```

---

## Task 5: Animation Data Builder

**Goal:** Build bone hierarchy and keyframes from bend sequence.

**Files:**
- Create: `include/openpanelcam/phase5/animation_builder.h`
- Create: `src/phase5/animation_builder.cpp`
- Create: `tests/phase5/test_animation_builder.cpp`

### Step 5.1: AnimationBuilder class
```cpp
class AnimationBuilder {
public:
    VisualizationData build(const MachineProgram& program,
                            const std::vector<phase1::BendFeature>& bends);

    std::vector<AnimationBone> buildBoneHierarchy(
        const std::vector<phase1::BendFeature>& bends);

    std::vector<AnimationKeyframe> buildKeyframes(
        const MachineProgram& program);
};
```

### Step 5.2: Bone hierarchy
- Root bone = base face
- Each bend creates a child bone at bend position
- Rotation axis = bend direction

### Step 5.3: Keyframes from instructions
- Cumulative time from instruction durations
- BEND → rotation keyframe on corresponding bone
- ROTATE → rotation keyframe on root bone

### Step 5.4: Tests, commit
```bash
git commit -m "feat(phase5): implement AnimationBuilder (bones + keyframes)"
```

---

## Task 6-7: PB-XML Generator

**Goal:** Generate PB-XML format output for Salvagnini P4.

**Files:**
- Create: `include/openpanelcam/phase5/xml_generator.h`
- Create: `src/phase5/xml_generator.cpp`
- Create: `tests/phase5/test_xml_generator.cpp`

### Step 6.1: PBXMLGenerator class
```cpp
class PBXMLGenerator {
public:
    explicit PBXMLGenerator(const PostProcessorConfig& config = PostProcessorConfig());

    std::string generate(const MachineProgram& program);

private:
    void writeHeader(pugi::xml_node& job, const MachineProgram& program);
    void writeProcessSequence(pugi::xml_node& job, const MachineProgram& program);
    void writeStep(pugi::xml_node& parent, const MachineInstruction& instr);
    void writeAdaptiveControl(pugi::xml_node& parent, const MachineProgram& program);
    void writeMetadata(pugi::xml_node& job, const MachineProgram& program);
};
```

### Step 6.2: XML Schema
```xml
<Job>
  <Header>
    <Material thickness="1.5" type="AISI304"/>
    <Tooling upper="Standard" lower="ABA"/>
    <JobInfo partName="..." program="OpenPanelCAM" timestamp="..."/>
  </Header>
  <ProcessSequence>
    <Step id="1" type="Bend">
      <BendLine><Start x="0" y="50"/><End x="200" y="50"/></BendLine>
      <Angle target="90" compensated="92.3"/>
      <Force>15.5</Force>
      <Springback>2.3</Springback>
      <Duration>2.5</Duration>
    </Step>
    ...
  </ProcessSequence>
  <AdaptiveControl>
    <AngleFeedback enabled="true" tolerance="0.5"/>
    <ForceLimit max="25.0"/>
    <SpeedControl adaptive="true"/>
  </AdaptiveControl>
  <Metadata>
    <TotalCycleTime>45.2</TotalCycleTime>
    <BendCount>6</BendCount>
    <RepositionCount>0</RepositionCount>
  </Metadata>
</Job>
```

### Step 7: Tests (parse generated XML, verify structure)
```bash
git commit -m "feat(phase5): implement PBXMLGenerator (PB-XML format)"
```

---

## Task 8-9: JSON/VDM Generator

**Goal:** Generate JSON visualization data for HMI.

**Files:**
- Create: `include/openpanelcam/phase5/json_generator.h`
- Create: `src/phase5/json_generator.cpp`
- Create: `tests/phase5/test_json_generator.cpp`

### Step 8.1: VDMGenerator class
```cpp
class VDMGenerator {
public:
    explicit VDMGenerator(const PostProcessorConfig& config = PostProcessorConfig());

    std::string generate(const VisualizationData& data);

private:
    nlohmann::json buildScene(const VisualizationData& data);
    nlohmann::json buildSkeleton(const VisualizationData& data);
    nlohmann::json buildAnimation(const VisualizationData& data);
    nlohmann::json buildMetadata(const VisualizationData& data);
};
```

### Step 8.2: JSON Schema
```json
{
  "scene": { "camera": {...}, "lights": [...] },
  "skeleton": { "bones": [...] },
  "animation": { "duration": 45.2, "fps": 30, "keyframes": [...] },
  "metadata": { "partName": "...", "cycleTime": 45.2, "bendCount": 6 }
}
```

### Step 9: Tests (parse generated JSON, verify structure)
```bash
git commit -m "feat(phase5): implement VDMGenerator (JSON visualization data)"
```

---

## Task 10-11: Output Validator

**Goal:** Validate generated output against machine limits.

**Files:**
- Create: `include/openpanelcam/phase5/output_validator.h`
- Create: `src/phase5/output_validator.cpp`
- Create: `tests/phase5/test_output_validator.cpp`

### Step 10.1: OutputValidator class
```cpp
class OutputValidator {
public:
    explicit OutputValidator(const PostProcessorConfig& config = PostProcessorConfig());

    struct ValidationResult {
        bool valid = true;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    ValidationResult validate(const MachineProgram& program);

    bool checkAngleLimits(const MachineInstruction& instr) const;
    bool checkForceLimits(const MachineInstruction& instr) const;
    bool checkPartSize(const MachineProgram& program) const;
    bool checkCycleTimeConsistency(const MachineProgram& program) const;
    bool checkStepIdUniqueness(const MachineProgram& program) const;
};
```

### Step 10.2: Validation checks
- Bend angles within [-135°, +135°]
- Forces within machine capacity
- Part dimensions within machine limits
- Total cycle time = sum of step durations
- No duplicate step IDs

### Step 11: Tests
```bash
git commit -m "feat(phase5): implement OutputValidator (machine limits, sanity checks)"
```

---

## Task 12-13: PostProcessor Pipeline

**Goal:** Main pipeline class combining all Phase 5 components.

**Files:**
- Create: `include/openpanelcam/phase5/post_processor.h`
- Create: `src/phase5/post_processor.cpp`
- Create: `tests/phase5/test_post_processor.cpp`

### Step 12.1: PostProcessor class
```cpp
class PostProcessor {
public:
    explicit PostProcessor(const PostProcessorConfig& config = PostProcessorConfig());

    Phase5Output process(const phase4::Phase4Output& validated,
                         const phase3::Phase3Output& sequence,
                         const std::vector<phase1::BendFeature>& bends);

    void setConfig(const PostProcessorConfig& config);
};
```

### Step 12.2: Pipeline
```
1. InstructionBuilder → MachineProgram
2. AnimationBuilder → VisualizationData
3. OutputValidator → validate MachineProgram
4. PBXMLGenerator → XML string
5. VDMGenerator → JSON string
6. Package into Phase5Output
```

### Step 13: Tests, commit
```bash
git commit -m "feat(phase5): implement PostProcessor pipeline"
```

---

## Task 14-15: Edge Cases and Performance Tests

**Goal:** Test edge cases and benchmark performance.

**Files:**
- Create: `tests/phase5/test_phase5_edge_cases.cpp`
- Create: `tests/phase5/test_phase5_performance.cpp`

### Test cases:
- Empty sequence → minimal valid output
- Single bend → correct XML and JSON
- 10 bends → all instructions present
- Out-of-range angles → validation catches
- Excessive force → validation catches
- Negative angles → correct sign in output
- All step types (bend, rotate, ABA, repo)

### Performance targets:
- < 5ms for XML generation (10 bends)
- < 5ms for JSON generation (10 bends)
- < 20ms for full pipeline (10 bends)

```bash
git commit -m "test(phase5): add edge case and performance tests"
```

---

## Task 16-17: Integration Tests

**Goal:** Full pipeline test Phase 2 → 3 → 4 → 5.

**Files:**
- Create: `tests/phase5/test_integration.cpp`

### Test cases:
```cpp
TEST_CASE("Full pipeline: Phase 2 → 3 → 4 → 5") {
    // Phase 2
    ConstraintSolver solver;
    auto p2 = solver.solve(bends);

    // Phase 3
    Sequencer seq;
    auto p3 = seq.sequence(p2, bends);

    // Phase 4
    SequenceValidator validator;
    auto p4 = validator.validate(p3, bends);

    // Phase 5
    PostProcessor pp;
    auto p5 = pp.process(p4, p3, bends);

    REQUIRE(p5.success == true);
    REQUIRE(!p5.xmlOutput.empty());
    REQUIRE(!p5.jsonOutput.empty());
}
```

```bash
git commit -m "test(phase5): add full pipeline integration tests (Phase 2→3→4→5)"
```

---

## Task 18-20: Documentation

**Files:**
- Create: `docs/Phase5_Architecture.md`

### Content:
- Module overview and pipeline flow
- PB-XML schema reference
- JSON/VDM schema reference
- API reference for all classes
- Machine limits and configuration
- Performance data
- Test coverage summary

```bash
git commit -m "docs(phase5): add architecture documentation"
```

---

## Summary

### Total Tasks: 20

| Task Range | Component | Description |
|------------|-----------|-------------|
| 1-2 | Core Types | MachineInstruction, MachineProgram, VisualizationData, Config |
| 3-4 | Instruction Builder | Phase4Output → MachineInstruction[] |
| 5 | Animation Builder | Bone hierarchy + keyframes |
| 6-7 | XML Generator | PB-XML format for Salvagnini P4 |
| 8-9 | JSON Generator | VDM format for HMI visualization |
| 10-11 | Validation | Machine limits, sanity checks |
| 12-13 | Pipeline | PostProcessor main class |
| 14-15 | Testing | Edge cases, performance benchmarks |
| 16-17 | Integration | Full Phase 2→3→4→5 pipeline |
| 18-20 | Documentation | Architecture docs |

### Key Algorithms

1. **Bend Force Calculation**: F = (σ_y × L × t²) / (4 × W) kN
2. **Bone Hierarchy**: Root = base face, children = flanges at bend positions
3. **Keyframe Generation**: Cumulative time from instruction durations
4. **XML Generation**: PugiXML DOM tree construction
5. **JSON Generation**: nlohmann/json object composition

### Dependencies

- Phase 4 output (validated sequence + springback table)
- Phase 3 output (bend sequence + actions)
- Phase 1 mock (BendFeature geometry)
- PugiXML (XML generation)
- nlohmann/json (JSON generation)
- NO OpenCASCADE dependency

### Performance Targets

- < 5ms XML generation (10 bends)
- < 5ms JSON generation (10 bends)
- < 20ms full pipeline (10 bends)

---

**Plan complete and saved to `docs/plans/2026-02-08-phase5-post-processor.md`**
