# Phase 5: Post-Processor - Architecture

## Overview

Phase 5 is the **Translator** of the OpenPanelCAM pipeline. It converts validated bend sequences (Phase 4 output) into machine-readable formats: PB-XML for the Salvagnini P4 Panel Bender controller and JSON/VDM for HMI visualization. No OpenCASCADE dependency — pure C++17 with optional PugiXML and nlohmann/json.

## Architecture

```
PostProcessor (main pipeline)
├── InstructionBuilder
│   ├── Phase4Output + Phase3Output → MachineProgram
│   ├── BendInstruction (angle, force, springback compensation)
│   ├── RotationInstruction (0°, 90°, 180°, 270°)
│   ├── ABASetupInstruction (segment reconfiguration)
│   └── RepositionInstruction (re-grip)
│
├── AnimationBuilder
│   ├── BendFeatures → Bone hierarchy (root=base, children=flanges)
│   └── MachineProgram → Keyframes (cumulative timeline)
│
├── OutputValidator
│   ├── Angle limits check (±135°)
│   ├── Force capacity check (≤25 kN)
│   ├── Part size limits (2500×1250 mm)
│   ├── Cycle time consistency
│   └── Step ID uniqueness
│
├── PBXMLGenerator
│   ├── Job → Header + ProcessSequence + AdaptiveControl + Metadata
│   ├── PugiXML-based (when HAS_PUGIXML=1)
│   └── Manual string fallback
│
└── VDMGenerator
    ├── Scene + Skeleton + Animation + Metadata → JSON
    ├── nlohmann/json-based (when HAS_NLOHMANN_JSON=1)
    └── Manual string fallback
```

## Pipeline Flow

```
Phase4Output ─┐
Phase3Output ─┼──→ InstructionBuilder ──→ MachineProgram ──┬──→ OutputValidator ──→ errors/warnings
BendFeatures ─┘         │                                  ├──→ PBXMLGenerator ──→ XML string
                         │                                  └──→ AnimationBuilder ──→ VisualizationData
                         │                                                              │
                         │                                                              └──→ VDMGenerator ──→ JSON string
                         │
                         └──→ Phase5Output { program, visualization, xmlOutput, jsonOutput, valid }
```

## Key Algorithms

### Bend Force Calculation

```
F = (σ_y × L × t²) / (4 × W) / 1000   [kN]
W = 8 × t                                [V-die opening, mm]
```

Where:
- σ_y = yield strength (MPa, default 200 for AISI304)
- L = bend length (mm)
- t = material thickness (mm, default 1.5)

### Bone Hierarchy

- **Root bone** (`base`): index 0, parentIndex -1, at origin
- **Flange bones** (`flange_N`): index N+1, parentIndex 0
  - Origin = bend position from BendFeature
  - Rotation axis = bend direction from BendFeature

### Keyframe Generation

Cumulative time from instruction durations:
- `BEND` → rotation keyframe on bone (bendId + 1), angle = targetAngle
- `ROTATE` → rotation keyframe on root bone (index 0)
- `ABA_SETUP` → skipped (no visual effect)
- `REPOSITION` → pause keyframe on root bone

### Springback Compensation

Inherited from Phase 4 SpringbackData:
- `compensatedAngle = targetAngle + springbackAngle`
- Passed through to PB-XML as both `target` and `compensated` attributes

## PB-XML Schema

```xml
<Job>
  <Header>
    <Material thickness="1.5" type="AISI304" yieldStrength="200"/>
    <Tooling upper="Standard" lower="ABA"/>
    <JobInfo partName="BOX001" program="OpenPanelCAM"
             machine="P4" timestamp="2026-02-08T10:00:00"/>
  </Header>
  <ProcessSequence>
    <Step id="1" type="Bend">
      <BendLine>
        <Start x="0" y="50"/>
        <End x="200" y="50"/>
      </BendLine>
      <Angle target="90" compensated="92.75"/>
      <Force>0.94</Force>
      <Springback>2.75</Springback>
      <Duration>2.5</Duration>
    </Step>
    <Step id="2" type="Rotation">
      <Rotation angle="90"/>
      <Duration>1.5</Duration>
    </Step>
    <Step id="3" type="ABASetup">
      <ABAConfig value="255"/>
      <Duration>0.8</Duration>
    </Step>
    <Step id="4" type="Reposition">
      <Description>Re-grip</Description>
      <Duration>3.0</Duration>
    </Step>
  </ProcessSequence>
  <AdaptiveControl>
    <AngleFeedback enabled="true" tolerance="0.5"/>
    <ForceLimit max="25"/>
    <SpeedControl adaptive="true"/>
  </AdaptiveControl>
  <Metadata>
    <TotalCycleTime>7.8</TotalCycleTime>
    <BendCount>2</BendCount>
    <RepositionCount>1</RepositionCount>
    <RotationCount>1</RotationCount>
  </Metadata>
</Job>
```

## JSON/VDM Schema

```json
{
  "scene": {
    "camera": {
      "position": [0, -500, 300],
      "lookAt": [0, 0, 0],
      "fov": 45
    },
    "lights": [
      {"type": "ambient", "intensity": 0.4},
      {"type": "directional", "direction": [1, -1, -1], "intensity": 0.8}
    ]
  },
  "skeleton": {
    "bones": [
      {"index": 0, "name": "base", "parent": -1, "origin": [0, 0, 0], "axis": [0, 0, 1]},
      {"index": 1, "name": "flange_0", "parent": 0, "origin": [0, 50, 0], "axis": [1, 0, 0]}
    ]
  },
  "animation": {
    "duration": 7.5,
    "fps": 30,
    "keyframes": [
      {"time": 0.0, "bone": 0, "angle": 0, "action": "Start"},
      {"time": 2.5, "bone": 1, "angle": 90, "action": "Bend 0"}
    ]
  },
  "metadata": {
    "partName": "BOX001",
    "cycleTime": 7.5,
    "bendCount": 2,
    "generator": "OpenPanelCAM",
    "version": "5.0"
  }
}
```

## API Reference

### PostProcessor

```cpp
class PostProcessor {
    PostProcessor(const PostProcessorConfig& config = PostProcessorConfig());
    Phase5Output process(const Phase4Output& validated,
                         const Phase3Output& sequence,
                         const std::vector<BendFeature>& bends);
    void setConfig(const PostProcessorConfig& config);
};
```

### InstructionBuilder

```cpp
class InstructionBuilder {
    InstructionBuilder(const PostProcessorConfig& config = PostProcessorConfig());
    MachineProgram build(const Phase4Output& validated,
                         const Phase3Output& sequence,
                         const std::vector<BendFeature>& bends);
    double computeBendForce(double length, double thickness, double yieldStrength) const;
};
```

### AnimationBuilder

```cpp
class AnimationBuilder {
    AnimationBuilder(const PostProcessorConfig& config = PostProcessorConfig());
    VisualizationData build(const MachineProgram& program,
                            const std::vector<BendFeature>& bends);
    std::vector<AnimationBone> buildBoneHierarchy(const std::vector<BendFeature>& bends);
    std::vector<AnimationKeyframe> buildKeyframes(const MachineProgram& program);
};
```

### OutputValidator

```cpp
class OutputValidator {
    OutputValidator(const PostProcessorConfig& config = PostProcessorConfig());
    ValidationResult validate(const MachineProgram& program);
    bool checkAngleLimits(const MachineInstruction& instr) const;
    bool checkForceLimits(const MachineInstruction& instr) const;
    bool checkPartSize(const MachineProgram& program) const;
    bool checkCycleTimeConsistency(const MachineProgram& program) const;
    bool checkStepIdUniqueness(const MachineProgram& program) const;
};
```

### PBXMLGenerator / VDMGenerator

```cpp
class PBXMLGenerator {
    PBXMLGenerator(const PostProcessorConfig& config = PostProcessorConfig());
    std::string generate(const MachineProgram& program);
};

class VDMGenerator {
    VDMGenerator(const PostProcessorConfig& config = PostProcessorConfig());
    std::string generate(const VisualizationData& data);
};
```

## Machine Limits (PostProcessorConfig)

| Parameter | Default | Description |
|-----------|---------|-------------|
| maxBendAngle | 135° | Maximum bend angle |
| minBendAngle | -135° | Minimum bend angle |
| maxForce | 25 kN | Maximum bending force |
| maxPartLength | 2500 mm | Maximum part length |
| maxPartWidth | 1250 mm | Maximum part width |
| animationFPS | 30 | Animation frame rate |
| defaultThickness | 1.5 mm | Material thickness |
| defaultMaterial | AISI304 | Material type |
| defaultYieldStrength | 200 MPa | Yield strength |

## Conditional Compilation

Phase 5 supports two optional dependencies:

| Library | CMake Target | Compile Definition | Fallback |
|---------|-------------|-------------------|----------|
| PugiXML | `pugixml::pugixml` or `pugixml` | `HAS_PUGIXML=1` | Manual string building |
| nlohmann/json | `nlohmann_json::nlohmann_json` | `HAS_NLOHMANN_JSON=1` | Manual string building |

Both fallback paths produce identical output — the library versions just provide safer string escaping and better error handling.

## Performance

Measured on Release build (MSVC 17, x64):

| Operation | 10 bends | Target |
|-----------|----------|--------|
| XML generation | < 1ms | < 5ms |
| JSON generation | < 1ms | < 5ms |
| Full pipeline | < 5ms | < 20ms |
| 20 bends pipeline | < 10ms | < 50ms |

## Test Coverage

| Module | Test Cases | Assertions |
|--------|-----------|------------|
| Types | 14 | 64 |
| InstructionBuilder | 14 | 59 |
| AnimationBuilder | 8 | 32 |
| PBXMLGenerator | 10 | 28 |
| VDMGenerator | 8 | 21 |
| OutputValidator | 16 | 26 |
| PostProcessor | 9 | 26 |
| Edge Cases | 8 | 25 |
| Performance | 4 | 9 |
| Integration (P2→5) | 4 | 27 |
| **Total Phase 5** | **95** | **317** |

## File Structure

```
include/openpanelcam/phase5/
├── types.h                 # Core data structures
├── instruction_builder.h   # Phase4 → MachineProgram
├── animation_builder.h     # Bone hierarchy + keyframes
├── output_validator.h      # Machine limits validation
├── xml_generator.h         # PB-XML output
├── json_generator.h        # JSON/VDM output
└── post_processor.h        # Main pipeline

src/phase5/
├── instruction_builder.cpp
├── animation_builder.cpp
├── output_validator.cpp
├── xml_generator.cpp
├── json_generator.cpp
├── post_processor.cpp
└── CMakeLists.txt

tests/phase5/
├── test_types.cpp
├── test_instruction_builder.cpp
├── test_animation_builder.cpp
├── test_xml_generator.cpp
├── test_json_generator.cpp
├── test_output_validator.cpp
├── test_post_processor.cpp
├── test_phase5_edge_cases.cpp
├── test_phase5_performance.cpp
├── test_integration.cpp
└── CMakeLists.txt
```
