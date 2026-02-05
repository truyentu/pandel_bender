# System Architecture Overview

> **Project**: OpenPanelCAM
> **Version**: 1.0
> **Last Updated**: 2026-02-04

---

## 1. High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           OpenPanelCAM System                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌─────────────────────────────────────────────────────────────────────┐   │
│   │                        External Interfaces                           │   │
│   ├─────────────────────────────────────────────────────────────────────┤   │
│   │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐ │   │
│   │  │  STEP    │  │   CLI    │  │   API    │  │   Machine Control    │ │   │
│   │  │  Files   │  │Interface │  │  Server  │  │   (TwinCAT/ADS)      │ │   │
│   │  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────────┬───────────┘ │   │
│   └───────┼─────────────┼─────────────┼────────────────────┼────────────┘   │
│           │             │             │                    │                 │
│   ┌───────▼─────────────▼─────────────▼────────────────────▼────────────┐   │
│   │                        Application Layer                             │   │
│   ├─────────────────────────────────────────────────────────────────────┤   │
│   │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐   │   │
│   │  │   Session    │  │    Job       │  │      Export              │   │   │
│   │  │   Manager    │  │   Manager    │  │      Manager             │   │   │
│   │  └──────────────┘  └──────────────┘  └──────────────────────────┘   │   │
│   └─────────────────────────────┬───────────────────────────────────────┘   │
│                                 │                                            │
│   ┌─────────────────────────────▼───────────────────────────────────────┐   │
│   │                         Core Pipeline                                │   │
│   ├─────────────────────────────────────────────────────────────────────┤   │
│   │                                                                      │   │
│   │   ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐   ┌────────┐   │   │
│   │   │Phase 1 │──▶│Phase 2 │──▶│Phase 3 │──▶│Phase 4 │──▶│Phase 5 │   │   │
│   │   │Parser  │   │Solver  │   │Sequencer│   │Physics │   │Output  │   │   │
│   │   └────────┘   └────────┘   └────────┘   └────────┘   └────────┘   │   │
│   │                                                                      │   │
│   └─────────────────────────────┬───────────────────────────────────────┘   │
│                                 │                                            │
│   ┌─────────────────────────────▼───────────────────────────────────────┐   │
│   │                        Foundation Layer                              │   │
│   ├─────────────────────────────────────────────────────────────────────┤   │
│   │  ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐    │   │
│   │  │  Geometry  │  │   Math     │  │  Logging   │  │   Config   │    │   │
│   │  │  (OCCT)    │  │  (Eigen)   │  │  (spdlog)  │  │  Manager   │    │   │
│   │  └────────────┘  └────────────┘  └────────────┘  └────────────┘    │   │
│   └─────────────────────────────────────────────────────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Module Dependencies

```
                    ┌─────────────┐
                    │   main()    │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │ Application │
                    │   Layer     │
                    └──────┬──────┘
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
   ┌─────▼─────┐    ┌──────▼──────┐   ┌─────▼─────┐
   │  Phase 1  │    │   Phase 2   │   │  Phase 5  │
   │  Parser   │───▶│   Solver    │   │  Output   │
   └─────┬─────┘    └──────┬──────┘   └─────▲─────┘
         │                 │                 │
         │          ┌──────▼──────┐          │
         │          │   Phase 3   │          │
         │          │  Sequencer  │          │
         │          └──────┬──────┘          │
         │                 │                 │
         │          ┌──────▼──────┐          │
         │          │   Phase 4   │──────────┘
         │          │   Physics   │
         │          └──────┬──────┘
         │                 │
         └────────┬────────┘
                  │
         ┌────────▼────────┐
         │   Foundation    │
         │ (OCCT, Eigen)   │
         └─────────────────┘
```

---

## 3. Directory Structure

```
Salvagnini_controller/
│
├── docs/                           # Documentation
│   ├── MASTER_CONTEXT.md          # Project context (this session)
│   ├── ROADMAP.md                 # Development roadmap
│   ├── architecture/
│   │   └── system-overview.md     # This file
│   ├── data-structures/
│   │   ├── cross-phase-types.md   # Shared types
│   │   └── phase1-types.md        # Phase 1 specific types
│   └── phases/
│       ├── phase1-design.md       # Phase 1 design document
│       ├── phase2-design.md       # Phase 2 design document
│       └── ...
│
├── include/                        # Public headers
│   └── openpanelcam/
│       ├── core/
│       │   ├── types.h            # Core type definitions
│       │   ├── geometry.h         # Geometry utilities
│       │   └── constants.h        # Global constants
│       ├── phase1/
│       │   ├── parser.h           # Main parser interface
│       │   ├── fag.h              # Face-Adjacency Graph
│       │   └── bend_classifier.h  # Bend classification
│       ├── phase2/
│       │   └── ...
│       └── ...
│
├── src/                            # Implementation
│   ├── core/
│   │   ├── geometry.cpp
│   │   └── logging.cpp
│   ├── phase1/
│   │   ├── step_reader.cpp
│   │   ├── geometry_healer.cpp
│   │   ├── fag_builder.cpp
│   │   ├── base_face_identifier.cpp
│   │   └── bend_classifier.cpp
│   ├── phase2/
│   │   └── ...
│   └── main.cpp                    # Entry point
│
├── tests/                          # Unit and integration tests
│   ├── phase1/
│   │   ├── test_step_reader.cpp
│   │   ├── test_fag_builder.cpp
│   │   └── test_bend_classifier.cpp
│   ├── phase2/
│   │   └── ...
│   └── integration/
│       └── test_full_pipeline.cpp
│
├── samples/                        # Sample data
│   └── step_files/
│       ├── simple_box.step
│       ├── l_bracket.step
│       └── ...
│
├── research_paper/                 # Source research documents
│   └── *.pdf
│
├── CMakeLists.txt                  # Build configuration
├── vcpkg.json                      # Package dependencies
└── README.md                       # Project readme
```

---

## 4. Build Configuration

### 4.1 CMake Structure

```cmake
# CMakeLists.txt (root)

cmake_minimum_required(VERSION 3.20)
project(OpenPanelCAM VERSION 0.1.0 LANGUAGES CXX)

# C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build examples" ON)

# Dependencies
find_package(OpenCASCADE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

# Core library
add_subdirectory(src/core)

# Phase libraries
add_subdirectory(src/phase1)
add_subdirectory(src/phase2)
add_subdirectory(src/phase3)
add_subdirectory(src/phase4)
add_subdirectory(src/phase5)

# Main executable
add_executable(openpanelcam src/main.cpp)
target_link_libraries(openpanelcam
    PRIVATE
        openpanelcam_core
        openpanelcam_phase1
        openpanelcam_phase2
        openpanelcam_phase3
        openpanelcam_phase4
        openpanelcam_phase5
)

# Tests
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
```

### 4.2 Phase 1 CMake

```cmake
# src/phase1/CMakeLists.txt

add_library(openpanelcam_phase1
    step_reader.cpp
    geometry_healer.cpp
    fag_builder.cpp
    base_face_identifier.cpp
    bend_classifier.cpp
)

target_include_directories(openpanelcam_phase1
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase1
    PUBLIC
        openpanelcam_core
        ${OpenCASCADE_LIBRARIES}
    PRIVATE
        spdlog::spdlog
)
```

### 4.3 vcpkg Dependencies

```json
{
  "name": "openpanelcam",
  "version": "0.1.0",
  "dependencies": [
    "eigen3",
    "spdlog",
    "catch2",
    "nlohmann-json",
    {
      "name": "opencascade",
      "features": ["vtk"]
    }
  ]
}
```

---

## 5. Data Flow

### 5.1 Complete Pipeline

```
┌──────────────────────────────────────────────────────────────────────┐
│                        Data Flow Pipeline                             │
├──────────────────────────────────────────────────────────────────────┤
│                                                                       │
│  ┌─────────────┐                                                      │
│  │  STEP File  │                                                      │
│  │  (*.step)   │                                                      │
│  └──────┬──────┘                                                      │
│         │                                                             │
│         ▼                                                             │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ PHASE 1: Geometric Parser                                        │ │
│  │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐               │ │
│  │ │ STEP Reader │─▶│ Geo Healer │─▶│ FAG Builder │               │ │
│  │ └─────────────┘  └─────────────┘  └─────────────┘               │ │
│  │                                          │                       │ │
│  │                  ┌─────────────┐  ┌──────▼──────┐               │ │
│  │                  │ Bend Class. │◀─│ Base Finder │               │ │
│  │                  └──────┬──────┘  └─────────────┘               │ │
│  └─────────────────────────┼───────────────────────────────────────┘ │
│                            │                                          │
│                            ▼                                          │
│                   ┌─────────────────┐                                 │
│                   │  Phase1Output   │                                 │
│                   │ - FAG           │                                 │
│                   │ - BendFeatures  │                                 │
│                   │ - BaseFace      │                                 │
│                   └────────┬────────┘                                 │
│                            │                                          │
│                            ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ PHASE 2: Constraint Solver                                       │ │
│  │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐               │ │
│  │ │ Precedence  │  │   Grasp     │  │    ABA      │               │ │
│  │ │  Analyzer   │  │ Validator   │  │  Analyzer   │               │ │
│  │ └──────┬──────┘  └──────┬──────┘  └──────┬──────┘               │ │
│  │        └────────────────┼────────────────┘                       │ │
│  │                         ▼                                        │ │
│  │                  ┌─────────────┐                                 │ │
│  │                  │ DAG Builder │                                 │ │
│  │                  └─────────────┘                                 │ │
│  └─────────────────────────┼───────────────────────────────────────┘ │
│                            │                                          │
│                            ▼                                          │
│                   ┌─────────────────┐                                 │
│                   │  Phase2Output   │                                 │
│                   │ - PrecedenceDAG │                                 │
│                   │ - Constraints   │                                 │
│                   └────────┬────────┘                                 │
│                            │                                          │
│                            ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ PHASE 3: Sequencer                                               │ │
│  │                  ┌─────────────┐                                 │ │
│  │                  │  A* Search  │                                 │ │
│  │                  │ Masked Time │                                 │ │
│  │                  └─────────────┘                                 │ │
│  └─────────────────────────┼───────────────────────────────────────┘ │
│                            │                                          │
│                            ▼                                          │
│                   ┌─────────────────┐                                 │
│                   │  BendSequence   │                                 │
│                   │ - Steps[]       │                                 │
│                   │ - Rotations[]   │                                 │
│                   └────────┬────────┘                                 │
│                            │                                          │
│                            ▼                                          │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ PHASE 4: Physics Validator                                       │ │
│  │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐               │ │
│  │ │   Swept     │  │  Collision  │  │   Grasp     │               │ │
│  │ │   Volume    │  │   Detect    │  │  Physics    │               │ │
│  │ └─────────────┘  └─────────────┘  └─────────────┘               │ │
│  └─────────────────────────┼───────────────────────────────────────┘ │
│                            │                                          │
│                   ┌────────┴────────┐                                 │
│                   │                 │                                 │
│             ┌─────▼─────┐    ┌──────▼──────┐                         │
│             │   VALID   │    │  INVALID    │                         │
│             └─────┬─────┘    │ (Feedback)  │                         │
│                   │          └─────────────┘                         │
│                   ▼                                                   │
│  ┌─────────────────────────────────────────────────────────────────┐ │
│  │ PHASE 5: Post-Processor                                          │ │
│  │ ┌─────────────┐  ┌─────────────┐                                │ │
│  │ │ XML/Binary  │  │    HMI      │                                │ │
│  │ │  Generator  │  │  Exporter   │                                │ │
│  │ └──────┬──────┘  └──────┬──────┘                                │ │
│  └────────┼────────────────┼───────────────────────────────────────┘ │
│           │                │                                          │
│           ▼                ▼                                          │
│   ┌─────────────┐  ┌─────────────┐                                   │
│   │ Machine     │  │ Visualization│                                   │
│   │ Instructions│  │    Data     │                                   │
│   │   (XML)     │  │   (JSON)    │                                   │
│   └─────────────┘  └─────────────┘                                   │
│                                                                       │
└──────────────────────────────────────────────────────────────────────┘
```

### 5.2 Error Handling Flow

```
┌───────────────────────────────────────────────────┐
│              Error Handling Strategy               │
├───────────────────────────────────────────────────┤
│                                                    │
│  Each Phase returns:                               │
│  ┌────────────────────────────────────────────┐   │
│  │  Result<OutputT>                            │   │
│  │  ├── Success: OutputT data                  │   │
│  │  └── Failure: ErrorInfo                     │   │
│  │       ├── code                              │   │
│  │       ├── message                           │   │
│  │       ├── severity (INFO/WARN/ERROR/FATAL)  │   │
│  │       └── context                           │   │
│  └────────────────────────────────────────────┘   │
│                                                    │
│  Error Propagation:                                │
│                                                    │
│  Phase 1 ──Error──▶ ┌─────────────────┐           │
│                     │ Error Handler   │           │
│  Phase 2 ──Error──▶ │                 │──▶ Log    │
│                     │ - Aggregate     │           │
│  Phase 3 ──Error──▶ │ - Classify      │──▶ UI    │
│                     │ - Route         │           │
│  Phase 4 ──Error──▶ └─────────────────┘──▶ Abort │
│                                                    │
└───────────────────────────────────────────────────┘
```

---

## 6. Interface Contracts

### 6.1 Phase Interface

```cpp
// Every phase implements this template
template<typename InputT, typename OutputT>
class IPhase {
public:
    virtual ~IPhase() = default;

    // Core processing
    virtual Result<OutputT> process(const InputT& input) = 0;

    // Pre-validation
    virtual ValidationResult validateInput(const InputT& input) = 0;

    // Post-validation
    virtual ValidationResult validateOutput(const OutputT& output) = 0;

    // Metadata
    virtual std::string getName() const = 0;
    virtual std::string getVersion() const = 0;

    // Progress reporting (for long operations)
    virtual void setProgressCallback(ProgressCallback cb) = 0;

    // Cancellation support
    virtual void cancel() = 0;
    virtual bool isCancelled() const = 0;
};
```

### 6.2 Phase Connections

```cpp
// Phase 1
class GeometricParser : public IPhase<std::string, Phase1Output> {
    // Input: filepath to STEP file
    // Output: FAG + BendFeatures
};

// Phase 2
class ConstraintSolver : public IPhase<Phase1Output, Phase2Output> {
    // Input: FAG from Phase 1
    // Output: PrecedenceDAG + Constraints
};

// Phase 3
class BendSequencer : public IPhase<Phase2Output, BendSequence> {
    // Input: DAG + Constraints
    // Output: Ordered BendSequence
};

// Phase 4
class PhysicsValidator : public IPhase<BendSequence, SequenceValidation> {
    // Input: Proposed sequence
    // Output: Validation result (pass/fail with details)
};

// Phase 5
class PostProcessor : public IPhase<BendSequence, MachineProgram> {
    // Input: Validated sequence
    // Output: Machine code + HMI data
};
```

---

## 7. External Dependencies

### 7.1 Required Libraries

| Library | Version | Purpose | License |
|---------|---------|---------|---------|
| OpenCASCADE | 7.6+ | CAD kernel, STEP parsing | LGPL |
| Eigen | 3.4+ | Linear algebra | MPL2 |
| spdlog | 1.x | Logging | MIT |
| nlohmann/json | 3.x | JSON serialization | MIT |
| Catch2 | 3.x | Unit testing | BSL |

### 7.2 Optional Libraries

| Library | Purpose | When Needed |
|---------|---------|-------------|
| CGAL | Computational geometry | Phase 2, 4 (LER, collision) |
| FCL | Collision detection | Phase 4 |
| Boost Graph | Graph algorithms | Phase 2, 3 (if needed) |
| TinyXML2 | XML generation | Phase 5 |

### 7.3 OCCT Modules

```cpp
// Required OCCT toolkits
// TKernel      - Core
// TKMath       - Math utilities
// TKG3d        - 3D geometry
// TKBRep       - Boundary representation
// TKTopAlgo    - Topological algorithms
// TKShHealing  - Shape healing
// TKSTEP       - STEP import
// TKXSBase     - Translation services
// TKBO         - Boolean operations (Phase 4)
// TKMesh       - Meshing (Phase 4)
// TKPrim       - Primitives (Phase 4 - MakeRevol)
```

---

## 8. Threading Model

```
┌─────────────────────────────────────────────────────────────────┐
│                    Threading Architecture                        │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Main Thread                                                     │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │  Application Logic                                          │ │
│  │  - Job management                                           │ │
│  │  - User interface                                           │ │
│  │  - Pipeline orchestration                                   │ │
│  └────────────────────────────────────────────────────────────┘ │
│                         │                                        │
│                         ▼                                        │
│  Worker Thread Pool (for long operations)                        │
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐       │
│  │ Worker 1  │ │ Worker 2  │ │ Worker 3  │ │ Worker 4  │       │
│  │           │ │           │ │           │ │           │       │
│  │ Phase 1   │ │ Phase 3   │ │ Phase 4   │ │ (idle)    │       │
│  │ (STEP     │ │ (A*       │ │ (Collision│ │           │       │
│  │  parsing) │ │  search)  │ │  detect)  │ │           │       │
│  └───────────┘ └───────────┘ └───────────┘ └───────────┘       │
│                                                                  │
│  Notes:                                                          │
│  - Each phase can run in its own thread                         │
│  - Phases are sequential (no parallel execution of phases)       │
│  - OCCT is NOT thread-safe for shape modification                │
│  - Read-only operations can be parallelized within a phase       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 9. Configuration System

```cpp
// Configuration hierarchy
//
// 1. Defaults (compiled in)
// 2. Config file (config.json)
// 3. Environment variables
// 4. Command-line arguments
// 5. Runtime API calls

struct AppConfig {
    // Paths
    std::string dataDir;
    std::string outputDir;
    std::string logDir;

    // Machine
    MachineConfig machine;

    // Processing
    struct {
        double geometryTolerance = 1e-6;
        bool enableHealing = true;
        bool verboseLogging = false;
    } processing;

    // Performance
    struct {
        int threadCount = 4;
        size_t cacheSize = 100 * 1024 * 1024;  // 100 MB
    } performance;

    // Load from file
    static AppConfig load(const std::string& path);

    // Apply command-line overrides
    void applyArgs(int argc, char** argv);
};
```

---

## 10. Logging Strategy

```cpp
// Log levels:
//   TRACE  - Very detailed debugging
//   DEBUG  - Debugging information
//   INFO   - General information
//   WARN   - Warnings (recoverable issues)
//   ERROR  - Errors (operation failed)
//   FATAL  - Critical errors (application must stop)

// Log format:
// [2026-02-04 14:30:00.123] [Phase1] [INFO] Parsing STEP file: test.step

// Example usage:
namespace logging {
    void init(const std::string& logFile, spdlog::level::level_enum level);

    // Get logger for a specific phase
    std::shared_ptr<spdlog::logger> getLogger(const std::string& name);
}

// In Phase 1:
auto logger = logging::getLogger("Phase1");
logger->info("Loaded {} faces from STEP file", faceCount);
logger->debug("Face {} classified as {}", faceId, faceType);
logger->error("Failed to heal geometry: {}", error.message);
```
