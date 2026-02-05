# OpenPanelCAM

High-performance CAM kernel for Salvagnini P4 Panel Bender - Automatic bend sequencing from STEP to machine code.

## Overview

OpenPanelCAM is a comprehensive CAM (Computer-Aided Manufacturing) system that automatically generates optimal bend sequences for panel bending machines. The system takes 3D STEP files as input and produces validated machine instructions with collision avoidance and cycle time optimization.

### Features

- **Phase 1: Geometric Parser** - STEP file parsing, Face-Adjacency Graph construction, bend classification
- **Phase 2: Constraint Solver** - Precedence analysis, grasp validation, ABA constraints
- **Phase 3: Sequencer** - A* search with masked time optimization
- **Phase 4: Physics Validator** - Collision detection, swept volume analysis
- **Phase 5: Post-Processor** - Machine code generation (PB-XML), HMI visualization

### Target Machine

Salvagnini P4 Panel Bender (P4-2520)
- Max sheet size: 2500 x 1500 mm
- Thickness range: 0.4 - 3.2 mm
- Bend angle range: -135° to +135°

## Building

### Prerequisites

- CMake 3.20+
- C++17 compiler (MSVC 2022 / GCC 11+ / Clang 14+)
- vcpkg (for dependency management)

### Dependencies

All dependencies are managed via vcpkg:

- OpenCASCADE 7.6+ (STEP parsing, geometry)
- Eigen 3.4+ (linear algebra)
- CGAL 5.x (computational geometry)
- FCL 0.7+ (collision detection)
- spdlog 1.x (logging)
- Catch2 3.x (testing)

### Build Instructions

#### Windows (with vcpkg)

```bash
# Set vcpkg root
set VCPKG_ROOT=C:\path\to\vcpkg

# Configure
cmake --preset windows-debug

# Build
cmake --build build/windows-debug

# Run tests
ctest --preset windows-debug
```

#### Linux (with vcpkg)

```bash
# Set vcpkg root
export VCPKG_ROOT=/path/to/vcpkg

# Configure
cmake --preset linux-debug

# Build
cmake --build build/linux-debug

# Run tests
ctest --preset linux-debug
```

### Manual Build (without presets)

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build .
ctest
```

## Project Structure

```
Salvagnini_controller/
├── CMakeLists.txt          # Root CMake configuration
├── vcpkg.json              # Dependency manifest
├── CMakePresets.json       # Build presets
├── include/                # Public headers
│   └── openpanelcam/
│       ├── core/           # Core utilities
│       ├── phase1/         # Geometric parser
│       ├── phase2/         # Constraint solver
│       ├── phase3/         # Sequencer
│       ├── phase4/         # Physics validator
│       └── phase5/         # Post-processor
├── src/                    # Implementation
│   ├── core/
│   ├── phase1/
│   ├── phase2/
│   ├── phase3/
│   ├── phase4/
│   └── phase5/
├── tests/                  # Test suite
│   ├── unit/               # Unit tests
│   └── integration/        # Integration tests
├── samples/                # Example applications
│   └── step_files/         # Test STEP files
├── docs/                   # Documentation
│   ├── ROADMAP.md
│   ├── MASTER_CONTEXT.md
│   ├── phases/             # Phase-specific design docs
│   ├── data-structures/    # Type specifications
│   └── architecture/       # System architecture
└── research_paper/         # Source research papers
```

## Usage

### Example: Parse STEP File

```cpp
#include <openpanelcam/phase1/step_reader.h>
#include <openpanelcam/phase1/fag_builder.h>

int main() {
    // Load STEP file
    openpanelcam::STEPReader reader;
    reader.load("part.step");

    // Build Face-Adjacency Graph
    openpanelcam::FAGBuilder builder;
    auto fag = builder.build(reader.getShape());

    // Extract bends
    auto bends = fag.extractBendFeatures();

    std::cout << "Found " << bends.size() << " bends\n";
    return 0;
}
```

## Development Status

- [x] **Documentation** - Design documents complete
- [x] **Project Setup** - Build system configured
- [ ] **Phase 1** - In development
- [ ] **Phase 2** - Not started
- [ ] **Phase 3** - Not started
- [ ] **Phase 4** - Not started
- [ ] **Phase 5** - Not started

## Documentation

Comprehensive documentation is available in the `docs/` directory:

- `ROADMAP.md` - Development roadmap for all phases
- `MASTER_CONTEXT.md` - Project context and architecture
- `phases/phase1-design.md` - Phase 1 detailed design
- `data-structures/` - Type specifications

## License

MIT License - See LICENSE file for details

## Research Foundation

This project is based on extensive research papers covering:
- Face-Adjacency Graph construction
- Bend feature recognition
- Constraint analysis
- A* sequencing optimization
- Collision detection
- Machine instruction generation

See `research_paper/` directory for source materials.

## Contributing

Contributions are welcome! Please follow the coding standards defined in the documentation.

## Contact

Project maintained by [Your Name/Organization]

---

**Status**: Active Development
**Version**: 0.1.0
**Last Updated**: 2026-02-04
