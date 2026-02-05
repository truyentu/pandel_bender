# Build Instructions

## Prerequisites

### Required Software

1. **CMake** (version 3.20 or higher)
   - Download from: https://cmake.org/download/
   - Windows: Add to PATH during installation
   - Linux: `sudo apt install cmake` or `sudo yum install cmake`

2. **vcpkg** (Package Manager)
   ```bash
   # Clone vcpkg
   git clone https://github.com/Microsoft/vcpkg.git
   cd vcpkg

   # Bootstrap vcpkg
   # Windows:
   .\bootstrap-vcpkg.bat
   # Linux/Mac:
   ./bootstrap-vcpkg.sh

   # Set environment variable
   # Windows:
   set VCPKG_ROOT=C:\path\to\vcpkg
   # Linux/Mac:
   export VCPKG_ROOT=/path/to/vcpkg
   ```

3. **Compiler**
   - Windows: Visual Studio 2022 (MSVC 19.30+)
   - Linux: GCC 11+ or Clang 14+
   - Mac: Xcode 13+ or Clang 14+

4. **Ninja** (Optional, recommended for faster builds)
   - Windows: `vcpkg install ninja`
   - Linux: `sudo apt install ninja-build`

## Installing Dependencies

### Option 1: Automatic (via vcpkg manifest)

Dependencies are automatically installed when you configure the project:

```bash
cmake --preset windows-debug  # or linux-debug
```

vcpkg will read `vcpkg.json` and install all required packages.

### Option 2: Manual Installation

```bash
cd $VCPKG_ROOT

# Core dependencies
vcpkg install opencascade:x64-windows
vcpkg install eigen3:x64-windows
vcpkg install cgal:x64-windows
vcpkg install fcl:x64-windows
vcpkg install nlohmann-json:x64-windows
vcpkg install spdlog:x64-windows
vcpkg install catch2:x64-windows
vcpkg install fmt:x64-windows
vcpkg install boost-graph:x64-windows
vcpkg install pugixml:x64-windows

# For Linux, replace :x64-windows with :x64-linux
```

**Note**: OpenCASCADE is a large package (~2GB) and may take 30-60 minutes to compile.

## Building the Project

### Using CMake Presets (Recommended)

#### Windows

```bash
# Debug build
cmake --preset windows-debug
cmake --build build/windows-debug

# Release build
cmake --preset windows-release
cmake --build build/windows-release
```

#### Linux

```bash
# Debug build
cmake --preset linux-debug
cmake --build build/linux-debug

# Release build
cmake --preset linux-release
cmake --build build/linux-release
```

### Manual Configuration

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake .. \
  -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DOPENPANELCAM_BUILD_TESTS=ON \
  -DOPENPANELCAM_BUILD_SAMPLES=ON

# Build
cmake --build . --config Debug

# Or for multi-config generators (Visual Studio):
cmake --build . --config Debug
cmake --build . --config Release
```

## Running Tests

### Using CTest (Recommended)

```bash
# With presets
ctest --preset windows-debug --output-on-failure

# Manual
cd build
ctest --output-on-failure
```

### Running Test Executables Directly

```bash
# Unit tests
./build/bin/test_phase1
./build/bin/test_core

# Integration tests
./build/bin/test_integration
```

## Running Samples

After building, you can run the Phase 1 sample applications:

### Parse STEP File (Full Pipeline)

```bash
# Windows
.\build\bin\sample_parse_step.exe path\to\part.step --verbose

# Linux/macOS
./build/bin/sample_parse_step path/to/part.step --verbose
```

**Options:**
- `--verbose` - Enable detailed logging
- `--thickness <mm>` - Set sheet thickness (default: 2.0)
- `--no-heal` - Disable geometry healing

**Example:**
```bash
./sample_parse_step bracket.step --thickness 2.5 --verbose
```

### Demo Individual Modules

```bash
# Windows
.\build\bin\sample_phase1_modules.exe part.step

# Linux/macOS
./build/bin/sample_phase1_modules part.step
```

Shows how to use each Phase 1 module separately.

### Test Core Utilities

```bash
# Windows
.\build\bin\test_core.exe

# Linux/macOS
./build/bin/test_core
```

See `samples/README.md` for detailed sample documentation.

## Build Options

Configure these options with `-D` flag during CMake configuration:

| Option | Default | Description |
|--------|---------|-------------|
| `OPENPANELCAM_BUILD_TESTS` | ON | Build test suite |
| `OPENPANELCAM_BUILD_SAMPLES` | ON | Build sample applications |
| `OPENPANELCAM_BUILD_DOCS` | OFF | Build documentation (requires Doxygen) |
| `OPENPANELCAM_ENABLE_WARNINGS` | ON | Enable compiler warnings |
| `OPENPANELCAM_WARNINGS_AS_ERRORS` | OFF | Treat warnings as errors |

Example:

```bash
cmake --preset windows-debug \
  -DOPENPANELCAM_BUILD_TESTS=OFF \
  -DOPENPANELCAM_WARNINGS_AS_ERRORS=ON
```

## Troubleshooting

### OpenCASCADE Not Found

**Problem**: `Could not find OpenCASCADE`

**Solution**:
1. Verify vcpkg installed OpenCASCADE: `vcpkg list | grep opencascade`
2. Ensure `CMAKE_TOOLCHAIN_FILE` points to vcpkg cmake
3. Try clean configure: `rm -rf build && cmake --preset windows-debug`

### Link Errors with OpenCASCADE

**Problem**: Undefined symbols from OCCT libraries

**Solution**:
1. Ensure all OCCT components are listed in `CMakeLists.txt`
2. Check vcpkg triplet matches your platform (x64-windows vs x64-linux)
3. Verify Release/Debug mismatch - rebuild vcpkg packages if needed

### Catch2 Not Found

**Problem**: `Could not find Catch2`

**Solution**:
```bash
vcpkg install catch2:x64-windows
# Force reconfigure
cmake --preset windows-debug -DCMAKE_FIND_DEBUG_MODE=ON
```

### Out of Memory During Build

**Problem**: Compiler crashes or system freezes

**Solution**:
1. Close other applications
2. Limit parallel jobs: `cmake --build build -- -j2` (use 2 cores)
3. Use Release build (smaller object files)

### Windows: MSVC Version Mismatch

**Problem**: vcpkg packages built with different MSVC version

**Solution**:
```bash
# Rebuild packages with current toolchain
vcpkg remove opencascade:x64-windows
vcpkg install opencascade:x64-windows --triplet x64-windows
```

## Build Performance Tips

1. **Use Ninja**: Faster than MSBuild/make
   ```bash
   cmake --preset windows-debug -G Ninja
   ```

2. **Enable Parallel Builds**:
   ```bash
   cmake --build build -- -j8  # Use 8 cores
   ```

3. **Precompiled Headers**: Enabled automatically for Phase 1+

4. **Incremental Builds**: Only rebuild changed files
   ```bash
   cmake --build build --target openpanelcam_phase1
   ```

5. **ccache** (Linux): Cache compilation results
   ```bash
   sudo apt install ccache
   export CMAKE_CXX_COMPILER_LAUNCHER=ccache
   ```

## IDE Integration

### Visual Studio

Open the project folder in Visual Studio 2022:
- File → Open → CMake → Select root `CMakeLists.txt`
- VS will auto-configure using `CMakePresets.json`

### Visual Studio Code

1. Install extensions:
   - CMake Tools
   - C/C++

2. Open project folder

3. Select preset from CMake Tools sidebar

4. Build/Debug from command palette

### CLion

1. Open project folder
2. CLion auto-detects CMake presets
3. Select desired preset from configuration dropdown

## Clean Build

```bash
# Remove build directory
rm -rf build

# Or with CMake
cmake --build build --target clean
```

## Installation

```bash
cmake --build build --target install
# Default install location: ./install/

# Custom install prefix:
cmake --preset windows-debug -DCMAKE_INSTALL_PREFIX=/custom/path
cmake --build build --target install
```

---

**Next Steps**: After successful build, see README.md for usage examples.
