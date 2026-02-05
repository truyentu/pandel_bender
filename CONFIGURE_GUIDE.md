# Configuration Guide - Without vcpkg Installation

Vì OpenCASCADE và các dependencies lớn (5GB+, build 60-90 phút), chúng ta sẽ tạo **minimal configuration** để test build system mà không cần install dependencies ngay.

## Current Situation

Project đã có:
- ✅ Complete CMake build system
- ✅ vcpkg.json manifest
- ✅ Placeholder source files (compiles without dependencies)
- ⚠️ vcpkg chưa được cài đặt
- ⚠️ Dependencies chưa được install

## Option 1: Test CMake Structure Only (5 minutes)

Test xem CMake có generate đúng không, KHÔNG build code:

```bash
cd E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller

# Create minimal CMakeLists test
cmake -E echo "Testing CMake structure..."

# Verify CMakeLists.txt syntax
cmake -P -E echo "CMake files are valid"
```

## Option 2: Dry-Run Configuration (10 minutes)

Kiểm tra CMake configuration KHÔNG cần dependencies:

### Step 1: Create standalone test

Tạo file `test_cmake.bat`:

```batch
@echo off
cd /d E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller

echo Testing CMake configuration...
cmake --version
cmake -E capabilities

echo.
echo Checking CMakeLists.txt syntax...
cmake -P CMakeLists.txt 2>&1 | findstr /C:"error" && echo ERRORS FOUND || echo SYNTAX OK

echo.
echo Project structure:
tree /F /A src include tests

pause
```

### Step 2: Run test

```bash
test_cmake.bat
```

## Option 3: Mock Dependencies (20 minutes)

Tạo mock targets thay cho real dependencies để test linking:

### Create mock_dependencies.cmake

```cmake
# Mock OpenCASCADE
if(NOT TARGET TKernel)
    add_library(TKernel INTERFACE)
    add_library(TKMath INTERFACE)
    add_library(TKG3d INTERFACE)
    # ... etc
    set(OpenCASCADE_FOUND TRUE)
    set(OpenCASCADE_LIBRARIES TKernel TKMath TKG3d)
    set(OpenCASCADE_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
endif()

# Mock Eigen3
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3 INTERFACE)
    add_library(Eigen3::Eigen ALIAS Eigen3)
endif()

# ... mock other dependencies
```

### Modify root CMakeLists.txt

Add before `find_package()`:

```cmake
option(OPENPANELCAM_USE_MOCK_DEPS "Use mock dependencies for testing" ON)

if(OPENPANELCAM_USE_MOCK_DEPS)
    include(${CMAKE_SOURCE_DIR}/cmake/mock_dependencies.cmake)
else()
    # Original find_package() calls
endif()
```

## Option 4: Install vcpkg + Dependencies (60-90 minutes)

Nếu muốn FULL build với real dependencies:

### Step 1: Install vcpkg

```powershell
# Download vcpkg
cd C:\
git clone https://github.com/Microsoft/vcpkg.git
cd vcpkg

# Bootstrap
.\bootstrap-vcpkg.bat

# Set environment variable (PERMANENT)
[Environment]::SetEnvironmentVariable("VCPKG_ROOT", "C:\vcpkg", "User")

# Refresh current session
$env:VCPKG_ROOT = "C:\vcpkg"
```

### Step 2: Integrate with Visual Studio (Optional)

```powershell
.\vcpkg integrate install
```

### Step 3: Install dependencies

⚠️ **WARNING**: Quá trình này sẽ:
- Download ~3GB source code
- Build ~2GB binaries
- Mất 60-90 phút (OpenCASCADE rất lớn)
- Cần ~10GB disk space

```powershell
cd E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller

# Install all dependencies from vcpkg.json
cmake --preset windows-debug
# vcpkg sẽ tự động install tất cả packages
```

### Step 4: Build project

```powershell
cmake --build build/windows-debug
```

## Recommended Approach for Now

Vì mục đích **test build system structure**, tôi khuyến nghị:

1. ✅ **Option 2** - Dry-run configuration (10 phút)
   - Verify CMake syntax
   - Check file structure
   - No dependencies needed

2. ⏸️ **Option 4** - Full install (làm sau)
   - Khi sẵn sàng implement code
   - Có thời gian chờ build
   - Cần disk space

## What We Can Verify Now (Without Dependencies)

- ✅ CMakeLists.txt syntax
- ✅ Project structure
- ✅ File organization
- ✅ CMake presets
- ✅ vcpkg.json manifest
- ❌ Actual compilation (cần dependencies)
- ❌ Linking (cần dependencies)
- ❌ Tests (cần dependencies)

## Next Steps

**Bạn muốn:**

1. **Test CMake syntax only** (5 phút) - Verify structure
2. **Create mock dependencies** (20 phút) - Test linking without real libs
3. **Install vcpkg + dependencies** (90 phút) - Full build
4. **Skip for now** - Proceed to code implementation

Cho tôi biết option nào phù hợp!
