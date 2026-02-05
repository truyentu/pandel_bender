@echo off
REM Build OpenPanelCAM project

echo ========================================
echo OpenPanelCAM - Build Script
echo ========================================

set VCPKG_ROOT=C:\vcpkg
set PROJECT_ROOT=%~dp0
set BUILD_DIR=%PROJECT_ROOT%build

REM Check if vcpkg exists
if not exist "%VCPKG_ROOT%\vcpkg.exe" (
    echo ERROR: vcpkg not found at %VCPKG_ROOT%
    echo Please install vcpkg first or update VCPKG_ROOT variable
    pause
    exit /b 1
)

echo.
echo Project root: %PROJECT_ROOT%
echo vcpkg root:   %VCPKG_ROOT%
echo Build dir:    %BUILD_DIR%
echo.

REM Create build directory
if not exist "%BUILD_DIR%" (
    echo Creating build directory...
    mkdir "%BUILD_DIR%"
)

cd "%BUILD_DIR%"

echo.
echo ========================================
echo Step 1: Configure CMake
echo ========================================
echo.

cmake .. ^
    -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DOPENPANELCAM_BUILD_TESTS=OFF ^
    -DOPENPANELCAM_BUILD_SAMPLES=ON ^
    -DOPENPANELCAM_ENABLE_WARNINGS=ON

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: CMake configuration failed!
    echo.
    echo Common issues:
    echo   - Dependencies not installed: run install_dependencies.bat
    echo   - Wrong vcpkg path: check VCPKG_ROOT variable
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo Step 2: Build Project
echo ========================================
echo.

cmake --build . --config Release

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Build failed!
    echo Check the error messages above for details.
    echo.
    pause
    exit /b 1
)

echo.
echo ========================================
echo SUCCESS: Build completed!
echo ========================================
echo.
echo Executables built in:
echo   %BUILD_DIR%\bin\Release\
echo.
echo Sample programs:
echo   - test_core.exe
echo   - sample_parse_step.exe
echo   - sample_phase1_modules.exe
echo.
echo To run a sample:
echo   cd %BUILD_DIR%\bin\Release
echo   sample_parse_step.exe path\to\your\file.step --verbose
echo.
pause
