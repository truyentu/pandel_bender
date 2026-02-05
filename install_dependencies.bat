@echo off
REM Install OpenPanelCAM Phase 1 dependencies via vcpkg
REM This script will take approximately 60-90 minutes due to OpenCASCADE compilation

echo ========================================
echo OpenPanelCAM - Dependency Installation
echo ========================================
echo.
echo This will install the following packages:
echo   - OpenCASCADE (large, ~60 min compile time)
echo   - Eigen3
echo   - CGAL
echo   - FCL
echo   - nlohmann-json
echo   - spdlog
echo   - fmt
echo   - boost-graph
echo   - pugixml
echo   - Catch2
echo.
echo Estimated time: 60-90 minutes
echo.
pause

set VCPKG_ROOT=C:\vcpkg

echo.
echo [1/10] Installing OpenCASCADE (this will take a while...)
%VCPKG_ROOT%\vcpkg.exe install opencascade:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install opencascade
    pause
    exit /b 1
)

echo.
echo [2/10] Installing Eigen3...
%VCPKG_ROOT%\vcpkg.exe install eigen3:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install eigen3
    pause
    exit /b 1
)

echo.
echo [3/10] Installing CGAL...
%VCPKG_ROOT%\vcpkg.exe install cgal:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install cgal
    pause
    exit /b 1
)

echo.
echo [4/10] Installing FCL...
%VCPKG_ROOT%\vcpkg.exe install fcl:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install fcl
    pause
    exit /b 1
)

echo.
echo [5/10] Installing nlohmann-json...
%VCPKG_ROOT%\vcpkg.exe install nlohmann-json:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install nlohmann-json
    pause
    exit /b 1
)

echo.
echo [6/10] Installing spdlog...
%VCPKG_ROOT%\vcpkg.exe install spdlog:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install spdlog
    pause
    exit /b 1
)

echo.
echo [7/10] Installing fmt...
%VCPKG_ROOT%\vcpkg.exe install fmt:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install fmt
    pause
    exit /b 1
)

echo.
echo [8/10] Installing boost-graph...
%VCPKG_ROOT%\vcpkg.exe install boost-graph:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install boost-graph
    pause
    exit /b 1
)

echo.
echo [9/10] Installing pugixml...
%VCPKG_ROOT%\vcpkg.exe install pugixml:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install pugixml
    pause
    exit /b 1
)

echo.
echo [10/10] Installing Catch2...
%VCPKG_ROOT%\vcpkg.exe install catch2:x64-windows
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: Failed to install catch2
    pause
    exit /b 1
)

echo.
echo ========================================
echo SUCCESS: All dependencies installed!
echo ========================================
echo.
echo Next steps:
echo   1. cd to project directory
echo   2. Run: build.bat
echo.
pause
