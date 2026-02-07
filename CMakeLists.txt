cmake_minimum_required(VERSION 3.20)
project(OpenPanelCAM
    VERSION 0.1.0
    DESCRIPTION "High-performance CAM kernel for Salvagnini P4 Panel Bender"
    LANGUAGES CXX
)

#==============================================================================
# Project Options
#==============================================================================

option(OPENPANELCAM_BUILD_TESTS "Build test suite" ON)
option(OPENPANELCAM_BUILD_SAMPLES "Build sample applications" ON)
option(OPENPANELCAM_BUILD_DOCS "Build documentation" OFF)
option(OPENPANELCAM_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(OPENPANELCAM_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

#==============================================================================
# C++ Standard
#==============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#==============================================================================
# Output Directories
#==============================================================================

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

#==============================================================================
# Find Dependencies
#==============================================================================

message(STATUS "Finding dependencies...")

# OpenCASCADE
find_package(OpenCASCADE CONFIG)

if(NOT OpenCASCADE_FOUND)
    message(WARNING "OpenCASCADE not found - Phase 1 will not be built")
endif()

# Eigen3
find_package(Eigen3 3.4 NO_MODULE)

# CGAL
find_package(CGAL COMPONENTS Core)

# FCL (Flexible Collision Library)
find_package(fcl)

# nlohmann_json
find_package(nlohmann_json 3.11)

# spdlog
find_package(spdlog)

# fmt
find_package(fmt)

# Boost (Graph library)
find_package(Boost COMPONENTS graph)

# PugiXML
find_package(pugixml)

# Catch2 (for testing)
if(OPENPANELCAM_BUILD_TESTS)
    find_package(Catch2 3 REQUIRED)
    include(Catch)
endif()

#==============================================================================
# Compiler Warnings
#==============================================================================

if(OPENPANELCAM_ENABLE_WARNINGS)
    if(MSVC)
        add_compile_options(/W4)
        if(OPENPANELCAM_WARNINGS_AS_ERRORS)
            add_compile_options(/WX)
        endif()
    else()
        add_compile_options(-Wall -Wextra -Wpedantic)
        if(OPENPANELCAM_WARNINGS_AS_ERRORS)
            add_compile_options(-Werror)
        endif()
    endif()
endif()

#==============================================================================
# Global Definitions
#==============================================================================

# Disable OCCT deprecation warnings
add_definitions(-DOCCT_NO_DEPRECATED)

# Unicode support for Windows
if(MSVC)
    add_definitions(-DUNICODE -D_UNICODE)
endif()

#==============================================================================
# Include Directories
#==============================================================================

include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

if(OpenCASCADE_FOUND)
    include_directories(${OpenCASCADE_INCLUDE_DIR})
endif()

#==============================================================================
# Subdirectories
#==============================================================================

# Core library (needs dependencies)
if(OpenCASCADE_FOUND AND Eigen3_FOUND AND spdlog_FOUND AND fmt_FOUND)
    add_subdirectory(src/core)
endif()

# Phase modules
if(OpenCASCADE_FOUND)
    add_subdirectory(src/phase1)
endif()
add_subdirectory(src/phase2)
add_subdirectory(src/phase3)
add_subdirectory(src/phase4)
# add_subdirectory(src/phase5)

# Tests
if(OPENPANELCAM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Samples (need core and phase1)
if(OPENPANELCAM_BUILD_SAMPLES AND OpenCASCADE_FOUND AND Eigen3_FOUND AND spdlog_FOUND AND fmt_FOUND)
    add_subdirectory(samples)
endif()

#==============================================================================
# Installation
#==============================================================================

install(DIRECTORY include/openpanelcam
    DESTINATION include
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

#==============================================================================
# Summary
#==============================================================================

message(STATUS "========================================")
message(STATUS "OpenPanelCAM Configuration Summary")
message(STATUS "========================================")
message(STATUS "Version: ${PROJECT_VERSION}")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Tests: ${OPENPANELCAM_BUILD_TESTS}")
message(STATUS "Build Samples: ${OPENPANELCAM_BUILD_SAMPLES}")
message(STATUS "Build Docs: ${OPENPANELCAM_BUILD_DOCS}")
message(STATUS "OpenCASCADE: ${OpenCASCADE_VERSION}")
message(STATUS "Eigen3: ${Eigen3_VERSION}")
message(STATUS "========================================")
