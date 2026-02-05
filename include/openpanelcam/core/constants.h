#pragma once

/**
 * @file constants.h
 * @brief Global constants for OpenPanelCAM
 */

namespace openpanelcam {
namespace constants {

//=============================================================================
// Geometric Tolerances
//=============================================================================

constexpr double LINEAR_TOLERANCE = 1e-6;      // mm
constexpr double ANGULAR_TOLERANCE = 0.01;     // radians (~0.57 degrees)
constexpr double AREA_TOLERANCE = 0.01;        // mm²
constexpr double COPLANARITY_TOLERANCE = 0.1;  // mm

//=============================================================================
// Edge Classification
//=============================================================================

constexpr double MIN_BEND_RADIUS = 0.1;        // mm
constexpr double MAX_BEND_RADIUS = 50.0;       // mm

//=============================================================================
// Bend Classification
//=============================================================================

constexpr double HEM_ANGLE_THRESHOLD = 45.0;   // degrees
constexpr double ACUTE_ANGLE_THRESHOLD = 90.0; // degrees
constexpr double RIGHT_ANGLE_TOLERANCE = 2.0;  // degrees

//=============================================================================
// SDF Validation
//=============================================================================

constexpr double SDF_EPSILON = 0.1;            // mm
constexpr double SDF_TOLERANCE = 1e-6;         // mm

//=============================================================================
// Machine Limits (Salvagnini P4-2520)
//=============================================================================

constexpr double MAX_SHEET_LENGTH = 2500.0;    // mm
constexpr double MAX_SHEET_WIDTH = 1500.0;     // mm
constexpr double MIN_THICKNESS = 0.4;          // mm
constexpr double MAX_THICKNESS = 3.2;          // mm
constexpr double MIN_BEND_ANGLE = -135.0;      // degrees
constexpr double MAX_BEND_ANGLE = 135.0;       // degrees

//=============================================================================
// Grasp Constraints
//=============================================================================

constexpr double MIN_GRIP_AREA = 100.0;        // mm²
constexpr double MAX_COM_DISTANCE = 500.0;     // mm

//=============================================================================
// Version
//=============================================================================

constexpr const char* VERSION = "0.1.0";

} // namespace constants
} // namespace openpanelcam
