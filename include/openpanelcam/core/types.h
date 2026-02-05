#pragma once

/**
 * @file types.h
 * @brief Core type definitions for OpenPanelCAM
 */

#include <string>
#include <vector>
#include <optional>
#include <memory>

// OpenCASCADE includes
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax1.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <Bnd_Box.hxx>

namespace openpanelcam {

//=============================================================================
// Enumerations
//=============================================================================

enum class FaceType {
    PLANAR,
    CYLINDRICAL,
    CONICAL,
    FREEFORM,
    UNKNOWN
};

enum class BendDirection {
    BEND_UP,
    BEND_DOWN,
    HEM,
    UNKNOWN
};

enum class BendConvexity {
    CONVEX,
    CONCAVE,
    UNKNOWN
};

//=============================================================================
// Type Aliases
//=============================================================================

using FaceId = int;
using EdgeId = int;
using NodeId = int;

//=============================================================================
// Common Structures
//=============================================================================

struct MaterialProperties {
    std::string name;
    double yieldStrength;    // MPa
    double thickness;        // mm
    double density;          // kg/m³
};

struct ValidationResult {
    bool isValid;
    std::string errorMessage;
    std::vector<std::string> warnings;
};

} // namespace openpanelcam
