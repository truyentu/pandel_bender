#pragma once

/**
 * @file geometry_utils.h
 * @brief OCCT geometry utility functions
 *
 * Các helper functions để làm việc với OpenCASCADE geometry:
 * - Surface analysis (planar, cylindrical detection)
 * - Normal computation với SDF validation
 * - Edge classification
 * - Dihedral angle calculation
 * - Bend axis extraction
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/core/constants.h>

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Circ.hxx>
#include <gp_Lin.hxx>
#include <Bnd_Box.hxx>
#include <TopAbs_State.hxx>

#include <optional>

namespace openpanelcam {

//=============================================================================
// Surface Analysis Results
//=============================================================================

struct SurfaceProperties {
    FaceType type;

    // For all types
    gp_Pnt centroid;
    double area;
    Bnd_Box boundingBox;

    // For planar faces
    gp_Pln plane;
    gp_Dir normal;
    bool normalValidated;        // SDF validation passed

    // For cylindrical faces
    gp_Cylinder cylinder;
    gp_Ax1 axis;
    double radius;
    gp_Pnt axisStart;            // Bend line start
    gp_Pnt axisEnd;              // Bend line end

    // Orientation info
    TopAbs_Orientation orientation;
    bool wasReversed;            // True if normal was flipped
};

struct EdgeProperties {
    bool isLinear;
    bool isCircular;
    double length;
    gp_Pnt startPoint;
    gp_Pnt endPoint;
    gp_Pnt midpoint;

    // For linear edges
    gp_Lin line;

    // For circular edges
    gp_Circ circle;
    double radius;
};

struct DihedralAngleResult {
    double angle;                // In degrees (0-360)
    double openingAngle;         // Angle between normals
    double bendAngle;            // 180 - opening angle
    gp_Dir crossProduct;         // N1 × N2
    bool isReflex;               // True if > 180°

    double getBendAngle() const {
        return 180.0 - openingAngle;
    }
};

struct SDFValidationResult {
    bool isValid;                // True if normal points outward
    TopAbs_State classifierState;// OUT, IN, or ON
    double signedDistance;       // Approximate SDF value
    bool needsReversal;          // True if normal should be flipped

    std::string getStateString() const;
};

//=============================================================================
// Surface Analyzer Class
//=============================================================================

/**
 * @brief Utility class for analyzing OCCT surfaces
 *
 * Static methods for:
 * - Face type classification
 * - Material-outward normal computation (with TopAbs_REVERSED correction)
 * - SDF validation (from S3)
 * - Bend axis extraction (from S1)
 */
class SurfaceAnalyzer {
public:
    // Analyze a face and return all properties
    static SurfaceProperties analyze(const TopoDS_Face& face);

    // Type checks
    static bool isPlanar(const TopoDS_Face& face);
    static bool isCylindrical(const TopoDS_Face& face);
    static bool isConical(const TopoDS_Face& face);

    // Get specific surface primitives
    static std::optional<gp_Pln> getPlane(const TopoDS_Face& face);
    static std::optional<gp_Cylinder> getCylinder(const TopoDS_Face& face);

    // Measurements
    static double computeArea(const TopoDS_Face& face);
    static gp_Pnt computeCentroid(const TopoDS_Face& face);
    static Bnd_Box computeBoundingBox(const TopoDS_Face& face);

    /**
     * @brief Get material-outward normal at face centroid
     * @param face The face to analyze
     * @return Unit normal vector pointing OUT of material
     *
     * IMPORTANT: Corrects for TopAbs_REVERSED orientation!
     */
    static gp_Dir getMaterialOutwardNormal(const TopoDS_Face& face);

    /**
     * @brief Validate normal direction using SDF (from S3)
     * @param face The face
     * @param normal The computed normal
     * @param solid The containing solid
     * @return Validation result with classifier state
     *
     * Algorithm:
     * 1. Create test point: P_test = centroid + epsilon * normal
     * 2. Classify P_test relative to solid (IN, OUT, ON)
     * 3. If OUT → normal is outward (correct)
     * 4. If IN → normal is inward (needs reversal)
     */
    static SDFValidationResult validateNormal(
        const TopoDS_Face& face,
        const gp_Dir& normal,
        const TopoDS_Solid& solid
    );

    /**
     * @brief Extract bend axis from cylindrical face (from S1)
     * @param cylindricalFace The bend surface
     * @return Axis of rotation (origin + direction)
     *
     * Extracts:
     * - Cylinder axis direction
     * - Axis position
     * - Start/end points along axis
     */
    static gp_Ax1 extractBendAxis(const TopoDS_Face& cylindricalFace);

    /**
     * @brief Get UV parameter bounds of face
     */
    static void getUVBounds(
        const TopoDS_Face& face,
        double& uMin, double& uMax,
        double& vMin, double& vMax
    );
};

//=============================================================================
// Edge Analyzer Class
//=============================================================================

/**
 * @brief Utility class for analyzing OCCT edges
 */
class EdgeAnalyzer {
public:
    // Analyze an edge
    static EdgeProperties analyze(const TopoDS_Edge& edge);

    // Type checks
    static bool isLinear(const TopoDS_Edge& edge);
    static bool isCircular(const TopoDS_Edge& edge);

    // Get geometric primitives
    static std::optional<gp_Lin> getLine(const TopoDS_Edge& edge);
    static std::optional<gp_Circ> getCircle(const TopoDS_Edge& edge);

    // Measurements
    static double computeLength(const TopoDS_Edge& edge);
    static gp_Pnt getStartPoint(const TopoDS_Edge& edge);
    static gp_Pnt getEndPoint(const TopoDS_Edge& edge);
    static gp_Pnt getMidpoint(const TopoDS_Edge& edge);
};

//=============================================================================
// Angle Calculator Class
//=============================================================================

/**
 * @brief Utility class for bend angle calculations
 */
class AngleCalculator {
public:
    /**
     * @brief Calculate dihedral angle between two faces (from S2)
     * @param face1 First face
     * @param face2 Second face
     * @param sharedEdge Common edge
     * @return Complete angle result with opening/bend angles
     *
     * Algorithm:
     * 1. Get material-outward normals N1, N2
     * 2. Compute geometric angle: acos(N1·N2)
     * 3. Determine sign using cross product: N1×N2
     * 4. Return both opening angle and bend angle
     */
    static DihedralAngleResult calculateDihedralAngle(
        const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& sharedEdge
    );

    /**
     * @brief Calculate angle between two normals
     * @param normal1 First normal
     * @param normal2 Second normal
     * @param referenceAxis Optional axis for sign determination
     * @return Angle in degrees
     */
    static double angleFromNormals(
        const gp_Dir& normal1,
        const gp_Dir& normal2,
        const gp_Dir& referenceAxis
    );

    /**
     * @brief Calculate signed bend angle
     * @param baseNormal Base face normal
     * @param flangeNormal Flange face normal
     * @param bendAxis Bend axis direction
     * @return Signed angle (positive = up, negative = down)
     */
    static double calculateSignedAngle(
        const gp_Dir& baseNormal,
        const gp_Dir& flangeNormal,
        const gp_Dir& bendAxis
    );
};

//=============================================================================
// Geometry Utilities
//=============================================================================

namespace GeometryUtils {

/**
 * @brief Compute centroid of a solid
 */
gp_Pnt computeSolidCentroid(const TopoDS_Solid& solid);

/**
 * @brief Compute volume of a solid
 */
double computeVolume(const TopoDS_Solid& solid);

/**
 * @brief Compute bounding box of a shape
 */
Bnd_Box computeBoundingBox(const TopoDS_Shape& shape);

/**
 * @brief Check if two points are coincident within tolerance
 */
bool arePointsCoincident(
    const gp_Pnt& p1,
    const gp_Pnt& p2,
    double tolerance = constants::LINEAR_TOLERANCE
);

/**
 * @brief Check if two directions are parallel
 */
bool areDirectionsParallel(
    const gp_Dir& d1,
    const gp_Dir& d2,
    double angleTolerance = constants::ANGULAR_TOLERANCE
);

/**
 * @brief Clamp value to range [-1, 1] (for acos safety)
 */
inline double clamp(double value, double min, double max) {
    return (value < min) ? min : (value > max) ? max : value;
}

} // namespace GeometryUtils

} // namespace openpanelcam
