/**
 * @file geometry_utils.cpp
 * @brief Geometry utility functions implementation
 */

#include <openpanelcam/core/geometry_utils.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>

// OCCT includes
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_CurveType.hxx>

#include <cmath>

namespace openpanelcam {

//=============================================================================
// SDFValidationResult helpers
//=============================================================================

std::string SDFValidationResult::getStateString() const {
    switch (classifierState) {
        case TopAbs_OUT: return "OUTSIDE";
        case TopAbs_IN: return "INSIDE";
        case TopAbs_ON: return "ON_BOUNDARY";
        default: return "UNKNOWN";
    }
}

//=============================================================================
// SurfaceAnalyzer Implementation
//=============================================================================

bool SurfaceAnalyzer::isPlanar(const TopoDS_Face& face) {
    if (face.IsNull()) return false;

    BRepAdaptor_Surface adaptor(face);
    return adaptor.GetType() == GeomAbs_Plane;
}

bool SurfaceAnalyzer::isCylindrical(const TopoDS_Face& face) {
    if (face.IsNull()) return false;

    BRepAdaptor_Surface adaptor(face);
    return adaptor.GetType() == GeomAbs_Cylinder;
}

bool SurfaceAnalyzer::isConical(const TopoDS_Face& face) {
    if (face.IsNull()) return false;

    BRepAdaptor_Surface adaptor(face);
    return adaptor.GetType() == GeomAbs_Cone;
}

std::optional<gp_Pln> SurfaceAnalyzer::getPlane(const TopoDS_Face& face) {
    if (!isPlanar(face)) {
        return std::nullopt;
    }

    BRepAdaptor_Surface adaptor(face);
    return adaptor.Plane();
}

std::optional<gp_Cylinder> SurfaceAnalyzer::getCylinder(const TopoDS_Face& face) {
    if (!isCylindrical(face)) {
        return std::nullopt;
    }

    BRepAdaptor_Surface adaptor(face);
    return adaptor.Cylinder();
}

double SurfaceAnalyzer::computeArea(const TopoDS_Face& face) {
    if (face.IsNull()) {
        return 0.0;
    }

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.Mass();  // Mass() returns area for surface
}

gp_Pnt SurfaceAnalyzer::computeCentroid(const TopoDS_Face& face) {
    if (face.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    GProp_GProps props;
    BRepGProp::SurfaceProperties(face, props);
    return props.CentreOfMass();
}

Bnd_Box SurfaceAnalyzer::computeBoundingBox(const TopoDS_Face& face) {
    Bnd_Box box;
    if (!face.IsNull()) {
        BRepBndLib::Add(face, box);
    }
    return box;
}

gp_Dir SurfaceAnalyzer::getMaterialOutwardNormal(const TopoDS_Face& face) {
    if (face.IsNull()) {
        THROW_PHASE1(ErrorCode::OCCT_EXCEPTION,
                    "Cannot compute normal of null face",
                    "Face is null");
    }

    // Get UV bounds
    double uMin, uMax, vMin, vMax;
    getUVBounds(face, uMin, uMax, vMin, vMax);

    // Compute mid-point parameters
    double uMid = (uMin + uMax) / 2.0;
    double vMid = (vMin + vMax) / 2.0;

    // Get surface handle
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face);

    // Compute surface properties at mid-point
    GeomLProp_SLProps props(surface, uMid, vMid, 1, constants::LINEAR_TOLERANCE);

    if (!props.IsNormalDefined()) {
        THROW_PHASE1(ErrorCode::OCCT_EXCEPTION,
                    "Normal not defined at face center",
                    "Surface is degenerate or singular");
    }

    // Get geometric normal
    gp_Dir geoNormal = props.Normal();

    // CRITICAL: Correct for face orientation
    // TopAbs_REVERSED faces have flipped normals
    if (face.Orientation() == TopAbs_REVERSED) {
        geoNormal.Reverse();
    }

    return geoNormal;
}

SDFValidationResult SurfaceAnalyzer::validateNormal(
    const TopoDS_Face& face,
    const gp_Dir& normal,
    const TopoDS_Solid& solid
) {
    SDFValidationResult result;
    result.isValid = true;
    result.needsReversal = false;
    result.signedDistance = 0.0;

    if (solid.IsNull()) {
        LOG_WARNING("Cannot validate normal - solid is null");
        result.classifierState = TopAbs_UNKNOWN;
        return result;
    }

    // Get face centroid
    gp_Pnt centroid = computeCentroid(face);

    // Create test point slightly offset along normal
    const double epsilon = constants::SDF_EPSILON;  // 0.1 mm
    gp_Pnt testPoint = centroid.Translated(gp_Vec(normal.XYZ() * epsilon));

    // Compute Signed Distance Function (SDF)
    BRepClass3d_SolidClassifier classifier(solid);
    classifier.Perform(testPoint, constants::SDF_TOLERANCE);

    result.classifierState = classifier.State();

    // Check classifier state
    if (classifier.State() == TopAbs_OUT) {
        // Test point is outside → normal is OUTWARD (correct)
        result.isValid = true;
        result.needsReversal = false;
    } else if (classifier.State() == TopAbs_IN) {
        // Test point is inside → normal is INWARD (wrong!)
        result.isValid = false;
        result.needsReversal = true;
        LOG_WARNING("Normal validation failed - points inward");
    } else {
        // ON boundary → inconclusive, accept
        result.isValid = true;
        result.needsReversal = false;
    }

    return result;
}

gp_Ax1 SurfaceAnalyzer::extractBendAxis(const TopoDS_Face& cylindricalFace) {
    if (cylindricalFace.IsNull()) {
        THROW_PHASE1(ErrorCode::OCCT_EXCEPTION,
                    "Cannot extract bend axis from null face",
                    "Face is null");
    }

    // Verify it's cylindrical
    BRepAdaptor_Surface adaptor(cylindricalFace);
    if (adaptor.GetType() != GeomAbs_Cylinder) {
        THROW_PHASE1(ErrorCode::OCCT_EXCEPTION,
                    "Cannot extract bend axis - not a cylindrical surface",
                    "Face type is not CYLINDRICAL");
    }

    // Get cylinder parameters
    gp_Cylinder cylinder = adaptor.Cylinder();
    gp_Ax1 axis = cylinder.Axis();

    // Get UV bounds to find axis endpoints
    double uMin, uMax, vMin, vMax;
    getUVBounds(cylindricalFace, uMin, uMax, vMin, vMax);

    // For cylinder: U = angle around axis, V = height along axis
    // Bend line runs along V direction
    Handle(Geom_Surface) surface = BRep_Tool::Surface(cylindricalFace);
    gp_Pnt startPoint = surface->Value(0, vMin);

    return gp_Ax1(startPoint, axis.Direction());
}

void SurfaceAnalyzer::getUVBounds(
    const TopoDS_Face& face,
    double& uMin, double& uMax,
    double& vMin, double& vMax
) {
    if (face.IsNull()) {
        uMin = uMax = vMin = vMax = 0.0;
        return;
    }

    BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
}

SurfaceProperties SurfaceAnalyzer::analyze(const TopoDS_Face& face) {
    SurfaceProperties props;

    if (face.IsNull()) {
        props.type = FaceType::UNKNOWN;
        return props;
    }

    // Classify face type
    BRepAdaptor_Surface adaptor(face);
    GeomAbs_SurfaceType surfType = adaptor.GetType();

    switch (surfType) {
        case GeomAbs_Plane:
            props.type = FaceType::PLANAR;
            props.plane = adaptor.Plane();
            props.normal = getMaterialOutwardNormal(face);
            props.normalValidated = false;  // Call validateNormal() separately
            break;

        case GeomAbs_Cylinder:
            props.type = FaceType::CYLINDRICAL;
            props.cylinder = adaptor.Cylinder();
            props.axis = props.cylinder.Axis();
            props.radius = props.cylinder.Radius();

            // Get axis endpoints
            {
                double uMin, uMax, vMin, vMax;
                getUVBounds(face, uMin, uMax, vMin, vMax);
                Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
                props.axisStart = surf->Value(0, vMin);
                props.axisEnd = surf->Value(0, vMax);
            }
            break;

        case GeomAbs_Cone:
            props.type = FaceType::CONICAL;
            break;

        case GeomAbs_BSplineSurface:
        case GeomAbs_BezierSurface:
            props.type = FaceType::FREEFORM;
            break;

        default:
            props.type = FaceType::UNKNOWN;
            break;
    }

    // Compute common properties
    props.centroid = computeCentroid(face);
    props.area = computeArea(face);
    props.boundingBox = computeBoundingBox(face);

    // Orientation info
    props.orientation = face.Orientation();
    props.wasReversed = (face.Orientation() == TopAbs_REVERSED);

    return props;
}

//=============================================================================
// EdgeAnalyzer Implementation
//=============================================================================

bool EdgeAnalyzer::isLinear(const TopoDS_Edge& edge) {
    if (edge.IsNull()) return false;

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.GetType() == GeomAbs_Line;
}

bool EdgeAnalyzer::isCircular(const TopoDS_Edge& edge) {
    if (edge.IsNull()) return false;

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.GetType() == GeomAbs_Circle;
}

std::optional<gp_Lin> EdgeAnalyzer::getLine(const TopoDS_Edge& edge) {
    if (!isLinear(edge)) {
        return std::nullopt;
    }

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.Line();
}

std::optional<gp_Circ> EdgeAnalyzer::getCircle(const TopoDS_Edge& edge) {
    if (!isCircular(edge)) {
        return std::nullopt;
    }

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.Circle();
}

double EdgeAnalyzer::computeLength(const TopoDS_Edge& edge) {
    if (edge.IsNull()) {
        return 0.0;
    }

    GProp_GProps props;
    BRepGProp::LinearProperties(edge, props);
    return props.Mass();  // Mass() returns length for curves
}

gp_Pnt EdgeAnalyzer::getStartPoint(const TopoDS_Edge& edge) {
    if (edge.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.Value(adaptor.FirstParameter());
}

gp_Pnt EdgeAnalyzer::getEndPoint(const TopoDS_Edge& edge) {
    if (edge.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    BRepAdaptor_Curve adaptor(edge);
    return adaptor.Value(adaptor.LastParameter());
}

gp_Pnt EdgeAnalyzer::getMidpoint(const TopoDS_Edge& edge) {
    if (edge.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    BRepAdaptor_Curve adaptor(edge);
    double tMid = (adaptor.FirstParameter() + adaptor.LastParameter()) / 2.0;
    return adaptor.Value(tMid);
}

EdgeProperties EdgeAnalyzer::analyze(const TopoDS_Edge& edge) {
    EdgeProperties props;

    if (edge.IsNull()) {
        props.isLinear = false;
        props.isCircular = false;
        props.length = 0.0;
        return props;
    }

    BRepAdaptor_Curve adaptor(edge);
    GeomAbs_CurveType curveType = adaptor.GetType();

    props.isLinear = (curveType == GeomAbs_Line);
    props.isCircular = (curveType == GeomAbs_Circle);
    props.length = computeLength(edge);
    props.startPoint = getStartPoint(edge);
    props.endPoint = getEndPoint(edge);
    props.midpoint = getMidpoint(edge);

    if (props.isLinear) {
        props.line = adaptor.Line();
    }

    if (props.isCircular) {
        props.circle = adaptor.Circle();
        props.radius = props.circle.Radius();
    } else {
        props.radius = 0.0;
    }

    return props;
}

//=============================================================================
// AngleCalculator Implementation
//=============================================================================

DihedralAngleResult AngleCalculator::calculateDihedralAngle(
    const TopoDS_Face& face1,
    const TopoDS_Face& face2,
    const TopoDS_Edge& sharedEdge
) {
    DihedralAngleResult result;
    result.angle = 0.0;
    result.openingAngle = 0.0;
    result.bendAngle = 0.0;
    result.isReflex = false;

    // Get material-outward normals
    gp_Dir N1 = SurfaceAnalyzer::getMaterialOutwardNormal(face1);
    gp_Dir N2 = SurfaceAnalyzer::getMaterialOutwardNormal(face2);

    // Compute geometric angle between normals
    double cosTheta = N1.Dot(N2);
    cosTheta = GeometryUtils::clamp(cosTheta, -1.0, 1.0);  // Safety for acos
    double angle = std::acos(cosTheta);  // Radians

    // Determine sign using cross product
    result.crossProduct = N1.Crossed(N2);

    // Get edge direction
    if (!sharedEdge.IsNull()) {
        BRepAdaptor_Curve curve(sharedEdge);
        gp_Pnt P1 = curve.Value(curve.FirstParameter());
        gp_Pnt P2 = curve.Value(curve.LastParameter());
        gp_Vec edgeVec(P1, P2);

        if (edgeVec.Magnitude() > constants::LINEAR_TOLERANCE) {
            gp_Dir edgeDir(edgeVec);

            // Check alignment of cross product with edge direction
            double dot = result.crossProduct.Dot(edgeDir);
            if (dot < 0) {
                // Normals form "outward" angle (reflex)
                angle = 2.0 * M_PI - angle;
                result.isReflex = true;
            }
        }
    }

    // Convert to degrees
    result.openingAngle = angle * 180.0 / M_PI;
    result.bendAngle = 180.0 - result.openingAngle;
    result.angle = result.openingAngle;

    return result;
}

double AngleCalculator::angleFromNormals(
    const gp_Dir& normal1,
    const gp_Dir& normal2,
    const gp_Dir& referenceAxis
) {
    // Compute angle between normals
    double cosTheta = normal1.Dot(normal2);
    cosTheta = GeometryUtils::clamp(cosTheta, -1.0, 1.0);
    double angle = std::acos(cosTheta);

    // Determine sign using cross product
    gp_Dir cross = normal1.Crossed(normal2);
    double dot = cross.Dot(referenceAxis);

    if (dot < 0) {
        angle = -angle;
    }

    // Convert to degrees
    return angle * 180.0 / M_PI;
}

double AngleCalculator::calculateSignedAngle(
    const gp_Dir& baseNormal,
    const gp_Dir& flangeNormal,
    const gp_Dir& bendAxis
) {
    return angleFromNormals(baseNormal, flangeNormal, bendAxis);
}

//=============================================================================
// GeometryUtils namespace
//=============================================================================

namespace GeometryUtils {

gp_Pnt computeSolidCentroid(const TopoDS_Solid& solid) {
    if (solid.IsNull()) {
        return gp_Pnt(0, 0, 0);
    }

    GProp_GProps props;
    BRepGProp::VolumeProperties(solid, props);
    return props.CentreOfMass();
}

double computeVolume(const TopoDS_Solid& solid) {
    if (solid.IsNull()) {
        return 0.0;
    }

    GProp_GProps props;
    BRepGProp::VolumeProperties(solid, props);
    return props.Mass();  // Mass() returns volume for solids
}

Bnd_Box computeBoundingBox(const TopoDS_Shape& shape) {
    Bnd_Box box;
    if (!shape.IsNull()) {
        BRepBndLib::Add(shape, box);
    }
    return box;
}

bool arePointsCoincident(const gp_Pnt& p1, const gp_Pnt& p2, double tolerance) {
    return p1.Distance(p2) < tolerance;
}

bool areDirectionsParallel(const gp_Dir& d1, const gp_Dir& d2, double angleTolerance) {
    double dot = std::abs(d1.Dot(d2));
    // If parallel, dot product = ±1
    return (1.0 - dot) < angleTolerance;
}

} // namespace GeometryUtils

} // namespace openpanelcam
