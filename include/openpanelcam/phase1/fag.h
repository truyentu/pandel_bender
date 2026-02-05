#pragma once

/**
 * @file fag.h
 * @brief Face-Adjacency Graph data structures
 *
 * Core data model for Phase 1:
 * - FAG_Node: Represents a face in the part
 * - FAG_Edge: Represents connection between faces (bend or sharp edge)
 * - BendFeature: Complete description of a bend operation
 * - FaceAdjacencyGraph: Graph container with analysis methods
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/core/constants.h>

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Pln.hxx>
#include <gp_Lin.hxx>
#include <Bnd_Box.hxx>

#include <string>
#include <vector>
#include <map>
#include <unordered_map>

namespace openpanelcam {

// Forward declarations
class FaceAdjacencyGraph;
struct BendFeature;

//=============================================================================
// FAG_Node - Represents a face in the part
//=============================================================================

struct FAG_Node {
    //=========================================================================
    // Identity
    //=========================================================================
    int id;                          // Unique ID (0-based index)
    std::string name;                // Debug name: "Face_0", "Flange_A"

    //=========================================================================
    // OCCT Reference
    //=========================================================================
    TopoDS_Face face;                // Handle to OCCT face
                                     // NOTE: Reference-counted, safe to copy

    //=========================================================================
    // Classification
    //=========================================================================
    FaceType type;                   // PLANAR, CYLINDRICAL, etc.

    // Face role in part
    bool isBaseFace;                 // True if this is the identified base
    bool isFlange;                   // True if connected to base via bend
    bool isBendSurface;              // True if cylindrical (bend radius)
    int flangeLevel;                 // Distance from base (0=base, 1=direct, etc.)

    //=========================================================================
    // Geometric Properties (computed once, cached)
    //=========================================================================
    gp_Dir normal;                   // Material-outward normal
                                     // IMPORTANT: Corrected for TopAbs_REVERSED

    gp_Pnt centroid;                 // Geometric center of face
    double area;                     // Surface area in mm²
    Bnd_Box boundingBox;             // Axis-aligned bounding box

    // For planar faces
    gp_Pln plane;                    // The underlying plane (if PLANAR)

    // For cylindrical faces
    gp_Ax1 cylinderAxis;             // Axis of cylinder (if CYLINDRICAL)
    double cylinderRadius;           // Radius (if CYLINDRICAL)

    //=========================================================================
    // Adjacency
    //=========================================================================
    std::vector<int> adjacentEdges;  // IDs of edges connected to this node

    //=========================================================================
    // Methods
    //=========================================================================

    // Check if this is a valid, non-degenerate face
    bool isValid() const {
        return !face.IsNull() && area > constants::AREA_TOLERANCE;
    }

    // Default constructor
    FAG_Node()
        : id(-1)
        , type(FaceType::UNKNOWN)
        , isBaseFace(false)
        , isFlange(false)
        , isBendSurface(false)
        , flangeLevel(-1)
        , area(0.0)
        , cylinderRadius(0.0)
    {}
};

//=============================================================================
// FAG_Edge - Connection between faces (bend or sharp edge)
//=============================================================================

struct FAG_Edge {
    //=========================================================================
    // Bend Type Enum
    //=========================================================================
    enum class BendType {
        ACUTE,                       // < 90°
        RIGHT,                       // = 90° (±tolerance)
        OBTUSE,                      // > 90°
        HEM,                         // < 45° (return flange)
        UNKNOWN
    };

    //=========================================================================
    // Identity
    //=========================================================================
    int id;                          // Unique ID (0-based index)

    //=========================================================================
    // Connected Nodes
    //=========================================================================
    int node1;                       // First face ID
    int node2;                       // Second face ID

    //=========================================================================
    // Edge Type
    //=========================================================================
    bool isBend;                     // true = bend, false = sharp edge

    //=========================================================================
    // Orientation Tracking (from S1)
    //=========================================================================
    bool isReversed;                 // Edge traversal direction
    bool isInternal;                 // Internal vs boundary edge

    //=========================================================================
    // Shared Geometry
    //=========================================================================
    TopoDS_Edge sharedEdge;          // Common edge (for sharp edges)
    TopoDS_Face bendFace;            // Cylindrical surface (for bends)

    //=========================================================================
    // Bend Properties (valid if isBend == true)
    //=========================================================================

    // Geometry
    gp_Ax1 bendAxis;                 // Axis of rotation
    gp_Pnt bendAxisStart;            // Start point of bend line
    gp_Pnt bendAxisEnd;              // End point of bend line
    double bendLength;               // Length along bend line (mm)
    double bendRadius;               // Internal radius (mm)

    // Angle
    double bendAngle;                // Absolute angle (0-180°)
    double signedAngle;              // Signed: +up, -down

    // Classification
    BendConvexity convexity;         // CONVEX or CONCAVE
    BendDirection direction;         // UP, DOWN, or HEM
    BendType bendType;               // ACUTE, RIGHT, OBTUSE, HEM

    //=========================================================================
    // Sharp Edge Properties (valid if isBend == false)
    //=========================================================================
    double edgeLength;               // Length of shared edge
    gp_Pnt edgeMidpoint;            // Center point of edge

    //=========================================================================
    // Methods
    //=========================================================================

    // Get the "other" node given one node ID
    int otherNode(int nodeId) const {
        return (nodeId == node1) ? node2 : node1;
    }

    // Check validity
    bool isValid() const {
        return node1 >= 0 && node2 >= 0 && node1 != node2;
    }

    // Check if this is a hem
    bool isHem() const {
        return bendType == BendType::HEM || std::abs(bendAngle) < 45.0;
    }

    // Get bend line as line segment
    gp_Lin getBendLineSegment() const {
        gp_Vec vec(bendAxisStart, bendAxisEnd);
        if (vec.Magnitude() > constants::LINEAR_TOLERANCE) {
            gp_Dir dir(vec);
            return gp_Lin(bendAxisStart, dir);
        }
        return gp_Lin(bendAxisStart, gp_Dir(0, 0, 1));  // Fallback
    }

    // Default constructor
    FAG_Edge()
        : id(-1)
        , node1(-1)
        , node2(-1)
        , isBend(false)
        , isReversed(false)
        , isInternal(true)
        , bendLength(0.0)
        , bendRadius(0.0)
        , bendAngle(0.0)
        , signedAngle(0.0)
        , convexity(BendConvexity::UNKNOWN)
        , direction(BendDirection::UNKNOWN)
        , bendType(BendType::UNKNOWN)
        , edgeLength(0.0)
    {}
};

//=============================================================================
// BendFeature - Complete description of a bend operation
//=============================================================================

struct BendFeature {
    //=========================================================================
    // Identity & References
    //=========================================================================
    int id;                          // Unique bend ID
    int fagEdgeId;                   // Reference to FAG_Edge
    std::string name;                // e.g., "Bend_1"

    //=========================================================================
    // Participating Faces
    //=========================================================================

    // The "fixed" side - connected to base
    FaceId baseFaceId;               // Index in FAG
    TopoDS_Face baseFace;            // Direct handle

    // The cylindrical bend surface
    FaceId bendFaceId;               // Index in FAG (-1 for zero-radius)
    TopoDS_Face bendFace;            // Direct handle

    // The "moving" side - the flange
    FaceId flangeFaceId;             // Index in FAG
    TopoDS_Face flangeFace;          // Direct handle

    //=========================================================================
    // Base Side Geometry
    //=========================================================================
    gp_Pnt baseCentroid;             // Center of base face
    gp_Dir baseNormal;               // Material-outward normal

    //=========================================================================
    // Flange Side Geometry
    //=========================================================================
    gp_Pnt flangeCentroid;           // Center of flange face
    gp_Dir flangeNormal;             // Material-outward normal

    // Flange dimensions
    double flangeLength;             // Distance from bend to far edge (mm)
    double flangeWidth;              // Width along bend line (mm)
    double flangeArea;               // Surface area (mm²)

    //=========================================================================
    // Bend Line Definition
    //=========================================================================
    gp_Lin bendLine;                 // The axis of rotation
    gp_Pnt bendLineStart;            // Start point
    gp_Pnt bendLineEnd;              // End point
    double bendLineLength;           // Length (mm)

    //=========================================================================
    // Bend Parameters
    //=========================================================================

    // Angles
    double targetAngle;              // Absolute angle (0-180°)
    double signedAngle;              // Signed: +up, -down

    // Radius
    double internalRadius;           // Inside radius (mm)
    double kFactor;                  // K-factor for this bend

    //=========================================================================
    // Classification
    //=========================================================================
    BendDirection direction;         // UP, DOWN, HEM
    BendConvexity convexity;         // CONVEX, CONCAVE

    // Classification confidence
    double signedDistance;           // SDF value used for classification

    //=========================================================================
    // Constraints (populated by Phase 2)
    //=========================================================================
    std::vector<int> mustBendBefore; // Bends that must happen first
    std::vector<int> mustBendAfter;  // Bends that must happen after

    //=========================================================================
    // Sequence Info (populated by Phase 3)
    //=========================================================================
    int sequencePosition;            // Order in sequence (-1 = not set)

    //=========================================================================
    // Methods
    //=========================================================================

    // Get bend axis as OCCT type
    gp_Ax1 getBendAxis() const {
        gp_Pnt midpoint((bendLineStart.XYZ() + bendLineEnd.XYZ()) / 2.0);
        gp_Vec vec(bendLineStart, bendLineEnd);
        if (vec.Magnitude() > constants::LINEAR_TOLERANCE) {
            return gp_Ax1(midpoint, gp_Dir(vec));
        }
        return gp_Ax1(midpoint, gp_Dir(0, 0, 1));  // Fallback
    }

    // Check if this is a hem
    bool isHem() const {
        return direction == BendDirection::HEM || targetAngle < 45.0;
    }

    // Default constructor
    BendFeature()
        : id(-1)
        , fagEdgeId(-1)
        , baseFaceId(-1)
        , bendFaceId(-1)
        , flangeFaceId(-1)
        , flangeLength(0.0)
        , flangeWidth(0.0)
        , flangeArea(0.0)
        , bendLineLength(0.0)
        , targetAngle(0.0)
        , signedAngle(0.0)
        , internalRadius(0.0)
        , kFactor(0.5)
        , direction(BendDirection::UNKNOWN)
        , convexity(BendConvexity::UNKNOWN)
        , signedDistance(0.0)
        , sequencePosition(-1)
    {}
};

//=============================================================================
// FaceAdjacencyGraph - Main container class
//=============================================================================

class FaceAdjacencyGraph {
public:
    //=========================================================================
    // Construction
    //=========================================================================

    FaceAdjacencyGraph();

    // Clear all data
    void clear();

    // Reserve memory (optimization)
    void reserve(size_t nodeCount, size_t edgeCount);

    //=========================================================================
    // Building the Graph
    //=========================================================================

    // Add a face node, returns the new node ID
    int addNode(const TopoDS_Face& face);

    // Add a sharp edge between two nodes
    int addEdge(int node1, int node2, const TopoDS_Edge& sharedEdge);

    // Add a bend edge (with cylindrical face)
    int addBendEdge(int node1, int node2, const TopoDS_Face& bendFace);

    // Finalize construction (compute all derived properties)
    void finalize();

    //=========================================================================
    // Accessors
    //=========================================================================

    // Node access
    const FAG_Node& getNode(int id) const;
    FAG_Node& getNode(int id);
    size_t nodeCount() const { return m_nodes.size(); }

    // Edge access
    const FAG_Edge& getEdge(int id) const;
    FAG_Edge& getEdge(int id);
    size_t edgeCount() const { return m_edges.size(); }

    // Iteration
    const std::vector<FAG_Node>& nodes() const { return m_nodes; }
    const std::vector<FAG_Edge>& edges() const { return m_edges; }

    //=========================================================================
    // Queries
    //=========================================================================

    // Get all edges for a node
    std::vector<int> getEdgesForNode(int nodeId) const;

    // Get all adjacent nodes (connected by any edge)
    std::vector<int> getAdjacentNodes(int nodeId) const;

    // Get adjacent nodes connected by bends only
    std::vector<int> getBendAdjacentNodes(int nodeId) const;

    // Find edge between two nodes (returns -1 if none)
    int findEdgeBetween(int node1, int node2) const;

    // Get all bend edges
    std::vector<int> getBendEdges() const;

    // Get number of bends
    size_t bendCount() const;

    //=========================================================================
    // Analysis
    //=========================================================================

    // Set base face and compute flange levels
    void setBaseFace(int nodeId);

    // Get base face ID
    int getBaseFaceId() const { return m_baseFaceId; }

    // Check if graph is connected
    bool isConnected() const;

    // Get part bounding box (union of all face boxes)
    Bnd_Box getPartBoundingBox() const;

    //=========================================================================
    // Validation
    //=========================================================================

    // Full validation with error reporting
    ValidationResult validate() const;

    // Quick validity check
    bool isValid() const;

private:
    //=========================================================================
    // Data
    //=========================================================================
    std::vector<FAG_Node> m_nodes;
    std::vector<FAG_Edge> m_edges;

    // Quick lookup: (node1, node2) -> edge ID
    std::map<std::pair<int,int>, int> m_nodePairToEdge;

    // Identified base face
    int m_baseFaceId;

    // Finalization flag
    bool m_finalized;

    //=========================================================================
    // Internal Methods
    //=========================================================================

    // Compute geometric properties for a node
    void computeNodeGeometry(FAG_Node& node);

    // Compute properties for a bend edge
    void computeBendProperties(FAG_Edge& edge);

    // Compute flange levels via BFS from base
    void computeFlangeLevels();
};

} // namespace openpanelcam
