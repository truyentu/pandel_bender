# Phase 1: Data Structure Specifications

> **Version**: 2.0
> **Phase**: Geometric Parser
> **Last Updated**: 2026-02-04
> **Changes**: Added fields from research papers S1-S4 (orientation tracking, SDF validation, bend axis details)

---

## 1. Overview

Document này mô tả chi tiết các data structures sử dụng trong Phase 1, bao gồm memory layout, relationships, và usage patterns.

---

## 2. Face-Adjacency Graph (FAG)

### 2.1 Conceptual Model

```
FAG là một undirected graph:
  - Nodes = Faces của part (planar hoặc cylindrical)
  - Edges = Relationships giữa các faces (bend hoặc sharp edge)

Ví dụ: Simple Box (4 bends)

        ┌─────────────┐
        │   TOP (0)   │
        └──────┬──────┘
               │ bend
        ┌──────┴──────┐
        │  BASE (1)   │◄── Base Face
        └──────┬──────┘
               │ bend
        ┌──────┴──────┐
        │  BOTTOM (2) │
        └─────────────┘

Graph representation:
  Node 0 (TOP)    ──bend──  Node 1 (BASE)
  Node 1 (BASE)   ──bend──  Node 2 (BOTTOM)
  Node 1 (BASE)   ──bend──  Node 3 (LEFT)
  Node 1 (BASE)   ──bend──  Node 4 (RIGHT)
```

### 2.2 Node Structure

```cpp
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
                                     // NOTE: This is a reference-counted handle
                                     // Safe to copy, but underlying shape must
                                     // remain valid

    //=========================================================================
    // Classification
    //=========================================================================
    FaceType type;                   // PLANAR, CYLINDRICAL, etc.

    // Face role in part
    bool isBaseFace;                 // True if this is the identified base
    bool isFlange;                   // True if connected to base via bend
    bool isBendSurface;              // True if cylindrical (bend radius)
    int flangeLevel;                 // Distance from base (0=base, 1=direct flange, etc.)

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
    // Adjacency (indices into FAG edge list)
    //=========================================================================
    std::vector<int> adjacentEdges;  // IDs of edges connected to this node

    //=========================================================================
    // Methods
    //=========================================================================

    // Check if this is a valid, non-degenerate face
    bool isValid() const {
        return !face.IsNull() && area > Constants::AREA_TOLERANCE;
    }

    // Get number of bend edges connected
    int bendEdgeCount() const;

    // Get list of adjacent nodes (through edges)
    std::vector<int> getAdjacentNodes(const FaceAdjacencyGraph& fag) const;
};

// Memory layout: ~200 bytes per node (excluding OCCT handle overhead)
```

### 2.3 Edge Structure

```cpp
struct FAG_Edge {
    //=========================================================================
    // Identity
    //=========================================================================
    int id;                          // Unique ID (0-based index)

    //=========================================================================
    // Connected Nodes
    //=========================================================================
    int node1;                       // First face ID
    int node2;                       // Second face ID
                                     // Order is arbitrary for undirected edges

    //=========================================================================
    // Edge Type
    //=========================================================================
    bool isBend;                     // true = bend, false = sharp edge

    //=========================================================================
    // NEW: Orientation Tracking (from S1)
    //=========================================================================
    bool isReversed;                 // Edge traversal direction relative to source
                                     // Critical for Phase 2 precedence analysis
    bool isInternal;                 // Internal edge vs boundary edge
                                     // Boundary edges are part of part outline

    //=========================================================================
    // Shared Geometry
    //=========================================================================
    TopoDS_Edge sharedEdge;          // The common edge between faces
                                     // Only valid for sharp edges

    TopoDS_Face bendFace;            // The cylindrical surface
                                     // Only valid for bends (isBend == true)

    //=========================================================================
    // Bend Properties (only valid if isBend == true)
    //=========================================================================

    // Geometry
    gp_Ax1 bendAxis;                 // Axis of rotation (bend line)
                                     // CRITICAL: Direction must be consistent!

    // NEW: Bend line endpoints (from S1, S2)
    gp_Pnt bendAxisStart;            // Start point of bend line
    gp_Pnt bendAxisEnd;              // End point of bend line
    double bendLength;               // Length along bend line (mm)

    double bendRadius;               // Internal radius (mm)

    // Angle
    double bendAngle;                // Absolute angle (0-180°)
    double signedAngle;              // Signed: positive=up, negative=down

    // Classification
    BendConvexity convexity;         // CONVEX or CONCAVE
    BendDirection direction;         // UP, DOWN, or HEM

    // NEW: Bend type classification (from S2)
    enum BendType {
        ACUTE,                       // < 90°
        RIGHT,                       // = 90° (±tolerance)
        OBTUSE,                      // > 90°
        HEM,                         // < 45° (return flange)
        UNKNOWN
    } bendType;

    //=========================================================================
    // Sharp Edge Properties (only valid if isBend == false)
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

    // NEW: Check if this is a hem
    bool isHem() const {
        return bendType == BendType::HEM || std::abs(bendAngle) < 45.0;
    }

    // NEW: Get bend line as line segment
    gp_Lin getBendLineSegment() const {
        gp_Dir dir(bendAxisEnd.XYZ() - bendAxisStart.XYZ());
        return gp_Lin(bendAxisStart, dir);
    }
};

// Memory layout: ~200 bytes per edge (was ~150, added orientation fields)
// Memory increase: +50 bytes per edge for new tracking data
```

### 2.4 Graph Class

```cpp
class FaceAdjacencyGraph {
public:
    //=========================================================================
    // Construction
    //=========================================================================

    FaceAdjacencyGraph() = default;

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
    const FAG_Node& node(int id) const;
    FAG_Node& node(int id);
    size_t nodeCount() const { return m_nodes.size(); }

    // Edge access
    const FAG_Edge& edge(int id) const;
    FAG_Edge& edge(int id);
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

    // Find the base face using scoring algorithm
    int findBaseFace() const;

    // Set base face and compute flange levels
    void setBaseFace(int nodeId);

    // Extract all bends as BendFeature objects
    std::vector<BendFeature> extractBendFeatures() const;

    // Check if graph is connected
    bool isConnected() const;

    // Check for consistent normals (all pointing outward)
    bool hasConsistentNormals() const;

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

    // Quick lookup: face hash -> node ID
    std::unordered_map<size_t, int> m_faceToNode;

    // Quick lookup: (node1, node2) -> edge ID
    std::map<std::pair<int,int>, int> m_nodePairToEdge;

    // Identified base face
    int m_baseFaceId = -1;

    // Finalization flag
    bool m_finalized = false;

    //=========================================================================
    // Internal Methods
    //=========================================================================

    // Compute geometric properties for a node
    void computeNodeGeometry(FAG_Node& node);

    // Compute properties for a bend edge
    void computeBendProperties(FAG_Edge& edge);

    // Classify face type based on surface
    FaceType classifyFace(const TopoDS_Face& face);

    // Hash function for TopoDS_Face
    size_t faceHash(const TopoDS_Face& face) const;
};
```

---

## 3. Bend Feature

### 3.1 Complete Structure

```cpp
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
    FaceId bendFaceId;               // Index in FAG (may be -1 for zero-radius)
    TopoDS_Face bendFace;            // Direct handle

    // The "moving" side - the flange
    FaceId flangeFaceId;             // Index in FAG
    TopoDS_Face flangeFace;          // Direct handle

    //=========================================================================
    // Base Side Geometry
    //=========================================================================
    gp_Pnt baseCentroid;             // Center of base face
    gp_Dir baseNormal;               // Material-outward normal

    // Edge where base meets bend
    gp_Lin baseEdgeLine;             // The line along the edge
    double baseEdgeLength;           // Length of edge

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

    // Position relative to part
    gp_Pnt bendLineMidpoint;         // Center of bend line

    //=========================================================================
    // Bend Parameters
    //=========================================================================

    // Angles
    double targetAngle;              // Absolute angle (0-180°)
    double signedAngle;              // Signed: +up, -down

    // Radius
    double internalRadius;           // Inside radius (mm)
    double neutralRadius;            // At neutral axis
    double externalRadius;           // Outside radius

    // Material
    double kFactor;                  // For this specific bend
    double bendAllowance;            // BA = (π/180) × angle × (radius + k×thickness)
    double bendDeduction;            // BD = 2 × setback - BA

    //=========================================================================
    // Classification
    //=========================================================================
    BendType type;                   // STANDARD, ACUTE, HEM, JOGGLE
    BendDirection direction;         // UP, DOWN
    BendConvexity convexity;         // CONVEX, CONCAVE

    // Classification confidence
    double classificationConfidence; // 0.0 - 1.0

    // SDF value used for classification
    double signedDistance;           // (P_flange - P_base) · N_base

    //=========================================================================
    // Constraints (populated by Phase 2)
    //=========================================================================
    std::vector<int> mustBendBefore; // Bends that must happen first
    std::vector<int> mustBendAfter;  // Bends that must happen after

    // Special flags
    bool isClosingBend;              // True if this closes a box
    bool requiresRepo;               // True if grip change needed before
    bool hasInterference;            // True if potential collision detected

    //=========================================================================
    // Sequence Info (populated by Phase 3)
    //=========================================================================
    int sequencePosition;            // Order in final sequence (-1 = not set)
    int rotationBefore;              // Degrees to rotate before this bend
    double abaWidthRequired;         // Tool width needed

    //=========================================================================
    // Validation (populated by Phase 4)
    //=========================================================================
    bool isValidated;
    bool isCollisionFree;
    double minClearance;             // Minimum clearance during bend (mm)
    std::string validationNotes;

    //=========================================================================
    // Methods
    //=========================================================================

    // Get bend axis as OCCT type
    gp_Ax1 getBendAxis() const {
        return gp_Ax1(bendLineMidpoint, bendLine.Direction());
    }

    // Get the angle between base and flange
    double getFlangeAngle() const {
        return baseNormal.Angle(flangeNormal) * 180.0 / M_PI;
    }

    // Check if this is a hem (closed fold)
    bool isHem() const {
        return type == BendType::HEM || targetAngle < 45.0;
    }

    // Get direction as string
    std::string getDirectionString() const {
        switch (direction) {
            case BendDirection::UP: return "UP";
            case BendDirection::DOWN: return "DOWN";
            default: return "UNKNOWN";
        }
    }
};
```

### 3.2 BendFeature Relationships

```
┌─────────────────────────────────────────────────────────────────┐
│                        BendFeature                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│   baseFace ◄─────── baseCentroid    baseNormal                  │
│      │                    ↓              ↓                       │
│      │              ┌─────────────────────────────┐              │
│      │              │     BASE FACE (Fixed)       │              │
│      └──────────────┤                             │              │
│                     │   P_base      N_base ↑      │              │
│                     └───────────┬───────────────-─┘              │
│                                 │                                │
│   bendFace ◄───── bendLine ────┼──── bendLineLength             │
│      │                         │                                 │
│      │              ╔══════════╧══════════╗                      │
│      └──────────────║   BEND SURFACE      ║ radius, angle       │
│                     ║   (Cylindrical)     ║                      │
│                     ╚══════════╤══════════╝                      │
│                                │                                 │
│   flangeFace ◄───────────────-─┼─────── flangeCentroid          │
│      │              ┌──────────┴──────────────────┐              │
│      │              │     FLANGE FACE (Moving)    │              │
│      └──────────────┤                             │              │
│                     │   P_flange   N_flange ↗     │              │
│                     └─────────────────────────────┘              │
│                                                                  │
│   signedDistance = (P_flange - P_base) · N_base                 │
│   direction = (signedDistance > 0) ? UP : DOWN                  │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 4. Phase1Output

```cpp
struct Phase1Output {
    //=========================================================================
    // Main Data Structures
    //=========================================================================

    // The Face-Adjacency Graph
    FaceAdjacencyGraph fag;

    // All identified bends
    std::vector<BendFeature> bends;

    //=========================================================================
    // Identified Features
    //=========================================================================

    // Base face
    int baseFaceId;                  // Index in FAG
    TopoDS_Face baseFace;            // Direct reference

    // All flange faces (ordered by distance from base)
    std::vector<int> flangeFaceIds;

    //=========================================================================
    // Part Properties
    //=========================================================================

    // Detected sheet thickness (mm)
    double thickness;

    // Part geometry
    gp_Pnt partCentroid;             // Overall centroid
    Bnd_Box boundingBox;             // Axis-aligned bounding box

    // Dimensions
    double partLength;               // Max extent in X
    double partWidth;                // Max extent in Y
    double partHeight;               // Max extent in Z

    //=========================================================================
    // Material (if detected from STEP metadata)
    //=========================================================================
    std::optional<MaterialProperties> material;

    //=========================================================================
    // Source Information
    //=========================================================================
    std::string sourceFile;          // Path to STEP file
    std::string partName;            // Part name from STEP
    std::string stepVersion;         // AP203, AP214, etc.

    //=========================================================================
    // Processing Status
    //=========================================================================
    bool isValid;                    // True if parsing succeeded
    std::string errorMessage;        // Error if not valid
    std::vector<std::string> warnings;

    // Statistics
    int totalFaces;
    int planarFaces;
    int cylindricalFaces;
    int bendCount;
    double processingTimeMs;

    //=========================================================================
    // Methods
    //=========================================================================

    // Get bend by ID
    const BendFeature* getBend(int id) const;

    // Get bends connected to a face
    std::vector<const BendFeature*> getBendsForFace(int faceId) const;

    // Validation
    bool validate() const;

    // Summary for logging
    std::string getSummary() const;
};
```

---

## 5. Helper Classes

### 5.0 Edge Classification (NEW - from S1, S2)

```cpp
//=============================================================================
// Edge Classification Result
//=============================================================================

struct EdgeClassification {
    // Type
    bool isBend;                     // True if bend, false if sharp edge
    EdgeType edgeType;               // BEND_EDGE, SHARP_EDGE, BOUNDARY_EDGE

    // Geometry
    double radius;                   // Bend radius (0 if sharp)
    double angle;                    // Dihedral angle between faces
    gp_Ax1 axis;                     // Bend axis (if bend)

    // Confidence
    double confidence;               // 0.0-1.0, classification confidence
    std::string reasoning;           // Debug info

    // Methods
    bool isValidBend() const {
        return isBend && radius > Constants::MIN_BEND_RADIUS;
    }

    bool isSharpCorner() const {
        return !isBend && std::abs(angle - 90.0) < 5.0;
    }
};

//=============================================================================
// Classification Constants (from S2)
//=============================================================================

namespace Constants {
    // Edge classification
    constexpr double MIN_BEND_RADIUS = 0.1;      // mm - below this = sharp edge
    constexpr double MAX_BEND_RADIUS = 50.0;     // mm - above this = suspicious

    // Angle classification
    constexpr double HEM_ANGLE_THRESHOLD = 45.0;  // degrees
    constexpr double ACUTE_ANGLE_THRESHOLD = 90.0;
    constexpr double RIGHT_ANGLE_TOLERANCE = 2.0; // ±2° for "right angle"

    // Tolerances
    constexpr double AREA_TOLERANCE = 0.01;       // mm² - minimum face area
    constexpr double LENGTH_TOLERANCE = 1e-6;     // mm - geometric comparison
    constexpr double ANGLE_TOLERANCE = 0.01;      // radians (~0.57°)
    constexpr double COPLANARITY_TOLERANCE = 0.1; // mm - max distance for coplanar

    // SDF validation (from S3)
    constexpr double SDF_EPSILON = 0.1;           // mm - offset for SDF test
    constexpr double SDF_TOLERANCE = 1e-6;        // mm - SDF classifier tolerance
}

//=============================================================================
// Dihedral Angle Calculation (from S2)
//=============================================================================

struct DihedralAngleResult {
    double angle;                    // In degrees (0-360)
    double openingAngle;             // Angle between normals
    double bendAngle;                // 180 - opening angle
    gp_Dir crossProduct;             // N1 × N2
    bool isReflex;                   // True if > 180°

    // Convert to bend angle
    double getBendAngle() const {
        return 180.0 - openingAngle;
    }
};

//=============================================================================
// SDF Validation Result (from S3)
//=============================================================================

struct SDFValidationResult {
    bool isValid;                    // True if normal points outward
    TopAbs_State classifierState;    // OUT, IN, or ON
    double signedDistance;           // Approximate SDF value
    bool needsReversal;              // True if normal should be flipped

    std::string getStateString() const {
        switch (classifierState) {
            case TopAbs_OUT: return "OUTSIDE";
            case TopAbs_IN: return "INSIDE";
            case TopAbs_ON: return "ON_BOUNDARY";
            default: return "UNKNOWN";
        }
    }
};

```

### 5.1 Surface Analyzer

```cpp
class SurfaceAnalyzer {
public:
    // Analyze a face and return its properties
    static SurfaceProperties analyze(const TopoDS_Face& face);

    // Specific surface type checks
    static bool isPlanar(const TopoDS_Face& face);
    static bool isCylindrical(const TopoDS_Face& face);
    static bool isConical(const TopoDS_Face& face);

    // Get plane from planar face
    static std::optional<gp_Pln> getPlane(const TopoDS_Face& face);

    // Get cylinder from cylindrical face
    static std::optional<gp_Cylinder> getCylinder(const TopoDS_Face& face);

    // Compute area
    static double computeArea(const TopoDS_Face& face);

    // Compute centroid
    static gp_Pnt computeCentroid(const TopoDS_Face& face);

    // Get material-outward normal at centroid
    static gp_Dir getMaterialOutwardNormal(const TopoDS_Face& face);

    // NEW: SDF validation (from S3)
    static SDFValidationResult validateNormal(
        const TopoDS_Face& face,
        const gp_Dir& normal,
        const TopoDS_Solid& solid
    );

    // NEW: Extract bend axis from cylindrical face (from S1)
    static gp_Ax1 extractBendAxis(const TopoDS_Face& cylindricalFace);

    // NEW: Get UV bounds
    static void getUVBounds(
        const TopoDS_Face& face,
        double& uMin, double& uMax,
        double& vMin, double& vMax
    );
};

struct SurfaceProperties {
    FaceType type;

    // For all types
    gp_Pnt centroid;
    double area;
    Bnd_Box boundingBox;

    // For planar
    gp_Pln plane;
    gp_Dir normal;
    bool normalValidated;            // NEW: SDF validation passed

    // For cylindrical
    gp_Cylinder cylinder;
    gp_Ax1 axis;
    double radius;
    gp_Pnt axisStart;                // NEW: Bend line start
    gp_Pnt axisEnd;                  // NEW: Bend line end

    // NEW: Orientation info (from S3)
    TopAbs_Orientation orientation;  // FORWARD or REVERSED
    bool wasReversed;                // True if normal was flipped
};
```

### 5.2 Edge Analyzer

```cpp
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

struct EdgeProperties {
    bool isLinear;
    bool isCircular;
    double length;
    gp_Pnt startPoint;
    gp_Pnt endPoint;
    gp_Pnt midpoint;

    // For linear
    gp_Lin line;

    // For circular
    gp_Circ circle;
    double radius;
};
```

### 5.3 Angle Calculator

```cpp
class AngleCalculator {
public:
    // Calculate angle between two planar faces at their shared bend
    static double calculateBendAngle(
        const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Face& bendFace
    );

    // Calculate signed angle (positive = up, negative = down)
    static double calculateSignedAngle(
        const gp_Dir& baseNormal,
        const gp_Dir& flangeNormal,
        const gp_Dir& bendAxis
    );

    // NEW: Calculate dihedral angle (from S2)
    static DihedralAngleResult calculateDihedralAngle(
        const TopoDS_Face& face1,
        const TopoDS_Face& face2,
        const TopoDS_Edge& sharedEdge
    );

    // Determine if angle is acute (> 120°), standard, or hem (< 45°)
    static BendType classifyByAngle(double angle);

    // NEW: Compute angle using cross product method (from S2)
    static double angleFromNormals(
        const gp_Dir& normal1,
        const gp_Dir& normal2,
        const gp_Dir& referenceAxis  // For sign determination
    );
};
```

---

## 6. Memory Considerations

### 6.1 Estimated Memory Usage

```
For a typical part with N faces and M bends:

FAG_Node:   ~200 bytes × N
FAG_Edge:   ~150 bytes × (N + M)
BendFeature: ~400 bytes × M

Example: Part with 50 faces, 15 bends
  Nodes:  200 × 50 = 10 KB
  Edges:  150 × 65 = 10 KB
  Bends:  400 × 15 = 6 KB
  Overhead: ~5 KB
  -----------------------
  Total: ~31 KB

Note: OCCT handles (TopoDS_Face, etc.) are reference-counted.
      The underlying shape data is shared, not duplicated.
```

### 6.2 Memory Management Guidelines

```cpp
// 1. OCCT handles are reference-counted - safe to copy
TopoDS_Face face1 = someFace;
TopoDS_Face face2 = face1;  // Both point to same data

// 2. Never store raw pointers to OCCT data
// BAD:
gp_Pnt* ptr = &someHandle->Location();  // Dangerous!

// GOOD:
gp_Pnt point = someHandle->Location();  // Copy the value

// 3. Clear large structures when done
void cleanup() {
    fag.clear();
    bends.clear();
    // OCCT handles will be released automatically
}

// 4. Use move semantics for large returns
Phase1Output processFile(const std::string& path) {
    Phase1Output result;
    // ... fill result ...
    return result;  // Move, not copy
}
```

---

## 7. Thread Safety

```cpp
// FaceAdjacencyGraph is NOT thread-safe by default
// For concurrent access:

class ThreadSafeFAG {
public:
    // Read operations (multiple readers OK)
    const FAG_Node& node(int id) const {
        std::shared_lock lock(m_mutex);
        return m_fag.node(id);
    }

    // Write operations (exclusive access)
    int addNode(const TopoDS_Face& face) {
        std::unique_lock lock(m_mutex);
        return m_fag.addNode(face);
    }

private:
    FaceAdjacencyGraph m_fag;
    mutable std::shared_mutex m_mutex;
};
```

---

## 9. Usage Examples (NEW)

### 9.1 Creating and Validating FAG_Edge

```cpp
// Example: Create a bend edge with full validation

FAG_Edge createBendEdge(
    int nodeId1,
    int nodeId2,
    const TopoDS_Face& cylindricalFace,
    const FaceAdjacencyGraph& fag
) {
    FAG_Edge edge;
    edge.node1 = nodeId1;
    edge.node2 = nodeId2;
    edge.isBend = true;
    edge.bendFace = cylindricalFace;

    // Extract bend axis (NEW - from S1)
    edge.bendAxis = SurfaceAnalyzer::extractBendAxis(cylindricalFace);

    // Get cylinder properties
    auto cylinder = SurfaceAnalyzer::getCylinder(cylindricalFace);
    if (cylinder) {
        edge.bendRadius = cylinder->Radius();
    }

    // Compute bend line endpoints (NEW - from S1)
    double uMin, uMax, vMin, vMax;
    SurfaceAnalyzer::getUVBounds(cylindricalFace, uMin, uMax, vMin, vMax);

    Handle(Geom_Surface) surf = BRep_Tool::Surface(cylindricalFace);
    edge.bendAxisStart = surf->Value(0, vMin);
    edge.bendAxisEnd = surf->Value(0, vMax);
    edge.bendLength = edge.bendAxisStart.Distance(edge.bendAxisEnd);

    // Compute dihedral angle (NEW - from S2)
    const FAG_Node& n1 = fag.node(nodeId1);
    const FAG_Node& n2 = fag.node(nodeId2);

    gp_Dir normal1 = n1.normal;
    gp_Dir normal2 = n2.normal;

    double cosAngle = normal1.Dot(normal2);
    edge.bendAngle = std::acos(std::clamp(cosAngle, -1.0, 1.0)) * 180.0 / M_PI;

    // Classify bend type (NEW - from S2)
    if (edge.bendAngle < Constants::HEM_ANGLE_THRESHOLD) {
        edge.bendType = FAG_Edge::BendType::HEM;
    } else if (std::abs(edge.bendAngle - 90.0) < Constants::RIGHT_ANGLE_TOLERANCE) {
        edge.bendType = FAG_Edge::BendType::RIGHT;
    } else if (edge.bendAngle < Constants::ACUTE_ANGLE_THRESHOLD) {
        edge.bendType = FAG_Edge::BendType::ACUTE;
    } else {
        edge.bendType = FAG_Edge::BendType::OBTUSE;
    }

    // Orientation tracking (NEW - from S1)
    edge.isReversed = false;  // Default, may be updated later
    edge.isInternal = true;   // Assume internal unless on boundary

    return edge;
}
```

### 9.2 SDF Normal Validation

```cpp
// Example: Validate material normal with SDF (from S3)

gp_Dir validateAndCorrectNormal(
    const TopoDS_Face& face,
    const TopoDS_Solid& solid
) {
    // Get initial normal
    gp_Dir normal = SurfaceAnalyzer::getMaterialOutwardNormal(face);

    // Validate with SDF
    SDFValidationResult sdfResult =
        SurfaceAnalyzer::validateNormal(face, normal, solid);

    if (!sdfResult.isValid) {
        LOG_WARNING("Normal validation failed for face - reversing");
        normal.Reverse();

        // Re-validate
        sdfResult = SurfaceAnalyzer::validateNormal(face, normal, solid);

        if (!sdfResult.isValid) {
            LOG_ERROR("Normal still invalid after reversal!");
        }
    }

    LOG_DEBUG("Normal validation: " + sdfResult.getStateString());
    return normal;
}
```

### 9.3 Edge Classification

```cpp
// Example: Classify edge as BEND or SHARP (from S1, S2)

EdgeClassification classifyEdge(
    const TopoDS_Edge& edge,
    const TopoDS_Face& face1,
    const TopoDS_Face& face2
) {
    EdgeClassification result;

    // Analyze edge geometry
    BRepAdaptor_Curve adaptor(edge);
    GeomAbs_CurveType curveType = adaptor.GetType();

    // Check for circular edge (potential bend)
    if (curveType == GeomAbs_Circle) {
        gp_Circ circle = adaptor.Circle();
        double radius = circle.Radius();

        if (radius > Constants::MIN_BEND_RADIUS) {
            result.isBend = true;
            result.radius = radius;
            result.axis = gp_Ax1(circle.Location(), circle.Axis().Direction());

            // Compute dihedral angle
            auto angleResult = AngleCalculator::calculateDihedralAngle(
                face1, face2, edge
            );
            result.angle = angleResult.bendAngle;
            result.confidence = 0.95;  // High confidence for circular bend
            result.reasoning = "Circular edge with radius " +
                               std::to_string(radius) + " mm";
        } else {
            result.isBend = false;
            result.radius = 0.0;
            result.confidence = 0.8;
            result.reasoning = "Radius below MIN_BEND_RADIUS threshold";
        }
    } else {
        // Sharp edge
        result.isBend = false;
        result.radius = 0.0;

        // Still compute dihedral angle for sharp corner
        auto angleResult = AngleCalculator::calculateDihedralAngle(
            face1, face2, edge
        );
        result.angle = angleResult.openingAngle;
        result.confidence = 1.0;
        result.reasoning = "Linear edge - sharp corner";
    }

    return result;
}
```

### 9.4 Memory Management Example

```cpp
// Example: Proper OCCT handle management

void processMultipleFiles(const std::vector<std::string>& files) {
    for (const auto& file : files) {
        // Local scope for each file
        {
            Phase1Output output;

            // Load and process
            output = parseSTEPFile(file);

            // Use output...
            analyzeOutput(output);

            // output goes out of scope here
            // OCCT handles automatically released
        }

        // Memory from previous file is freed
    }
}

// BAD: Don't store raw pointers
void badExample() {
    TopoDS_Face face = getFace();
    gp_Pnt* ptr = new gp_Pnt(face.Location());  // DON'T DO THIS!
    // Face goes out of scope, pointer now dangling
}

// GOOD: Store values or handles
void goodExample() {
    TopoDS_Face face = getFace();  // Handle, reference-counted
    gp_Pnt point = face.Location(); // Value, safe copy
    // Both are safe
}
```

---

## 10. Changelog (Version 2.0)

### Added Fields

**FAG_Edge:**
- `isReversed` - Edge orientation tracking for precedence analysis
- `isInternal` - Internal vs boundary edge classification
- `bendAxisStart`, `bendAxisEnd` - Precise bend line endpoints
- `bendType` enum - ACUTE, RIGHT, OBTUSE, HEM classification
- New methods: `isHem()`, `getBendLineSegment()`

**SurfaceProperties:**
- `normalValidated` - SDF validation flag
- `axisStart`, `axisEnd` - Cylinder axis endpoints
- `orientation`, `wasReversed` - Orientation tracking

### Added Types

- `EdgeClassification` - Result of edge classification algorithm
- `DihedralAngleResult` - Detailed angle computation result
- `SDFValidationResult` - SDF validation outcome
- `Constants` namespace - All thresholds and tolerances

### Added Methods

**SurfaceAnalyzer:**
- `validateNormal()` - SDF-based normal validation
- `extractBendAxis()` - Bend axis from cylindrical face
- `getUVBounds()` - UV parameter bounds

**AngleCalculator:**
- `calculateDihedralAngle()` - Complete dihedral angle computation
- `angleFromNormals()` - Cross product method

### Memory Impact

- FAG_Edge: +50 bytes per edge (150 → 200 bytes)
- SurfaceProperties: +40 bytes
- New helper structs: ~100 bytes each (when used)

### Research Paper References

- S1: Edge orientation, bend axis extraction
- S2: Edge classification, dihedral angle, bend types
- S3: SDF validation, normal correction
- S4: (Healing - covered in phase1-design.md)

---

**End of Phase 1 Types Document**

```cpp
// For debugging and persistence

namespace Phase1Serialization {

// Save to JSON (human-readable, for debugging)
void saveToJson(const Phase1Output& output, const std::string& path);
Phase1Output loadFromJson(const std::string& path);

// Save to binary (compact, for caching)
void saveToBinary(const Phase1Output& output, const std::string& path);
Phase1Output loadFromBinary(const std::string& path);

// Note: TopoDS_Shape handles require OCCT serialization (BRepTools)
void saveShapes(const TopoDS_Shape& shape, const std::string& path);
TopoDS_Shape loadShapes(const std::string& path);

}
```
