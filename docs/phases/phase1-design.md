# Phase 1: Geometric Parser - Design Document

> **Version**: 2.0
> **Status**: IN DESIGN
> **Last Updated**: 2026-02-04
> **Changes**: Added detailed algorithms from research papers (S1-S4)

---

## 1. Overview

### 1.1 Purpose
Phase 1 là "The Eyes" của hệ thống - đọc và hiểu geometry từ STEP file, xây dựng Face-Adjacency Graph (FAG) và classify từng bend.

### 1.2 Input
- STEP file (ISO 10303)
  - AP203: Configuration Controlled 3D Design
  - AP214: Automotive Design
  - AP242: Managed Model-based 3D Engineering

### 1.3 Output
```cpp
struct Phase1Output {
    FaceAdjacencyGraph fag;           // Graph structure
    std::vector<BendFeature> bends;   // All identified bends
    TopoDS_Face baseFace;             // Identified base face
    MaterialDirection grainDir;       // Optional: grain direction
};
```

---

## 2. Architecture

### 2.1 Module Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                        Phase 1: Geometric Parser                 │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ STEP Reader  │───▶│ Geometry     │───▶│ FAG Builder  │       │
│  │              │    │ Healer       │    │              │       │
│  └──────────────┘    └──────────────┘    └──────┬───────┘       │
│                                                  │               │
│                                                  ▼               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │ Phase1Output │◀───│ Bend         │◀───│ Base Face    │       │
│  │              │    │ Classifier   │    │ Identifier   │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Class Diagram

```
┌─────────────────────────────────┐
│        GeometricParser          │
├─────────────────────────────────┤
│ - stepReader: STEPReader        │
│ - healer: GeometryHealer        │
│ - fagBuilder: FAGBuilder        │
│ - baseFinder: BaseFaceIdentifier│
│ - classifier: BendClassifier    │
├─────────────────────────────────┤
│ + parse(filepath): Phase1Output │
│ + validate(): bool              │
│ + getLastError(): string        │
└─────────────────────────────────┘
           │
           │ uses
           ▼
┌─────────────────────────────────┐
│        FaceAdjacencyGraph       │
├─────────────────────────────────┤
│ - nodes: vector<FAG_Node>       │
│ - edges: vector<FAG_Edge>       │
│ - adjacency: map<int,vector>    │
├─────────────────────────────────┤
│ + addNode(face): int            │
│ + addEdge(n1, n2, bend): void   │
│ + getNeighbors(nodeId): vector  │
│ + findBaseFace(): int           │
│ + getBends(): vector<BendFeature>│
└─────────────────────────────────┘
```

---

## 3. Data Structures

### 3.1 Core Types

```cpp
// Forward declarations
class FaceAdjacencyGraph;
class BendFeature;

//=============================================================================
// Enumerations
//=============================================================================

enum class FaceType {
    PLANAR,         // Flat face (base or flange)
    CYLINDRICAL,    // Bend surface
    CONICAL,        // Rare: tapered bend
    FREEFORM,       // B-spline (not supported)
    UNKNOWN
};

enum class BendDirection {
    BEND_UP,        // Positive bend (lower blade)
    BEND_DOWN,      // Negative bend (upper blade)
    HEM,            // Fold-back (< 45° opening)
    UNKNOWN
};

enum class BendConvexity {
    CONVEX,         // Material on inside of bend radius
    CONCAVE,        // Material on outside of bend radius
    UNKNOWN
};

//=============================================================================
// FAG Node - Represents a face in the part
//=============================================================================

struct FAG_Node {
    int id;                          // Unique identifier
    TopoDS_Face face;                // OCCT face handle
    FaceType type;                   // Classification

    // Geometric properties
    gp_Dir normal;                   // Material-outward normal (corrected)
    gp_Pnt centroid;                 // Face centroid
    double area;                     // Surface area (mm²)

    // Classification
    bool isBaseFace;                 // True if identified as base
    bool isFlange;                   // True if connected to base via bend
    int bendDepth;                   // Distance from base (in bends)

    // For debugging
    std::string debugName;           // e.g., "Face_1", "Flange_A"
};

//=============================================================================
// FAG Edge - Represents connection between faces (bend or shared edge)
//=============================================================================

struct FAG_Edge {
    int id;                          // Unique identifier
    int sourceNodeId;                // First face
    int targetNodeId;                // Second face

    // Shared geometry
    TopoDS_Edge sharedEdge;          // Common edge (if sharp)
    TopoDS_Face bendFace;            // Cylindrical face (if bend)

    // Edge type
    bool isBend;                     // True if this is a bend, false if sharp edge

    // **NEW: Orientation tracking (from S1)**
    bool isReversed;                 // Edge traversal direction relative to source
    bool isInternal;                 // Internal edge vs boundary edge

    // Bend properties (only valid if isBend == true)
    double bendAngle;                // Signed angle in degrees (-180 to +180)
    double bendRadius;               // Internal radius (mm)
    double bendLength;               // Length along bend line (mm)
    BendConvexity convexity;         // Convex or concave

    // **NEW: Bend axis (from S1, S2)**
    gp_Ax1 bendAxis;                 // Axis of rotation (origin + direction)
    gp_Pnt bendAxisStart;            // Start point of bend line
    gp_Pnt bendAxisEnd;              // End point of bend line

    // For sharp edges
    double edgeLength;               // Length of shared edge

    // **NEW: Bend type classification (from S2)**
    enum BendType {
        ACUTE,                       // < 90°
        RIGHT,                       // = 90° (±tolerance)
        OBTUSE,                      // > 90°
        HEM                          // < 45° (return flange)
    } bendType;
};

//=============================================================================
// Bend Feature - Complete description of a bend operation
//=============================================================================

struct BendFeature {
    int id;                          // Unique identifier
    int fagEdgeId;                   // Reference to FAG_Edge

    // Participating faces
    TopoDS_Face baseFace;            // The "fixed" side
    TopoDS_Face bendFace;            // The cylindrical surface
    TopoDS_Face flangeFace;          // The "moving" side

    // Geometry - Base side
    gp_Pnt baseCentroid;
    gp_Dir baseNormal;               // Material-outward, corrected

    // Geometry - Flange side
    gp_Pnt flangeCentroid;
    gp_Dir flangeNormal;             // Material-outward, corrected

    // Bend line definition
    gp_Lin bendLine;                 // The bend axis as a line
    double bendLineLength;           // Length of bend line (mm)

    // Bend parameters
    double targetAngle;              // Absolute angle (0-180°)
    double signedAngle;              // Signed angle (+up, -down)
    double internalRadius;           // Bend radius (mm)
    double kFactor;                  // Estimated K-factor

    // Classification
    BendDirection direction;         // UP, DOWN, HEM
    double signedDistance;           // SDF value for classification

    // Flange properties
    double flangeLength;             // Distance from bend to flange edge
    double flangeWidth;              // Width of flange (along bend line)

    // Constraints (populated in Phase 2)
    std::vector<int> mustBendBefore; // Precedence
    std::vector<int> mustBendAfter;
};

//=============================================================================
// Face Adjacency Graph - Main data structure
//=============================================================================

class FaceAdjacencyGraph {
public:
    // Construction
    void clear();
    int addNode(const TopoDS_Face& face);
    void addEdge(int node1, int node2, const TopoDS_Edge& sharedEdge);
    void addBendEdge(int node1, int node2, const TopoDS_Face& bendFace);

    // Queries
    const FAG_Node& getNode(int id) const;
    const FAG_Edge& getEdge(int id) const;
    std::vector<int> getNeighbors(int nodeId) const;
    std::vector<int> getBendNeighbors(int nodeId) const;

    // Analysis
    int findBaseFace() const;
    std::vector<BendFeature> extractBendFeatures() const;
    void computeBendDepths(int baseFaceId);

    // Validation
    bool isConnected() const;
    bool hasConsistentNormals() const;

    // Accessors
    const std::vector<FAG_Node>& nodes() const { return m_nodes; }
    const std::vector<FAG_Edge>& edges() const { return m_edges; }
    size_t nodeCount() const { return m_nodes.size(); }
    size_t edgeCount() const { return m_edges.size(); }
    size_t bendCount() const;

private:
    std::vector<FAG_Node> m_nodes;
    std::vector<FAG_Edge> m_edges;
    std::map<int, std::vector<int>> m_adjacency;  // nodeId -> [edgeIds]

    // Internal helpers
    void classifyFace(FAG_Node& node);
    void computeNodeGeometry(FAG_Node& node);
    void computeEdgeGeometry(FAG_Edge& edge);
};

//=============================================================================
// Phase 1 Output - Complete result of parsing
//=============================================================================

struct Phase1Output {
    // Main data
    FaceAdjacencyGraph fag;
    std::vector<BendFeature> bends;

    // Identified features
    int baseFaceId;                  // Index in FAG
    TopoDS_Face baseFace;            // Direct reference

    // Part properties
    double thickness;                // Detected sheet thickness (mm)
    gp_Pnt partCentroid;            // Overall part centroid
    Bnd_Box boundingBox;            // Part bounding box

    // Metadata
    std::string sourceFile;
    std::string partName;

    // Validation
    bool isValid;
    std::string errorMessage;
    std::vector<std::string> warnings;
};
```

### 3.2 Helper Types

```cpp
//=============================================================================
// Geometry Utilities
//=============================================================================

struct SurfaceProperties {
    FaceType type;
    gp_Dir normal;           // For planar
    gp_Ax1 axis;             // For cylindrical
    double radius;           // For cylindrical
    gp_Pnt centroid;
    double area;
};

struct EdgeProperties {
    bool isLinear;
    bool isCircular;
    gp_Lin line;             // If linear
    gp_Circ circle;          // If circular
    double length;
    gp_Pnt startPoint;
    gp_Pnt endPoint;
};

//=============================================================================
// Classification Results
//=============================================================================

struct BaseFaceCandidate {
    int nodeId;
    double score;

    // Scoring components
    double areaScore;        // Larger = higher
    double flatnessScore;    // More neighbors = higher
    double centralityScore;  // More central = higher
    double orientationScore; // Horizontal preferred
};

struct BendClassification {
    BendDirection direction;
    double confidence;       // 0.0 - 1.0
    double signedDistance;
    std::string reasoning;   // For debugging
};
```

---

## 4. Algorithms

### 4.1 FAG Construction Algorithm

```
Algorithm: BuildFaceAdjacencyGraph
Input:  TopoDS_Shape (healed solid)
Output: FaceAdjacencyGraph

1. EXTRACT all faces from shape
   faces = TopExp::MapShapes(shape, TopAbs_FACE)

2. BUILD edge-to-face mapping (O(n))
   edgeToFaces = TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE)

3. FOR each face in faces:
   3.1 CREATE node
       node.face = face
       node.type = ClassifyFace(face)
       node.normal = GetMaterialOutwardNormal(face)
       node.centroid = ComputeCentroid(face)
       node.area = ComputeArea(face)
   3.2 ADD node to graph
       nodeId = fag.addNode(node)
       faceToNode[face] = nodeId

4. FOR each edge in edgeToFaces:
   4.1 GET adjacent faces
       adjacentFaces = edgeToFaces[edge]
   4.2 IF count(adjacentFaces) == 2:
       face1, face2 = adjacentFaces
       node1 = faceToNode[face1]
       node2 = faceToNode[face2]
   4.3 IF edge is cylindrical (bend):
       fag.addBendEdge(node1, node2, edge)
   4.4 ELSE (sharp edge):
       fag.addEdge(node1, node2, edge)

5. RETURN fag
```

**Complexity**: O(n) where n = number of faces + edges

### 4.2 Face Classification Algorithm

```
Algorithm: ClassifyFace
Input:  TopoDS_Face
Output: FaceType

1. GET underlying surface
   surface = BRep_Tool::Surface(face)
   adaptor = BRepAdaptor_Surface(face)

2. SWITCH adaptor.GetType():
   CASE GeomAbs_Plane:
       RETURN FaceType::PLANAR

   CASE GeomAbs_Cylinder:
       RETURN FaceType::CYLINDRICAL

   CASE GeomAbs_Cone:
       RETURN FaceType::CONICAL

   CASE GeomAbs_BSplineSurface:
   CASE GeomAbs_BezierSurface:
       // Check if it's actually a degenerate cylinder
       IF IsDegenerateCylinder(surface):
           RETURN FaceType::CYLINDRICAL
       RETURN FaceType::FREEFORM

   DEFAULT:
       RETURN FaceType::UNKNOWN
```

### 4.3 Material-Outward Normal Algorithm

```
Algorithm: GetMaterialOutwardNormal
Input:  TopoDS_Face
Output: gp_Dir (unit vector pointing out of material)

1. GET UV bounds of face
   BRepTools::UVBounds(face, uMin, uMax, vMin, vMax)

2. COMPUTE mid-point parameters
   uMid = (uMin + uMax) / 2
   vMid = (vMin + vMax) / 2

3. GET surface handle
   surface = BRep_Tool::Surface(face)

4. COMPUTE surface properties at mid-point
   props = GeomLProp_SLProps(surface, uMid, vMid, 1, 1e-7)

5. GET geometric normal
   geoNormal = props.Normal()

6. CORRECT for face orientation
   IF face.Orientation() == TopAbs_REVERSED:
       geoNormal.Reverse()

7. RETURN geoNormal
```

**Critical Note**: Step 6 is essential. OCCT faces can have reversed orientation, which flips the normal direction. Without this correction, bend classification will be wrong.

### 4.3.1 SDF Validation Algorithm (from S3)

**Purpose**: Verify that the computed normal actually points OUTWARD from the solid material.

```
Algorithm: ValidateMaterialNormal
Input:  TopoDS_Face, gp_Dir normal, TopoDS_Solid solid
Output: bool (true if normal points outward)

1. GET face centroid
   P = ComputeCentroid(face)

2. CREATE test point slightly offset along normal
   epsilon = 0.1  // mm
   P_test = P + epsilon * normal

3. COMPUTE Signed Distance Function (SDF)
   // SDF > 0 if outside solid, < 0 if inside
   distanceToSolid = BRepClass3d_SolidClassifier()
   classifier.Perform(P_test, solid, tolerance)

4. CHECK classifier state
   IF classifier.State() == TopAbs_OUT:
       // P_test is outside → normal is OUTWARD
       RETURN true
   ELSE IF classifier.State() == TopAbs_IN:
       // P_test is inside → normal is INWARD (WRONG!)
       RETURN false
   ELSE:
       // ON boundary → inconclusive, accept
       RETURN true
```

**When to use**: After GetMaterialOutwardNormal(), especially for complex or badly-oriented STEP files.

### 4.3.2 Bend Axis Extraction Algorithm (from S1, S2)

**Purpose**: Extract the precise axis of rotation for a cylindrical bend face.

```
Algorithm: ExtractBendAxis
Input:  TopoDS_Face (cylindrical bend face)
Output: gp_Ax1 (axis of rotation)

1. GET underlying surface
   surface = BRep_Tool::Surface(face)
   adaptor = BRepAdaptor_Surface(face)

2. VERIFY it's cylindrical
   IF adaptor.GetType() != GeomAbs_Cylinder:
       THROW error("Not a cylindrical surface")

3. GET cylinder parameters
   cylinder = adaptor.Cylinder()
   axis = cylinder.Axis()        // gp_Ax1: origin + direction
   radius = cylinder.Radius()

4. COMPUTE bend line endpoints
   // Get U,V bounds of the cylindrical face
   BRepTools::UVBounds(face, uMin, uMax, vMin, vMax)

   // Cylinder parametrization: U = angle, V = height along axis
   // Bend line endpoints at V_min and V_max
   gp_Pnt P_start = surface->Value(0, vMin)
   gp_Pnt P_end = surface->Value(0, vMax)

5. ENSURE consistent direction
   // Axis direction should follow source → target node order
   // If needed, reverse axis direction for consistency

6. RETURN gp_Ax1(P_start, axis.Direction())
```

**Critical**: Bend axis direction must be consistent across the entire FAG to ensure correct precedence analysis in Phase 2.

### 4.4 Base Face Identification Algorithm

```
Algorithm: FindBaseFace
Input:  FaceAdjacencyGraph
Output: int (nodeId of base face)

1. COLLECT all planar faces
   candidates = []
   FOR node in fag.nodes():
       IF node.type == PLANAR:
           candidates.append(node)

2. SCORE each candidate
   FOR candidate in candidates:
       score = 0.0

       // Area score (larger is better)
       areaScore = candidate.area / maxArea
       score += areaScore * WEIGHT_AREA  // WEIGHT_AREA = 0.3

       // Connectivity score (more bends connected is better)
       bendCount = CountBendEdges(candidate.id)
       connectScore = bendCount / maxBendCount
       score += connectScore * WEIGHT_CONNECT  // WEIGHT_CONNECT = 0.3

       // Centrality score (closer to centroid is better)
       distToCentroid = Distance(candidate.centroid, partCentroid)
       centralScore = 1.0 - (distToCentroid / maxDist)
       score += centralScore * WEIGHT_CENTRAL  // WEIGHT_CENTRAL = 0.2

       // Orientation score (horizontal preferred)
       verticalDot = abs(candidate.normal.Dot(Z_AXIS))
       orientScore = verticalDot  // 1.0 if horizontal
       score += orientScore * WEIGHT_ORIENT  // WEIGHT_ORIENT = 0.2

       candidate.score = score

3. SORT candidates by score descending

4. RETURN candidates[0].nodeId
```

### 4.5 Bend Direction Classification (SDF)

```
Algorithm: ClassifyBendDirection
Input:  BendFeature (with base and flange faces set)
Output: BendDirection

1. COMPUTE centroids
   P_base = ComputeCentroid(baseFace)
   P_flange = ComputeCentroid(flangeFace)

2. GET base face normal (material-outward)
   N_base = GetMaterialOutwardNormal(baseFace)

3. COMPUTE signed distance
   // SDF: positive if flange is in direction of normal
   D = (P_flange - P_base).Dot(N_base)

4. CLASSIFY based on sign and magnitude
   IF abs(D) < THICKNESS_TOLERANCE:
       // Coplanar faces, likely a hem
       IF ComputeAngle(baseFace, flangeFace) < 45°:
           RETURN BendDirection::HEM
       RETURN BendDirection::UNKNOWN

   IF D > 0:
       RETURN BendDirection::BEND_UP
   ELSE:
       RETURN BendDirection::BEND_DOWN
```

**Visualization**:
```
          N_base (material outward)
             ↑
             │
    ─────────┼─────────  Base Face (horizontal)
             │
             │
    ═════════╪═════════  Flange UP (D > 0)  → BEND_UP
             ○ P_base

    ─────────┼─────────  Base Face
             │
             ○ P_base
    ═════════╪═════════  Flange DOWN (D < 0) → BEND_DOWN
             │
             ↓
```

### 4.6 Geometry Healing Algorithm

```
Algorithm: HealGeometry
Input:  TopoDS_Shape (raw from STEP)
Output: TopoDS_Shape (healed)

1. FIX basic shape issues
   fixer = ShapeFix_Shape(shape)
   fixer.SetPrecision(TOLERANCE)  // TOLERANCE = 1e-6
   fixer.Perform()
   shape = fixer.Shape()

2. UNIFY same-domain faces
   // Merges adjacent planar faces that are coplanar
   unifier = ShapeUpgrade_UnifySameDomain(shape)
   unifier.SetLinearTolerance(TOLERANCE)
   unifier.SetAngularTolerance(ANGULAR_TOL)  // ~0.01 radians
   unifier.Build()
   shape = unifier.Shape()

3. SEW faces (close gaps)
   sewer = BRepBuilderAPI_Sewing(TOLERANCE)
   sewer.Add(shape)
   sewer.Perform()
   shape = sewer.SewedShape()

4. VALIDATE result
   analyzer = BRepCheck_Analyzer(shape)
   IF NOT analyzer.IsValid():
       LOG_WARNING("Shape has remaining issues")
       // Continue anyway, log issues

5. RETURN shape
```

### 4.7 Edge Classification Algorithm (from S1, S2)

**Purpose**: Determine if an edge connecting two faces is a bend or a sharp edge.

```
Algorithm: ClassifyEdge
Input:  TopoDS_Edge, TopoDS_Face face1, TopoDS_Face face2
Output: EdgeClassification {isBend, bendRadius, bendAngle}

1. ANALYZE edge geometry
   adaptor = BRepAdaptor_Curve(edge)
   curveType = adaptor.GetType()

2. CLASSIFY adjacent faces
   type1 = ClassifyFace(face1)
   type2 = ClassifyFace(face2)

3. CHECK for cylindrical connection (BEND)
   // Case 1: Edge is part of a cylindrical face boundary
   IF curveType == GeomAbs_Circle:
       // This could be a bend edge
       circle = adaptor.Circle()
       radius = circle.Radius()

       // Verify both adjacent faces are planar
       IF type1 == PLANAR AND type2 == PLANAR:
           // Compute dihedral angle between faces
           angle = ComputeDihedralAngle(face1, face2, edge)

           // Check if radius is significant (not chamfer)
           IF radius > MIN_BEND_RADIUS:  // e.g., 0.1mm
               RETURN {isBend: true, radius, angle}

   // Case 2: Edge is straight, but connects via cylindrical face
   IF curveType == GeomAbs_Line:
       // Check if there's a cylindrical face between f1 and f2
       // (More complex case - requires topology traversal)

4. SHARP EDGE classification
   // If not a bend, it's a sharp edge
   angle = ComputeDihedralAngle(face1, face2, edge)
   RETURN {isBend: false, radius: 0, angle}
```

**Key Decision**: `MIN_BEND_RADIUS`
- Too small: Chamfers classified as bends
- Too large: Small-radius bends classified as sharp
- Recommended: 0.1mm for typical sheet metal

### 4.8 Dihedral Angle Calculation (from S2)

**Purpose**: Compute the angle between two planar faces sharing an edge.

```
Algorithm: ComputeDihedralAngle
Input:  TopoDS_Face face1, TopoDS_Face face2, TopoDS_Edge sharedEdge
Output: double (angle in degrees, 0-180)

1. GET material-outward normals
   N1 = GetMaterialOutwardNormal(face1)
   N2 = GetMaterialOutwardNormal(face2)

2. COMPUTE geometric angle
   // Angle between normals
   cosTheta = N1.Dot(N2)
   angle = acos(clamp(cosTheta, -1.0, 1.0))  // Radians

3. DETERMINE sign using cross product
   // Cross product points along shared edge direction
   cross = N1.Crossed(N2)

   // Get edge direction
   BRepAdaptor_Curve curve(sharedEdge)
   gp_Pnt P1 = curve.Value(curve.FirstParameter())
   gp_Pnt P2 = curve.Value(curve.LastParameter())
   edgeDir = gp_Dir(P2.XYZ() - P1.XYZ())

   // Check alignment
   dot = cross.Dot(edgeDir)
   IF dot < 0:
       // Normals form "outward" angle
       angle = 2*PI - angle

4. CONVERT to degrees
   angleDegrees = angle * 180 / PI

5. RETURN angleDegrees
```

**Note**: This gives the "opening angle" between faces. For bend angle:
- Opening = 180° → Bend = 0° (flat)
- Opening = 90° → Bend = 90°
- Opening = 0° → Bend = 180° (folded flat)

Formula: `bendAngle = 180 - openingAngle`

---

## 5. Module Specifications

### 5.1 STEPReader

```cpp
class STEPReader {
public:
    // Load STEP file
    // Returns: true if successful
    // Throws: std::runtime_error on parse failure
    bool load(const std::string& filepath);

    // Get the loaded shape
    // Pre: load() returned true
    TopoDS_Shape getShape() const;

    // Get metadata
    std::string getPartName() const;
    std::string getUnits() const;  // "MM" or "INCH"

    // Error handling
    std::string getLastError() const;

private:
    STEPControl_Reader m_reader;
    TopoDS_Shape m_shape;
    std::string m_lastError;

    bool validateShape();
    void convertUnits();  // Ensure MM
};
```

### 5.2 GeometryHealer

```cpp
class GeometryHealer {
public:
    // Configuration
    void setTolerance(double tol);
    void setAngularTolerance(double tol);

    // Healing operations
    TopoDS_Shape heal(const TopoDS_Shape& input);

    // Specific fixes
    TopoDS_Shape fixShape(const TopoDS_Shape& input);
    TopoDS_Shape unifySameDomain(const TopoDS_Shape& input);
    TopoDS_Shape sewFaces(const TopoDS_Shape& input);

    // Diagnostics
    std::vector<std::string> getIssuesFound() const;
    bool hadCriticalIssues() const;

private:
    double m_tolerance = 1e-6;
    double m_angularTolerance = 0.01;
    std::vector<std::string> m_issues;
};
```

### 5.3 FAGBuilder

```cpp
class FAGBuilder {
public:
    // Build the graph
    FaceAdjacencyGraph build(const TopoDS_Shape& shape);

    // Configuration
    void setMinBendRadius(double r);   // Below this = sharp edge
    void setMinFaceArea(double a);     // Below this = ignored

    // Diagnostics
    int getIgnoredFaceCount() const;
    std::vector<std::string> getWarnings() const;

private:
    double m_minBendRadius = 0.1;  // mm
    double m_minFaceArea = 0.01;   // mm²

    // Internal methods
    void extractFaces(const TopoDS_Shape& shape);
    void buildAdjacency();
    void classifyEdges();
    SurfaceProperties analyzeSurface(const TopoDS_Face& face);
    EdgeProperties analyzeEdge(const TopoDS_Edge& edge);
};
```

### 5.4 BaseFaceIdentifier

```cpp
class BaseFaceIdentifier {
public:
    // Find the base face
    int identify(const FaceAdjacencyGraph& fag);

    // Get all candidates with scores
    std::vector<BaseFaceCandidate> getCandidates() const;

    // Configuration
    void setWeights(double area, double connect, double central, double orient);

    // For manual override
    void setPreferredNormal(const gp_Dir& normal);

private:
    double m_weightArea = 0.3;
    double m_weightConnect = 0.3;
    double m_weightCentral = 0.2;
    double m_weightOrient = 0.2;

    std::optional<gp_Dir> m_preferredNormal;
    std::vector<BaseFaceCandidate> m_candidates;

    double scoreCandidate(const FAG_Node& node, const FaceAdjacencyGraph& fag);
};
```

### 5.5 BendClassifier

```cpp
class BendClassifier {
public:
    // Classify all bends in the FAG
    std::vector<BendFeature> classify(
        const FaceAdjacencyGraph& fag,
        int baseFaceId
    );

    // Classify a single bend edge
    BendClassification classifyBend(
        const FAG_Edge& edge,
        const FAG_Node& baseNode,
        const FAG_Node& flangeNode
    );

    // Configuration
    void setThickness(double t);  // For coplanarity checks
    void setHemAngleThreshold(double angle);  // Default: 45°

private:
    double m_thickness = 2.0;
    double m_hemAngleThreshold = 45.0;

    BendDirection computeDirection(
        const gp_Pnt& baseCentroid,
        const gp_Dir& baseNormal,
        const gp_Pnt& flangeCentroid
    );

    double computeBendAngle(const FAG_Edge& edge);
    gp_Lin extractBendLine(const TopoDS_Face& bendFace);
};
```

---

## 6. Error Handling

### 6.1 Error Categories

```cpp
enum class Phase1Error {
    // File errors
    FILE_NOT_FOUND,
    FILE_READ_ERROR,
    INVALID_STEP_FORMAT,
    UNSUPPORTED_AP_VERSION,

    // Geometry errors
    EMPTY_SHAPE,
    NO_SOLID_FOUND,
    MULTIPLE_SOLIDS,  // Warning, take first
    HEALING_FAILED,

    // Analysis errors
    NO_PLANAR_FACES,
    NO_BENDS_FOUND,
    BASE_FACE_AMBIGUOUS,
    INCONSISTENT_NORMALS,

    // Internal errors
    OCCT_EXCEPTION,
    MEMORY_ERROR
};

struct Phase1ErrorInfo {
    Phase1Error code;
    std::string message;
    std::string details;
    bool isFatal;
};
```

### 6.2 Error Handling Strategy

```cpp
class Phase1ErrorHandler {
public:
    void addError(Phase1Error code, const std::string& details);
    void addWarning(const std::string& message);

    bool hasFatalError() const;
    std::vector<Phase1ErrorInfo> getErrors() const;
    std::vector<std::string> getWarnings() const;

    // For user display
    std::string getSummary() const;
};
```

---

## 7. Test Cases

### 7.1 Unit Tests

```cpp
// test_step_reader.cpp
TEST(STEPReader, LoadsValidFile) { ... }
TEST(STEPReader, RejectsInvalidFile) { ... }
TEST(STEPReader, HandlesUnicodeFilename) { ... }

// test_geometry_healer.cpp
TEST(GeometryHealer, FixesSplitFaces) { ... }
TEST(GeometryHealer, UnifiesCoplanarFaces) { ... }
TEST(GeometryHealer, PreservesValidGeometry) { ... }

// test_fag_builder.cpp
TEST(FAGBuilder, BuildsSimpleBox) { ... }
TEST(FAGBuilder, IdentifiesBendEdges) { ... }
TEST(FAGBuilder, HandlesSharpEdges) { ... }

// test_base_face_identifier.cpp
TEST(BaseFaceIdentifier, FindsLargestFace) { ... }
TEST(BaseFaceIdentifier, PrefersHorizontal) { ... }
TEST(BaseFaceIdentifier, HandlesTie) { ... }

// test_bend_classifier.cpp
TEST(BendClassifier, ClassifiesUpBend) { ... }
TEST(BendClassifier, ClassifiesDownBend) { ... }
TEST(BendClassifier, ClassifiesHem) { ... }
TEST(BendClassifier, HandlesReversedNormal) { ... }
```

### 7.2 Integration Tests

```cpp
// test_phase1_integration.cpp
TEST(Phase1Integration, SimpleBox4Bends) {
    // Given: simple_box.step with 4 90° bends
    Phase1Output result = parseSTEP("test_data/simple_box.step");

    // Then: should identify 4 bends
    EXPECT_EQ(result.bends.size(), 4);

    // And: all bends should be 90°
    for (const auto& bend : result.bends) {
        EXPECT_NEAR(bend.targetAngle, 90.0, 0.1);
    }

    // And: should find base face (bottom)
    EXPECT_TRUE(result.baseFace.IsNull() == false);
}

TEST(Phase1Integration, LBracket1Bend) { ... }
TEST(Phase1Integration, ComplexPanel8Bends) { ... }
TEST(Phase1Integration, PartWithHoles) { ... }
TEST(Phase1Integration, PartWithHems) { ... }
```

### 7.3 Test Data Files

| File | Description | Expected Bends |
|------|-------------|----------------|
| simple_box.step | 4-sided box | 4 UP |
| l_bracket.step | L-shaped bracket | 1 UP |
| u_channel.step | U-channel | 2 UP |
| z_bracket.step | Z-shaped | 2 (1 UP, 1 DOWN) |
| complex_panel.step | 8 bends mixed | 8 |
| hem_example.step | With hem flange | 2 (1 regular, 1 HEM) |
| dirty_geometry.step | Needs healing | 4 |
| holes_cutouts.step | With holes | 4 |

---

## 8. Performance Requirements

| Operation | Target | Max Allowed |
|-----------|--------|-------------|
| STEP Load | < 200ms | 500ms |
| Geometry Healing | < 300ms | 1000ms |
| FAG Construction | < 100ms | 300ms |
| Base Face ID | < 10ms | 50ms |
| Bend Classification | < 50ms | 200ms |
| **Total Phase 1** | < 700ms | 2000ms |

*For typical parts (< 50 faces, < 20 bends)*

---

## 9. Dependencies

### 9.1 External Libraries

| Library | Version | Purpose |
|---------|---------|---------|
| OpenCASCADE | 7.6+ | STEP parsing, geometry |
| Eigen | 3.4+ | Matrix operations |
| spdlog | 1.x | Logging |
| Catch2 | 3.x | Unit testing |

### 9.2 OCCT Modules Required

```cmake
# CMake: Required OCCT components
find_package(OpenCASCADE REQUIRED COMPONENTS
    TKernel        # Core
    TKMath         # Math
    TKG3d          # 3D geometry
    TKBRep         # Boundary representation
    TKTopAlgo      # Topological algorithms
    TKShHealing    # Shape healing
    TKSTEP         # STEP reader
    TKSTEPBase
    TKXSBase       # Translation services
)
```

---

## 10. Open Questions

### 10.1 Implementation Strategies (from Research Papers)

#### Strategy 1: Face-First vs Edge-First Construction

**Face-First (Current approach):**
```cpp
1. Extract all faces
2. For each face pair, find shared edge
3. Classify edge as bend or sharp
```
- Pro: Simple, matches OCCT topology
- Con: O(n²) in worst case

**Edge-First (Recommended by S1):**
```cpp
1. Use TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE)
2. Iterate edges once (O(n))
3. For each edge, get adjacent faces (O(1) lookup)
```
- Pro: O(n) complexity, faster
- Con: Requires understanding of OCCT maps

**Decision**: Use Edge-First (TopExp::MapShapesAndAncestors) per research paper S1.

#### Strategy 2: Normal Orientation Consistency

**Problem**: OCCT faces can have arbitrary orientation (TopAbs_FORWARD vs TopAbs_REVERSED).

**Solution (from S3)**:
```cpp
// ALWAYS check orientation before using normal
gp_Dir GetMaterialOutwardNormal(const TopoDS_Face& face) {
    // ... compute geometric normal ...

    // CRITICAL STEP:
    if (face.Orientation() == TopAbs_REVERSED) {
        normal.Reverse();
    }

    // OPTIONAL: Validate with SDF
    if (!ValidateMaterialNormal(face, normal, solid)) {
        LOG_WARNING("Normal validation failed, reversing");
        normal.Reverse();
    }

    return normal;
}
```

**Best Practice**:
1. Always correct for TopAbs_REVERSED
2. Optionally validate with SDF (adds ~10ms overhead)
3. Store corrected normals in FAG_Node for reuse

#### Strategy 3: Handling Degenerate Geometry (from S4)

**Common Issues**:
1. **Split coplanar faces** - Should be one face
2. **Tiny sliver faces** - Artifacts from CAD export
3. **Zero-area faces** - Degenerate geometry
4. **Disconnected shells** - Multiple bodies

**Healing Pipeline (from S4)**:
```cpp
TopoDS_Shape HealingPipeline(TopoDS_Shape input) {
    // Step 1: Basic fixes (invalid vertices, edges)
    ShapeFix_Shape fixer(input);
    fixer.Perform();
    shape = fixer.Shape();

    // Step 2: Unify coplanar faces (CRITICAL for sheet metal)
    ShapeUpgrade_UnifySameDomain unifier(shape);
    unifier.SetLinearTolerance(1e-6);
    unifier.SetAngularTolerance(0.01);  // ~0.57 degrees
    unifier.Build();
    shape = unifier.Shape();

    // Step 3: Remove small faces (< threshold)
    ShapeUpgrade_RemoveInternalWires remover;
    // ... custom implementation to remove tiny faces ...

    // Step 4: Sew gaps
    BRepBuilderAPI_Sewing sewer(1e-6);
    sewer.Add(shape);
    sewer.Perform();
    shape = sewer.SewedShape();

    return shape;
}
```

**Tolerance Guidelines**:
- Linear: 1e-6 mm (1 nanometer) - standard precision
- Angular: 0.01 rad (~0.57°) - coplanarity tolerance
- Sewing: 1e-6 mm - gap closing

### 10.2 Design Decisions Pending

1. **Multiple Solids**: If STEP contains multiple solids, process all or just first?
   - Current decision: Process first, warn user
   - Alternative: Let user select

2. **Freeform Surfaces**: How to handle B-spline bends?
   - Current decision: Reject as unsupported
   - Alternative: Approximate as cylinder

3. **Zero-Radius Bends**: Treat as sharp edge or special bend?
   - Current decision: Sharp edge (no bend face)
   - Note: Will need special handling in Phase 4

### 10.2 Assumptions

1. Input STEP files are valid ISO 10303
2. Parts are single-thickness sheet metal
3. All bends are cylindrical (no compound curves)
4. Units are either MM or INCH (auto-convert to MM)

---

## 11. Implementation Checklist

### 11.0 Complete Phase 1 Pipeline (Pseudocode)

**Master Algorithm**: End-to-end flow from STEP to Phase1Output

```cpp
Phase1Output ParseSTEPFile(string filepath) {
    Phase1Output output;

    try {
        // STEP 1: Load STEP file
        LOG_INFO("Loading STEP file: " + filepath);
        STEPReader reader;
        if (!reader.load(filepath)) {
            output.isValid = false;
            output.errorMessage = reader.getLastError();
            return output;
        }
        TopoDS_Shape rawShape = reader.getShape();
        LOG_INFO("Loaded shape with " + CountFaces(rawShape) + " faces");

        // STEP 2: Geometry Healing
        LOG_INFO("Healing geometry...");
        GeometryHealer healer;
        TopoDS_Shape healedShape = healer.heal(rawShape);

        // Check if healing introduced issues
        if (healer.hadCriticalIssues()) {
            output.warnings.push_back("Geometry healing had critical issues");
        }
        LOG_INFO("Healed: " + healer.getIssuesFound().size() + " issues fixed");

        // STEP 3: Extract solid
        TopoDS_Solid solid = ExtractSolid(healedShape);
        if (solid.IsNull()) {
            output.isValid = false;
            output.errorMessage = "No solid found in STEP file";
            return output;
        }

        // STEP 4: Build Face-Adjacency Graph
        LOG_INFO("Building FAG...");
        FAGBuilder fagBuilder;
        FaceAdjacencyGraph fag = fagBuilder.build(solid);
        LOG_INFO("FAG: " + fag.nodeCount() + " nodes, " + fag.edgeCount() + " edges");

        // Validate FAG connectivity
        if (!fag.isConnected()) {
            output.warnings.push_back("FAG has disconnected components");
        }

        // STEP 5: Identify Base Face
        LOG_INFO("Identifying base face...");
        BaseFaceIdentifier baseFinder;
        int baseFaceId = baseFinder.identify(fag);

        if (baseFaceId < 0) {
            output.isValid = false;
            output.errorMessage = "Could not identify base face";
            return output;
        }

        const FAG_Node& baseNode = fag.getNode(baseFaceId);
        LOG_INFO("Base face: " + baseNode.debugName +
                 " (area: " + baseNode.area + " mm²)");

        // Log all candidates for debugging
        auto candidates = baseFinder.getCandidates();
        for (const auto& c : candidates) {
            LOG_DEBUG("  Candidate " + c.nodeId +
                      " score: " + c.score);
        }

        // STEP 6: Classify Bends
        LOG_INFO("Classifying bends...");
        BendClassifier classifier;
        classifier.setThickness(DetectThickness(solid));

        std::vector<BendFeature> bends = classifier.classify(fag, baseFaceId);
        LOG_INFO("Found " + bends.size() + " bends");

        // Log each bend
        for (size_t i = 0; i < bends.size(); i++) {
            const auto& bend = bends[i];
            LOG_INFO("  Bend " + i + ": " +
                     bend.signedAngle + "° " +
                     DirectionToString(bend.direction) +
                     " (radius: " + bend.internalRadius + " mm)");
        }

        // STEP 7: Validate Results
        if (bends.empty()) {
            output.warnings.push_back("No bends found - is this a flat part?");
        }

        // Check for normal consistency
        if (!fag.hasConsistentNormals()) {
            output.warnings.push_back("FAG has inconsistent normals");
        }

        // STEP 8: Populate Output
        output.fag = std::move(fag);
        output.bends = std::move(bends);
        output.baseFaceId = baseFaceId;
        output.baseFace = baseNode.face;
        output.thickness = DetectThickness(solid);
        output.partCentroid = ComputeCentroid(solid);
        output.boundingBox = ComputeBoundingBox(solid);
        output.sourceFile = filepath;
        output.partName = reader.getPartName();
        output.isValid = true;

        LOG_INFO("Phase 1 complete successfully");
        return output;

    } catch (const Standard_Failure& e) {
        output.isValid = false;
        output.errorMessage = "OCCT Exception: " + string(e.GetMessageString());
        LOG_ERROR(output.errorMessage);
        return output;

    } catch (const std::exception& e) {
        output.isValid = false;
        output.errorMessage = "Exception: " + string(e.what());
        LOG_ERROR(output.errorMessage);
        return output;
    }
}
```

**Helper Functions**:

```cpp
// Extract first solid from compound
TopoDS_Solid ExtractSolid(const TopoDS_Shape& shape) {
    TopExp_Explorer exp(shape, TopAbs_SOLID);
    if (exp.More()) {
        return TopoDS::Solid(exp.Current());
    }
    return TopoDS_Solid();  // Null
}

// Detect sheet thickness from geometry
double DetectThickness(const TopoDS_Solid& solid) {
    // Method 1: Find thinnest dimension of bounding box
    Bnd_Box bbox = ComputeBoundingBox(solid);
    double dx, dy, dz;
    bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    dx = xmax - xmin;
    dy = ymax - ymin;
    dz = zmax - zmin;

    // Assume thickness is smallest dimension
    return std::min({dx, dy, dz});

    // Method 2 (more accurate): Analyze planar faces
    // Find most common distance between parallel planar faces
    // (More complex, implement if Method 1 insufficient)
}

// Convert direction enum to string
string DirectionToString(BendDirection dir) {
    switch (dir) {
        case BEND_UP: return "UP";
        case BEND_DOWN: return "DOWN";
        case HEM: return "HEM";
        default: return "UNKNOWN";
    }
}
```

**Error Recovery Strategies**:

1. **STEP load failure**:
   - Try different STEP readers (AP203, AP214, AP242)
   - Check file encoding (UTF-8 vs ASCII)
   - Validate STEP syntax with external tool

2. **Healing failure**:
   - Try more aggressive healing (higher tolerances)
   - Skip UnifySameDomain if it fails
   - Continue with best-effort healed shape

3. **No base face found**:
   - Allow user to manually select base face
   - Use first planar face as fallback
   - Prompt for material orientation

4. **No bends found**:
   - Verify part is actually bent sheet metal
   - Check MIN_BEND_RADIUS threshold
   - May be a flat pattern → skip to Phase 5

### 11.1 Core Components
- [ ] STEPReader class
- [ ] GeometryHealer class
- [ ] FAGBuilder class
- [ ] BaseFaceIdentifier class
- [ ] BendClassifier class
- [ ] Phase1Output struct

### 11.2 Algorithms
- [ ] GetMaterialOutwardNormal()
- [ ] **NEW: ValidateMaterialNormal() - SDF check**
- [ ] **NEW: ExtractBendAxis() - Cylindrical axis extraction**
- [ ] BuildFaceAdjacencyGraph() - **with TopExp::MapShapesAndAncestors**
- [ ] ClassifyFace()
- [ ] **NEW: ClassifyEdge() - BEND vs SHARP distinction**
- [ ] **NEW: ComputeDihedralAngle() - Signed angle between faces**
- [ ] FindBaseFace()
- [ ] ClassifyBendDirection()
- [ ] HealGeometry()
- [ ] **NEW: DetectThickness() - Auto-detect sheet thickness**
- [ ] **NEW: ParseSTEPFile() - Complete pipeline**

### 11.3 Tests
- [ ] Unit tests for each class
- [ ] Integration tests with test STEP files
- [ ] Performance benchmarks
- [ ] **NEW: SDF validation tests (reversed normals)**
- [ ] **NEW: Bend axis orientation consistency tests**
- [ ] **NEW: Edge classification tests (bend vs sharp)**
- [ ] **NEW: Dihedral angle accuracy tests**
- [ ] **NEW: Healing pipeline robustness tests**
- [ ] **NEW: Thickness detection accuracy tests**

### 11.4 Documentation
- [x] Design document (this file)
- [ ] API reference
- [ ] Usage examples

---

## 12. References to Research Papers

This design document is based on the following research papers:

### Primary References (Phase 1 Specific)

**S1: Building Face-Adjacency Graph**
- File: `research_paper/S1 Building Face-Adjacency Graph.pdf`
- Key Contributions:
  - TopExp::MapShapesAndAncestors() O(n) algorithm
  - Edge orientation consistency tracking
  - Internal vs boundary edge classification
  - Bend axis extraction from cylindrical faces

**S2: Sheet Metal Feature Recognition**
- File: `research_paper/S2 Sheet Metal Feature Recognition.pdf`
- Key Contributions:
  - Edge classification algorithm (BEND vs SHARP)
  - Bend type taxonomy (ACUTE, RIGHT, OBTUSE, HEM)
  - Dihedral angle calculation
  - Bend radius extraction

**S3: OCCT Normal Consistency for Bending**
- File: `research_paper/S3 OCCT Normal Consistency for Bending.pdf`
- Key Contributions:
  - GetMaterialOutwardNormal() with TopAbs_REVERSED correction
  - SDF (Signed Distance Function) validation
  - Material-aware normal orientation
  - BRepClass3d_SolidClassifier usage

**S4: OCCT Dirty Geometry Handling for FAG**
- File: `research_paper/S4 OCCT Dirty Geometry Handling for FAG.pdf`
- Key Contributions:
  - Geometry healing pipeline
  - ShapeUpgrade_UnifySameDomain for coplanar faces
  - Tolerance selection guidelines
  - Degenerate geometry handling strategies

### Supporting References

**General Framework**:
- Salvagnini STREAMBEND CAM Logic (comprehensive mathematical framework)
- Architectural Deconstruction of Automated Bend Sequencing Logic

**Related Phases**:
- See `phase2-design.md` for constraint analysis (depends on Phase 1 FAG)
- See `phase3-design.md` for sequencing (uses bend features from Phase 1)

---

## 13. Appendix: OCCT API Quick Reference

### Common OCCT Classes Used in Phase 1

```cpp
// STEP I/O
STEPControl_Reader              // Load STEP files
STEPControl_Writer              // Export STEP files

// Shape Healing
ShapeFix_Shape                  // General shape fixing
ShapeUpgrade_UnifySameDomain    // Merge coplanar faces
BRepBuilderAPI_Sewing           // Close gaps

// Topology Exploration
TopExp                          // Topology utilities
TopExp_Explorer                 // Iterate shapes
TopExp::MapShapes()             // Extract shapes
TopExp::MapShapesAndAncestors() // Build edge→face map

// Topology Data
TopoDS_Shape                    // Base shape class
TopoDS_Solid                    // 3D solid
TopoDS_Face                     // 2D face
TopoDS_Edge                     // 1D edge
TopoDS_Vertex                   // 0D vertex

// Geometry Access
BRep_Tool::Surface()            // Get surface from face
BRep_Tool::Curve()              // Get curve from edge
BRepTools::UVBounds()           // Get UV parameter bounds

// Geometry Adaptors
BRepAdaptor_Surface             // Surface queries
BRepAdaptor_Curve               // Curve queries

// Geometric Entities
gp_Pnt                          // 3D point
gp_Dir                          // 3D direction (unit vector)
gp_Vec                          // 3D vector
gp_Lin                          // 3D line
gp_Circ                         // 3D circle
gp_Ax1                          // Axis (point + direction)
gp_Cylinder                     // Cylindrical surface

// Surface Properties
GeomLProp_SLProps               // Local surface properties (normals)

// Geometric Primitives
BRepPrimAPI_MakeCylinder        // Create cylinder
BRepPrimAPI_MakeRevol           // Revolve profile

// Classification
BRepClass3d_SolidClassifier     // Point-in-solid test (SDF)

// Analysis
GProp_GProps                    // Global properties
BRepGProp::VolumeProperties()   // Compute volume, centroid
BRepGProp::SurfaceProperties()  // Compute area

// Bounding Boxes
Bnd_Box                         // Axis-aligned bounding box
BRepBndLib::Add()               // Add shape to bbox
```

### Common Pitfalls

1. **Forgetting to check TopAbs_REVERSED orientation**
   ```cpp
   // WRONG:
   gp_Dir normal = props.Normal();

   // CORRECT:
   gp_Dir normal = props.Normal();
   if (face.Orientation() == TopAbs_REVERSED)
       normal.Reverse();
   ```

2. **Not handling null shapes**
   ```cpp
   // ALWAYS check:
   if (shape.IsNull()) {
       // Handle error
   }
   ```

3. **Mixing Handle<> and TopoDS types**
   ```cpp
   // OCCT uses both:
   Handle<Geom_Surface> surf;   // Reference counted
   TopoDS_Face face;            // Value type (lightweight)
   ```

4. **Tolerance issues**
   ```cpp
   // Use consistent tolerances:
   constexpr double TOLERANCE = 1e-6;  // mm

   // Not magic numbers everywhere:
   if (distance < 0.00001) { ... }  // BAD
   if (distance < TOLERANCE) { ... } // GOOD
   ```

---

**End of Phase 1 Design Document**
