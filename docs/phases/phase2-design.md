# Phase 2: Constraint Solver - Design Document

> **Version**: 1.0
> **Status**: IN DESIGN
> **Last Updated**: 2026-02-05
> **Based on**: Research papers P2S1, P2S2, P2S3 + ROADMAP.md

---

## 1. Overview

### 1.1 Purpose
Phase 2 là "The Rules" của hệ thống - phân tích Face-Adjacency Graph từ Phase 1 để xác định tất cả ràng buộc vật lý và xây dựng Precedence DAG cho bend sequencing.

### 1.2 Input
```cpp
struct Phase1Output {
    FaceAdjacencyGraph fag;           // Graph structure from Phase 1
    std::vector<BendFeature> bends;   // All identified bends
    int baseFaceId;                   // Base face node ID
    double thickness;                 // Sheet thickness (mm)
    TopoDS_Shape originalShape;       // For 3D validation
};
```

### 1.3 Output
```cpp
struct Phase2Output {
    PrecedenceDAG dag;                                // Precedence relationships
    std::map<int, GraspConstraint> graspConstraints;  // Per bend state
    std::map<int, ABAConstraint> abaConstraints;      // Per bend
    std::vector<int> topologicalOrder;                // Valid bend order (if acyclic)

    bool success;
    std::string errorMessage;
    std::vector<std::string> warnings;
};
```

### 1.4 Key Objectives
1. ✅ Identify **geometric precedence** constraints (which bends must go first)
2. ✅ Compute **valid grip regions** at each bend state
3. ✅ Determine **ABA tool configurations** needed
4. ✅ Build **Precedence DAG** (must be acyclic)
5. ✅ Detect **box closing scenarios** (impossible bends)

---

## 2. Architecture

### 2.1 Module Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                   Phase 2: Constraint Solver                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐    ┌──────────────────┐                   │
│  │  Phase1Output    │───▶│  Geometric       │                   │
│  │  (FAG + Bends)   │    │  Precedence      │                   │
│  └──────────────────┘    │  Analyzer        │                   │
│                          └────────┬─────────┘                   │
│                                   │                             │
│  ┌──────────────────┐             │  ┌──────────────────┐       │
│  │  Grasp           │◀────────────┼─▶│  ABA             │       │
│  │  Constraint      │             │  │  Constraint      │       │
│  │  Generator       │             │  │  Analyzer        │       │
│  └────────┬─────────┘             │  └────────┬─────────┘       │
│           │                       │           │                 │
│           └───────────┬───────────┴───────────┘                 │
│                       ▼                                         │
│            ┌──────────────────┐                                 │
│            │  Precedence DAG  │                                 │
│            │  Builder         │                                 │
│            └────────┬─────────┘                                 │
│                     │                                           │
│                     ▼                                           │
│            ┌──────────────────┐                                 │
│            │  Phase2Output    │                                 │
│            └──────────────────┘                                 │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 Class Diagram

```
┌──────────────────────────────────┐
│      ConstraintSolver            │
├──────────────────────────────────┤
│ - fag: FaceAdjacencyGraph        │
│ - bends: vector<BendFeature>     │
│ - precedenceAnalyzer: GeoPrecAna │
│ - graspGenerator: GraspGen       │
│ - abaAnalyzer: ABAAnalyzer       │
│ - dagBuilder: DAGBuilder         │
├──────────────────────────────────┤
│ + analyze(): Phase2Output        │
│ + validate(): bool               │
│ + getDAG(): PrecedenceDAG        │
└──────────────────────────────────┘
         │
         │ creates
         ▼
┌──────────────────────────────────┐
│      PrecedenceDAG               │
├──────────────────────────────────┤
│ - nodes: vector<PrecedenceNode>  │
│ - edges: vector<PrecedenceEdge>  │
│ - adjacencyList: map             │
├──────────────────────────────────┤
│ + addConstraint(): void          │
│ + isAcyclic(): bool              │
│ + topologicalSort(): vector      │
│ + detectCycles(): vector         │
└──────────────────────────────────┘
```

---

## 3. Data Structures

### 3.1 Core Types

#### PrecedenceNode
```cpp
struct PrecedenceNode {
    int id;                    // Node ID (same as bend ID)
    int bendId;                // Associated bend

    // Incoming/outgoing constraints
    std::vector<int> predecessors;  // Bends that must come before
    std::vector<int> successors;    // Bends that must come after

    // Metadata
    int level;                 // Topological level (computed)
    bool visited;              // For graph traversal
};
```

#### PrecedenceEdge
```cpp
enum ConstraintType {
    GEOMETRIC,        // Corner overlap, geometric interference
    BOX_CLOSING,      // 3-sided box scenario
    SEQUENTIAL,       // Bent flanges blocking access
    USER_DEFINED      // Manual override
};

struct PrecedenceEdge {
    int id;
    int fromBend;             // Bend that must come first
    int toBend;               // Bend that must come second
    ConstraintType type;      // Type of constraint
    double confidence;        // 0.0-1.0 (for debugging)
    std::string reasoning;    // Human-readable explanation

    // Geometric data (optional, for visualization)
    gp_Pnt conflictPoint;     // Where interference occurs
};
```

#### DeadZone
```cpp
enum DeadZoneType {
    FLANGE,           // Standing bent flange
    SAFETY_MARGIN,    // Buffer around future bends
    ABA_TOOL,         // ABA interference zone
    BOUNDARY          // Part boundary (no grip possible)
};

struct DeadZone {
    int id;
    DeadZoneType type;
    std::vector<gp_Pnt2d> polygon;  // 2D polygon on base plane
    int causedByBend;                // Which bend created this (-1 if not applicable)
    double safetyMargin;             // Additional buffer (mm)

    // Methods
    bool contains(gp_Pnt2d point) const;
    double area() const;
};
```

#### GraspConstraint
```cpp
struct Rectangle2D {
    gp_Pnt2d bottomLeft;
    gp_Pnt2d topRight;
    double width;
    double height;
    double area;
    gp_Pnt2d center;
};

struct Polygon2D {
    std::vector<gp_Pnt2d> vertices;
    std::vector<std::vector<gp_Pnt2d>> holes;  // Inner holes (if any)

    double area() const;
    gp_Pnt2d centroid() const;
    bool contains(gp_Pnt2d point) const;
};

struct GraspConstraint {
    int stateId;                        // After which bend configuration
    std::vector<int> bentBends;         // Bends already completed

    // Dead zones
    std::vector<DeadZone> deadZones;    // All forbidden regions

    // Valid grip region
    Polygon2D validRegion;              // Valid grip area
    double validArea;                   // Total valid area (mm²)

    // Optimal grip placement (MIR algorithm result)
    Rectangle2D maxInscribedRect;       // Largest rectangle in valid region
    gp_Pnt2d optimalGripCenter;         // Recommended grip point

    // Physics validation
    bool hasValidGrip;                  // Can we grip at all?
    double minRequiredArea;             // Minimum area threshold (100 mm²)
    gp_Pnt2d centerOfMass;              // COM of current bent shape

    // Warnings
    std::vector<std::string> warnings;  // E.g., "Grip region shrinking"
};
```

#### ABAConstraint
```cpp
struct ABAConstraint {
    int bendId;

    // Requirements
    double bendLength;                  // Length of bend line (mm)
    double requiredWidth;               // Minimum tool width needed
    double clearance;                   // Safety clearance (typically 10-20mm)

    // Solution (subset sum result)
    std::vector<int> segmentSolution;   // e.g., {100, 150, 200}
    int totalSegments;                  // Number of segments
    double totalWidth;                  // Sum of segments

    // Feasibility
    bool feasible;                      // Solution exists?
    bool isBoxClosing;                  // Would this close a 3-sided box?

    // If infeasible
    std::string reason;                 // Why not feasible
    std::vector<int> suggestedAlternatives;  // Other possible widths
};
```

#### PrecedenceDAG
```cpp
class PrecedenceDAG {
public:
    // Construction
    void addNode(int bendId);
    void addEdge(int from, int to, ConstraintType type, const std::string& reason = "");
    void finalize();  // Build adjacency structures

    // Validation
    bool isAcyclic() const;
    std::vector<std::vector<int>> detectCycles() const;

    // Traversal
    std::vector<int> topologicalSort() const;
    std::vector<int> getPredecessors(int bendId) const;
    std::vector<int> getSuccessors(int bendId) const;

    // Query
    int nodeCount() const { return static_cast<int>(m_nodes.size()); }
    int edgeCount() const { return static_cast<int>(m_edges.size()); }
    const PrecedenceNode& getNode(int id) const;
    const PrecedenceEdge& getEdge(int id) const;

    // Export
    std::string toDOT() const;  // Graphviz format for visualization

private:
    std::vector<PrecedenceNode> m_nodes;
    std::vector<PrecedenceEdge> m_edges;
    std::map<int, std::vector<int>> m_adjacencyList;  // bendId -> successors
    bool m_finalized;

    // Internal
    bool detectCyclesUtil(int node, std::vector<bool>& visited,
                          std::vector<bool>& recStack,
                          std::vector<int>& cycle) const;
};
```

### 3.2 Phase2Output
```cpp
struct Phase2Output {
    // Core results
    PrecedenceDAG dag;
    std::map<int, GraspConstraint> graspConstraints;  // Key = state ID
    std::map<int, ABAConstraint> abaConstraints;      // Key = bend ID

    // Derived results
    std::vector<int> topologicalOrder;  // Valid ordering (if acyclic)
    bool isAcyclic;                     // DAG validation result

    // Statistics
    int geometricConstraintCount;
    int boxClosingCount;
    int sequentialConstraintCount;
    int totalConstraints;

    // Status
    bool success;
    std::string errorMessage;
    std::vector<std::string> warnings;

    // Timing
    double analysisTime;  // Time spent in Phase 2 (ms)
};
```

---

## 4. Module Implementations

### 4.1 Geometric Precedence Analyzer

**Purpose**: Detect which bends must happen before others due to geometry.

#### 4.1.1 Type 1: Corner Overlap Detection

**Algorithm**: Ray Casting (from P2S1)

```cpp
class GeometricPrecedenceAnalyzer {
public:
    std::vector<PrecedenceEdge> analyze(
        const std::vector<BendFeature>& bends,
        const FaceAdjacencyGraph& fag
    );

private:
    // Type 1: Corner overlap
    bool checkCornerOverlap(
        const BendFeature& bi,
        const BendFeature& bj,
        const BentState& state
    );

    // Type 2: Box closing
    bool isBoxClosing(
        const BendFeature& bend,
        const BentState& state
    );

    // Type 3: Sequential blocking
    bool isBlocked(
        const BendFeature& bi,
        const BendFeature& bj,
        const BentState& state
    );

    // Helpers
    std::vector<gp_Pnt> getFlangeCorners(
        const BendFeature& bend,
        const BentState& state
    );

    gp_Vec predictMotionPath(
        const BendFeature& bend,
        const BentState& state
    );

    bool rayIntersectsFlange(
        const gp_Pnt& origin,
        const gp_Vec& direction,
        const TopoDS_Face& flange
    );
};
```

**Corner Overlap Detection Algorithm**:
```
For each pair of bends (Bi, Bj):
    1. Simulate bending Bi first
    2. Get 4 corner points of Bi's flange in bent state
    3. For each corner:
        a. Cast ray along motion path during bending
        b. Check if ray intersects Bj's flange
        c. If YES → Bi must bend before Bj
    4. Record constraint if found
```

#### 4.1.2 Type 2: Box Closing Detection

**Algorithm**: 2D Polygon Analysis

```cpp
bool GeometricPrecedenceAnalyzer::isBoxClosing(
    const BendFeature& bend,
    const BentState& state
) {
    // Project all bent flanges to 2D
    std::vector<Polygon2D> bentFlanges2D;
    for (const auto& bentBend : state.bentBends) {
        bentFlanges2D.push_back(projectTo2D(bentBend.flangeFace));
    }

    // Check if they form 3 sides of rectangle/box
    if (forms3SidedBox(bentFlanges2D)) {
        // Project next flange
        Polygon2D nextFlange2D = projectTo2D(bend.flangeFace);

        // Would it close the 4th side?
        if (wouldClose4thSide(bentFlanges2D, nextFlange2D)) {
            return true;  // TRAP! Tool cannot escape
        }
    }

    return false;
}
```

**3-Sided Box Detection**:
```
1. Project bent flanges onto base plane
2. Check if they form U-shape or 3 walls:
   - Three flanges roughly perpendicular to base
   - Their 2D projections form 3 sides of rectangle
   - Gap exists on 4th side
3. If YES, mark as "dangerous state" - next bend might trap tool
```

#### 4.1.3 Type 3: Sequential Blocking

**Algorithm**: Accessibility Analysis

```cpp
bool GeometricPrecedenceAnalyzer::isBlocked(
    const BendFeature& bi,  // Bend to check
    const BendFeature& bj,  // Potential blocker
    const BentState& state
) {
    // Simulate bending Bj first
    BentState tempState = state;
    tempState.applyBend(bj);

    // Check if Bi's bend line is accessible
    gp_Lin bendLine = bi.getBendLine();
    gp_Dir toolApproach(0, 0, 1);  // Vertical approach

    // Does Bj's bent flange block tool access?
    for (const auto& bentBend : tempState.bentBends) {
        if (bentBend.id == bj.id) {
            // Check if flange intersects tool path
            if (flangeBlocksAccess(bentBend.flangeFace, bendLine, toolApproach)) {
                return true;  // Bj blocks Bi → Bi must bend first
            }
        }
    }

    return false;
}
```

---

### 4.2 Grasp Constraint Generator

**Purpose**: Compute valid grip regions at each bend state.

#### 4.2.1 Dead Zone Calculation

```cpp
class GraspConstraintGenerator {
public:
    std::map<int, GraspConstraint> generateConstraints(
        const std::vector<BendFeature>& bends,
        const FaceAdjacencyGraph& fag,
        double thickness
    );

private:
    // Dead zone computation
    std::vector<DeadZone> computeDeadZones(
        const BentState& state,
        const std::vector<BendFeature>& remainingBends
    );

    DeadZone projectFlangeToDeadZone(
        const TopoDS_Face& flange,
        double safetyMargin = 20.0  // mm
    );

    // Valid region computation
    Polygon2D computeValidRegion(
        const Polygon2D& baseFace,
        const std::vector<DeadZone>& deadZones
    );

    // MIR algorithm
    Rectangle2D findMaximumInscribedRectangle(
        const Polygon2D& validRegion
    );

    // Physics validation
    bool validateGripPhysics(
        const Rectangle2D& grip,
        const BentState& state,
        double minArea = 100.0  // mm²
    );
};
```

**Dead Zone Generation Algorithm**:
```
For each bent state:
    1. Initialize dead zones:
        a. For each bent flange:
            - Project flange geometry onto base plane
            - Add safety margin (20mm) around projection
            - Mark as FLANGE dead zone

        b. For each future bend:
            - Project bend line area onto base plane
            - Add safety margin
            - Mark as SAFETY_MARGIN dead zone

        c. Part boundaries:
            - Mark edges of base face as BOUNDARY

    2. Union all dead zones into forbidden regions

    3. Compute valid grip area:
        Valid = Base_Face_Area - Union(all_dead_zones)
```

#### 4.2.2 Maximum Inscribed Rectangle (MIR)

**Algorithm**: Sweep Line + Dynamic Programming (from P2S2)

```cpp
Rectangle2D GraspConstraintGenerator::findMaximumInscribedRectangle(
    const Polygon2D& validRegion
) {
    Rectangle2D maxRect;
    double maxArea = 0.0;

    // Get all x-coordinates from polygon vertices
    std::vector<double> xCoords;
    for (const auto& vertex : validRegion.vertices) {
        xCoords.push_back(vertex.X());
    }
    std::sort(xCoords.begin(), xCoords.end());

    // Sweep vertical lines
    for (size_t i = 0; i < xCoords.size(); i++) {
        for (size_t j = i+1; j < xCoords.size(); j++) {
            double x1 = xCoords[i];
            double x2 = xCoords[j];
            double width = x2 - x1;

            // Find max height at this x-range
            double maxHeight = findMaxHeightInRange(validRegion, x1, x2);

            double area = width * maxHeight;
            if (area > maxArea) {
                maxArea = area;
                maxRect = Rectangle2D{
                    gp_Pnt2d(x1, 0),  // Will adjust Y
                    gp_Pnt2d(x2, maxHeight),
                    width,
                    maxHeight,
                    area,
                    gp_Pnt2d((x1+x2)/2, maxHeight/2)
                };
            }
        }
    }

    return maxRect;
}
```

**MIR Reference**:
- "Computing the Largest Empty Rectangle" - Naamad et al.
- Complexity: O(n² log n) where n = polygon vertices

#### 4.2.3 Grip Physics Validation

```cpp
bool GraspConstraintGenerator::validateGripPhysics(
    const Rectangle2D& grip,
    const BentState& state,
    double minArea
) {
    // Check 1: Minimum area
    if (grip.area < minArea) {
        return false;  // Too small to grip
    }

    // Check 2: Center of mass within grip
    gp_Pnt2d com = state.getCenterOfMass2D();
    if (!grip.contains(com)) {
        return false;  // Would tip during handling
    }

    // Check 3: Torque stability
    double leverArm = com.Distance(grip.center);
    double mass = state.getMass();  // kg
    double torque = mass * 9.81 * leverArm;  // N⋅m

    const double MAX_TORQUE = 10.0;  // N⋅m (configurable)
    if (torque > MAX_TORQUE) {
        return false;  // Would tilt during grip
    }

    // Check 4: Shear resistance
    double shearForce = mass * 9.81;  // N (worst case: vertical)
    double frictionCoeff = 0.6;  // Vacuum grip on steel
    double normalForce = 500.0;  // N (grip pressure)
    double frictionForce = frictionCoeff * normalForce;

    if (shearForce > frictionForce) {
        return false;  // Would slip from grip
    }

    return true;  // All checks passed
}
```

---

### 4.3 ABA Constraint Analyzer

**Purpose**: Determine tool configuration for each bend.

#### 4.3.1 ABA Width Calculation

```cpp
class ABAConstraintAnalyzer {
public:
    std::map<int, ABAConstraint> analyzeABARequirements(
        const std::vector<BendFeature>& bends,
        const std::vector<int>& availableSegments = {50, 100, 150, 200, 250, 300, 400, 500}
    );

private:
    // Width calculation
    double computeRequiredWidth(
        const BendFeature& bend,
        double clearance = 15.0  // mm
    );

    // Subset sum solver
    std::vector<int> solveSubsetSum(
        double targetWidth,
        const std::vector<int>& segments
    );

    // Box closing check
    bool wouldCloseBox(
        const BendFeature& bend,
        const BentState& state
    );
};
```

**Required Width Formula**:
```cpp
double ABAConstraintAnalyzer::computeRequiredWidth(
    const BendFeature& bend,
    double clearance
) {
    // Bend line length
    double bendLength = bend.bendLineLength;

    // Add clearance on both sides
    double requiredWidth = bendLength + 2 * clearance;

    return requiredWidth;
}
```

#### 4.3.2 Subset Sum Solver

**Algorithm**: Dynamic Programming

```cpp
std::vector<int> ABAConstraintAnalyzer::solveSubsetSum(
    double targetWidth,
    const std::vector<int>& segments
) {
    int n = segments.size();
    int target = static_cast<int>(std::ceil(targetWidth));

    // DP table: dp[i][w] = can we make width w using first i segments?
    std::vector<std::vector<bool>> dp(n+1, std::vector<bool>(target+1, false));
    dp[0][0] = true;  // Base case: 0 segments = 0 width

    // Fill DP table
    for (int i = 1; i <= n; i++) {
        int seg = segments[i-1];
        for (int w = 0; w <= target; w++) {
            // Option 1: Don't use segment i
            dp[i][w] = dp[i-1][w];

            // Option 2: Use segment i (if possible)
            if (w >= seg && dp[i-1][w - seg]) {
                dp[i][w] = true;
            }
        }
    }

    // Check if solution exists
    if (!dp[n][target]) {
        return {};  // No solution
    }

    // Backtrack to find solution with minimum segments
    std::vector<int> solution;
    int w = target;
    for (int i = n; i > 0; i--) {
        // Did we use segment i?
        if (w >= segments[i-1] && dp[i-1][w - segments[i-1]] && !dp[i-1][w]) {
            solution.push_back(segments[i-1]);
            w -= segments[i-1];
        }
    }

    return solution;
}
```

**Complexity**: O(n × W) where n = segments, W = target width

---

### 4.4 Precedence DAG Builder

**Purpose**: Combine all constraints into unified graph.

```cpp
class PrecedenceDAGBuilder {
public:
    PrecedenceDAG build(
        const std::vector<PrecedenceEdge>& geometricConstraints,
        const std::map<int, ABAConstraint>& abaConstraints,
        int bendCount
    );

private:
    bool detectCycles(
        const PrecedenceDAG& dag,
        std::vector<std::vector<int>>& cycles
    );

    std::vector<int> topologicalSort(const PrecedenceDAG& dag);
};
```

**DAG Construction Algorithm**:
```
1. Create nodes:
    For each bend:
        Add PrecedenceNode with bend ID

2. Add edges from geometric constraints:
    For each PrecedenceEdge:
        Add directed edge (from → to) with constraint type

3. Add edges from ABA constraints:
    For bends requiring same ABA config:
        Group together (optional optimization)

4. Validate acyclic property:
    Run cycle detection (DFS with recursion stack)
    If cycles found → report error

5. Compute topological order:
    If acyclic → run topological sort
    This gives one valid ordering
```

**Cycle Detection (DFS)**:
```cpp
bool PrecedenceDAGBuilder::detectCycles(
    const PrecedenceDAG& dag,
    std::vector<std::vector<int>>& cycles
) {
    int n = dag.nodeCount();
    std::vector<bool> visited(n, false);
    std::vector<bool> recStack(n, false);  // Recursion stack
    std::vector<int> currentPath;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            if (detectCyclesUtil(dag, i, visited, recStack, currentPath, cycles)) {
                return true;  // Cycle found
            }
        }
    }

    return false;  // No cycles
}

bool detectCyclesUtil(
    const PrecedenceDAG& dag,
    int node,
    std::vector<bool>& visited,
    std::vector<bool>& recStack,
    std::vector<int>& path,
    std::vector<std::vector<int>>& cycles
) {
    visited[node] = true;
    recStack[node] = true;
    path.push_back(node);

    for (int successor : dag.getSuccessors(node)) {
        if (!visited[successor]) {
            if (detectCyclesUtil(dag, successor, visited, recStack, path, cycles)) {
                return true;
            }
        } else if (recStack[successor]) {
            // Back edge found → cycle!
            std::vector<int> cycle;
            auto it = std::find(path.begin(), path.end(), successor);
            cycle.assign(it, path.end());
            cycles.push_back(cycle);
            return true;
        }
    }

    recStack[node] = false;
    path.pop_back();
    return false;
}
```

---

## 5. Algorithms Reference

### 5.1 Ray Casting for Intersection

```cpp
bool rayIntersectsFlange(
    const gp_Pnt& origin,
    const gp_Vec& direction,
    const TopoDS_Face& flange
) {
    // Create ray as infinite line
    gp_Lin ray(origin, gp_Dir(direction));

    // Use IntCurvesFace to find intersections
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(flange, Precision::Confusion());
    intersector.Perform(ray, 0.0, Precision::Infinite());

    // Check if any intersections
    return intersector.NbPnt() > 0;
}
```

### 5.2 2D Polygon Projection

```cpp
Polygon2D projectTo2D(const TopoDS_Face& face) {
    Polygon2D poly;

    // Get outer wire
    TopoDS_Wire outerWire = BRepTools::OuterWire(face);

    // Iterate edges
    BRepTools_WireExplorer explorer(outerWire);
    for (; explorer.More(); explorer.Next()) {
        const TopoDS_Edge& edge = explorer.Current();

        // Get edge geometry
        Standard_Real first, last;
        Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

        // Sample points (simple approach)
        gp_Pnt p1 = curve->Value(first);
        gp_Pnt p2 = curve->Value(last);

        // Project to XY plane (Z=0)
        poly.vertices.push_back(gp_Pnt2d(p1.X(), p1.Y()));
        poly.vertices.push_back(gp_Pnt2d(p2.X(), p2.Y()));
    }

    // Remove duplicates
    poly.removeDuplicates();

    return poly;
}
```

### 5.3 Polygon Union (for dead zones)

```cpp
Polygon2D unionPolygons(const std::vector<Polygon2D>& polygons) {
    // Use CGAL for robust polygon operations
    #include <CGAL/Polygon_2.h>
    #include <CGAL/Boolean_set_operations_2.h>

    typedef CGAL::Exact_predicates_exact_constructions_kernel K;
    typedef CGAL::Polygon_2<K> CGAL_Polygon;
    typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;

    // Convert to CGAL polygons
    std::vector<CGAL_Polygon> cgal_polys;
    for (const auto& poly : polygons) {
        CGAL_Polygon cgal_poly;
        for (const auto& vertex : poly.vertices) {
            cgal_poly.push_back(K::Point_2(vertex.X(), vertex.Y()));
        }
        cgal_polys.push_back(cgal_poly);
    }

    // Compute union
    Polygon_with_holes result = cgal_polys[0];
    for (size_t i = 1; i < cgal_polys.size(); i++) {
        Polygon_with_holes temp;
        CGAL::join(result, cgal_polys[i], temp);
        result = temp;
    }

    // Convert back to Polygon2D
    return convertFromCGAL(result);
}
```

---

## 6. Testing Strategy

### 6.1 Unit Tests

#### Geometric Precedence Tests
```cpp
TEST_CASE("Corner overlap detection") {
    // Setup: Two bends where B1 corners overlap B2 flange
    BendFeature b1, b2;
    // ... setup geometry

    GeometricPrecedenceAnalyzer analyzer;
    auto constraints = analyzer.analyze({b1, b2}, fag);

    // Assert: B1 must bend before B2
    REQUIRE(constraints.size() == 1);
    REQUIRE(constraints[0].fromBend == b1.id);
    REQUIRE(constraints[0].toBend == b2.id);
    REQUIRE(constraints[0].type == GEOMETRIC);
}

TEST_CASE("Box closing detection") {
    // Setup: 3-sided box scenario
    // ... create bent state with 3 sides

    GeometricPrecedenceAnalyzer analyzer;
    bool isClosing = analyzer.isBoxClosing(bend4, bentState);

    REQUIRE(isClosing == true);
}
```

#### Grasp Constraint Tests
```cpp
TEST_CASE("Dead zone calculation") {
    // Setup: Part with 2 bent flanges
    BentState state;
    // ... add bent flanges

    GraspConstraintGenerator generator;
    auto zones = generator.computeDeadZones(state, remainingBends);

    // Assert: 2 FLANGE zones + safety margins
    REQUIRE(zones.size() >= 2);
}

TEST_CASE("MIR algorithm") {
    // Setup: L-shaped valid region
    Polygon2D validRegion;
    // ... create L-shape

    GraspConstraintGenerator generator;
    Rectangle2D mir = generator.findMaximumInscribedRectangle(validRegion);

    // Assert: Rectangle fits inside and has maximum area
    REQUIRE(mir.area > 0);
    REQUIRE(validRegion.contains(mir.center));
}
```

#### ABA Constraint Tests
```cpp
TEST_CASE("Subset sum solver") {
    std::vector<int> segments = {50, 100, 150, 200};
    double target = 275.0;

    ABAConstraintAnalyzer analyzer;
    auto solution = analyzer.solveSubsetSum(target, segments);

    // Assert: Solution exists and sums correctly
    REQUIRE(!solution.empty());
    int sum = std::accumulate(solution.begin(), solution.end(), 0);
    REQUIRE(sum >= target);
}

TEST_CASE("No solution case") {
    std::vector<int> segments = {100, 200, 300};
    double target = 75.0;  // Impossible with these segments

    ABAConstraintAnalyzer analyzer;
    auto solution = analyzer.solveSubsetSum(target, segments);

    REQUIRE(solution.empty());
}
```

#### DAG Tests
```cpp
TEST_CASE("Acyclic DAG") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, GEOMETRIC);
    dag.addEdge(1, 2, GEOMETRIC);
    dag.finalize();

    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("Cycle detection") {
    PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addEdge(0, 1, GEOMETRIC);
    dag.addEdge(1, 0, SEQUENTIAL);  // Creates cycle!
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);
    auto cycles = dag.detectCycles();
    REQUIRE(cycles.size() > 0);
}

TEST_CASE("Topological sort") {
    PrecedenceDAG dag;
    // ... build valid DAG
    dag.finalize();

    auto order = dag.topologicalSort();

    // Assert: All nodes present, precedence respected
    REQUIRE(order.size() == dag.nodeCount());
    // ... check ordering
}
```

### 6.2 Integration Tests

```cpp
TEST_CASE("Phase 1 + Phase 2 integration") {
    // Parse STEP file
    Phase1Output phase1 = parseSTEPFile("test_box.step");
    REQUIRE(phase1.success);

    // Run Phase 2
    ConstraintSolver solver;
    Phase2Output phase2 = solver.analyze(phase1);

    // Validate results
    REQUIRE(phase2.success);
    REQUIRE(phase2.dag.isAcyclic());
    REQUIRE(!phase2.graspConstraints.empty());
    REQUIRE(!phase2.abaConstraints.empty());
}
```

### 6.3 Test Parts

| Part | Bends | Expected Constraints | Test Focus |
|------|-------|---------------------|------------|
| **L-bracket** | 1 | None | Baseline (no constraints) |
| **U-channel** | 2 | None | Simple parallel bends |
| **Box (4-sided)** | 4 | Box closing | Detect impossible 4th bend |
| **Complex panel** | 8 | Multiple geometric | Corner overlaps, blocking |
| **Interference test** | 3 | Sequential | Bent flanges block access |

---

## 7. Performance Requirements

### 7.1 Time Complexity

| Algorithm | Complexity | Typical Runtime |
|-----------|-----------|----------------|
| Corner overlap (all pairs) | O(n²) | < 500ms for n=20 |
| MIR per state | O(v² log v) | < 100ms for v=50 vertices |
| Subset sum | O(n × W) | < 10ms for n=10, W=500 |
| Cycle detection | O(V + E) | < 50ms for V=20, E=40 |
| **Total Phase 2** | O(n²) | **< 2 seconds** |

Where:
- n = number of bends
- v = polygon vertices
- W = target width
- V = DAG vertices
- E = DAG edges

### 7.2 Memory Requirements

- Precedence DAG: ~1 KB per bend
- Grasp constraints: ~5 KB per state
- Dead zones: ~2 KB per zone
- **Total**: < 1 MB for typical parts (< 20 bends)

---

## 8. Error Handling

### 8.1 Error Cases

```cpp
enum Phase2Error {
    NO_ERROR = 0,

    // DAG errors
    CYCLIC_DEPENDENCIES = 100,
    EMPTY_DAG,
    INVALID_NODE_ID,

    // Grasp errors
    NO_VALID_GRIP = 200,
    GRIP_TOO_SMALL,
    COM_OUTSIDE_GRIP,
    TORQUE_EXCEEDED,

    // ABA errors
    NO_ABA_SOLUTION = 300,
    BOX_CLOSING_DETECTED,
    TOOL_TOO_WIDE,

    // General
    INVALID_INPUT = 400,
    COMPUTATION_FAILED
};
```

### 8.2 Error Messages

```cpp
std::string getErrorMessage(Phase2Error code) {
    switch (code) {
        case CYCLIC_DEPENDENCIES:
            return "Precedence DAG contains cycles - conflicting constraints";
        case NO_VALID_GRIP:
            return "No valid grip region found - consider repositioning";
        case NO_ABA_SOLUTION:
            return "Cannot configure ABA for required width - no segment combination";
        case BOX_CLOSING_DETECTED:
            return "Bend would close 3-sided box - tool would be trapped";
        default:
            return "Unknown error";
    }
}
```

---

## 9. Output Examples

### 9.1 Simple Box (4 bends)

```cpp
Phase2Output result;

// DAG: Linear precedence
result.dag.nodes = {0, 1, 2, 3};  // 4 bends
result.dag.edges = {
    {from: 0, to: 2, type: GEOMETRIC},     // Bottom before top
    {from: 1, to: 3, type: GEOMETRIC},     // Right before left
    {from: 0, to: 3, type: BOX_CLOSING},   // Repo needed before left
};

// Topological order (one valid sequence)
result.topologicalOrder = {0, 1, 2};  // 3rd bend requires repo

// Grasp constraints
result.graspConstraints[0] = {
    validArea: 5000.0,           // mm²
    maxInscribedRect: {100 x 50},
    hasValidGrip: true
};

result.graspConstraints[1] = {
    validArea: 3500.0,           // Shrinking
    maxInscribedRect: {80 x 40},
    hasValidGrip: true
};

result.graspConstraints[2] = {
    validArea: 800.0,            // Critical!
    maxInscribedRect: {40 x 20},
    hasValidGrip: false,         // Below threshold
    warnings: {"Grip too small - reposition required"}
};

// ABA constraints
result.abaConstraints[0] = {
    requiredWidth: 275.0,
    segmentSolution: {100, 175},
    feasible: true
};
```

---

## 10. Dependencies

### 10.1 Required Libraries

- **OpenCASCADE**: Geometry operations, ray casting
- **CGAL**: Polygon operations, MIR algorithm
- **Boost Graph**: DAG traversal, cycle detection
- **Eigen**: Matrix operations (optional, for optimization)

### 10.2 Phase 1 Integration

```cpp
// Phase 2 consumes Phase 1 output
Phase1Output phase1 = parseSTEPFile("part.step");

if (phase1.success) {
    ConstraintSolver solver;
    Phase2Output phase2 = solver.analyze(phase1);

    if (phase2.success && phase2.dag.isAcyclic()) {
        // Ready for Phase 3!
        LOG_INFO("Precedence DAG valid: {} constraints",
                 phase2.totalConstraints);
    } else {
        LOG_ERROR("Phase 2 failed: {}", phase2.errorMessage);
    }
}
```

---

## 11. Implementation Checklist

### Week 1-2: Design & Setup
- [ ] Review Phase 1 output structures
- [ ] Design Phase 2 data structures (this document)
- [ ] Create header files
- [ ] Setup Catch2 unit test framework

### Week 3: Geometric Precedence
- [ ] Implement `GeometricPrecedenceAnalyzer` class
- [ ] Ray casting algorithm
- [ ] Corner overlap detection
- [ ] Box closing detection
- [ ] Sequential blocking analysis
- [ ] Unit tests (10+ test cases)

### Week 4: Grasp Constraints
- [ ] Implement `GraspConstraintGenerator` class
- [ ] Dead zone calculation
- [ ] 2D polygon projection
- [ ] MIR algorithm (CGAL integration)
- [ ] Grip physics validation
- [ ] Unit tests (10+ test cases)

### Week 5: ABA & DAG
- [ ] Implement `ABAConstraintAnalyzer` class
- [ ] Subset sum DP solver
- [ ] Implement `PrecedenceDAGBuilder` class
- [ ] Cycle detection (DFS)
- [ ] Topological sort
- [ ] Unit tests (10+ test cases)

### Week 6: Integration & Testing
- [ ] Integrate all modules
- [ ] `ConstraintSolver` main class
- [ ] Phase 1 + Phase 2 integration tests
- [ ] Test with 5+ different parts
- [ ] Performance profiling
- [ ] Documentation (Doxygen comments)
- [ ] Bug fixes

---

## 12. Success Criteria

Phase 2 considered **COMPLETE** when:

✅ **Functionality**:
- [ ] Precedence DAG is acyclic for valid parts
- [ ] All 3 constraint types detected correctly
- [ ] Grip validation works (MIR + physics)
- [ ] ABA solver handles all feasible cases
- [ ] Box closing detected with 100% accuracy

✅ **Performance**:
- [ ] < 2 seconds for typical parts (10-20 bends)
- [ ] < 5 seconds for complex parts (30+ bends)
- [ ] Memory usage < 1 MB

✅ **Quality**:
- [ ] All unit tests pass (30+ tests)
- [ ] Integration tests pass (5+ parts)
- [ ] Code coverage > 80%
- [ ] Documentation complete (API + examples)
- [ ] Zero memory leaks (Valgrind clean)

✅ **Integration**:
- [ ] Phase 1 → Phase 2 pipeline works
- [ ] Phase 2 output ready for Phase 3 consumption
- [ ] Error handling robust (no crashes)

---

## 13. References

### Research Papers
- **P2S1**: "Automated Bend Sequencing using Geometric Precedence Analysis"
- **P2S2**: "Maximum Inscribed Rectangle for Optimal Grip Placement"
- **P2S3**: "ABA Tool Configuration via Subset Sum Optimization"

### CGAL Documentation
- Polygon operations: https://doc.cgal.org/latest/Boolean_set_operations_2/
- Polygon with holes: https://doc.cgal.org/latest/Polygon/

### Boost Graph
- Cycle detection: https://www.boost.org/doc/libs/release/libs/graph/
- Topological sort: https://www.boost.org/doc/libs/release/libs/graph/doc/topological_sort.html

---

**Next**: Begin Phase 2 implementation after Phase 1 compilation and testing complete.

**Estimated Completion**: 5-6 weeks from start.

**Status**: Design document ready ✅
