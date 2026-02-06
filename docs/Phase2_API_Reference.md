# Phase 2 API Reference

**Version:** 1.0
**Last Updated:** 2026-02-06
**Namespace:** `openpanelcam::phase2`

---

## Table of Contents

1. [ConstraintSolver](#constraintsolver)
2. [Phase2Output](#phase2output)
3. [PrecedenceDAG](#precedencedag)
4. [GeometricPrecedenceAnalyzer](#geometricprecedenceanalyzer)
5. [GraspConstraintGenerator](#graspconstraintgenerator)
6. [ABAConstraintAnalyzer](#abaconstraintanalyzer)
7. [Data Structures](#data-structures)
8. [Enumerations](#enumerations)
9. [Usage Examples](#usage-examples)

---

## ConstraintSolver

Main entry point for Phase 2 constraint analysis. Orchestrates all analysis modules to generate optimal bend sequences.

### Header

```cpp
#include "openpanelcam/phase2/constraint_solver.h"
```

### Constructor

```cpp
ConstraintSolver();
```

Creates a new constraint solver with default configuration.

### Methods

#### solve

```cpp
Phase2Output solve(const std::vector<phase1::BendFeature>& bends);
```

Analyzes all bends and generates optimal sequence.

**Parameters:**
- `bends`: Vector of bend features from Phase 1

**Returns:** `Phase2Output` with complete solution

**Workflow:**
1. Geometric precedence analysis (corner overlap, box closing, blocking)
2. Build precedence DAG from constraints
3. Grasp constraint analysis (dead zones, valid grip regions)
4. ABA tool feasibility analysis (segment selection)
5. Cycle detection and resolution
6. Topological sort for optimal sequence
7. Generate analysis summary

#### getStatistics

```cpp
const Statistics& getStatistics() const;
```

Returns timing and count statistics from last solve.

**Statistics Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `totalSolveTimeMs` | `double` | Total solve time in milliseconds |
| `geometricAnalysisTimeMs` | `double` | Time for geometric analysis |
| `graspAnalysisTimeMs` | `double` | Time for grasp analysis |
| `abaAnalysisTimeMs` | `double` | Time for ABA analysis |
| `graphBuildTimeMs` | `double` | Time to build DAG |
| `topologicalSortTimeMs` | `double` | Time for topological sort |
| `totalBends` | `int` | Number of bends analyzed |
| `geometricConstraints` | `int` | Geometric constraints found |
| `graspStatesAnalyzed` | `int` | Number of grasp states analyzed |
| `abaConstraintsGenerated` | `int` | ABA constraints generated |
| `totalEdges` | `int` | Total edges in DAG |
| `hasInfeasibleBends` | `bool` | Whether infeasible bends exist |
| `hasBoxClosing` | `bool` | Whether box closing detected |
| `hasGraspIssues` | `bool` | Whether grasp issues detected |

#### reset

```cpp
void reset();
```

Clears internal state for reuse. Call before solving a new part.

---

## Phase2Output

Complete output from constraint analysis.

### Header

```cpp
#include "openpanelcam/phase2/constraint_solver.h"
```

### Fields

| Field | Type | Description |
|-------|------|-------------|
| `precedenceGraph` | `PrecedenceDAG` | Complete constraint graph |
| `graspConstraints` | `vector<GraspConstraint>` | Grip analysis for each state |
| `abaConstraints` | `vector<ABAConstraint>` | Tool analysis for each bend |
| `bendSequence` | `vector<int>` | Optimal bend order (bend IDs) |
| `success` | `bool` | Whether valid solution found |
| `analysisSummary` | `string` | Human-readable summary |
| `errors` | `vector<string>` | Critical error messages |
| `warnings` | `vector<string>` | Non-critical warnings |

### Notes

- `Phase2Output` uses move semantics (non-copyable due to `PrecedenceDAG`)
- `bendSequence` is empty if `success` is false
- Check `errors` vector for failure reasons

---

## PrecedenceDAG

Directed acyclic graph of bend dependencies with cycle detection and topological sorting.

### Header

```cpp
#include "openpanelcam/phase2/precedence_dag.h"
```

### Constructor

```cpp
PrecedenceDAG();
```

Creates an empty DAG.

### Methods

| Method | Signature | Description |
|--------|-----------|-------------|
| `addNode` | `int addNode(int bendId)` | Add bend node, returns node ID |
| `addEdge` | `int addEdge(int from, int to, ConstraintType type, double conf, const string& reason)` | Add constraint edge |
| `getNode` | `const PrecedenceNode* getNode(int nodeId) const` | Get node by ID (nullptr if not found) |
| `getEdge` | `const PrecedenceEdge* getEdge(int edgeId) const` | Get edge by ID (nullptr if not found) |
| `nodeCount` | `int nodeCount() const` | Number of nodes |
| `edgeCount` | `int edgeCount() const` | Number of edges |
| `isFinalized` | `bool isFinalized() const` | Check if finalized |
| `clear` | `void clear()` | Remove all nodes and edges |
| `finalize` | `bool finalize()` | Finalize graph (calculate levels) |
| `isAcyclic` | `bool isAcyclic()` | Check for cycles (DFS 3-color) |
| `resolveCycles` | `vector<PrecedenceEdge> resolveCycles()` | Remove weak edges to break cycles |
| `topologicalSort` | `vector<int> topologicalSort()` | Get valid sequence (Kahn's algorithm) |

### Algorithms

- **Level Calculation:** BFS propagation, O(V+E)
- **Cycle Detection:** DFS 3-color algorithm, O(V+E)
- **Topological Sort:** Kahn's algorithm, O(V+E)
- **Cycle Resolution:** Iterative lowest-confidence edge removal

---

## GeometricPrecedenceAnalyzer

Analyzes geometric conflicts between bends to determine precedence constraints.

### Header

```cpp
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"
```

### Methods

#### analyze

```cpp
std::vector<PrecedenceEdge> analyze(const std::vector<phase1::BendFeature>& bends);
```

Analyzes all bend pairs for geometric conflicts.

**Returns:** Vector of precedence edges (constraints)

**Detection Types:**
- **GEOMETRIC:** Corner overlap during bending motion
- **BOX_CLOSING:** Tool would be trapped (3-sided box)
- **SEQUENTIAL:** Access obstructed by bent flange

#### checkCornerOverlap

```cpp
bool checkCornerOverlap(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
);
```

Checks if bi's flange corners would overlap bj during bending.

#### isBoxClosing

```cpp
bool isBoxClosing(
    const phase1::BendFeature& bend,
    const BentState& state
);
```

Checks if bending would create a box-closing scenario (tool trap).

#### isBlocked

```cpp
bool isBlocked(
    const phase1::BendFeature& bi,
    const phase1::BendFeature& bj,
    const BentState& state
);
```

Checks if bj blocks access to bi.

#### getStatistics

```cpp
const Statistics& getStatistics() const;
```

**Statistics Fields:**
- `totalPairsChecked`: Number of bend pairs analyzed
- `cornerOverlapCount`: Corner overlaps detected
- `boxClosingCount`: Box closing scenarios detected
- `sequentialBlockCount`: Sequential blocking detected
- `analysisTimeMs`: Total analysis time
- `avgPairTimeMs`: Average time per pair

---

## GraspConstraintGenerator

Analyzes bent part states to determine valid grip regions and dead zones.

### Header

```cpp
#include "openpanelcam/phase2/grasp_constraint_generator.h"
```

### Methods

#### analyze

```cpp
GraspConstraint analyze(
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<int>& bentBends
);
```

Generates grasp constraint for a specific bent state.

**Parameters:**
- `bends`: All bend features
- `bentBends`: List of bend IDs already bent

**Returns:** `GraspConstraint` with dead zones, valid region, and MIR

#### getStatistics

```cpp
const Statistics& getStatistics() const;
```

**Statistics Fields:**
- `totalStatesAnalyzed`: Number of states analyzed
- `totalDeadZonesGenerated`: Dead zones created
- `validGripCount`: States with valid grip
- `invalidGripCount`: States without valid grip
- `analysisTimeMs`: Total analysis time

### Key Algorithms

- **Dead Zone Calculation:** Projects standing flanges to base plane
- **Valid Region:** Sheet bounds minus dead zones (polygon subtraction)
- **Maximum Inscribed Rectangle (MIR):** Finds largest grip rectangle
- **Grip Physics:** Validates COM, friction, stability

---

## ABAConstraintAnalyzer

Analyzes whether each bend can be performed using ABA (Adaptive Bending Arm) tooling.

### Header

```cpp
#include "openpanelcam/phase2/aba_constraint_analyzer.h"
```

### Methods

#### analyze

```cpp
std::vector<ABAConstraint> analyze(const std::vector<phase1::BendFeature>& bends);
```

Analyzes all bends for ABA tool feasibility.

**Returns:** Vector of ABA constraints (one per bend)

#### getStatistics

```cpp
const Statistics& getStatistics() const;
```

**Statistics Fields:**
- `totalBendsAnalyzed`: Number of bends analyzed
- `feasibleCount`: Bends with valid ABA solution
- `infeasibleCount`: Bends without solution
- `boxClosingCount`: Bends with box closing
- `analysisTimeMs`: Total analysis time

### ABA Segment Sizes

Standard Salvagnini configuration: `{25, 50, 75, 100, 125, 150, 175, 200}` mm

### Key Algorithms

- **Subset Sum DP:** Finds optimal segment combination
- **Greedy Fallback:** For targets > 1000mm
- **Box Closing Detection:** Identifies tool trap scenarios

---

## Data Structures

### GraspConstraint

Grip feasibility for a bent state.

```cpp
struct GraspConstraint {
    int stateId;                      // State identifier
    std::vector<int> bentBends;       // Bends already completed
    std::vector<DeadZone> deadZones;  // Forbidden grip regions
    Polygon2D validRegion;            // Available grip area
    double validArea;                 // Area of valid region (mm^2)
    Rectangle2D maxInscribedRect;     // Largest grip rectangle (MIR)
    Point2D optimalGripCenter;        // Optimal grip location
    bool hasValidGrip;                // Whether gripping is possible
    double minRequiredArea;           // Minimum area (default 100mm^2)
    Point2D centerOfMass;             // Part COM projection
    std::vector<std::string> warnings;
};
```

### ABAConstraint

ABA tool feasibility for a bend.

```cpp
struct ABAConstraint {
    int bendId;                       // Bend identifier
    double bendLength;                // Bend line length (mm)
    double requiredWidth;             // Minimum tool width (mm)
    double clearance;                 // Safety clearance (mm)
    std::vector<int> segmentSolution; // Selected segments
    int totalSegments;                // Number of segments used
    double totalWidth;                // Total width of segments
    bool feasible;                    // Whether solution exists
    bool isBoxClosing;                // Would trap tool
    std::string reason;               // Explanation
    std::vector<int> suggestedAlternatives;
};
```

### DeadZone

Forbidden grip region on base plane.

```cpp
struct DeadZone {
    int id;
    DeadZoneType type;
    Polygon2D polygon;          // 2D footprint
    int causedByBend;           // Which bend created this
    double safetyMargin;        // Safety buffer (mm)

    double area() const;
    bool contains(const Point2D& point) const;
};
```

### PrecedenceNode

Node in precedence DAG.

```cpp
struct PrecedenceNode {
    int id;                     // Node ID in DAG
    int bendId;                 // Bend ID from Phase 1
    std::vector<int> predecessors;
    std::vector<int> successors;
    int level;                  // Topological level
    bool visited;               // For traversal
};
```

### PrecedenceEdge

Edge in precedence DAG.

```cpp
struct PrecedenceEdge {
    int id;
    int fromBend;               // Must bend first
    int toBend;                 // Must bend second
    ConstraintType type;
    double confidence;          // 0.0 - 1.0
    std::string reasoning;      // Human-readable
    Point3D conflictPoint;      // Location of conflict
};
```

### Polygon2D

2D polygon with geometric operations.

```cpp
struct Polygon2D {
    std::vector<Point2D> vertices;
    std::vector<std::vector<Point2D>> holes;

    double area() const;                    // Shoelace formula
    bool contains(const Point2D& p) const;  // Ray casting
    Point2D centroid() const;               // Centroid calculation
};
```

### Rectangle2D

Axis-aligned bounding box.

```cpp
struct Rectangle2D {
    Point2D bottomLeft;
    Point2D topRight;
    double width;
    double height;
    double area;
    Point2D center;

    bool contains(const Point2D& point) const;
};
```

### Point2D / Point3D

Basic geometric points.

```cpp
struct Point2D {
    double x, y;
    Point2D(double x_ = 0, double y_ = 0);
};

struct Point3D {
    double x, y, z;
    Point3D(double x_ = 0, double y_ = 0, double z_ = 0);
};
```

### BentState

Bend state simulator (helper class).

```cpp
class BentState {
public:
    void applyBend(int bendId);           // Mark bend as bent
    bool isBent(int bendId) const;        // Check if bent
    const std::vector<int>& getBentBends() const;
    int count() const;
    void reset();                         // Reset to flat
};
```

---

## Enumerations

### ConstraintType

```cpp
enum class ConstraintType {
    GEOMETRIC,    // Corner overlap constraint
    BOX_CLOSING,  // Tool trap constraint
    SEQUENTIAL    // Access blocking constraint
};
```

### DeadZoneType

```cpp
enum class DeadZoneType {
    STANDING_FLANGE,  // Standing flange from bent bend
    SAFETY_MARGIN,    // Safety buffer zone
    ABA_INTERFERENCE  // ABA tool interference zone
};
```

---

## Usage Examples

### Example 1: Basic Usage

```cpp
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;

int main() {
    // Prepare bends from Phase 1
    std::vector<phase1::BendFeature> bends;
    // ... populate bends ...

    // Create solver and analyze
    ConstraintSolver solver;
    Phase2Output output = solver.solve(bends);

    if (output.success) {
        std::cout << "Optimal bend sequence: ";
        for (int id : output.bendSequence) {
            std::cout << id << " ";
        }
        std::cout << std::endl;
    } else {
        for (const auto& err : output.errors) {
            std::cerr << "Error: " << err << std::endl;
        }
    }

    return 0;
}
```

### Example 2: Accessing Statistics

```cpp
ConstraintSolver solver;
Phase2Output output = solver.solve(bends);

const auto& stats = solver.getStatistics();
std::cout << "Analysis time: " << stats.totalSolveTimeMs << " ms" << std::endl;
std::cout << "Geometric constraints: " << stats.geometricConstraints << std::endl;
std::cout << "Grasp states analyzed: " << stats.graspStatesAnalyzed << std::endl;
```

### Example 3: Checking Grasp Constraints

```cpp
Phase2Output output = solver.solve(bends);

for (const auto& grasp : output.graspConstraints) {
    std::cout << "State " << grasp.stateId << ": ";
    if (grasp.hasValidGrip) {
        std::cout << "Valid grip, MIR area = "
                  << grasp.maxInscribedRect.area << " mm^2" << std::endl;
    } else {
        std::cout << "NO VALID GRIP" << std::endl;
        for (const auto& warning : grasp.warnings) {
            std::cout << "  Warning: " << warning << std::endl;
        }
    }
}
```

### Example 4: Checking ABA Constraints

```cpp
Phase2Output output = solver.solve(bends);

for (const auto& aba : output.abaConstraints) {
    std::cout << "Bend " << aba.bendId << ": ";
    if (aba.feasible) {
        std::cout << "Feasible with " << aba.totalSegments << " segments";
        std::cout << " (total width: " << aba.totalWidth << " mm)" << std::endl;
    } else {
        std::cout << "INFEASIBLE - " << aba.reason << std::endl;
    }
}
```

### Example 5: Building Custom DAG

```cpp
PrecedenceDAG dag;

// Add nodes
dag.addNode(0);  // Bend 0
dag.addNode(1);  // Bend 1
dag.addNode(2);  // Bend 2

// Add constraints
dag.addEdge(0, 1, ConstraintType::GEOMETRIC, 1.0, "Corner overlap");
dag.addEdge(1, 2, ConstraintType::BOX_CLOSING, 0.95, "Box closing risk");

// Finalize and check
dag.finalize();

if (dag.isAcyclic()) {
    std::vector<int> order = dag.topologicalSort();
    // order = [0, 1, 2]
} else {
    auto removed = dag.resolveCycles();
    // Handle removed edges
}
```

---

## Performance Characteristics

### Complexity Analysis

| Operation | Complexity | Notes |
|-----------|------------|-------|
| `ConstraintSolver::solve()` | O(n^2) | Dominated by geometric analysis |
| `GeometricPrecedenceAnalyzer::analyze()` | O(n^2) | Pairwise bend comparison |
| `GraspConstraintGenerator::analyze()` | O(n) per state | Linear in dead zones |
| `ABAConstraintAnalyzer::analyze()` | O(n * W) | W = max tool width |
| `PrecedenceDAG::finalize()` | O(V+E) | BFS level calculation |
| `PrecedenceDAG::isAcyclic()` | O(V+E) | DFS 3-color |
| `PrecedenceDAG::topologicalSort()` | O(V+E) | Kahn's algorithm |

### Typical Performance (Debug Mode)

| Bends | Time |
|-------|------|
| 2 | < 1 ms |
| 5 | < 5 ms |
| 10 | < 1000 ms |
| 20 | < 5000 ms |

Release mode is typically 10-100x faster.

### Memory Usage

- Minimal, scales linearly with bend count
- Typical: < 10 KB for 20 bends

---

## Error Handling

### Common Errors

1. **Cyclic dependencies:** Graph has cycles (resolved automatically)
2. **No valid grip:** Dead zones cover entire sheet
3. **Infeasible bend:** No ABA segment combination exists
4. **Box closing:** Tool would be trapped

### Best Practices

- Always check `output.success` before using `output.bendSequence`
- Review `output.errors` for critical issues
- Review `output.warnings` for potential problems
- Use `output.analysisSummary` for human-readable report

---

## See Also

- [ConstraintSolver_Usage.md](ConstraintSolver_Usage.md) - Detailed usage guide
- [GeometricPrecedenceAnalyzer_Usage.md](GeometricPrecedenceAnalyzer_Usage.md) - Geometric analysis guide
- [PHASE2_PROGRESS.md](PHASE2_PROGRESS.md) - Implementation progress

---

**Document Version:** 1.0
**Author:** Claude Code (Anthropic)
**Project:** OpenPanelCAM - Salvagnini Controller
