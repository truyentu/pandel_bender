# Phase 2: Constraint Solver - Next Steps After Phase 1

**Date**: 2026-02-05
**Status**: Ready to Begin After Phase 1 Compilation Test
**Estimated Duration**: 5-6 weeks

---

## 🎯 Objective

Analyze Face-Adjacency Graph (FAG) từ Phase 1 để xác định các ràng buộc vật lý và xây dựng **Precedence DAG** (Directed Acyclic Graph) cho bend sequencing.

---

## 📋 Phase 2 Overview

### What Phase 2 Does

Phase 2 nhận **Phase1Output** (FAG + classified bends) và tạo ra:

1. **Geometric Precedence Constraints** - Bends nào phải bend trước/sau dựa trên geometry
2. **Grasp Constraints** - Vùng nào có thể kẹp part mà không va chạm
3. **ABA Tool Constraints** - Tool configuration cần thiết cho mỗi bend
4. **Precedence DAG** - Graph chỉ định thứ tự hợp lệ

### Why It's Critical

- Ngăn ngừa **collision** và **impossible sequences**
- Đảm bảo part luôn có **valid grip point**
- Tối ưu hóa **tool configuration**
- Foundation cho Phase 3 sequencing

---

## 🔧 Phase 2 Modules

### Module 1: Geometric Precedence Analyzer

**Input**: FAG with bend features
**Output**: Precedence constraints (Bi before Bj)

**3 Types of Constraints**:

#### Type 1: Corner Overlap Detection
```cpp
// Ray casting from flange corners
if (corner_of(Bi) overlaps flange(Bj))
    → Bi must bend before Bj
```

**Algorithm**: Cast rays từ góc flange, detect intersection.

#### Type 2: Box Closing Constraint
```cpp
// Detect 3-sided enclosures
if (3_sides_bent && 4th_side_unbent)
    → 4th side IMPOSSIBLE (tool trapped)
    → Requires repositioning
```

**Algorithm**: Project bent flanges to 2D, check for U-shape.

#### Type 3: Sequential Dependencies
```cpp
// Bent flanges block access to other bends
if (Bj_blocked_by_bent_flange_of_Bi)
    → Bi must bend before Bj
```

**Algorithm**: Accessibility analysis from tool approach direction.

---

### Module 2: Grasp Constraint Generator

**Input**: Bent state after each bend
**Output**: Valid grip regions

**Key Algorithm: Dead Zone Calculation**

```cpp
Valid_Grip_Area = Base_Area - Union(all_dead_zones)

Dead zones include:
- Standing flanges (already bent)
- Safety margins around future bends (20mm)
- ABA tool interference zones
```

**Maximum Inscribed Rectangle (MIR)**:
- Find largest rectangle trong valid grip region
- Salvagnini grippers are rectangular
- Use sweep line + dynamic programming

**Grip Physics Validation**:
```cpp
✓ Minimum area >= 100 mm²
✓ Center of mass within grip bounds
✓ Torque stability check
✓ Shear resistance check
```

---

### Module 3: ABA Constraint Analyzer

**Input**: Bend length + clearance
**Output**: ABA segment combination

**Problem**: Subset Sum
```
Available segments: {50, 100, 150, 200, 250, 300, 400, 500} mm

Required width: 275mm
Solution: [100, 175] or [50, 100, 125]
Optimize: Minimize segment count (faster reconfiguration)
```

**Algorithm**: Dynamic programming subset sum solver

**Box Closing Detection**:
- Check if bend creates 3-sided enclosure
- 4th bend would trap tool inside
- Mark as UNBENDABLE or require repo

---

### Module 4: Precedence DAG Builder

**Input**: All 3 constraint types
**Output**: Unified DAG

```
Nodes = Bends
Edges = Precedence (Bi → Bj means Bi before Bj)

Validation:
✓ Must be ACYCLIC (no circular dependencies)
✓ Cycle detection → report conflicts
```

---

## 📊 Data Structures

### New Types for Phase 2

```cpp
// Precedence constraint
struct PrecedenceEdge {
    int fromBend;           // Bend ID that must come first
    int toBend;             // Bend ID that must come second
    ConstraintType type;    // GEOMETRIC | BOX_CLOSING | SEQUENTIAL
    double confidence;      // 0.0-1.0
    std::string reasoning;  // Debug info
};

// Dead zone polygon
struct DeadZone {
    std::vector<gp_Pnt2d> polygon;  // 2D polygon on base plane
    int causedByBend;                // Which bend created this zone
    DeadZoneType type;               // FLANGE | SAFETY | ABA
};

// Grasp constraint
struct GraspConstraint {
    int afterBend;                   // Valid after this bend
    std::vector<DeadZone> deadZones; // Forbidden regions
    Polygon2D validRegion;           // Remaining valid area
    Rectangle2D maxInscribedRect;    // MIR result
    bool hasValidGrip;               // Can we grip?
};

// ABA constraint
struct ABAConstraint {
    int bendId;
    double requiredWidth;            // Minimum tool width
    std::vector<int> segmentSolution; // e.g., [100, 150, 200]
    int segmentCount;                // Number of segments
    bool feasible;                   // Solution exists?
};

// Precedence DAG
struct PrecedenceDAG {
    std::vector<PrecedenceNode> nodes;  // One per bend
    std::vector<PrecedenceEdge> edges;  // Precedence relationships
    bool isAcyclic;                     // Validation result
    std::vector<int> topologicalOrder;  // Valid ordering (if acyclic)
};

// Phase 2 Output
struct Phase2Output {
    PrecedenceDAG dag;
    std::map<int, GraspConstraint> graspConstraints;  // Per bend
    std::map<int, ABAConstraint> abaConstraints;      // Per bend
    bool success;
    std::string errorMessage;
    std::vector<std::string> warnings;
};
```

---

## 🔬 Key Algorithms to Implement

### 1. Ray Casting for Corner Overlap

```cpp
bool CheckCornerOverlap(BendFeature bi, BendFeature bj, BentState state) {
    // Get 4 corner points của bi's flange
    auto corners = GetFlangeCorners(bi, state);

    // Cast ray từ mỗi corner theo predicted motion path
    for (auto corner : corners) {
        gp_Vec motionPath = PredictMotionPath(bi, state);

        // Check intersection với bj's flange
        if (RayIntersectsFlange(corner, motionPath, bj.flangeFace)) {
            return true;  // bi must bend before bj
        }
    }

    return false;
}
```

### 2. Maximum Inscribed Rectangle (MIR)

```cpp
Rectangle2D FindMIR(Polygon2D validRegion) {
    // Sweep line algorithm
    // Reference: "Computing the Largest Empty Rectangle" paper

    Rectangle2D maxRect;
    double maxArea = 0.0;

    // Sweep horizontally
    for (auto& line : SweepLines(validRegion)) {
        // Dynamic programming to find max rectangle at this sweep
        auto rect = FindMaxRectAtLine(line, validRegion);
        if (rect.area > maxArea) {
            maxRect = rect;
            maxArea = rect.area;
        }
    }

    return maxRect;
}
```

### 3. Subset Sum for ABA Segments

```cpp
std::vector<int> SolveABASegments(double requiredWidth,
                                   std::vector<int> availableSegs) {
    int n = availableSegs.size();
    int target = static_cast<int>(requiredWidth);

    // DP table: dp[i][w] = can we make width w using first i segments?
    std::vector<std::vector<bool>> dp(n+1, std::vector<bool>(target+1, false));
    dp[0][0] = true;

    // Fill DP table
    for (int i = 1; i <= n; i++) {
        int seg = availableSegs[i-1];
        for (int w = 0; w <= target; w++) {
            dp[i][w] = dp[i-1][w];  // Don't use segment i
            if (w >= seg && dp[i-1][w-seg]) {
                dp[i][w] = true;     // Use segment i
            }
        }
    }

    // Backtrack to find solution
    if (!dp[n][target]) return {};  // No solution

    std::vector<int> solution;
    int w = target;
    for (int i = n; i > 0; i--) {
        if (dp[i][w] && !dp[i-1][w]) {
            solution.push_back(availableSegs[i-1]);
            w -= availableSegs[i-1];
        }
    }

    return solution;
}
```

### 4. Box Closing Detection

```cpp
bool IsBoxClosing(BendFeature bend, BentState current) {
    // Project all bent flanges to 2D base plane
    std::vector<Polygon2D> bentFlanges2D;
    for (auto& bentBend : current.bentBends) {
        bentFlanges2D.push_back(Project2D(bentBend.flangeFace));
    }

    // Check if they form 3 sides of rectangle
    if (Forms3SidedBox(bentFlanges2D)) {
        // Check if current bend would close 4th side
        Polygon2D nextFlange2D = Project2D(bend.flangeFace);
        if (WouldClose4thSide(bentFlanges2D, nextFlange2D)) {
            return true;  // TRAP! Tool would be enclosed
        }
    }

    return false;
}
```

---

## 🧪 Implementation Plan

### Week 1-2: Design & Setup
- [ ] Review Phase 1 output structures
- [ ] Design Phase 2 data structures
- [ ] Create header files
- [ ] Setup unit test framework

### Week 3: Geometric Precedence
- [ ] Implement ray casting algorithm
- [ ] Implement corner overlap detection
- [ ] Implement sequential dependency analysis
- [ ] Unit tests for precedence detection

### Week 4: Grasp Constraints
- [ ] Implement dead zone calculation
- [ ] Implement MIR algorithm
- [ ] Implement grip physics validation
- [ ] Unit tests for grip constraints

### Week 5: ABA & DAG
- [ ] Implement subset sum solver
- [ ] Implement box closing detection
- [ ] Build precedence DAG
- [ ] Cycle detection algorithm
- [ ] Unit tests

### Week 6: Integration & Testing
- [ ] Integrate all modules
- [ ] End-to-end testing với Phase 1
- [ ] Performance optimization
- [ ] Documentation

---

## 📦 Dependencies

### Libraries Cần Thêm

```json
{
  "cgal": "For polygon operations, MIR algorithm",
  "boost-graph": "For DAG operations, cycle detection"
}
```

(Already in vcpkg.json ✅)

### Phase 1 Integration

```cpp
// Phase 2 consumes Phase 1 output
Phase1Output phase1 = parseSTEPFile("part.step");

// Create Phase 2 analyzer
ConstraintSolver solver;
solver.setFAG(phase1.fag);
solver.setBends(phase1.bends);
solver.setThickness(phase1.thickness);

// Run analysis
Phase2Output phase2 = solver.analyze();

if (phase2.success) {
    // Phase 2 ready for Phase 3!
}
```

---

## ✅ Acceptance Criteria

Phase 2 considered complete when:

- [ ] **Precedence DAG is acyclic** - No circular dependencies
- [ ] **All 3 constraint types detected** - Geometric, box closing, sequential
- [ ] **Grip validation works** - Valid regions computed correctly
- [ ] **MIR finds optimal rectangles** - Maximum area for grip
- [ ] **ABA solver handles all cases** - Including no-solution
- [ ] **Box closing detected 100%** - No false negatives
- [ ] **Performance < 2 seconds** - For typical parts (10-20 bends)
- [ ] **Integration tests pass** - With Phase 1 output
- [ ] **Documentation complete** - API docs, examples

---

## 🎯 Expected Output Example

```cpp
Phase2Output result;

// Precedence DAG
result.dag.nodes = {
    {id: 0, bendId: 0},  // Bend 0 (bottom)
    {id: 1, bendId: 1},  // Bend 1 (right)
    {id: 2, bendId: 2},  // Bend 2 (top)
    {id: 3, bendId: 3},  // Bend 3 (left)
};

result.dag.edges = {
    {from: 0, to: 2, type: GEOMETRIC},      // Bottom before top
    {from: 1, to: 3, type: GEOMETRIC},      // Right before left
    {from: 0, to: 3, type: BOX_CLOSING},    // Need repo before left
};

// Grasp constraints
result.graspConstraints[0] = {
    afterBend: 0,
    validRegion: {area: 5000.0},  // mm²
    maxInscribedRect: {width: 50, height: 100, area: 5000},
    hasValidGrip: true
};

// ABA constraints
result.abaConstraints[0] = {
    bendId: 0,
    requiredWidth: 275.0,
    segmentSolution: {100, 175},
    segmentCount: 2,
    feasible: true
};
```

---

## 📚 References

- **ROADMAP.md** - Phase 2 detailed specs (lines 173-300)
- **Research Papers**:
  - P2S1: Ray casting for geometric constraints
  - P2S2: MIR algorithm for grip optimization
  - P2S3: Subset sum for ABA, box closing detection

---

## 🚀 Ready to Start?

**Prerequisites**:
1. ✅ Phase 1 compiled and tested
2. ✅ Dependencies installed (CGAL, Boost)
3. ✅ Sample STEP files with known constraints

**First Steps**:
1. Create `include/openpanelcam/phase2/` folder
2. Design data structures (types.h)
3. Implement precedence analyzer module
4. Write unit tests as you go

**Estimated Completion**: 5-6 weeks from start

---

**Phase 2 sẽ transform FAG thành actionable constraints cho sequencing!**
