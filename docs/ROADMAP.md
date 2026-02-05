# OpenPanelCAM Development Roadmap

> **Version**: 2.0
> **Created**: 2026-02-04
> **Updated**: 2026-02-04 - Added detailed algorithms from research papers

---

## Overview

Roadmap phát triển OpenPanelCAM theo phương pháp Phase-by-Phase, đảm bảo mỗi phase hoàn thành và test trước khi chuyển sang phase tiếp theo.

### Version 2.0 Changes (2026-02-04)

Sau khi phân tích toàn bộ 23 research papers, các cập nhật quan trọng:

**Phase 1 Enhancements:**
- Edge orientation consistency tracking
- Bend axis extraction for each edge
- SDF-based material normal validation
- Internal vs boundary edge classification

**Phase 2 Enhancements:**
- 3 types of precedence constraints (geometric, box-closing, sequential)
- Maximum Inscribed Rectangle (MIR) algorithm for grip optimization
- Subset Sum solver for ABA segment selection
- Dead zone projection with safety margins

**Phase 3 Enhancements:**
- Zobrist hashing for state deduplication (critical for performance)
- Grip state tracking in search nodes
- 3 research-backed heuristics (rotational entropy, tooling variance, grasp fragmentation)
- Detailed masked time cost function

**Phase 3.5 NEW:**
- Dedicated Repositioning Planner module
- 4 repo trigger conditions
- Optimal repo point finder with branch-and-bound
- Grip transition planning

**Phase 4 Enhancements:**
- Hierarchical collision detection (AABB → OBB → Exact)
- Torque and shear validation for grip stability
- Tool extraction path planning with trap detection
- Springback compensation formulas

**Phase 5 Enhancements:**
- Complete PB-XML schema with adaptive control parameters
- Skeletal animation structure for HMI
- Bone weight assignment algorithm
- Keyframe interpolation methods

**Timeline Adjustments:**
- Performance targets updated (more realistic)
- Phase 3.5 added (~3 weeks)
- Total timeline: ~11 months (was ~8 months)

---

## Phase 1: Geometric Parser ("The Eyes")

### Objective
Parse STEP file và xây dựng Face-Adjacency Graph (FAG) với classification chính xác cho từng bend.

### Deliverables
1. **STEP Reader Module**
   - Load STEP file (AP203, AP214, AP242)
   - Geometry healing (ShapeFix)
   - Face unification (UnifySameDomain)
   - Unit conversion (inch → mm if needed)
   - Metadata extraction (part name, material)

2. **Face-Adjacency Graph Builder**
   - Extract all faces from solid using `TopExp::MapShapesAndAncestors()`
   - Build adjacency relationships
   - Identify shared edges
   - **NEW: Edge orientation consistency tracking**
   - **NEW: Classify edges as BEND_EDGE vs SHARP_EDGE**
   - **NEW: Distinguish internal edges vs boundary edges**
   - **NEW: Extract bend axis (gp_Ax1) for each bend edge**

3. **Bend Classifier**
   - Identify cylindrical faces (bends) using surface type
   - Classify planar faces (base/flanges)
   - Compute bend angles using dihedral angle
   - **NEW: Extract bend radius from cylindrical surface**
   - **NEW: Classify bend types (ACUTE < 90°, RIGHT = 90°, OBTUSE > 90°)**
   - **NEW: Detect return flanges (hems) when angle < 45°**

4. **Base Face Identifier**
   - Weighted scoring algorithm with 4 factors:
     - Area score (largest face preferred)
     - Connectivity score (most neighbors)
     - Centrality score (geometric center position)
     - Orientation score (horizontal preferred)
   - Handle ambiguous cases with user override option
   - **NEW: Validate base face has minimum area threshold**

5. **Bend Direction Calculator**
   - GetMaterialOutwardNormal() with orientation correction
   - **NEW: Signed Distance Function (SDF) validation**
   - **NEW: Cross product method for bend direction**
   - UP/DOWN/HEM classification
   - **NEW: Material-aware normal verification (point OUTWARD from solid)**

### Data Structures
```cpp
// Core types defined in docs/data-structures/phase1-types.md
- FAG_Node
- FAG_Edge
- BendFeature
- FaceAdjacencyGraph
- Phase1Output
```

### Dependencies
- OpenCASCADE 7.6+
- Eigen 3.4+

### Test Cases
- [ ] Simple box (4 bends)
- [ ] L-bracket (1 bend)
- [ ] Complex panel (8+ bends)
- [ ] Part with holes/cutouts
- [ ] Part with return flanges (hems)
- [ ] Degenerate geometry handling
- [ ] **NEW: TopAbs_REVERSED face orientation**
- [ ] **NEW: Zero-radius sharp edges**
- [ ] **NEW: Coplanar adjacent faces**
- [ ] **NEW: Material normal validation (SDF test)**

### Key Algorithms (from Research Papers)

#### 1. Material Outward Normal (S3)
```cpp
gp_Dir GetMaterialOutwardNormal(const TopoDS_Face& face) {
    // Get surface parametric bounds
    // Compute normal at UV midpoint
    // CRITICAL: Check face.Orientation() == TopAbs_REVERSED
    // Validate using SDF (normal must point outward from solid)
}
```

#### 2. Bend Axis Extraction (S1)
```cpp
gp_Ax1 ExtractBendAxis(const TopoDS_Edge& bendEdge) {
    // Get cylindrical surface axis
    // Ensure axis direction consistency across FAG
    // Store in FAG_Edge for Phase 2 usage
}
```

#### 3. Edge Classification (S2)
```cpp
EdgeType ClassifyEdge(TopoDS_Edge e, TopoDS_Face f1, TopoDS_Face f2) {
    // If both faces planar + edge curve is line -> SHARP_EDGE
    // If one face cylindrical -> BEND_EDGE
    // Extract bend parameters (angle, radius, axis)
}
```

### Acceptance Criteria
- [ ] All test STEP files parse without crash
- [ ] FAG correctly represents topology
- [ ] Bend directions match visual inspection
- [ ] Base face identification >= 95% accuracy
- [ ] **UPDATED: Performance < 3 seconds for typical parts (was 1s)**
- [ ] **NEW: All bend axes have consistent orientation**
- [ ] **NEW: Material normals validated via SDF**

---

## Phase 2: Constraint Solver ("The Rules")

### Objective
Analyze FAG để xác định các ràng buộc vật lý và xây dựng Precedence DAG.

### Deliverables
1. **Geometric Precedence Analyzer**
   - **Type 1: Corner Overlap Detection (Ray Casting)**
     - Cast rays from flange corners to detect geometric interference
     - Generate precedence: if corner_of(Bi) overlaps flange(Bj) → Bi must before Bj
   - **Type 2: Box Closing Constraint**
     - Detect when 3 sides already bent → 4th side impossible (tool trapped)
     - Flag unbendable configurations or suggest repo
   - **Type 3: Sequential Dependencies**
     - Bent flanges create barriers blocking access to other bends
     - Build dependency chain based on geometric accessibility
   - Generate precedence edges for DAG

2. **Grasp Constraint Generator**
   - **Dead Zone Calculation (P2S2)**
     - Project all bent flanges onto base plane
     - Mark regions occupied by standing flanges
     - Mark safety margins around future bends (e.g., 20mm buffer)
     - Mark ABA tool interference zones
     - Compute: Valid_Grip = Base_Area - Union(all_dead_zones)
   - **Maximum Inscribed Rectangle (MIR) Algorithm**
     - Find largest rectangle fitting in valid grip region
     - Use sweep line + dynamic programming
     - Salvagnini grippers are rectangular → MIR is optimal
   - **Grip Physics Validation**
     - Minimum area: >= 100 mm² (configurable)
     - Center of mass within grip bounds
     - Torque stability check
     - Shear resistance check
   - Valid grip regions per bend state

3. **ABA Constraint Analyzer**
   - Minimum tool width per bend (based on bend length + clearance)
   - **Subset Sum Problem Solver (P2S3)**
     - ABA segments: discrete values {50, 100, 150, 200, 250, 300, 400, 500} mm
     - For required width W, find segment combination summing to W
     - Optimize: minimize number of segments (faster reconfiguration)
     - Example: W=275mm → [100, 175] or [50, 100, 125]
   - **Box Closing Detection**
     - Check if current bend creates 3-sided enclosure
     - Remaining 4th bend would trap tool inside
     - Mark as UNBENDABLE or require repo before closure
   - Generate ABA constraints per bend

4. **Precedence DAG Builder**
   - Merge all constraint types into unified DAG
   - Nodes = Bends, Edges = Precedence relationships
   - Topological validation (must be acyclic)
   - Cycle detection → report conflicting constraints
   - **CRITICAL: DAG must encode ALL three constraint types**

### Data Structures
```cpp
- PrecedenceNode (enhanced with constraint type tracking)
- PrecedenceEdge (with ConstraintType: GEOMETRIC | BOX_CLOSING | SEQUENTIAL)
- GraspConstraint (with dead zones polygon list)
- ABAConstraint (with segment combination solution)
- DeadZone (2D polygon on base plane)
- Phase2Output
```

### Key Algorithms (from Research Papers)

#### 1. Ray Casting for Corner Overlap (P2s1)
```cpp
bool CheckCornerOverlap(Bend bi, Bend bj, BentState state) {
    // Get corner points of bi's flange
    // Cast rays from corners along potential motion path
    // If ray intersects bj's flange → precedence: bi before bj
}
```

#### 2. Maximum Inscribed Rectangle (P2S2)
```cpp
Rectangle FindMIR(Polygon2D validRegion) {
    // Sweep line algorithm
    // Dynamic programming for maximum area
    // Returns largest rectangle for gripper placement
}
```

#### 3. Subset Sum for ABA (P2S3)
```cpp
std::vector<int> SolveABASegments(double requiredWidth,
                                   std::vector<int> availableSegs) {
    // Dynamic programming subset sum
    // Minimize number of segments used
    // Return segment combination or empty if impossible
}
```

#### 4. Box Closing Detection (P2S3)
```cpp
bool IsBoxClosing(Bend b, BentState current) {
    // Project all bent flanges to 2D
    // Check if adding b creates 3-sided enclosure
    // Return true if 4th side would be trapped
}
```

### Dependencies
- Phase 1 Output (FAG with bend axes)
- CGAL (for geometric algorithms, polygon operations)
- **NEW: 2D geometry library for MIR**

### Test Cases
- [ ] Box closing scenario (4-sided box)
- [ ] Corner interference detection (overlapping flanges)
- [ ] Dead zone validation (grip region shrinking)
- [ ] ABA segment selection (various widths)
- [ ] **NEW: 3-sided enclosure detection**
- [ ] **NEW: Multiple valid grip regions**
- [ ] **NEW: Subset sum with no solution**
- [ ] **NEW: Conflicting precedence constraints (cycle detection)**

### Acceptance Criteria
- [ ] Precedence DAG is acyclic
- [ ] All physical constraints captured
- [ ] No false positives in collision detection
- [ ] **NEW: Box closing detected with 100% accuracy**
- [ ] **NEW: MIR finds optimal grip rectangle**
- [ ] **NEW: ABA solver succeeds for all feasible widths**

---

## Phase 3: Sequencer ("The Brain")

### Objective
Tìm optimal bend sequence sử dụng A* search với Masked Time cost function.

### Deliverables
1. **State Space Definition**
   - **State encoding:**
     - `uint32_t bentMask` - Bitmask of completed bends (32 bends max)
     - `uint8_t orientation` - Part rotation: 0°, 90°, 180°, 270°
     - `uint16_t abaConfig` - Current ABA segment configuration
     - `gp_Pnt gripCenter` - Current grip point on base
     - `bool needsRepo` - Repositioning required flag
   - **Zobrist Hashing (P3S1)**
     - Precompute random values for state components
     - Hash = bentMask_hash XOR orientation_hash XOR aba_hash
     - Enables O(1) duplicate state detection
     - Critical for search performance

2. **A* Search Implementation**
   - Priority queue: `f(n) = g(n) + h(n)` ordering
   - **Successor generation:**
     - For each unbent bend respecting precedence DAG
     - Generate new state with bend added
     - Check grasp validity in new state
     - Compute rotation/ABA changes needed
   - **Goal testing:** All bends completed (bentMask == all_ones)
   - **State pruning:** Discard states with invalid grips

3. **Cost Function (Masked Time) - CORE ALGORITHM (P3S2)**
   ```cpp
   double g(current, next_bend) = g(current) + StepCost(current, next_bend)

   StepCost = t_bend + max(t_rotation, t_aba) + t_repo

   where:
     t_bend = 2.0 - 3.5 seconds (depends on angle, thickness)
     t_rotation = 1.5s (if orientation change needed, else 0)
     t_aba = 0.8s (if ABA reconfiguration needed, else 0)
     t_repo = 5.0s (if repositioning needed, else 0)

   CRITICAL: t_rotation and t_aba happen in PARALLEL
            → Use max(), not sum()
            → This is "Masked Time" optimization
   ```

4. **Heuristic Functions (P3S2) - Research-backed weights**

   **H1: Rotational Entropy**
   - Measures orientation diversity of remaining bends
   - Formula: `h1 = k1 * CountDistinctOrientations(remaining)`
   - Rationale: Prefer finishing same-orientation bends early
   - Weight: k1 = 0.8 seconds per distinct orientation

   **H2: Tooling Variance**
   - Measures ABA configuration changes needed
   - Formula: `h2 = k2 * StandardDeviation(remaining_aba_widths)`
   - Rationale: Prefer similar-width bends together
   - Weight: k2 = 0.3 seconds per mm stddev

   **H3: Grasp Fragmentation**
   - Measures grip region splitting
   - Formula: `h3 = k3 * (NumberOfDisconnectedRegions(validGrip) - 1)`
   - Rationale: Penalize sequences that break grip into islands
   - Weight: k3 = 2.0 seconds per extra region

   **Combined Heuristic:**
   ```cpp
   h(state) = h1 + h2 + h3  // Admissible if weights tuned correctly
   ```

5. **Repo Injection Logic (P3S3)**
   - **Trigger Detection:**
     - Valid grip area < MIN_AREA threshold
     - No feasible grip point for next bend
     - Center of mass outside current grip
   - **Optimal Repo Point Finding:**
     - Search for best intermediate state to re-grip
     - Minimize total cycle time including repo overhead
     - Use branch-and-bound to avoid full search
   - **Grip Transition Planning:**
     - Old grip → release → rotate/flip → new grip
     - Validate new grip region exists

### Data Structures
```cpp
- SearchNode (with f, g, h scores)
- SearchState (enhanced with grip state and hash)
- CostFunction (Masked Time implementation)
- Heuristic (3-component weighted sum)
- ZobristTable (hash lookup table)
- RepoAction (repositioning plan)
- Phase3Output
```

### Key Algorithms (from Research Papers)

#### 1. Zobrist Hashing (P3S1)
```cpp
uint64_t ComputeHash(SearchState s) {
    uint64_t h = 0;
    for (int i = 0; i < 32; i++) {
        if (s.bentMask & (1 << i))
            h ^= zobrist_bend[i];
    }
    h ^= zobrist_orientation[s.orientation];
    h ^= zobrist_aba[s.abaConfig];
    return h;
}
```

#### 2. Masked Time Cost (P3S2)
```cpp
double ComputeStepCost(State current, Bend next) {
    double t_bend = next.estimatedTime;
    double t_rot = (next.reqOrientation != current.orientation) ? 1.5 : 0.0;
    double t_aba = (next.abaWidth != current.abaWidth) ? 0.8 : 0.0;
    double t_repo = next.needsRepo ? 5.0 : 0.0;

    // CRITICAL: Parallel operations masked
    return t_bend + std::max(t_rot, t_aba) + t_repo;
}
```

#### 3. Heuristic Calculation (P3S2)
```cpp
double ComputeHeuristic(State s, std::vector<Bend> remaining) {
    double h1 = 0.8 * CountDistinctOrientations(remaining);
    double h2 = 0.3 * StdDev(GetABAWidths(remaining));
    double h3 = 2.0 * (CountGripRegions(s.validGrip) - 1);
    return h1 + h2 + h3;
}
```

#### 4. Repo Trigger (P3S3)
```cpp
bool NeedsRepo(State s, Bend next) {
    if (s.validGripArea < MIN_AREA) return true;
    if (!CanGrip(s.gripCenter, next.geometry)) return true;
    if (CenterOfMassOutside(s.gripCenter, s.bentShape)) return true;
    return false;
}
```

### Dependencies
- Phase 2 Output (DAG, Constraints, Grip regions)
- **NEW: Hash table library (std::unordered_map or custom)**

### Test Cases
- [ ] **NEW: Zobrist hash collision rate test**
- [ ] **NEW: Masked time optimization verification**
- [ ] **NEW: Heuristic admissibility test**
- [ ] **NEW: Repo injection trigger test**
- [ ] **NEW: State space explosion handling (>1M states)**

### Acceptance Criteria
- [ ] Finds valid sequence for all test cases
- [ ] Respects all precedence constraints
- [ ] Optimizes for cycle time
- [ ] Performance: < 5 seconds for typical parts
- [ ] **NEW: Zobrist hashing eliminates 95%+ duplicate states**
- [ ] **NEW: Heuristic reduces search space by 10x+**
- [ ] **NEW: Masked time correctly computes parallel operations**

---

## Phase 3.5: Repositioning Planner ("The Re-Gripper") - NEW

### Objective
Detect when repositioning is required và tìm optimal repo point trong sequence.

### Rationale (from Research Papers)
Research papers nhấn mạnh rằng repositioning (repo) là một **critical sub-problem** riêng biệt:
- Không phải tất cả parts có thể hoàn thành trong một lần kẹp
- Box closing scenarios BUỘC phải repo
- Grip region shrinking có thể khiến sequence không khả thi
- Repo adds significant cost (~5 seconds) → phải optimize vị trí repo

### Deliverables

1. **Repo Trigger Detector**
   - **Trigger Condition 1: Grip Area Exhausted**
     ```cpp
     if (validGripArea < MIN_AREA) → NEED_REPO
     ```
   - **Trigger Condition 2: Box Closing Detected**
     ```cpp
     if (Is3SidedEnclosure(state) && HasUnbentSide())
         → NEED_REPO before 4th side
     ```
   - **Trigger Condition 3: Center of Mass Outside Grip**
     ```cpp
     if (!GripContains(gripCenter, centerOfMass))
         → NEED_REPO
     ```
   - **Trigger Condition 4: No Valid Grip for Next Bend**
     ```cpp
     if (!ExistsValidGrip(state, nextBend))
         → NEED_REPO
     ```

2. **Optimal Repo Point Finder**
   - **Search Space:** After which bend should we repo?
   - **Objective:** Minimize total cycle time including repo overhead
   - **Algorithm: Branch and Bound**
     ```cpp
     BestRepoPoint FindOptimalRepo(Sequence current) {
         double bestCost = INFINITY;
         int bestRepoAfter = -1;

         for (int i = 0; i < current.bends.size(); i++) {
             // Try repo after bend i
             double cost = EstimateCostWithRepo(current, i);
             if (cost < bestCost) {
                 bestCost = cost;
                 bestRepoAfter = i;
             }
         }

         return {bestRepoAfter, bestCost};
     }
     ```
   - **Heuristic:** Prefer repo points where:
     - Valid grip region maximized after repo
     - Remaining bends balanced between before/after repo
     - Orientation changes minimized

3. **Grip Transition Planner**
   - **Old Grip → Release**
     - Lower part gently to table
     - Verify stable resting position
   - **Reorientation (if needed)**
     - Rotate part on table (manual or with manipulator)
     - New orientation for better access
   - **New Grip → Acquire**
     - Find valid grip region in new state
     - Use MIR algorithm from Phase 2
     - Verify new grip can complete remaining bends
   - **Validation:**
     - New grip area >= MIN_AREA
     - Remaining bends accessible from new grip
     - No new collisions introduced

4. **Repo Cost Model**
   ```cpp
   double RepoCost = t_release + t_reorient + t_acquire + t_safety_margin

   where:
     t_release = 2.0s    // Lower to table
     t_reorient = 0-3.0s // Depends on rotation needed
     t_acquire = 2.0s    // Re-grip
     t_safety = 1.0s     // Safety buffer
     TOTAL ≈ 5-8 seconds
   ```

5. **Repo Sequence Generator**
   - Input: Sequence with repo trigger point
   - Output: Modified sequence with repo action inserted
   - **Repo Action:**
     ```cpp
     struct RepoAction {
         int afterBendId;         // Repo after this bend
         gp_Pnt oldGripCenter;    // Release this grip
         gp_Pnt newGripCenter;    // Acquire new grip
         double rotationAngle;    // Reorient by this angle
         double estimatedTime;    // Repo cost
     };
     ```

### Data Structures
```cpp
- RepoTrigger (trigger condition + severity)
- RepoCandidate (possible repo points with costs)
- GripTransition (old_grip → new_grip plan)
- RepoAction (complete repo instruction)
- Phase3_5_Output (sequence with repo actions)
```

### Key Algorithms

#### 1. Box Closing Detection
```cpp
bool Is3SidedEnclosure(BentState state) {
    // Project all bent flanges to 2D
    // Check if they form U-shape or 3 sides of rectangle
    // Return true if 4th side would close the box
}
```

#### 2. Grip Region Prediction
```cpp
Polygon2D PredictGripRegion(BentState current, Bend next) {
    // Simulate bending next
    // Project new bent flange as dead zone
    // Compute: new_valid_grip = old_valid_grip - new_dead_zone
    return new_valid_grip;
}
```

#### 3. Optimal Repo Timing
```cpp
int FindOptimalRepoPoint(std::vector<Bend> sequence) {
    // Try repo after each bend
    // Evaluate cost: remaining_bends_accessible, repo_overhead
    // Use dynamic programming to avoid redundant computation
    // Return index minimizing total cost
}
```

### Dependencies
- Phase 2 Output (Grasp constraints, Dead zones)
- Phase 3 Output (Sequence candidates)
- **Requires tight integration with Phase 3 A* search**

### Integration with Phase 3
**Option A: Reactive Repo (Current approach)**
- Phase 3 generates sequence assuming no repo
- Phase 3.5 detects failures and inserts repo
- Iterate until valid

**Option B: Proactive Repo (Recommended by research)**
- Phase 3 search state includes `repoCount`
- Repo is a valid "action" in A* search
- Cost function includes repo overhead
- Search naturally finds optimal repo points
- **Implement this in Phase 3 refactor**

### Test Cases
- [ ] Box closing scenario (forced repo)
- [ ] Grip area exhaustion (gradual shrinking)
- [ ] COM drift outside grip
- [ ] Multiple repo points (complex parts)
- [ ] Repo point optimization (minimize cycle time)
- [ ] Grip transition feasibility

### Acceptance Criteria
- [ ] Detects all scenarios requiring repo
- [ ] Finds optimal repo point minimizing cycle time
- [ ] Validates new grip can complete remaining bends
- [ ] Repo actions correctly encoded in output
- [ ] Performance: < 1 second for repo planning

---

## Phase 4: Physics Validator ("The Gatekeeper")

### Objective
Validate sequence với collision detection và physics simulation.

### Deliverables
1. **Swept Volume Generator**
   - **BRepPrimAPI_MakeRevol for bend arcs (P4s2)**
     - Revolve moving flange around bend axis
     - Generate exact swept volume as TopoDS_Shape
     - Handle partial angles (not full 360°)
   - **Convex Hull Approximation**
     - For fast broad-phase collision
     - Use CGAL convex hull on key points
   - **Swept Volume Caching**
     - Compute once per bend, reuse across validation

2. **Collision Detection System - Hierarchical (P4s2)**

   **Level 1: AABB (Axis-Aligned Bounding Box) - Fast Rejection**
   ```cpp
   bool CheckAABB(Box a, Box b) {
       // O(1) overlap test
       // Rejects 90%+ of non-colliding pairs
   }
   ```

   **Level 2: OBB (Oriented Bounding Box) - Tighter Fit**
   ```cpp
   bool CheckOBB(OBB a, OBB b) {
       // SAT (Separating Axis Theorem)
       // 15 axes to test (3+3 face normals, 9 edge crosses)
       // Rejects 99%+ of non-colliding pairs
   }
   ```

   **Level 3: Exact Geometry - Final Validation**
   ```cpp
   bool CheckExact(Shape swept, Shape fixed) {
       // BRepAlgoAPI_Common for boolean intersection
       // If result non-empty → COLLISION
   }
   ```

   **Dynamic BVH Tree (P4s2)**
   - Bounding Volume Hierarchy updated per bend step
   - Efficient broad-phase culling
   - Refit boxes as geometry changes

3. **Grasp Physics Engine (P4s3)**

   **Area Validation**
   ```cpp
   if (gripArea < MIN_AREA) → FAIL
   // Typical: MIN_AREA = 100 mm²
   ```

   **Center of Mass Check**
   ```cpp
   gp_Pnt com = ComputeCenterOfMass(bentShape);
   if (Distance(com, gripCenter) > MAX_COM_DIST) → FAIL
   // Prevents tipping during manipulation
   ```

   **Torque/Tilt Stability (P4s3)**
   ```cpp
   double torque = Mass * g * Distance(com, gripCenter);
   if (torque > MAX_TORQUE) → FAIL
   // Ensures part doesn't tilt under gravity
   ```

   **Shear/Slip Resistance (P4s3)**
   ```cpp
   double shear = Mass * g;  // Worst case vertical
   double friction = μ * NormalForce;
   if (shear > friction) → FAIL
   // Prevents part slipping from vacuum grip
   ```

   **Largest Empty Rectangle (LER) for Grip Planning**
   - Find optimal grip placement within valid region
   - Maximize distance from dead zones and COM

4. **Tool Extraction Validator (P4s3)**

   **Trap Scenario Detection**
   ```cpp
   bool CanExtractTool(Bend b, BentState after) {
       // Define extraction path (typically vertical pullout)
       gp_Dir extractDir(0, 0, 1);

       // Ray cast from tool position along extraction path
       for (auto& flange : after.bentFlanges) {
           if (RayIntersects(extractDir, flange.geometry))
               return false;  // TRAPPED!
       }
       return true;
   }
   ```

   **Springback Compensation (P4s3)**
   - Material springs back after bending
   - Typical: 2-3° for steel, 4-6° for aluminum
   - Overbend by springback angle to achieve target
   - Formula: `bend_angle_machine = target_angle + springback`

   **Extraction Path Planning**
   - If vertical extraction blocked, try angled paths
   - Search for collision-free withdrawal trajectory
   - If none found → sequence INVALID

5. **Feedback Generator**
   - **Failure Mode Identification**
     - Collision type (swept volume vs fixed geometry)
     - Grip failure type (area, COM, torque, shear)
     - Extraction failure (trapped tool)
   - **Correction Suggestions**
     - Suggest bend reordering
     - Suggest repo before problematic bend
     - Suggest alternative grip points
   - **Repo Recommendations**
     - When grip invalid, suggest optimal repo point
     - Minimize total cycle time with repo included

### Data Structures
```cpp
- SweptVolume (with exact and approximate representations)
- CollisionResult (with contact points and penetration depth)
- GraspValidation (with all physics checks: area, COM, torque, shear)
- ExtractionPath (trajectory for tool removal)
- ValidationResult (pass/fail with detailed failure reasons)
- BVHNode (for hierarchical collision tree)
- Phase4Output
```

### Key Algorithms (from Research Papers)

#### 1. Swept Volume Generation (P4s2)
```cpp
TopoDS_Shape GenerateSweptVolume(Bend b, BentState state) {
    TopoDS_Face flange = b.movingFlange;
    gp_Ax1 axis = b.bendAxis;
    double angle = b.bendAngle;

    BRepPrimAPI_MakeRevol revol(flange, axis, angle);
    return revol.Shape();
}
```

#### 2. Hierarchical Collision (P4s2)
```cpp
bool CheckCollision(Shape a, Shape b) {
    // Level 1: AABB (fast rejection)
    if (!CheckAABB(GetAABB(a), GetAABB(b)))
        return false;

    // Level 2: OBB (tighter bounds)
    if (!CheckOBB(GetOBB(a), GetOBB(b)))
        return false;

    // Level 3: Exact geometry
    return CheckExactIntersection(a, b);
}
```

#### 3. Torque Validation (P4s3)
```cpp
bool ValidateTorque(gp_Pnt grip, gp_Pnt com, double mass) {
    double lever_arm = grip.Distance(com);
    double torque = mass * 9.81 * lever_arm;  // N⋅m
    return torque < MAX_TORQUE;  // Typical: 10 N⋅m
}
```

#### 4. Tool Extraction Check (P4s3)
```cpp
bool CanExtractTool(Bend b, BentState after) {
    gp_Dir pullDir(0, 0, 1);  // Vertical
    gp_Pnt toolPos = b.toolPosition;

    for (auto& flange : after.bentFlanges) {
        if (RayIntersects(toolPos, pullDir, flange))
            return false;
    }
    return true;
}
```

#### 5. Springback Compensation (P4s3)
```cpp
double CompensateSpringback(double targetAngle, Material mat) {
    // Empirical formulas from research
    double springback = 0.0;
    if (mat.type == STEEL)
        springback = 2.0 + 0.5 * mat.thickness;  // degrees
    else if (mat.type == ALUMINUM)
        springback = 4.0 + 0.8 * mat.thickness;

    return targetAngle + springback;
}
```

### Dependencies
- Phase 3 Output (Sequence)
- FCL library (optional, for faster collision)
- CGAL (for convex hull, geometric predicates)
- **NEW: OpenCASCADE Boolean operations**

### Test Cases
- [ ] **NEW: Hierarchical collision performance test**
- [ ] **NEW: AABB vs OBB vs Exact accuracy comparison**
- [ ] **NEW: Torque validation with various COM positions**
- [ ] **NEW: Shear validation with different materials**
- [ ] **NEW: Tool trap detection (enclosed geometry)**
- [ ] **NEW: Springback compensation accuracy**
- [ ] **NEW: Dynamic BVH tree update performance**

### Acceptance Criteria
- [ ] No false negatives (missed collisions)
- [ ] Minimal false positives (< 5%)
- [ ] Actionable feedback on failures
- [ ] Performance: < 100ms per step validation
- [ ] **NEW: AABB rejects 90%+ non-collisions in <1μs**
- [ ] **NEW: Torque/shear validation within 10% of FEA**
- [ ] **NEW: Springback compensation ±1° accuracy**

---

## Phase 5: Post-Processor ("The Translator")

### Objective
Generate machine instructions và HMI visualization data.

### Deliverables
1. **XML Generator (PB-XML Format - P5s1)**

   **Schema Structure (from research):**
   ```xml
   <Job>
     <Header>
       <Material thickness="1.5" type="AISI304"/>
       <Tooling upper="Standard" lower="ABA"/>
       <JobInfo partName="BOX001" program="AUTO_GEN"/>
     </Header>

     <ProcessSequence>
       <!-- Rotation step -->
       <Step id="1" type="Rotation">
         <Angle>90</Angle>
         <Duration>1.5</Duration>
       </Step>

       <!-- ABA reconfiguration -->
       <Step id="2" type="ABASetup">
         <Segments>
           <Seg pos="0" width="100"/>
           <Seg pos="100" width="150"/>
           <Seg pos="250" width="200"/>
         </Segments>
         <Duration>0.8</Duration>
       </Step>

       <!-- Actual bend -->
       <Step id="3" type="Bend">
         <BendLine>
           <Start x="0" y="50"/>
           <End x="200" y="50"/>
         </BendLine>
         <Angle target="90" compensated="92.3"/>
         <Force>15.5</Force>  <!-- kN -->
         <Springback>2.3</Springback>
         <Duration>2.5</Duration>
       </Step>

       <!-- Adaptive control parameters -->
       <AdaptiveControl>
         <AngleFeedback enabled="true" tolerance="0.5"/>
         <ForceLimit max="25.0"/>
         <SpeedControl adaptive="true"/>
       </AdaptiveControl>
     </ProcessSequence>

     <Metadata>
       <TotalCycleTime>45.2</TotalCycleTime>
       <BendCount>6</BendCount>
       <RepositionCount>0</RepositionCount>
     </Metadata>
   </Job>
   ```

   **Generator Tasks:**
   - Job header with material properties (thickness, type, yield strength)
   - Tooling configuration (upper blade, ABA segments)
   - Process sequence (rotations, ABA setups, bends, repos)
   - **NEW: Adaptive control parameters**
     - Angle feedback enable/disable
     - Force limits for safety
     - Speed control adaptive mode
   - **NEW: Springback compensation values**
   - **NEW: Metadata (cycle time, bend count, repo count)**

2. **JSON Generator (VDM - Visualization Data Model - P5s2)**

   **Schema Structure:**
   ```json
   {
     "scene": {
       "camera": {
         "position": [0, -500, 300],
         "lookAt": [0, 0, 0],
         "fov": 45
       },
       "lights": [
         {"type": "directional", "direction": [1, -1, -1]},
         {"type": "ambient", "intensity": 0.4}
       ]
     },

     "mesh": "part_box001.glb",

     "skeleton": {
       "bones": [
         {"id": 0, "name": "base", "parent": null},
         {"id": 1, "name": "flange_top", "parent": 0},
         {"id": 2, "name": "flange_right", "parent": 0},
         {"id": 3, "name": "flange_bottom", "parent": 0},
         {"id": 4, "name": "flange_left", "parent": 0}
       ]
     },

     "animation": {
       "duration": 45.2,
       "fps": 30,
       "keyframes": [
         {
           "time": 0.0,
           "transforms": [
             {"bone": 0, "rotation": [0,0,0], "translation": [0,0,0]},
             {"bone": 1, "rotation": [0,0,0]}
           ]
         },
         {
           "time": 3.5,
           "transforms": [
             {"bone": 1, "rotation": [90,0,0]}
           ]
         },
         {
           "time": 8.2,
           "transforms": [
             {"bone": 0, "rotation": [0,0,90]},
             {"bone": 2, "rotation": [90,0,0]}
           ]
         }
       ]
     },

     "metadata": {
       "partName": "BOX001",
       "cycleTime": 45.2,
       "bendCount": 6
     }
   }
   ```

   **Generator Tasks:**
   - Scene configuration (camera, lights)
   - **Bone map for skeletal animation**
     - Base bone = base face
     - Each flange = child bone
     - Rotation hierarchy matches bend sequence
   - **Keyframe data**
     - Time stamps from sequence cost function
     - Rotation transforms (quaternion or Euler)
     - Translation for repos
   - Metadata (part name, cycle time)

3. **HMI Data Exporter (P5s2)**
   - **GLB mesh with skinning**
     - Export part geometry as GLB/GLTF
     - Assign bone weights to vertices
     - Flange vertices weighted to flange bone
     - Base vertices weighted to base bone
   - **Animation keyframe file**
     - JSON format compatible with Three.js/Babylon.js
     - Smooth interpolation between keyframes
   - **Preview Renderer (optional)**
     - Validate animation looks correct
     - Screenshot at key steps

4. **Validation Layer**
   - **Schema validation**
     - XML schema (XSD) validation
     - JSON schema validation
   - **Machine compatibility check**
     - Verify bend angles within machine limits (-135° to +135°)
     - Verify forces within machine capacity
     - Verify ABA segments available on target machine
   - **Sanity checks**
     - Total cycle time > 0
     - Bend count matches sequence
     - No duplicate step IDs

### Data Structures
```cpp
- MachineInstruction (base class for Rotation, ABASetup, Bend, Repo)
- XMLJob (complete job structure)
- VisualizationData (VDM format)
- SkeletonBone (bone hierarchy)
- AnimationKeyframe (transform at time t)
- HMIExport (complete export package)
```

### Key Algorithms (from Research Papers)

#### 1. Springback Calculation (P5s1)
```cpp
double CalculateCompensatedAngle(double target, Material mat) {
    // Empirical model from research
    double k_factor = mat.yieldStrength / 200.0;  // Normalized
    double springback = (2.0 + mat.thickness * 0.5) * k_factor;
    return target + springback;
}
```

#### 2. Bone Weight Assignment (P5s2)
```cpp
void AssignBoneWeights(Mesh mesh, Skeleton skeleton) {
    for (auto& vertex : mesh.vertices) {
        // Determine which face vertex belongs to
        TopoDS_Face ownerFace = FindOwnerFace(vertex);

        // Find corresponding bone
        int boneId = skeleton.GetBoneForFace(ownerFace);

        // Assign weight
        vertex.boneWeights[boneId] = 1.0;
    }
}
```

#### 3. Keyframe Interpolation (P5s2)
```cpp
Keyframe InterpolateKeyframes(Keyframe k1, Keyframe k2, double t) {
    double alpha = (t - k1.time) / (k2.time - k1.time);

    Keyframe result;
    result.time = t;

    // Quaternion slerp for rotation
    result.rotation = Slerp(k1.rotation, k2.rotation, alpha);

    // Linear interpolation for translation
    result.translation = Lerp(k1.translation, k2.translation, alpha);

    return result;
}
```

### Dependencies
- Phase 4 Output (Validated Sequence)
- TinyXML2 or PugiXML (for XML generation)
- nlohmann/json (for JSON generation)
- **NEW: GLB/GLTF exporter library**
- **NEW: XML schema validator**

### Test Cases
- [ ] **NEW: XML schema validation with XSD**
- [ ] **NEW: JSON schema validation**
- [ ] **NEW: Bone hierarchy correctness**
- [ ] **NEW: Keyframe timing matches cycle time**
- [ ] **NEW: Springback compensation values**
- [ ] **NEW: Adaptive control parameter presence**
- [ ] **NEW: GLB mesh export with valid skinning**

### Acceptance Criteria
- [ ] Valid XML schema (validates against PB-XML XSD)
- [ ] All bends correctly encoded
- [ ] HMI data renders correctly (visual test)
- [ ] Adaptive control parameters present
- [ ] **NEW: Springback values within ±5% of empirical data**
- [ ] **NEW: Animation playback matches expected sequence**
- [ ] **NEW: Cycle time in metadata matches sum of step durations**

---

## Cross-Phase Integration

### Shared Types
All phases share common types defined in:
- `include/openpanelcam/types.h`
- `docs/data-structures/cross-phase-types.md`

### Interface Contracts
```cpp
// Each phase implements this interface
class IPhase {
public:
    virtual bool validate() = 0;
    virtual void execute() = 0;
    virtual std::string getErrorMessage() = 0;
};
```

---

## Development Timeline

```
Week 1-2:   Phase 1 Design (Documentation)
Week 3-5:   Phase 1 Implementation
Week 6:     Phase 1 Testing & Refinement

Week 7-8:   Phase 2 Design
Week 9-11:  Phase 2 Implementation
Week 12:    Phase 2 Testing

Week 13-14: Phase 3 Design
Week 15-18: Phase 3 Implementation
Week 19:    Phase 3 Testing

Week 20:    Phase 3.5 Design (NEW - Repo Planner)
Week 21-22: Phase 3.5 Implementation
Week 23:    Phase 3.5 Testing & Integration with Phase 3

Week 24-25: Phase 4 Design
Week 26-30: Phase 4 Implementation
Week 31:    Phase 4 Testing

Week 32-33: Phase 5 Design
Week 34-37: Phase 5 Implementation
Week 38:    Phase 5 Testing

Week 39-40: Integration Testing (All Phases)
Week 41-42: Performance Optimization
Week 43-44: Real-world Validation & Bug Fixes
```

**Total Duration: ~11 months**

### Milestones
- **Month 2:** Phase 1 Complete (Geometric Parser working)
- **Month 3:** Phase 2 Complete (Constraints extracted)
- **Month 5:** Phase 3 + 3.5 Complete (Optimal sequences with repo)
- **Month 8:** Phase 4 Complete (Physics validation working)
- **Month 10:** Phase 5 Complete (Machine code generation)
- **Month 11:** Production Ready (Integration + optimization done)

---

## Risk Mitigation

### Technical Risks
1. **OCCT Learning Curve**
   - Mitigation: Build small examples first
   - Reference: OCCT samples, FreeCAD source

2. **Collision Detection Performance**
   - Mitigation: Hierarchical culling, LOD
   - Fallback: Conservative approximations

3. **A* State Space Explosion**
   - Mitigation: Beam search, state pruning
   - Fallback: Greedy heuristic

### Design Risks
1. **Cross-Phase Incompatibility**
   - Mitigation: Define interfaces upfront
   - Review: Design review before implementation

2. **Missing Edge Cases**
   - Mitigation: Comprehensive test suite
   - Feedback: Iterative testing with real parts

---

## Quality Gates

### Per-Phase Gates
Before moving to next phase:
- [ ] All unit tests pass
- [ ] Integration tests with previous phase pass
- [ ] Documentation complete
- [ ] Code review completed
- [ ] Performance benchmarks met

### Final Integration Gate
- [ ] End-to-end test: STEP → Machine Code
- [ ] Validation against known-good sequences
- [ ] HMI visualization verified
