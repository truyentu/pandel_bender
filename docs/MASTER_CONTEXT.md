# OpenPanelCAM - Master Context Document

> **Purpose**: File này duy trì context xuyên suốt các session làm việc.
> **Last Updated**: 2026-02-04

---

## 1. PROJECT IDENTITY

**Project Name**: OpenPanelCAM Core Kernel
**Target Machine**: Salvagnini P4 Panel Bender
**Goal**: High-performance CAM kernel từ STEP file đến Machine Instructions

### Input/Output
- **Input**: Raw 3D STEP files (AP203/AP214/AP242)
- **Output**:
  - Validated Bend Sequence
  - Machine Instructions (XML/Binary)
  - HMI Visualization Data (JSON)

---

## 2. MACHINE PHYSICS (Critical Context)

### 2.1 Panel Bender vs Press Brake
| Aspect | Panel Bender (P4) | Press Brake |
|--------|-------------------|-------------|
| Sheet Position | Horizontal table | Vertical |
| Part Handling | Automatic manipulator | Manual/Robot |
| Part Flip | NOT required | Required for up/down bends |
| Tooling | Universal (ABA) | Fixed V-dies |

### 2.2 Bending Directions
```
BEND_UP (Positive):   Lower blade pushes UP    → Flange goes UP
BEND_DOWN (Negative): Upper blade pushes DOWN  → Flange goes DOWN

The manipulator CANNOT bend the side it's currently holding.
→ Must ROTATE part to access different sides
```

### 2.3 Masked Time Philosophy (Core Optimization)
```
Cycle_Time = Σ max(t_rotation, t_aba_change) + Σ t_bend

If t_rotation > t_aba_change:
    → ABA adjustment happens DURING rotation
    → Effective ABA cost = 0 (MASKED)
```

### 2.4 Machine Specifications (P4-2520 Reference)
| Parameter | Value |
|-----------|-------|
| Max Sheet Size | 2500 x 1500 mm |
| Thickness Range | 0.4 - 3.2 mm |
| Bend Angle Range | -135° to +135° |
| ABA Segmentation | 5mm increments |
| Rotation Angles | 0°, 90°, 180°, 270° |

---

## 3. SYSTEM ARCHITECTURE (5-Phase Pipeline)

```
┌─────────────────────────────────────────────────────────────────────┐
│                        OpenPanelCAM Pipeline                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  STEP File                                                           │
│      │                                                               │
│      ▼                                                               │
│  ┌──────────────────┐                                                │
│  │  PHASE 1         │  Geometric Parser                              │
│  │  "The Eyes"      │  STEP → Face-Adjacency Graph (FAG)            │
│  └────────┬─────────┘                                                │
│           │ FAG + BendFeatures                                       │
│           ▼                                                          │
│  ┌──────────────────┐                                                │
│  │  PHASE 2         │  Constraint Solver                             │
│  │  "The Rules"     │  FAG → Precedence DAG + Constraints           │
│  └────────┬─────────┘                                                │
│           │ DAG + GraspConstraints + ABAConstraints                  │
│           ▼                                                          │
│  ┌──────────────────┐                                                │
│  │  PHASE 3         │  Sequencer (A* Search)                         │
│  │  "The Brain"     │  DAG → Optimal Bend Sequence                  │
│  └────────┬─────────┘                                                │
│           │ BendSequence                                             │
│           ▼                                                          │
│  ┌──────────────────┐                                                │
│  │  PHASE 4         │  Physics Validator                             │
│  │  "The Gatekeeper"│  Sequence → Collision-Free Validated Sequence │
│  └────────┬─────────┘                                                │
│           │ ValidatedSequence                                        │
│           ▼                                                          │
│  ┌──────────────────┐                                                │
│  │  PHASE 5         │  Post-Processor                                │
│  │  "The Translator"│  Sequence → Machine Code + HMI Data           │
│  └────────┬─────────┘                                                │
│           │                                                          │
│           ▼                                                          │
│  Machine Instructions (XML) + HMI Visualization (JSON)               │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 4. TECH STACK

### 4.1 Core Technologies
| Component | Technology | Version |
|-----------|------------|---------|
| Language | C++ | 17+ |
| Geometry Kernel | OpenCASCADE (OCCT) | 7.6+ |
| Linear Algebra | Eigen | 3.4+ |
| Computational Geometry | CGAL | 5.x |
| Collision Detection | FCL | 0.7+ |
| Graph Algorithms | Boost Graph (optional) | 1.80+ |

### 4.2 Build System
- CMake 3.20+
- Compiler: MSVC 2022 / GCC 11+ / Clang 14+

### 4.3 Control System (Phase 5)
- Beckhoff TwinCAT 3
- EtherCAT fieldbus
- ADS protocol for variable injection

---

## 5. KEY ALGORITHMS BY PHASE

### Phase 1: Geometric Parser
- `TopExp::MapShapesAndAncestors()` - O(n) graph construction
- `GetMaterialOutwardNormal()` - Orientation-corrected normals
- `ShapeFix_Shape` - Geometry healing
- Signed Distance Function (SDF) for bend classification

### Phase 2: Constraint Solver
- Ray Casting + SAT for overlap detection
- Maximum Inscribed Circle for grasp validation
- Subset Sum for ABA segment selection

### Phase 3: Sequencer
- A* Search with Masked Time cost function
- Zobrist Hashing for state deduplication
- Dynamic Repo injection

### Phase 4: Physics Validator
- `BRepPrimAPI_MakeRevol` for swept volumes
- Dynamic BVH (AABB/OBB trees)
- Largest Empty Rectangle (LER) for grasp planning
- Springback modeling

### Phase 5: Post-Processor
- XML schema (PB-XML/K-Flow) generation
- Skeletal animation data for HMI
- ADS variable mapping

---

## 6. PROJECT STATUS

### Current Phase: Phase 1 - Design Stage

| Phase | Status | Progress |
|-------|--------|----------|
| Phase 1 | 🔵 IN DESIGN | Documentation |
| Phase 2 | ⚪ NOT STARTED | - |
| Phase 3 | ⚪ NOT STARTED | - |
| Phase 4 | ⚪ NOT STARTED | - |
| Phase 5 | ⚪ NOT STARTED | - |

### Milestones
- [ ] Phase 1 Design Complete
- [ ] Phase 1 Implementation Complete
- [ ] Phase 1 Tests Pass
- [ ] Phase 2 Design Complete
- [ ] ...

---

## 7. FILE STRUCTURE

```
Salvagnini_controller/
├── docs/
│   ├── MASTER_CONTEXT.md          # This file
│   ├── ROADMAP.md                 # Development roadmap
│   ├── architecture/
│   │   └── system-overview.md
│   ├── data-structures/
│   │   ├── cross-phase-types.md   # Shared types
│   │   └── phase1-types.md
│   └── phases/
│       ├── phase1-design.md
│       ├── phase2-design.md
│       └── ...
├── src/
│   ├── core/                      # Shared utilities
│   ├── phase1/                    # Geometric Parser
│   ├── phase2/                    # Constraint Solver
│   ├── phase3/                    # Sequencer
│   ├── phase4/                    # Physics Validator
│   └── phase5/                    # Post-Processor
├── include/
│   └── openpanelcam/
├── tests/
│   ├── phase1/
│   └── ...
├── samples/
│   └── step_files/                # Test STEP files
└── research_paper/                # Source documents
```

---

## 8. CRITICAL DESIGN DECISIONS

### 8.1 Data Flow Between Phases
```cpp
// Phase 1 → Phase 2
struct Phase1Output {
    FaceAdjacencyGraph fag;
    std::vector<BendFeature> bends;
    TopoDS_Face baseFace;
};

// Phase 2 → Phase 3
struct Phase2Output {
    PrecedenceDAG dag;
    std::vector<GraspConstraint> graspConstraints;
    std::vector<ABAConstraint> abaConstraints;
};

// Phase 3 → Phase 4
struct Phase3Output {
    std::vector<BendStep> sequence;
    std::vector<int> rotations;
    std::vector<double> abaWidths;
};

// Phase 4 → Phase 5
struct Phase4Output {
    ValidatedSequence sequence;
    std::vector<RepoAction> repoActions;
    bool isValid;
};
```

### 8.2 Coordinate System
```
Machine Coordinate System:
  X: Along bend line (left-right)
  Y: Perpendicular to bend line (front-back)
  Z: Vertical (up-down)

Origin: Center of bending line at table level
```

### 8.3 Units
- Length: millimeters (mm)
- Angle: degrees (°) for user-facing, radians internally
- Time: seconds (s)
- Force: Newtons (N)

---

## 9. KNOWN CONSTRAINTS & GOTCHAS

### OCCT Specific
1. **TopAbs_REVERSED faces**: Always check orientation before using normal
2. **Dirty geometry**: Use ShapeFix before any analysis
3. **Zero-radius bends**: Treated as sharp edges, need special handling
4. **Memory**: OCCT uses reference counting (Handle<>)

### Algorithm Specific
1. **First Match vs Best Match**: Always use closest match in geometric queries
2. **Tolerance**: 1e-6 for geometric comparisons
3. **Degenerate cases**: Single-bend parts, coplanar flanges

---

## 10. GLOSSARY

| Term | Definition |
|------|------------|
| FAG | Face-Adjacency Graph |
| DAG | Directed Acyclic Graph (Precedence) |
| ABA | Automatic Blankholder Adjustment |
| SDF | Signed Distance Function |
| LER | Largest Empty Rectangle |
| MAC | Material Attitude Correction |
| Repo | Repositioning (re-grip) operation |
| Masked Time | Parallel operation hiding latency |

---

## 11. SESSION LOG

### 2026-02-04
- Read all 23 research documents
- Created Knowledge Base summary
- Started Phase 1 design documentation

---

*Last Session Context*: Completed reading all research PDFs. Ready to design Phase 1 data structures and algorithms.
