# Phase 2: Constraint Solver Architecture

## Overview

Phase 2 analyzes bend features from Phase 1 and generates optimal bend sequences
by solving geometric, grasp, and tool constraints.

## Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                       ConstraintSolver                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────┐  ┌─────────────────────┐              │
│  │ GeometricPrecedence │  │ GraspConstraint     │              │
│  │ Analyzer            │  │ Generator           │              │
│  ├─────────────────────┤  ├─────────────────────┤              │
│  │ - Corner overlap    │  │ - Dead zones        │              │
│  │ - Box closing       │  │ - Valid regions     │              │
│  │ - Sequential block  │  │ - MIR algorithm     │              │
│  │ - Ray casting       │  │ - Grip physics      │              │
│  └─────────────────────┘  └─────────────────────┘              │
│                                                                 │
│  ┌─────────────────────┐  ┌─────────────────────┐              │
│  │ ABAConstraint       │  │ PrecedenceDAG       │              │
│  │ Analyzer            │  │                     │              │
│  ├─────────────────────┤  ├─────────────────────┤              │
│  │ - Subset sum DP     │  │ - Cycle detection   │              │
│  │ - Tool width calc   │  │ - Topological sort  │              │
│  │ - Segment selection │  │ - Level calculation │              │
│  │ - Box closing check │  │ - Edge management   │              │
│  └─────────────────────┘  └─────────────────────┘              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

```
Phase1Output (BendFeatures)
         │
         ▼
┌────────────────────┐
│ GeometricAnalyzer  │───▶ PrecedenceEdges
└────────────────────┘
         │
         ▼
┌────────────────────┐
│ PrecedenceDAG      │───▶ Build graph, detect cycles
└────────────────────┘
         │
         ├───▶ GraspGenerator ───▶ GraspConstraints
         │
         ├───▶ ABAAnalyzer ───▶ ABAConstraints
         │
         ▼
┌────────────────────┐
│ TopologicalSort    │───▶ Optimal bend sequence
└────────────────────┘
         │
         ▼
    Phase2Output
```

## Key Algorithms

### 1. Cycle Detection (DFS 3-Color)
- **Purpose:** Detect cyclic dependencies in bend ordering
- **Complexity:** O(V + E)
- **Location:** `precedence_dag.cpp:isAcyclic()`

### 2. Cycle Resolution (Confidence-Based)
- **Purpose:** Break cycles by removing lowest-confidence edges
- **Complexity:** O(V + E) per cycle
- **Location:** `precedence_dag.cpp:resolveCycles()`

### 3. Topological Sort (Kahn's Algorithm)
- **Purpose:** Generate valid bend sequence respecting all constraints
- **Complexity:** O(V + E)
- **Location:** `precedence_dag.cpp:topologicalSort()`

### 4. Maximum Inscribed Rectangle (Grid Sampling + Binary Search)
- **Purpose:** Find largest grip area in valid region
- **Complexity:** O(G² × log(D) × V)
- **Location:** `grasp_constraint_generator.cpp:findMaxInscribedRect()`

### 5. Subset Sum (Dynamic Programming)
- **Purpose:** Find optimal ABA segment combination
- **Complexity:** O(T × S) where T=target width, S=segments
- **Location:** `aba_constraint_analyzer.cpp:solveSubsetSum()`

## Performance Characteristics

| Operation | Complexity | Typical Time (7 bends) |
|-----------|------------|------------------------|
| Geometric Analysis | O(n²) | < 5 ms |
| Grasp Analysis | O(n × states) | < 10 ms |
| ABA Analysis | O(n × target) | < 5 ms |
| Graph Operations | O(V + E) | < 1 ms |
| **Total** | **O(n²)** | **< 20 ms** |

## Error Handling

### Recoverable Errors
- No valid grip region → Warning, continue
- ABA infeasible → Warning, suggest alternatives
- Weak constraints → Remove lowest confidence via resolveCycles()

### Critical Errors
- Cyclic dependencies (unresolvable) → Fail with error
- Empty bend sequence → Fail with error
- Graph finalization failure → Fail with error

## Extension Points

1. **Custom Constraint Types:** Add new `ConstraintType` enum values
2. **Alternative Algorithms:** Swap MIR, subset sum implementations
3. **Real Geometry:** Replace mock with OCCT TopoDS_Shape
4. **Visualization:** Add DOT export for Graphviz

## Configuration

Default thresholds (modifiable):
- Minimum grip area: 100 mm²
- Safety margin: 5 mm
- Tolerance for 90° detection: ±5°
- Max COM offset: 100 mm
- ABA segments: {25, 50, 75, 100, 125, 150, 175, 200} mm
