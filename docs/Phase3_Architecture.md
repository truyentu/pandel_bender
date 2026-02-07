# Phase 3: Sequencer - Architecture & API Reference

> **The Brain** of OpenPanelCAM — finds the optimal bend sequence using A* search.

## Overview

Phase 3 takes the precedence constraints from Phase 2 and finds the time-optimal ordering of bends, considering:
- **Rotation time** between orientations
- **ABA reconfiguration** time (parallel with rotation)
- **Repositioning** detection (box closing, grip exhaustion)
- **Precedence constraints** from the DAG

```
Phase 2 Output ──→ Sequencer ──→ Phase 3 Output
  PrecedenceDAG        A*         bendSequence[]
  GraspConstraints   Search       actions[]
  ABAConstraints    + Zobrist     totalCycleTime
                   + Heuristics   repoCount
```

## Architecture

```
Sequencer (entry point)
├── AStarSearch
│   ├── SuccessorGenerator
│   │   ├── PrecedenceDAG (from Phase 2)
│   │   ├── BendTimeEstimator
│   │   └── RepoTriggerDetector
│   ├── MaskedTimeCost
│   ├── CombinedHeuristic
│   │   ├── H1: RotationalEntropyHeuristic
│   │   ├── H2: ToolingVarianceHeuristic
│   │   └── H3: GraspFragmentationHeuristic
│   ├── ZobristTable (state deduplication)
│   └── BeamSearch (fallback)
└── SearchConfig
```

## Key Algorithms

### 1. Masked Time Cost Function

```
StepCost = t_bend + max(t_rotation, t_aba) + t_repo
```

**Critical insight**: Rotation and ABA reconfiguration happen in **parallel** on the Salvagnini P4, so we use `max()` not `sum()`.

| Operation | Default Time | Notes |
|-----------|-------------|-------|
| Base bend (90°) | 2.0s | + 0.01s per degree beyond 90° |
| 90° rotation | 1.5s | Shortest path (min steps) |
| ABA reconfig | 0.8s | Fixed time per change |
| Repositioning | 6.5s | Release + reorient + acquire + safety |

### 2. Zobrist Hashing

O(1) state deduplication using precomputed random 64-bit XOR values:
- 32 bend position hashes
- 4 orientation hashes
- 256 ABA config bucket hashes

Incremental updates: `newHash = oldHash ^ oldComponent ^ newComponent`

### 3. Three Research Heuristics

All admissible (never overestimate):

| Heuristic | Formula | Weight | Purpose |
|-----------|---------|--------|---------|
| H1: RotationalEntropy | k1 × (distinctOrientations - 1) | 0.8 | Minimize rotations |
| H2: ToolingVariance | k2 × stddev(abaWidths) | 0.003 | Group similar widths |
| H3: GraspFragmentation | k3 × (remainingBends / 3) | 2.0 | Predict repo risk |

Combined: `h = h1 + h2 + h3`

### 4. Beam Search Fallback

When A* exceeds `maxNodes` or `timeoutSeconds`:
1. Automatically falls back to beam search
2. Keeps only top-K nodes (by f-score) at each depth
3. Marks result as `optimal = false`
4. Default beam width: 100

## API Reference

### Sequencer

```cpp
#include "openpanelcam/phase3/sequencer.h"

Sequencer seq;

// Optional: configure search
SearchConfig config;
config.maxNodes = 1000000;
config.timeoutSeconds = 30.0;
config.useBeamSearch = true;
config.beamWidth = 100;
seq.setConfig(config);

// Run sequencing
Phase3Output result = seq.sequence(phase2Output, bends);

if (result.success) {
    for (int bendId : result.bendSequence) {
        // Process bend in order
    }
    std::cout << result.generateSummary();
}
```

### Phase3Output

| Field | Type | Description |
|-------|------|-------------|
| `bendSequence` | `vector<int>` | Optimal bend order (bend IDs) |
| `actions` | `vector<SequenceAction>` | Detailed actions (BEND, REPOSITION) |
| `totalCycleTime` | `double` | Total estimated time (seconds) |
| `repoCount` | `int` | Number of repositions |
| `repoAfterBends` | `vector<int>` | Repo after these bend IDs |
| `success` | `bool` | Solution found |
| `optimal` | `bool` | True if A* (false if beam search) |
| `stats` | `SequencerStatistics` | Search statistics |

### SearchState

Bitmask-based state encoding (max 32 bends):

```cpp
SearchState state;
state.markBent(0);           // Mark bend 0 complete
state.isBent(0);             // true
state.bentCount();           // 1
state.isGoal(totalBends);    // Check if all done
```

## Performance

Tested on Windows x64, Release build:

| Bends | Constraint | Time | Nodes Expanded |
|-------|-----------|------|----------------|
| 3 | None | < 1ms | ~10 |
| 5 | None | < 100ms | < 120 |
| 8 | None | < 5s | varies |
| 10 | Chain | < 1ms | 10 (trivial) |
| 10 | None | < 10s | varies |
| 20 | Chain | < 100ms | 20 (trivial) |

Deduplication eliminates significant duplicate states. Heuristic reduces 5-bend search from 120 (5!) to fewer nodes.

## File Structure

```
include/openpanelcam/phase3/
├── types.h                  # SearchState, SearchNode, Phase3Output
├── zobrist_table.h          # Zobrist hashing
├── cost_function.h          # MaskedTimeCost, BendTimeEstimator
├── heuristic.h              # H1, H2, H3, CombinedHeuristic
├── successor_generator.h    # SuccessorGenerator, RepoTriggerDetector
├── astar_search.h           # AStarSearch, SearchConfig
└── sequencer.h              # Sequencer (main entry point)

src/phase3/
├── zobrist_table.cpp
├── cost_function.cpp
├── heuristic.cpp
├── successor_generator.cpp
├── astar_search.cpp
└── sequencer.cpp

tests/phase3/
├── test_types.cpp           # 20 tests, 78 assertions
├── test_zobrist.cpp         # 8 tests, 8 assertions
├── test_cost_function.cpp   # 19 tests, 27 assertions
├── test_heuristic.cpp       # 18 tests, 35 assertions
├── test_successor.cpp       # 7 tests, 32 assertions
├── test_astar.cpp           # 9 tests, 38 assertions
├── test_sequencer.cpp       # 9 tests, 30 assertions
├── test_repo.cpp            # 8 tests, 15 assertions
├── test_performance.cpp     # 7 tests, 29 assertions
└── test_edge_cases.cpp      # 11 tests, 62 assertions
```

**Total: 116 tests, 354 assertions**

## Integration: Phase 2 → Phase 3

```cpp
// Phase 2
ConstraintSolver solver;
Phase2Output p2 = solver.solve(bends);

// Phase 3
Sequencer seq;
Phase3Output p3 = seq.sequence(p2, bends);

// Use result
for (const auto& action : p3.actions) {
    if (action.type == ActionType::BEND) {
        executeBend(action.bendId);
    } else if (action.type == ActionType::REPOSITION) {
        executeRepo();
    }
}
```
