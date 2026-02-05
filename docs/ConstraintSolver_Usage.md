# ConstraintSolver Usage Guide

## Overview

ConstraintSolver là main orchestration class cho Phase 2 constraint analysis. Nó tích hợp:
- GeometricPrecedenceAnalyzer: Phân tích geometric constraints
- GraspConstraintGenerator: Validate grip regions
- ABAConstraintAnalyzer: Check tool feasibility

## Quick Start

```cpp
#include "openpanelcam/phase2/constraint_solver.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

// Create solver
ConstraintSolver solver;

// Prepare bends (from Phase 1)
std::vector<BendFeature> bends = { /* your bends */ };

// Solve constraints
Phase2Output output = solver.solve(bends);

// Check result
if (output.success) {
    // Use output.bendSequence for manufacturing
    for (int bendId : output.bendSequence) {
        std::cout << "Bend " << bendId << std::endl;
    }
} else {
    // Handle errors
    for (const auto& error : output.errors) {
        std::cerr << "Error: " << error << std::endl;
    }
}
```

## API Reference

### ConstraintSolver Class

#### Methods

**solve(bends)**
```cpp
Phase2Output solve(const std::vector<BendFeature>& bends);
```
Phân tích tất cả constraints và generate bend sequence.

**getStatistics()**
```cpp
const Statistics& getStatistics() const;
```
Lấy thống kê performance.

**reset()**
```cpp
void reset();
```
Reset solver state.

### Phase2Output Structure

```cpp
struct Phase2Output {
    PrecedenceDAG precedenceGraph;          // DAG với tất cả constraints
    std::vector<GraspConstraint> graspConstraints;  // Grip validation
    std::vector<ABAConstraint> abaConstraints;      // Tool feasibility
    std::vector<int> bendSequence;          // Valid bend order
    bool success;                           // Success flag
    std::string analysisSummary;            // Human-readable summary
    std::vector<std::string> errors;        // Error messages
    std::vector<std::string> warnings;      // Warning messages
};
```

### Statistics Structure

```cpp
struct Statistics {
    // Timing (milliseconds)
    double totalSolveTimeMs;
    double geometricAnalysisTimeMs;
    double graspAnalysisTimeMs;
    double abaAnalysisTimeMs;
    double graphBuildTimeMs;
    double topologicalSortTimeMs;

    // Counts
    int totalBends;
    int geometricConstraints;
    int graspStatesAnalyzed;
    int abaConstraintsGenerated;
    int totalEdges;

    // Flags
    bool hasInfeasibleBends;
    bool hasBoxClosing;
    bool hasGraspIssues;
};
```

## Common Patterns

### Example 1: Simple L-Bracket

```cpp
ConstraintSolver solver;

BendFeature b0, b1;
b0.id = 0;
b0.angle = 90.0;
b0.length = 100.0;
b0.position.x = 0.0;
b0.position.y = 0.0;

b1.id = 1;
b1.angle = 90.0;
b1.length = 80.0;
b1.position.x = 0.0;
b1.position.y = 100.0;

auto output = solver.solve({ b0, b1 });

if (output.success) {
    std::cout << "Sequence: ";
    for (int id : output.bendSequence) {
        std::cout << id << " ";
    }
    std::cout << std::endl;
}
```

### Example 2: Error Handling

```cpp
auto output = solver.solve(bends);

if (!output.success) {
    std::cerr << "Constraint solving failed!" << std::endl;

    // Print errors
    for (const auto& err : output.errors) {
        std::cerr << "ERROR: " << err << std::endl;
    }

    // Print warnings
    for (const auto& warn : output.warnings) {
        std::cerr << "WARNING: " << warn << std::endl;
    }

    // Print summary
    std::cout << output.analysisSummary << std::endl;
}
```

### Example 3: Performance Monitoring

```cpp
auto output = solver.solve(bends);
auto stats = solver.getStatistics();

std::cout << "Performance Report:" << std::endl;
std::cout << "  Total time: " << stats.totalSolveTimeMs << " ms" << std::endl;
std::cout << "  Geometric: " << stats.geometricAnalysisTimeMs << " ms" << std::endl;
std::cout << "  Grasp: " << stats.graspAnalysisTimeMs << " ms" << std::endl;
std::cout << "  ABA: " << stats.abaAnalysisTimeMs << " ms" << std::endl;
std::cout << "  Total bends: " << stats.totalBends << std::endl;
std::cout << "  Constraints: " << stats.geometricConstraints << std::endl;
```

### Example 4: Constraint Analysis

```cpp
auto output = solver.solve(bends);

// Check ABA feasibility
std::cout << "ABA Analysis:" << std::endl;
for (const auto& aba : output.abaConstraints) {
    std::cout << "  Bend " << aba.bendId << ": ";
    if (aba.feasible) {
        std::cout << "OK (" << aba.totalSegments << " segments)" << std::endl;
    } else {
        std::cout << "INFEASIBLE - " << aba.reason << std::endl;
    }
}

// Check grasp constraints
std::cout << "Grasp Analysis:" << std::endl;
for (const auto& grasp : output.graspConstraints) {
    std::cout << "  State " << grasp.stateId << ": ";
    if (grasp.hasValidGrip) {
        std::cout << "Valid grip area: " << grasp.validArea << " mm²" << std::endl;
    } else {
        std::cout << "NO VALID GRIP" << std::endl;
    }
}
```

## Performance Characteristics

### Complexity
- **Geometric Analysis**: O(n²) cho n bends (pairwise checks)
- **Grasp Analysis**: O(k) cho k states
- **ABA Analysis**: O(n) cho n bends
- **Graph Operations**: O(V + E) với V = bends, E = constraints
- **Overall**: O(n²) dominated by geometric analysis

### Typical Performance (Debug Mode)
- 2 bends: < 1 ms
- 5 bends: < 5 ms
- 10 bends: < 50 ms
- 20 bends: < 500 ms

### Memory Usage
- Minimal: ~2KB for typical 7-bend part
- Scales linearly with number of bends and constraints

## Best Practices

### 1. Input Validation
- Ensure bends have unique IDs
- Provide geometry information (position, direction)
- Use reasonable bend angles (0-180°)

### 2. Error Handling
- Always check `output.success` before using results
- Log errors and warnings for debugging
- Review analysis summary for insights

### 3. Performance
- Reuse solver instance when possible
- Call `reset()` between unrelated solves
- Monitor statistics for performance issues

### 4. Debugging
- Check `output.analysisSummary` for detailed info
- Use statistics to identify bottlenecks
- Validate precedence graph is acyclic

## Troubleshooting

### Problem: Solver returns success = false

**Possible causes:**
1. Cyclic dependencies in precedence graph
2. Graph finalization failed

**Solution:**
- Check `output.errors` for specific error messages
- Review bend geometry for mutual blocking
- Adjust bend positions to avoid conflicts

### Problem: Empty bend sequence

**Causes:**
- Graph has cycles
- No valid topological order exists

**Solution:**
- Review geometric constraints
- Check for circular dependencies
- Adjust bend arrangement

### Problem: Warnings about infeasible bends

**Causes:**
- ABA tool segments cannot cover required width
- Box closing scenarios

**Solution:**
- Check `aba.reason` for specifics
- Consider alternative tool configurations
- Modify bend sequence manually if needed

### Problem: No valid grip regions

**Causes:**
- Standing flanges occupy entire base plane
- Dead zones too large

**Solution:**
- Review `graspConstraints` for specific states
- Consider different bend sequence
- Adjust flange dimensions

## Advanced Usage

### Multiple Solver Instances

```cpp
// Independent solvers for parallel processing
ConstraintSolver solver1;
ConstraintSolver solver2;

auto result1 = solver1.solve(part1_bends);
auto result2 = solver2.solve(part2_bends);
```

### Solver Reuse

```cpp
ConstraintSolver solver;

// First part
auto output1 = solver.solve(part1_bends);

// Reset for next part
solver.reset();

// Second part
auto output2 = solver.solve(part2_bends);
```

## See Also

- GeometricPrecedenceAnalyzer documentation
- GraspConstraintGenerator documentation
- ABAConstraintAnalyzer documentation
- PrecedenceDAG API reference
