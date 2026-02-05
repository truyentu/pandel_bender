# GeometricPrecedenceAnalyzer - Usage Guide

## Overview

`GeometricPrecedenceAnalyzer` phát hiện các ràng buộc thứ tự (precedence constraints) giữa các bends dựa trên geometric conflicts.

## Quick Start

```cpp
#include "openpanelcam/phase2/geometric_precedence_analyzer.h"

using namespace openpanelcam::phase2;
using namespace openpanelcam::phase1;

// Create analyzer
GeometricPrecedenceAnalyzer analyzer;

// Create bend features (từ Phase 1)
std::vector<BendFeature> bends;

BendFeature b0;
b0.id = 0;
b0.angle = 90.0;
b0.length = 100.0;
b0.position = {0.0, 0.0, 0.0};
b0.direction = {0.0, 1.0, 0.0};  // Along Y-axis
b0.normal = {0.0, 0.0, 1.0};     // Rotate around Z

bends.push_back(b0);

// Analyze all bends
auto constraints = analyzer.analyze(bends);

// Get statistics
auto stats = analyzer.getStatistics();
std::cout << "Total constraints: " << stats.totalConstraints << std::endl;
std::cout << "Analysis time: " << stats.analysisTimeMs << " ms" << std::endl;
```

## Constraint Types Detected

### 1. GEOMETRIC - Corner Overlap

**Trigger:** Khi corner của flange bi sẽ va chạm với flange bj trong quá trình bending.

**Example:**
```cpp
// Two bends close together
BendFeature b0, b1;
b0.position.x = 0.0;
b1.position.x = 10.0;  // 10mm apart

// If corners overlap → GEOMETRIC constraint
// b0 must bend before b1 (or vice versa)
```

**Algorithm:** Ray casting từ 4 góc của flange theo motion path.

**Confidence:** 1.0

### 2. BOX_CLOSING - Tool Trap

**Trigger:** Khi 3 bends đã tạo thành U-shape và bend thứ 4 sẽ đóng kín hộp → tool bị kẹt bên trong.

**Example:**
```cpp
// U-shaped configuration
//  Bend 0 (left)    Bend 2 (right)
//      |               |
//      +---------------+
//         Bend 1 (bottom)

// If Bend 3 (top) is added → BOX_CLOSING detected
```

**Algorithm:**
1. Detect 3-sided box (3 bends at 90°)
2. Check if next bend would close 4th side

**Confidence:** 1.0

### 3. SEQUENTIAL - Access Blocking

**Trigger:** Khi bending bj trước sẽ chắn tool không thể access bi.

**Example:**
```cpp
// Two parallel bends close together
BendFeature bi, bj;
bi.position.x = 0.0;
bj.position.x = 40.0;  // 40mm apart
// Both parallel (same direction)

// If bj bent first → blocks access to bi
// bi must bend first
```

**Algorithm:**
- Distance check (< 60mm access zone)
- Parallelism check (dot product > 0.9)

**Confidence:** 0.9

## API Reference

### Main Methods

#### `analyze(bends)`
```cpp
std::vector<PrecedenceEdge> analyze(
    const std::vector<BendFeature>& bends
);
```

**Input:** Vector of bend features từ Phase 1

**Output:** Vector of precedence edges (constraints)

**Complexity:** O(n²) where n = number of bends

**Example:**
```cpp
auto constraints = analyzer.analyze(bends);

for (const auto& edge : constraints) {
    std::cout << "Constraint: Bend " << edge.fromBend
              << " → Bend " << edge.toBend << std::endl;
    std::cout << "Type: " << static_cast<int>(edge.type) << std::endl;
    std::cout << "Reason: " << edge.reasoning << std::endl;
}
```

#### `getStatistics()`
```cpp
const Statistics& getStatistics() const;
```

**Returns:** Statistics struct with metrics:

```cpp
struct Statistics {
    int totalPairsChecked;        // Number of pairs analyzed
    int cornerOverlapCount;       // GEOMETRIC constraints
    int boxClosingCount;          // BOX_CLOSING detections
    int sequentialBlockCount;     // SEQUENTIAL constraints
    int totalConstraints;         // Sum of all constraints

    // Performance metrics
    double analysisTimeMs;        // Total time in milliseconds
    double avgPairTimeMs;         // Average time per pair
    int maxConstraintsPerPair;    // Max constraints from single pair
};
```

### Helper Methods (Advanced)

#### `checkCornerOverlap(bi, bj, state)`
```cpp
bool checkCornerOverlap(
    const BendFeature& bi,
    const BendFeature& bj,
    const BentState& state
);
```

Test xem bi có corner overlap với bj không.

#### `isBoxClosing(bend, state)`
```cpp
bool isBoxClosing(
    const BendFeature& bend,
    const BentState& state
);
```

Test xem bend này có đóng kín box không.

#### `isBlocked(bi, bj, state)`
```cpp
bool isBlocked(
    const BendFeature& bi,
    const BendFeature& bj,
    const BentState& state
);
```

Test xem bj có block access đến bi không.

## BentState Helper

Track trạng thái bends nào đã được bending.

```cpp
BentState state;

state.applyBend(0);     // Mark bend 0 as bent
state.applyBend(1);     // Mark bend 1 as bent

bool isBent = state.isBent(0);  // true
int count = state.count();       // 2

state.reset();          // Reset to flat state
```

## Performance Characteristics

### Time Complexity

- **analyze():** O(n²) - checks all pairs
- **checkCornerOverlap():** O(1) - simplified geometry
- **isBoxClosing():** O(n) - checks bent bends
- **isBlocked():** O(1) - distance + dot product

### Space Complexity

- O(n) for nodes
- O(k) for constraints (k ≤ n²)

### Typical Performance

| Bends | Pairs | Time (Debug) | Time (Release) |
|-------|-------|--------------|----------------|
| 2 | 2 | < 0.1 ms | < 0.01 ms |
| 4 | 12 | < 0.5 ms | < 0.05 ms |
| 8 | 56 | < 2 ms | < 0.2 ms |
| 16 | 240 | < 10 ms | < 1 ms |

*Measured on simplified geometry (mock BendFeature)*

## Common Patterns

### Pattern 1: Simple L-Bracket (No Constraints)

```cpp
// Two perpendicular bends far apart
BendFeature b0, b1;
b0.position = {0, 0, 0};
b0.direction = {0, 1, 0};  // Y-axis

b1.position = {100, 50, 0};  // Far away
b1.direction = {1, 0, 0};    // X-axis (perpendicular)

auto constraints = analyzer.analyze({b0, b1});
// constraints.empty() == true (no conflicts)
```

### Pattern 2: Parallel Close Bends (Sequential Blocking)

```cpp
// Two parallel bends close together
BendFeature b0, b1;
b0.position = {0, 0, 0};
b0.direction = {0, 1, 0};

b1.position = {35, 0, 0};    // 35mm apart
b1.direction = {0, 1, 0};    // Parallel

auto constraints = analyzer.analyze({b0, b1});
// Will generate SEQUENTIAL constraint
```

### Pattern 3: U-Channel (3 Bends)

```cpp
// Left, Bottom, Right
BendFeature left, bottom, right;

left.position = {0, 50, 0};
bottom.position = {100, 0, 0};
right.position = {200, 50, 0};

auto constraints = analyzer.analyze({left, bottom, right});
// May have some constraints depending on spacing
```

## Best Practices

1. **Always check statistics** để hiểu analysis results:
   ```cpp
   auto stats = analyzer.getStatistics();
   if (stats.totalConstraints == 0) {
       // No conflicts - any order is valid
   }
   ```

2. **Monitor performance** với complex parts:
   ```cpp
   if (stats.analysisTimeMs > 100.0) {
       std::cout << "Warning: Slow analysis for "
                 << bends.size() << " bends" << std::endl;
   }
   ```

3. **Validate constraint confidence**:
   ```cpp
   for (const auto& edge : constraints) {
       if (edge.confidence < 0.95) {
           // Low confidence - might need manual review
       }
   }
   ```

4. **Check for cycles** sau khi có constraints:
   ```cpp
   PrecedenceDAG dag;
   for (const auto& bend : bends) {
       dag.addNode(bend.id);
   }
   for (const auto& edge : constraints) {
       dag.addEdge(edge.fromBend, edge.toBend, edge.type,
                   edge.confidence, edge.reasoning);
   }
   dag.finalize();

   if (!dag.isAcyclic()) {
       // Circular dependency detected!
       auto cycles = dag.detectCycles();
   }
   ```

## Limitations (Current Implementation)

1. **Simplified Geometry:** Sử dụng mock geometry (Point3D) thay vì real OCCT shapes
2. **2D Analysis:** Chỉ check 2D projections, chưa có full 3D collision detection
3. **Conservative Detection:** Có thể có false positives (over-constraining)
4. **No Material Properties:** Chưa xét springback, material thickness variations

## Future Enhancements (with Phase 1 Integration)

1. **Real 3D Geometry:** Use OCCT TopoDS_Face for actual flange shapes
2. **Precise Ray Casting:** BRepIntCurveSurface_Inter for accurate intersection
3. **Advanced Box Detection:** Topological analysis of bent shape connectivity
4. **Clearance Calculation:** Precise tool access zone based on actual machine specs

## Troubleshooting

### Q: Too many constraints detected?

A: Giảm tolerance trong `rayIntersectsFlange()` hoặc tăng access zone radius.

### Q: Missing obvious conflicts?

A: Check bend positions và directions. Ensure geometry data đúng từ Phase 1.

### Q: Performance too slow?

A: Current O(n²) là optimal cho pairwise check. Có thể optimize bằng spatial partitioning nếu n > 50.

### Q: Circular dependencies?

A: Use `PrecedenceDAG::detectCycles()` để identify conflicting constraints. May need manual resolution.

## See Also

- `PrecedenceDAG` - DAG construction and topological sort
- `BentState` - Bend state simulation
- Phase 1 documentation - Bend feature extraction
