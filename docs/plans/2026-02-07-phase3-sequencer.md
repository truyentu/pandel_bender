# Phase 3: Sequencer Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Implement A* search algorithm with Masked Time cost function to find optimal bend sequences, including Zobrist hashing for state deduplication and integrated repositioning planning.

**Architecture:** Build a search engine that explores bend sequences respecting Phase 2 precedence constraints. Use Zobrist hashing for O(1) state lookup, Masked Time cost function to model parallel machine operations, and three research-backed heuristics. Integrate repo detection into the search rather than as a post-process.

**Tech Stack:** C++17, STL containers (priority_queue, unordered_map), Phase 2 output (PrecedenceDAG, GraspConstraints, ABAConstraints)

---

## Prerequisites

- Phase 2 complete (40/40 tasks)
- All Phase 2 tests passing
- Working directory: `E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller`

---

## Module Overview

```
Phase 3 Sequencer
├── Core Data Structures (Tasks 1-5)
│   ├── SearchState
│   ├── SearchNode
│   ├── ZobristTable
│   └── Phase3Output
│
├── Cost Functions (Tasks 6-10)
│   ├── MaskedTimeCost
│   ├── BendTimeEstimator
│   └── RepoCostModel
│
├── Heuristics (Tasks 11-15)
│   ├── RotationalEntropyHeuristic
│   ├── ToolingVarianceHeuristic
│   ├── GraspFragmentationHeuristic
│   └── CombinedHeuristic
│
├── A* Search Engine (Tasks 16-25)
│   ├── SuccessorGenerator
│   ├── PriorityQueue
│   ├── StateDeduplication
│   ├── GoalTesting
│   └── PathReconstruction
│
├── Repo Integration (Tasks 26-30)
│   ├── RepoTriggerDetector
│   ├── RepoAction
│   └── RepoAsSearchAction
│
├── Sequencer Class (Tasks 31-35)
│   ├── Sequencer main class
│   ├── Phase2 integration
│   └── Phase3Output generation
│
└── Testing & Documentation (Tasks 36-40)
    ├── Unit tests
    ├── Integration tests
    ├── Performance benchmarks
    └── Documentation
```

---

## Task 1: Core Enum Types

**Goal:** Define orientation and action type enums for Phase 3.

**Files:**
- Create: `include/openpanelcam/phase3/types.h`
- Create: `tests/phase3/test_types.cpp`
- Modify: `tests/phase3/CMakeLists.txt`

### Step 1.1: Create Phase 3 types header

```cpp
// include/openpanelcam/phase3/types.h
#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Part orientation on the machine (0°, 90°, 180°, 270°)
 */
enum class Orientation : uint8_t {
    DEG_0 = 0,
    DEG_90 = 1,
    DEG_180 = 2,
    DEG_270 = 3
};

/**
 * @brief Type of action in the sequence
 */
enum class ActionType {
    BEND,           // Perform a bend
    ROTATE,         // Rotate part
    ABA_RECONFIG,   // Reconfigure ABA segments
    REPOSITION      // Reposition (re-grip) part
};

/**
 * @brief Reason for repositioning
 */
enum class RepoReason {
    NONE,
    GRIP_AREA_EXHAUSTED,
    BOX_CLOSING,
    COM_OUTSIDE_GRIP,
    NO_VALID_GRIP
};

/**
 * @brief Convert orientation to degrees
 */
inline double orientationToDegrees(Orientation o) {
    return static_cast<double>(static_cast<uint8_t>(o)) * 90.0;
}

/**
 * @brief Get next orientation (rotate 90° clockwise)
 */
inline Orientation rotateClockwise(Orientation o) {
    return static_cast<Orientation>((static_cast<uint8_t>(o) + 1) % 4);
}

/**
 * @brief Get opposite orientation (rotate 180°)
 */
inline Orientation rotateOpposite(Orientation o) {
    return static_cast<Orientation>((static_cast<uint8_t>(o) + 2) % 4);
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 1.2: Create test file

```cpp
// tests/phase3/test_types.cpp
#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/types.h"

using namespace openpanelcam::phase3;

TEST_CASE("Orientation enum values", "[phase3][types]") {
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_0) == 0);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_90) == 1);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_180) == 2);
    REQUIRE(static_cast<uint8_t>(Orientation::DEG_270) == 3);
}

TEST_CASE("orientationToDegrees converts correctly", "[phase3][types]") {
    REQUIRE(orientationToDegrees(Orientation::DEG_0) == 0.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_90) == 90.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_180) == 180.0);
    REQUIRE(orientationToDegrees(Orientation::DEG_270) == 270.0);
}

TEST_CASE("rotateClockwise cycles through orientations", "[phase3][types]") {
    REQUIRE(rotateClockwise(Orientation::DEG_0) == Orientation::DEG_90);
    REQUIRE(rotateClockwise(Orientation::DEG_90) == Orientation::DEG_180);
    REQUIRE(rotateClockwise(Orientation::DEG_180) == Orientation::DEG_270);
    REQUIRE(rotateClockwise(Orientation::DEG_270) == Orientation::DEG_0);
}

TEST_CASE("rotateOpposite returns 180 degree rotation", "[phase3][types]") {
    REQUIRE(rotateOpposite(Orientation::DEG_0) == Orientation::DEG_180);
    REQUIRE(rotateOpposite(Orientation::DEG_90) == Orientation::DEG_270);
    REQUIRE(rotateOpposite(Orientation::DEG_180) == Orientation::DEG_0);
    REQUIRE(rotateOpposite(Orientation::DEG_270) == Orientation::DEG_90);
}

TEST_CASE("ActionType enum exists", "[phase3][types]") {
    ActionType bend = ActionType::BEND;
    ActionType rotate = ActionType::ROTATE;
    ActionType aba = ActionType::ABA_RECONFIG;
    ActionType repo = ActionType::REPOSITION;

    REQUIRE(bend != rotate);
    REQUIRE(rotate != aba);
    REQUIRE(aba != repo);
}

TEST_CASE("RepoReason enum exists", "[phase3][types]") {
    REQUIRE(RepoReason::NONE != RepoReason::GRIP_AREA_EXHAUSTED);
    REQUIRE(RepoReason::BOX_CLOSING != RepoReason::COM_OUTSIDE_GRIP);
}
```

### Step 1.3: Create CMakeLists.txt for Phase 3 tests

```cmake
# tests/phase3/CMakeLists.txt
find_package(Catch2 3 REQUIRED)

# Test for types
add_executable(test_phase3_types test_types.cpp)
target_link_libraries(test_phase3_types PRIVATE Catch2::Catch2WithMain)
target_include_directories(test_phase3_types PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
add_test(NAME test_phase3_types COMMAND test_phase3_types)
```

### Step 1.4: Update main tests CMakeLists.txt

Add to `tests/CMakeLists.txt`:
```cmake
add_subdirectory(phase3)
```

### Step 1.5: Run test to verify

```bash
cd build && cmake .. && cmake --build . --target test_phase3_types && ./bin/test_phase3_types
```

Expected: PASS - All 6 test cases pass

### Step 1.6: Commit

```bash
git add include/openpanelcam/phase3/types.h tests/phase3/test_types.cpp tests/phase3/CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat(phase3): add core enum types (Orientation, ActionType, RepoReason)"
```

---

## Task 2: SearchState Structure

**Goal:** Define the search state encoding with bentMask, orientation, ABA config.

**Files:**
- Modify: `include/openpanelcam/phase3/types.h`
- Modify: `tests/phase3/test_types.cpp`

### Step 2.1: Add SearchState to types.h

```cpp
// Add to include/openpanelcam/phase3/types.h

#include "../phase2/types.h"  // For Point2D

/**
 * @brief Search state in A* algorithm
 *
 * Encodes the complete state of the bending process:
 * - Which bends are completed (bitmask)
 * - Current part orientation
 * - Current ABA segment configuration
 * - Current grip center position
 * - Whether repo is needed
 */
struct SearchState {
    uint32_t bentMask = 0;          // Bitmask of completed bends (max 32 bends)
    Orientation orientation = Orientation::DEG_0;
    uint16_t abaConfig = 0;         // Encoded ABA segment configuration
    phase2::Point2D gripCenter;     // Current grip point on base
    bool needsRepo = false;         // Repositioning required flag
    RepoReason repoReason = RepoReason::NONE;

    /**
     * @brief Check if bend is completed
     */
    bool isBent(int bendId) const {
        if (bendId < 0 || bendId >= 32) return false;
        return (bentMask & (1u << bendId)) != 0;
    }

    /**
     * @brief Mark bend as completed
     */
    void markBent(int bendId) {
        if (bendId >= 0 && bendId < 32) {
            bentMask |= (1u << bendId);
        }
    }

    /**
     * @brief Count number of completed bends
     */
    int bentCount() const {
        int count = 0;
        uint32_t mask = bentMask;
        while (mask) {
            count += mask & 1;
            mask >>= 1;
        }
        return count;
    }

    /**
     * @brief Check if all bends completed
     */
    bool isGoal(int totalBends) const {
        if (totalBends <= 0 || totalBends > 32) return false;
        uint32_t goalMask = (1u << totalBends) - 1;
        return bentMask == goalMask;
    }

    /**
     * @brief Equality comparison for state deduplication
     */
    bool operator==(const SearchState& other) const {
        return bentMask == other.bentMask &&
               orientation == other.orientation &&
               abaConfig == other.abaConfig;
        // Note: gripCenter and needsRepo not included in equality
        // because they're derived from bentMask
    }
};
```

### Step 2.2: Add SearchState tests

```cpp
// Add to tests/phase3/test_types.cpp

TEST_CASE("SearchState default initialization", "[phase3][types][state]") {
    SearchState state;

    REQUIRE(state.bentMask == 0);
    REQUIRE(state.orientation == Orientation::DEG_0);
    REQUIRE(state.abaConfig == 0);
    REQUIRE(state.needsRepo == false);
    REQUIRE(state.repoReason == RepoReason::NONE);
}

TEST_CASE("SearchState isBent checks bitmask", "[phase3][types][state]") {
    SearchState state;
    state.bentMask = 0b1010;  // Bends 1 and 3 are done

    REQUIRE(state.isBent(0) == false);
    REQUIRE(state.isBent(1) == true);
    REQUIRE(state.isBent(2) == false);
    REQUIRE(state.isBent(3) == true);
    REQUIRE(state.isBent(4) == false);

    // Edge cases
    REQUIRE(state.isBent(-1) == false);
    REQUIRE(state.isBent(32) == false);
}

TEST_CASE("SearchState markBent sets bit", "[phase3][types][state]") {
    SearchState state;

    state.markBent(0);
    REQUIRE(state.bentMask == 0b0001);

    state.markBent(2);
    REQUIRE(state.bentMask == 0b0101);

    state.markBent(2);  // Already set, no change
    REQUIRE(state.bentMask == 0b0101);

    // Edge cases - should not crash
    state.markBent(-1);
    state.markBent(32);
    REQUIRE(state.bentMask == 0b0101);
}

TEST_CASE("SearchState bentCount counts bits", "[phase3][types][state]") {
    SearchState state;

    REQUIRE(state.bentCount() == 0);

    state.bentMask = 0b1010;
    REQUIRE(state.bentCount() == 2);

    state.bentMask = 0b11111111;
    REQUIRE(state.bentCount() == 8);

    state.bentMask = 0xFFFFFFFF;
    REQUIRE(state.bentCount() == 32);
}

TEST_CASE("SearchState isGoal checks completion", "[phase3][types][state]") {
    SearchState state;

    // 3 bends total
    REQUIRE(state.isGoal(3) == false);

    state.bentMask = 0b011;  // Only 2 done
    REQUIRE(state.isGoal(3) == false);

    state.bentMask = 0b111;  // All 3 done
    REQUIRE(state.isGoal(3) == true);

    // Edge cases
    REQUIRE(state.isGoal(0) == false);
    REQUIRE(state.isGoal(-1) == false);
    REQUIRE(state.isGoal(33) == false);
}

TEST_CASE("SearchState equality ignores grip", "[phase3][types][state]") {
    SearchState s1, s2;

    s1.bentMask = 0b101;
    s1.orientation = Orientation::DEG_90;
    s1.abaConfig = 42;
    s1.gripCenter = phase2::Point2D(100, 200);

    s2.bentMask = 0b101;
    s2.orientation = Orientation::DEG_90;
    s2.abaConfig = 42;
    s2.gripCenter = phase2::Point2D(999, 999);  // Different grip

    REQUIRE(s1 == s2);  // Still equal

    s2.bentMask = 0b111;
    REQUIRE(!(s1 == s2));  // Now different
}
```

### Step 2.3: Run test

```bash
cd build && cmake --build . --target test_phase3_types && ./bin/test_phase3_types
```

Expected: PASS

### Step 2.4: Commit

```bash
git add include/openpanelcam/phase3/types.h tests/phase3/test_types.cpp
git commit -m "feat(phase3): add SearchState structure with bitmask operations"
```

---

## Task 3: SearchNode Structure

**Goal:** Define the A* search node with f, g, h scores and parent tracking.

**Files:**
- Modify: `include/openpanelcam/phase3/types.h`
- Modify: `tests/phase3/test_types.cpp`

### Step 3.1: Add SearchNode to types.h

```cpp
// Add to include/openpanelcam/phase3/types.h

/**
 * @brief A* search node with cost scores
 */
struct SearchNode {
    SearchState state;
    double g = 0.0;     // Cost from start to this node
    double h = 0.0;     // Heuristic estimate to goal
    int parentId = -1;  // Index of parent node in closed set
    int lastBendId = -1; // Last bend performed to reach this state
    ActionType lastAction = ActionType::BEND;

    /**
     * @brief Total estimated cost (f = g + h)
     */
    double f() const {
        return g + h;
    }

    /**
     * @brief Comparison for priority queue (min-heap by f)
     */
    bool operator>(const SearchNode& other) const {
        return f() > other.f();
    }

    /**
     * @brief Create initial node (flat state)
     */
    static SearchNode createInitial() {
        SearchNode node;
        node.state = SearchState();
        node.g = 0.0;
        node.h = 0.0;
        node.parentId = -1;
        node.lastBendId = -1;
        return node;
    }
};

/**
 * @brief Action performed in sequence
 */
struct SequenceAction {
    ActionType type;
    int bendId = -1;            // For BEND action
    Orientation newOrientation; // For ROTATE action
    uint16_t newAbaConfig = 0;  // For ABA_RECONFIG action
    phase2::Point2D newGripCenter; // For REPOSITION action
    double duration = 0.0;      // Estimated time in seconds
    std::string description;
};
```

### Step 3.2: Add SearchNode tests

```cpp
// Add to tests/phase3/test_types.cpp

TEST_CASE("SearchNode default values", "[phase3][types][node]") {
    SearchNode node;

    REQUIRE(node.g == 0.0);
    REQUIRE(node.h == 0.0);
    REQUIRE(node.f() == 0.0);
    REQUIRE(node.parentId == -1);
    REQUIRE(node.lastBendId == -1);
}

TEST_CASE("SearchNode f() computes g + h", "[phase3][types][node]") {
    SearchNode node;
    node.g = 10.5;
    node.h = 5.3;

    REQUIRE(node.f() == Catch::Approx(15.8));
}

TEST_CASE("SearchNode comparison for min-heap", "[phase3][types][node]") {
    SearchNode low, high;
    low.g = 5.0;
    low.h = 2.0;  // f = 7

    high.g = 10.0;
    high.h = 3.0;  // f = 13

    REQUIRE(high > low);
    REQUIRE(!(low > high));
}

TEST_CASE("SearchNode createInitial returns flat state", "[phase3][types][node]") {
    auto node = SearchNode::createInitial();

    REQUIRE(node.state.bentMask == 0);
    REQUIRE(node.state.orientation == Orientation::DEG_0);
    REQUIRE(node.g == 0.0);
    REQUIRE(node.h == 0.0);
    REQUIRE(node.parentId == -1);
    REQUIRE(node.lastBendId == -1);
}

TEST_CASE("SequenceAction stores action details", "[phase3][types][action]") {
    SequenceAction action;
    action.type = ActionType::BEND;
    action.bendId = 3;
    action.duration = 2.5;
    action.description = "Bend flange 3 at 90 degrees";

    REQUIRE(action.type == ActionType::BEND);
    REQUIRE(action.bendId == 3);
    REQUIRE(action.duration == Catch::Approx(2.5));
}
```

### Step 3.3: Run test

```bash
cd build && cmake --build . --target test_phase3_types && ./bin/test_phase3_types
```

Expected: PASS

### Step 3.4: Commit

```bash
git add include/openpanelcam/phase3/types.h tests/phase3/test_types.cpp
git commit -m "feat(phase3): add SearchNode and SequenceAction structures"
```

---

## Task 4: ZobristTable for State Hashing

**Goal:** Implement Zobrist hashing for O(1) state deduplication.

**Files:**
- Create: `include/openpanelcam/phase3/zobrist_table.h`
- Create: `src/phase3/zobrist_table.cpp`
- Create: `tests/phase3/test_zobrist.cpp`
- Create: `src/phase3/CMakeLists.txt`

### Step 4.1: Create ZobristTable header

```cpp
// include/openpanelcam/phase3/zobrist_table.h
#pragma once

#include <cstdint>
#include <array>
#include <random>
#include "types.h"

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Zobrist hashing table for fast state deduplication
 *
 * Precomputes random 64-bit values for each state component.
 * Hash = XOR of all relevant component hashes.
 *
 * Enables O(1) duplicate state detection in A* search.
 */
class ZobristTable {
public:
    static constexpr int MAX_BENDS = 32;
    static constexpr int NUM_ORIENTATIONS = 4;
    static constexpr int ABA_CONFIG_BUCKETS = 256;  // Hash ABA config to buckets

    /**
     * @brief Initialize with random values
     * @param seed Random seed for reproducibility (0 = random)
     */
    explicit ZobristTable(uint64_t seed = 0);

    /**
     * @brief Compute hash for a search state
     */
    uint64_t hash(const SearchState& state) const;

    /**
     * @brief Incrementally update hash after bending
     * @param oldHash Previous hash value
     * @param bendId Bend that was just completed
     * @return New hash value
     */
    uint64_t updateBend(uint64_t oldHash, int bendId) const;

    /**
     * @brief Incrementally update hash after rotation
     */
    uint64_t updateOrientation(uint64_t oldHash,
                               Orientation oldOri,
                               Orientation newOri) const;

    /**
     * @brief Incrementally update hash after ABA reconfig
     */
    uint64_t updateAba(uint64_t oldHash,
                       uint16_t oldConfig,
                       uint16_t newConfig) const;

private:
    std::array<uint64_t, MAX_BENDS> m_bendHashes;
    std::array<uint64_t, NUM_ORIENTATIONS> m_orientationHashes;
    std::array<uint64_t, ABA_CONFIG_BUCKETS> m_abaHashes;

    uint16_t abaConfigToBucket(uint16_t config) const;
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 4.2: Create ZobristTable implementation

```cpp
// src/phase3/zobrist_table.cpp
#include "openpanelcam/phase3/zobrist_table.h"

namespace openpanelcam {
namespace phase3 {

ZobristTable::ZobristTable(uint64_t seed) {
    // Use seed or random device
    std::mt19937_64 rng;
    if (seed == 0) {
        std::random_device rd;
        rng.seed(rd());
    } else {
        rng.seed(seed);
    }

    // Generate random values for each bend position
    for (int i = 0; i < MAX_BENDS; i++) {
        m_bendHashes[i] = rng();
    }

    // Generate random values for each orientation
    for (int i = 0; i < NUM_ORIENTATIONS; i++) {
        m_orientationHashes[i] = rng();
    }

    // Generate random values for ABA config buckets
    for (int i = 0; i < ABA_CONFIG_BUCKETS; i++) {
        m_abaHashes[i] = rng();
    }
}

uint64_t ZobristTable::hash(const SearchState& state) const {
    uint64_t h = 0;

    // XOR in each bent bend
    for (int i = 0; i < MAX_BENDS; i++) {
        if (state.isBent(i)) {
            h ^= m_bendHashes[i];
        }
    }

    // XOR in orientation
    h ^= m_orientationHashes[static_cast<uint8_t>(state.orientation)];

    // XOR in ABA config (bucketed)
    h ^= m_abaHashes[abaConfigToBucket(state.abaConfig)];

    return h;
}

uint64_t ZobristTable::updateBend(uint64_t oldHash, int bendId) const {
    if (bendId < 0 || bendId >= MAX_BENDS) return oldHash;
    return oldHash ^ m_bendHashes[bendId];
}

uint64_t ZobristTable::updateOrientation(uint64_t oldHash,
                                         Orientation oldOri,
                                         Orientation newOri) const {
    // XOR out old, XOR in new
    uint64_t h = oldHash;
    h ^= m_orientationHashes[static_cast<uint8_t>(oldOri)];
    h ^= m_orientationHashes[static_cast<uint8_t>(newOri)];
    return h;
}

uint64_t ZobristTable::updateAba(uint64_t oldHash,
                                 uint16_t oldConfig,
                                 uint16_t newConfig) const {
    uint64_t h = oldHash;
    h ^= m_abaHashes[abaConfigToBucket(oldConfig)];
    h ^= m_abaHashes[abaConfigToBucket(newConfig)];
    return h;
}

uint16_t ZobristTable::abaConfigToBucket(uint16_t config) const {
    return config % ABA_CONFIG_BUCKETS;
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 4.3: Create ZobristTable tests

```cpp
// tests/phase3/test_zobrist.cpp
#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/zobrist_table.h"
#include <unordered_set>

using namespace openpanelcam::phase3;

TEST_CASE("ZobristTable produces consistent hashes", "[phase3][zobrist]") {
    ZobristTable table(12345);  // Fixed seed

    SearchState state;
    state.bentMask = 0b1010;
    state.orientation = Orientation::DEG_90;
    state.abaConfig = 100;

    uint64_t hash1 = table.hash(state);
    uint64_t hash2 = table.hash(state);

    REQUIRE(hash1 == hash2);
}

TEST_CASE("ZobristTable different states have different hashes", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.bentMask = 0b1010;
    s2.bentMask = 0b1011;  // One more bend

    REQUIRE(table.hash(s1) != table.hash(s2));
}

TEST_CASE("ZobristTable orientation changes hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.orientation = Orientation::DEG_0;
    s2.orientation = Orientation::DEG_90;

    REQUIRE(table.hash(s1) != table.hash(s2));
}

TEST_CASE("ZobristTable incremental update equals full hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState before;
    before.bentMask = 0b1010;
    before.orientation = Orientation::DEG_0;

    uint64_t hashBefore = table.hash(before);

    // Apply bend 2
    SearchState after = before;
    after.markBent(2);

    uint64_t hashFull = table.hash(after);
    uint64_t hashIncremental = table.updateBend(hashBefore, 2);

    REQUIRE(hashFull == hashIncremental);
}

TEST_CASE("ZobristTable orientation update is incremental", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState state;
    state.orientation = Orientation::DEG_0;

    uint64_t h1 = table.hash(state);

    state.orientation = Orientation::DEG_90;
    uint64_t h2Full = table.hash(state);

    uint64_t h2Inc = table.updateOrientation(h1, Orientation::DEG_0, Orientation::DEG_90);

    REQUIRE(h2Full == h2Inc);
}

TEST_CASE("ZobristTable low collision rate", "[phase3][zobrist]") {
    ZobristTable table(12345);
    std::unordered_set<uint64_t> hashes;

    // Generate 1000 different states
    for (int mask = 0; mask < 256; mask++) {
        for (int ori = 0; ori < 4; ori++) {
            SearchState state;
            state.bentMask = mask;
            state.orientation = static_cast<Orientation>(ori);
            hashes.insert(table.hash(state));
        }
    }

    // Should have very few collisions (ideally 1024 unique hashes)
    // Allow up to 1% collision rate
    REQUIRE(hashes.size() >= 1000);
}

TEST_CASE("ZobristTable ABA config affects hash", "[phase3][zobrist]") {
    ZobristTable table(12345);

    SearchState s1, s2;
    s1.abaConfig = 100;
    s2.abaConfig = 200;

    REQUIRE(table.hash(s1) != table.hash(s2));
}
```

### Step 4.4: Create Phase 3 source CMakeLists.txt

```cmake
# src/phase3/CMakeLists.txt
add_library(openpanelcam_phase3 STATIC
    zobrist_table.cpp
)

target_include_directories(openpanelcam_phase3 PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(openpanelcam_phase3 PUBLIC
    openpanelcam_phase2
)
```

### Step 4.5: Update tests CMakeLists.txt

```cmake
# Add to tests/phase3/CMakeLists.txt

add_executable(test_zobrist test_zobrist.cpp)
target_link_libraries(test_zobrist PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase3
)
add_test(NAME test_zobrist COMMAND test_zobrist)
```

### Step 4.6: Update main src CMakeLists.txt

Add to `src/CMakeLists.txt`:
```cmake
add_subdirectory(phase3)
```

### Step 4.7: Run test

```bash
cd build && cmake .. && cmake --build . --target test_zobrist && ./bin/test_zobrist
```

Expected: PASS

### Step 4.8: Commit

```bash
git add include/openpanelcam/phase3/zobrist_table.h src/phase3/zobrist_table.cpp src/phase3/CMakeLists.txt tests/phase3/test_zobrist.cpp tests/phase3/CMakeLists.txt src/CMakeLists.txt
git commit -m "feat(phase3): implement Zobrist hashing for O(1) state deduplication"
```

---

## Task 5: Phase3Output Structure

**Goal:** Define the output structure for Phase 3 sequencer.

**Files:**
- Modify: `include/openpanelcam/phase3/types.h`
- Modify: `tests/phase3/test_types.cpp`

### Step 5.1: Add Phase3Output to types.h

```cpp
// Add to include/openpanelcam/phase3/types.h

/**
 * @brief Statistics from the search
 */
struct SequencerStatistics {
    int nodesExpanded = 0;
    int nodesGenerated = 0;
    int nodesPruned = 0;
    int duplicatesSkipped = 0;
    double searchTimeMs = 0.0;
    double heuristicTimeMs = 0.0;
    int maxOpenSetSize = 0;
    int solutionDepth = 0;
};

/**
 * @brief Complete output from Phase 3 Sequencer
 */
struct Phase3Output {
    // Primary result
    std::vector<int> bendSequence;          // Optimal bend order (bend IDs)
    std::vector<SequenceAction> actions;    // Detailed action sequence
    double totalCycleTime = 0.0;            // Total estimated cycle time

    // Repo information
    int repoCount = 0;                      // Number of repositions needed
    std::vector<int> repoAfterBends;        // Repo after these bends

    // Status
    bool success = false;
    bool optimal = false;                   // True if search completed
    std::string errorMessage;
    std::vector<std::string> warnings;

    // Statistics
    SequencerStatistics stats;

    // Analysis summary
    std::string generateSummary() const {
        std::string summary = "Phase 3 Sequencer Result:\n";
        summary += "  Status: " + std::string(success ? "SUCCESS" : "FAILED") + "\n";
        summary += "  Bends: " + std::to_string(bendSequence.size()) + "\n";
        summary += "  Cycle Time: " + std::to_string(totalCycleTime) + " seconds\n";
        summary += "  Repositions: " + std::to_string(repoCount) + "\n";
        summary += "  Nodes Expanded: " + std::to_string(stats.nodesExpanded) + "\n";
        summary += "  Search Time: " + std::to_string(stats.searchTimeMs) + " ms\n";
        return summary;
    }
};
```

### Step 5.2: Add Phase3Output tests

```cpp
// Add to tests/phase3/test_types.cpp

TEST_CASE("SequencerStatistics default values", "[phase3][types][stats]") {
    SequencerStatistics stats;

    REQUIRE(stats.nodesExpanded == 0);
    REQUIRE(stats.nodesGenerated == 0);
    REQUIRE(stats.searchTimeMs == 0.0);
}

TEST_CASE("Phase3Output default values", "[phase3][types][output]") {
    Phase3Output output;

    REQUIRE(output.bendSequence.empty());
    REQUIRE(output.actions.empty());
    REQUIRE(output.totalCycleTime == 0.0);
    REQUIRE(output.repoCount == 0);
    REQUIRE(output.success == false);
    REQUIRE(output.optimal == false);
}

TEST_CASE("Phase3Output generateSummary", "[phase3][types][output]") {
    Phase3Output output;
    output.success = true;
    output.bendSequence = {0, 2, 1, 3};
    output.totalCycleTime = 15.5;
    output.repoCount = 1;
    output.stats.nodesExpanded = 150;
    output.stats.searchTimeMs = 25.3;

    std::string summary = output.generateSummary();

    REQUIRE(summary.find("SUCCESS") != std::string::npos);
    REQUIRE(summary.find("4") != std::string::npos);  // 4 bends
    REQUIRE(summary.find("15.5") != std::string::npos);
    REQUIRE(summary.find("150") != std::string::npos);
}
```

### Step 5.3: Run test

```bash
cd build && cmake --build . --target test_phase3_types && ./bin/test_phase3_types
```

Expected: PASS

### Step 5.4: Commit

```bash
git add include/openpanelcam/phase3/types.h tests/phase3/test_types.cpp
git commit -m "feat(phase3): add Phase3Output and SequencerStatistics structures"
```

---

## Task 6: MaskedTimeCost Class

**Goal:** Implement the Masked Time cost function for parallel operations.

**Files:**
- Create: `include/openpanelcam/phase3/cost_function.h`
- Create: `src/phase3/cost_function.cpp`
- Create: `tests/phase3/test_cost_function.cpp`

### Step 6.1: Create cost function header

```cpp
// include/openpanelcam/phase3/cost_function.h
#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Configuration for cost function timing
 */
struct CostConfig {
    // Bend time parameters
    double baseBendTime = 2.0;      // Base time for 90° bend (seconds)
    double angleMultiplier = 0.01;  // Additional time per degree beyond 90°
    double thicknessMultiplier = 0.5; // Additional time per mm thickness

    // Parallel operation times
    double rotationTime = 1.5;      // Time for 90° rotation
    double abaReconfigTime = 0.8;   // Time for ABA segment change

    // Repo times
    double repoReleaseTime = 2.0;   // Time to release part
    double repoReorientTime = 1.5;  // Time to reorient on table
    double repoAcquireTime = 2.0;   // Time to re-grip
    double repoSafetyMargin = 1.0;  // Safety buffer

    double totalRepoTime() const {
        return repoReleaseTime + repoReorientTime + repoAcquireTime + repoSafetyMargin;
    }
};

/**
 * @brief Masked Time cost function for A* search
 *
 * Implements: StepCost = t_bend + max(t_rotation, t_aba) + t_repo
 *
 * Key insight: Rotation and ABA reconfiguration happen in PARALLEL,
 * so we use max() not sum().
 */
class MaskedTimeCost {
public:
    explicit MaskedTimeCost(const CostConfig& config = CostConfig());

    /**
     * @brief Calculate step cost from current state to next state
     *
     * @param current Current search state
     * @param nextBend Bend to perform
     * @param nextState State after bending
     * @param bend Bend feature data
     * @return Cost in seconds
     */
    double stepCost(const SearchState& current,
                    int nextBendId,
                    const SearchState& nextState,
                    const phase1::BendFeature& bend) const;

    /**
     * @brief Calculate bend time based on geometry
     */
    double bendTime(const phase1::BendFeature& bend) const;

    /**
     * @brief Calculate rotation time (0 if no rotation needed)
     */
    double rotationTime(Orientation from, Orientation to) const;

    /**
     * @brief Calculate ABA reconfiguration time (0 if no change)
     */
    double abaTime(uint16_t fromConfig, uint16_t toConfig) const;

    /**
     * @brief Calculate repo time (0 if no repo needed)
     */
    double repoTime(bool needsRepo) const;

    const CostConfig& config() const { return m_config; }

private:
    CostConfig m_config;
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 6.2: Create cost function implementation

```cpp
// src/phase3/cost_function.cpp
#include "openpanelcam/phase3/cost_function.h"
#include <algorithm>
#include <cmath>

namespace openpanelcam {
namespace phase3 {

MaskedTimeCost::MaskedTimeCost(const CostConfig& config)
    : m_config(config) {}

double MaskedTimeCost::stepCost(const SearchState& current,
                                 int nextBendId,
                                 const SearchState& nextState,
                                 const phase1::BendFeature& bend) const {
    // Core formula: StepCost = t_bend + max(t_rotation, t_aba) + t_repo

    double t_bend = bendTime(bend);
    double t_rot = rotationTime(current.orientation, nextState.orientation);
    double t_aba = abaTime(current.abaConfig, nextState.abaConfig);
    double t_repo = repoTime(nextState.needsRepo);

    // CRITICAL: Parallel operations use max(), not sum()
    double t_parallel = std::max(t_rot, t_aba);

    return t_bend + t_parallel + t_repo;
}

double MaskedTimeCost::bendTime(const phase1::BendFeature& bend) const {
    double time = m_config.baseBendTime;

    // Additional time for angles beyond 90°
    double extraAngle = std::abs(bend.angle) - 90.0;
    if (extraAngle > 0) {
        time += extraAngle * m_config.angleMultiplier;
    }

    // Note: Thickness would be added here if available in BendFeature
    // time += bend.thickness * m_config.thicknessMultiplier;

    return time;
}

double MaskedTimeCost::rotationTime(Orientation from, Orientation to) const {
    if (from == to) return 0.0;

    // Calculate minimum rotation steps (0-2)
    int fromVal = static_cast<int>(from);
    int toVal = static_cast<int>(to);
    int diff = std::abs(fromVal - toVal);
    int steps = std::min(diff, 4 - diff);

    // Each 90° step takes rotation time
    return steps * m_config.rotationTime;
}

double MaskedTimeCost::abaTime(uint16_t fromConfig, uint16_t toConfig) const {
    if (fromConfig == toConfig) return 0.0;
    return m_config.abaReconfigTime;
}

double MaskedTimeCost::repoTime(bool needsRepo) const {
    if (!needsRepo) return 0.0;
    return m_config.totalRepoTime();
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 6.3: Create cost function tests

```cpp
// tests/phase3/test_cost_function.cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "openpanelcam/phase3/cost_function.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;
using Catch::Approx;

TEST_CASE("MaskedTimeCost uses default config", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.config().baseBendTime == 2.0);
    REQUIRE(cost.config().rotationTime == 1.5);
    REQUIRE(cost.config().abaReconfigTime == 0.8);
}

TEST_CASE("bendTime returns base time for 90 degree", "[phase3][cost]") {
    MaskedTimeCost cost;

    BendFeature bend;
    bend.angle = 90.0;

    REQUIRE(cost.bendTime(bend) == Approx(2.0));
}

TEST_CASE("bendTime increases for larger angles", "[phase3][cost]") {
    MaskedTimeCost cost;

    BendFeature bend90, bend120;
    bend90.angle = 90.0;
    bend120.angle = 120.0;

    REQUIRE(cost.bendTime(bend120) > cost.bendTime(bend90));
}

TEST_CASE("rotationTime is 0 for same orientation", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.rotationTime(Orientation::DEG_0, Orientation::DEG_0) == 0.0);
    REQUIRE(cost.rotationTime(Orientation::DEG_90, Orientation::DEG_90) == 0.0);
}

TEST_CASE("rotationTime for 90 degree rotation", "[phase3][cost]") {
    MaskedTimeCost cost;

    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_90);
    REQUIRE(time == Approx(1.5));
}

TEST_CASE("rotationTime takes shortest path", "[phase3][cost]") {
    MaskedTimeCost cost;

    // 0 -> 270 should be 1 step (90 CCW) not 3 steps (270 CW)
    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_270);
    REQUIRE(time == Approx(1.5));  // 1 step
}

TEST_CASE("rotationTime for 180 degree rotation", "[phase3][cost]") {
    MaskedTimeCost cost;

    double time = cost.rotationTime(Orientation::DEG_0, Orientation::DEG_180);
    REQUIRE(time == Approx(3.0));  // 2 steps
}

TEST_CASE("abaTime is 0 for same config", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.abaTime(100, 100) == 0.0);
}

TEST_CASE("abaTime is fixed for any change", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.abaTime(100, 200) == Approx(0.8));
    REQUIRE(cost.abaTime(0, 1) == Approx(0.8));
}

TEST_CASE("repoTime is 0 when not needed", "[phase3][cost]") {
    MaskedTimeCost cost;

    REQUIRE(cost.repoTime(false) == 0.0);
}

TEST_CASE("repoTime includes all phases", "[phase3][cost]") {
    MaskedTimeCost cost;

    // 2 + 1.5 + 2 + 1 = 6.5 seconds
    REQUIRE(cost.repoTime(true) == Approx(6.5));
}

TEST_CASE("stepCost uses max for parallel operations", "[phase3][cost]") {
    MaskedTimeCost cost;

    SearchState current;
    current.orientation = Orientation::DEG_0;
    current.abaConfig = 100;

    SearchState next;
    next.orientation = Orientation::DEG_90;  // Rotation needed (1.5s)
    next.abaConfig = 200;                     // ABA change needed (0.8s)
    next.needsRepo = false;

    BendFeature bend;
    bend.id = 0;
    bend.angle = 90.0;  // 2.0s bend time

    double totalCost = cost.stepCost(current, 0, next, bend);

    // Expected: 2.0 (bend) + max(1.5, 0.8) (parallel) + 0 (repo) = 3.5
    REQUIRE(totalCost == Approx(3.5));
}

TEST_CASE("stepCost with repo adds full repo time", "[phase3][cost]") {
    MaskedTimeCost cost;

    SearchState current, next;
    next.needsRepo = true;

    BendFeature bend;
    bend.angle = 90.0;

    double totalCost = cost.stepCost(current, 0, next, bend);

    // Expected: 2.0 (bend) + 0 (no parallel) + 6.5 (repo) = 8.5
    REQUIRE(totalCost == Approx(8.5));
}
```

### Step 6.4: Update CMakeLists.txt

```cmake
# Update src/phase3/CMakeLists.txt
add_library(openpanelcam_phase3 STATIC
    zobrist_table.cpp
    cost_function.cpp
)
```

```cmake
# Update tests/phase3/CMakeLists.txt
add_executable(test_cost_function test_cost_function.cpp)
target_link_libraries(test_cost_function PRIVATE
    Catch2::Catch2WithMain
    openpanelcam_phase3
)
add_test(NAME test_cost_function COMMAND test_cost_function)
```

### Step 6.5: Run test

```bash
cd build && cmake --build . --target test_cost_function && ./bin/test_cost_function
```

Expected: PASS

### Step 6.6: Commit

```bash
git add include/openpanelcam/phase3/cost_function.h src/phase3/cost_function.cpp tests/phase3/test_cost_function.cpp src/phase3/CMakeLists.txt tests/phase3/CMakeLists.txt
git commit -m "feat(phase3): implement MaskedTimeCost with parallel operation handling"
```

---

## Task 7: BendTimeEstimator

**Goal:** Estimate bend time based on angle, material, and thickness.

**Files:**
- Modify: `include/openpanelcam/phase3/cost_function.h`
- Modify: `src/phase3/cost_function.cpp`
- Modify: `tests/phase3/test_cost_function.cpp`

### Step 7.1: Add BendTimeEstimator class

```cpp
// Add to include/openpanelcam/phase3/cost_function.h

/**
 * @brief Estimates bend execution time based on geometry
 */
class BendTimeEstimator {
public:
    explicit BendTimeEstimator(const CostConfig& config = CostConfig());

    /**
     * @brief Estimate time for a single bend
     * @param bend Bend feature
     * @param thickness Material thickness (mm)
     * @return Estimated time in seconds
     */
    double estimate(const phase1::BendFeature& bend, double thickness = 1.5) const;

    /**
     * @brief Get required orientation for bend
     */
    Orientation requiredOrientation(const phase1::BendFeature& bend) const;

    /**
     * @brief Get required ABA config for bend
     */
    uint16_t requiredAbaConfig(const phase1::BendFeature& bend) const;

private:
    CostConfig m_config;
};
```

### Step 7.2: Implement BendTimeEstimator

```cpp
// Add to src/phase3/cost_function.cpp

BendTimeEstimator::BendTimeEstimator(const CostConfig& config)
    : m_config(config) {}

double BendTimeEstimator::estimate(const phase1::BendFeature& bend, double thickness) const {
    double time = m_config.baseBendTime;

    // Angle factor: more time for larger angles
    double angleFactor = std::abs(bend.angle) / 90.0;
    time *= angleFactor;

    // Thickness factor
    time += thickness * m_config.thicknessMultiplier;

    // Length factor: longer bends take slightly more time
    double lengthFactor = 1.0 + (bend.length / 1000.0) * 0.1;
    time *= lengthFactor;

    return time;
}

Orientation BendTimeEstimator::requiredOrientation(const phase1::BendFeature& bend) const {
    // Determine required orientation based on bend direction
    // Simplified: use bend direction vector to determine quadrant
    double dx = bend.direction.x;
    double dy = bend.direction.y;

    if (std::abs(dx) > std::abs(dy)) {
        return (dx > 0) ? Orientation::DEG_0 : Orientation::DEG_180;
    } else {
        return (dy > 0) ? Orientation::DEG_90 : Orientation::DEG_270;
    }
}

uint16_t BendTimeEstimator::requiredAbaConfig(const phase1::BendFeature& bend) const {
    // Encode required width as ABA config
    // Add 10mm clearance to bend length
    double requiredWidth = bend.length + 10.0;
    return static_cast<uint16_t>(requiredWidth);
}
```

### Step 7.3: Add tests

```cpp
// Add to tests/phase3/test_cost_function.cpp

TEST_CASE("BendTimeEstimator basic estimate", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend;
    bend.angle = 90.0;
    bend.length = 100.0;

    double time = estimator.estimate(bend);
    REQUIRE(time > 0.0);
    REQUIRE(time < 10.0);  // Reasonable range
}

TEST_CASE("BendTimeEstimator longer bend takes more time", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature short_bend, long_bend;
    short_bend.angle = 90.0;
    short_bend.length = 50.0;
    long_bend.angle = 90.0;
    long_bend.length = 500.0;

    REQUIRE(estimator.estimate(long_bend) > estimator.estimate(short_bend));
}

TEST_CASE("BendTimeEstimator requiredOrientation", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend;
    bend.direction.x = 1.0;
    bend.direction.y = 0.0;

    REQUIRE(estimator.requiredOrientation(bend) == Orientation::DEG_0);
}

TEST_CASE("BendTimeEstimator requiredAbaConfig", "[phase3][cost][estimator]") {
    BendTimeEstimator estimator;

    BendFeature bend;
    bend.length = 100.0;

    uint16_t config = estimator.requiredAbaConfig(bend);
    REQUIRE(config == 110);  // 100 + 10 clearance
}
```

### Step 7.4: Run test and commit

```bash
cd build && cmake --build . --target test_cost_function && ./bin/test_cost_function
git add -A && git commit -m "feat(phase3): add BendTimeEstimator for bend time calculation"
```

---

## Task 8: Heuristic Base Class

**Goal:** Create base interface for all heuristics.

**Files:**
- Create: `include/openpanelcam/phase3/heuristic.h`
- Create: `tests/phase3/test_heuristic.cpp`

### Step 8.1: Create heuristic header

```cpp
// include/openpanelcam/phase3/heuristic.h
#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Base class for A* heuristics
 */
class Heuristic {
public:
    virtual ~Heuristic() = default;

    /**
     * @brief Estimate remaining cost to goal
     * @param state Current search state
     * @param allBends All bend features
     * @return Estimated remaining cost (must be admissible)
     */
    virtual double estimate(const SearchState& state,
                           const std::vector<phase1::BendFeature>& allBends) const = 0;

    /**
     * @brief Get remaining (unbent) bends
     */
    static std::vector<phase1::BendFeature> getRemainingBends(
        const SearchState& state,
        const std::vector<phase1::BendFeature>& allBends
    ) {
        std::vector<phase1::BendFeature> remaining;
        for (const auto& bend : allBends) {
            if (!state.isBent(bend.id)) {
                remaining.push_back(bend);
            }
        }
        return remaining;
    }
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 8.2: Create test file

```cpp
// tests/phase3/test_heuristic.cpp
#include <catch2/catch_test_macros.hpp>
#include "openpanelcam/phase3/heuristic.h"

using namespace openpanelcam::phase3;
using namespace openpanelcam::phase1;

TEST_CASE("Heuristic::getRemainingBends filters correctly", "[phase3][heuristic]") {
    std::vector<BendFeature> allBends;
    for (int i = 0; i < 5; i++) {
        BendFeature b;
        b.id = i;
        allBends.push_back(b);
    }

    SearchState state;
    state.bentMask = 0b01010;  // Bends 1 and 3 done

    auto remaining = Heuristic::getRemainingBends(state, allBends);

    REQUIRE(remaining.size() == 3);  // 0, 2, 4 remaining

    // Check IDs
    bool has0 = false, has2 = false, has4 = false;
    for (const auto& b : remaining) {
        if (b.id == 0) has0 = true;
        if (b.id == 2) has2 = true;
        if (b.id == 4) has4 = true;
    }
    REQUIRE(has0);
    REQUIRE(has2);
    REQUIRE(has4);
}
```

### Step 8.3: Run test and commit

```bash
cd build && cmake --build . --target test_heuristic && ./bin/test_heuristic
git add -A && git commit -m "feat(phase3): add Heuristic base class interface"
```

---

## Task 9: RotationalEntropyHeuristic (H1)

**Goal:** Implement H1: penalize orientation diversity in remaining bends.

**Files:**
- Modify: `include/openpanelcam/phase3/heuristic.h`
- Create: `src/phase3/heuristic.cpp`
- Modify: `tests/phase3/test_heuristic.cpp`

### Step 9.1: Add RotationalEntropyHeuristic class

```cpp
// Add to include/openpanelcam/phase3/heuristic.h

/**
 * @brief H1: Rotational Entropy Heuristic
 *
 * Penalizes sequences that leave bends requiring different orientations.
 * Formula: h1 = k1 * CountDistinctOrientations(remaining)
 */
class RotationalEntropyHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 0.8;  // seconds per distinct orientation

    explicit RotationalEntropyHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
    BendTimeEstimator m_estimator;
};
```

### Step 9.2: Implement RotationalEntropyHeuristic

```cpp
// src/phase3/heuristic.cpp
#include "openpanelcam/phase3/heuristic.h"
#include "openpanelcam/phase3/cost_function.h"
#include <set>
#include <cmath>
#include <numeric>
#include <algorithm>

namespace openpanelcam {
namespace phase3 {

RotationalEntropyHeuristic::RotationalEntropyHeuristic(double weight)
    : m_weight(weight) {}

double RotationalEntropyHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.empty()) return 0.0;

    // Count distinct orientations needed
    std::set<Orientation> orientations;
    for (const auto& bend : remaining) {
        orientations.insert(m_estimator.requiredOrientation(bend));
    }

    // Estimate: one rotation per distinct orientation change
    int distinctCount = static_cast<int>(orientations.size());
    return m_weight * (distinctCount - 1);  // -1 because first doesn't need change
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 9.3: Add tests

```cpp
// Add to tests/phase3/test_heuristic.cpp
#include "openpanelcam/phase3/cost_function.h"

TEST_CASE("RotationalEntropyHeuristic returns 0 for empty", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    SearchState state;
    state.bentMask = 0b111;  // All 3 done

    std::vector<BendFeature> bends(3);
    for (int i = 0; i < 3; i++) bends[i].id = i;

    REQUIRE(h1.estimate(state, bends) == 0.0);
}

TEST_CASE("RotationalEntropyHeuristic penalizes diverse orientations", "[phase3][heuristic][h1]") {
    RotationalEntropyHeuristic h1;

    // Create bends with different directions
    std::vector<BendFeature> bends(4);
    bends[0].id = 0; bends[0].direction = {1, 0, 0};   // DEG_0
    bends[1].id = 1; bends[1].direction = {0, 1, 0};   // DEG_90
    bends[2].id = 2; bends[2].direction = {-1, 0, 0};  // DEG_180
    bends[3].id = 3; bends[3].direction = {0, -1, 0};  // DEG_270

    SearchState state;  // Nothing bent yet

    double h = h1.estimate(state, bends);

    // 4 distinct orientations -> (4-1) * 0.8 = 2.4
    REQUIRE(h > 0.0);
}
```

### Step 9.4: Run test and commit

```bash
cd build && cmake --build . --target test_heuristic && ./bin/test_heuristic
git add -A && git commit -m "feat(phase3): implement RotationalEntropyHeuristic (H1)"
```

---

## Task 10: ToolingVarianceHeuristic (H2)

**Goal:** Implement H2: penalize ABA width diversity.

**Files:**
- Modify: `include/openpanelcam/phase3/heuristic.h`
- Modify: `src/phase3/heuristic.cpp`
- Modify: `tests/phase3/test_heuristic.cpp`

### Step 10.1: Add ToolingVarianceHeuristic

```cpp
// Add to include/openpanelcam/phase3/heuristic.h

/**
 * @brief H2: Tooling Variance Heuristic
 *
 * Penalizes sequences with high variance in required ABA widths.
 * Formula: h2 = k2 * StandardDeviation(remaining_aba_widths)
 */
class ToolingVarianceHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 0.003;  // seconds per mm stddev

    explicit ToolingVarianceHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
};
```

### Step 10.2: Implement

```cpp
// Add to src/phase3/heuristic.cpp

ToolingVarianceHeuristic::ToolingVarianceHeuristic(double weight)
    : m_weight(weight) {}

double ToolingVarianceHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    auto remaining = getRemainingBends(state, allBends);
    if (remaining.size() < 2) return 0.0;

    // Calculate standard deviation of bend lengths (proxy for ABA width)
    std::vector<double> widths;
    for (const auto& bend : remaining) {
        widths.push_back(bend.length + 10.0);  // Add clearance
    }

    double mean = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();

    double sqSum = 0.0;
    for (double w : widths) {
        sqSum += (w - mean) * (w - mean);
    }
    double stddev = std::sqrt(sqSum / widths.size());

    return m_weight * stddev;
}
```

### Step 10.3: Add tests and commit

```bash
git add -A && git commit -m "feat(phase3): implement ToolingVarianceHeuristic (H2)"
```

---

## Task 11: GraspFragmentationHeuristic (H3)

**Goal:** Implement H3: penalize grip region splitting.

**Files:**
- Modify: `include/openpanelcam/phase3/heuristic.h`
- Modify: `src/phase3/heuristic.cpp`

### Step 11.1: Add GraspFragmentationHeuristic

```cpp
// Add to include/openpanelcam/phase3/heuristic.h

/**
 * @brief H3: Grasp Fragmentation Heuristic
 *
 * Penalizes states where valid grip region is fragmented.
 * Formula: h3 = k3 * (NumberOfDisconnectedRegions - 1)
 */
class GraspFragmentationHeuristic : public Heuristic {
public:
    static constexpr double DEFAULT_WEIGHT = 2.0;  // seconds per extra region

    explicit GraspFragmentationHeuristic(double weight = DEFAULT_WEIGHT);

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    double m_weight;
};
```

### Step 11.2: Implement (simplified - counts remaining bends as proxy)

```cpp
// Add to src/phase3/heuristic.cpp

GraspFragmentationHeuristic::GraspFragmentationHeuristic(double weight)
    : m_weight(weight) {}

double GraspFragmentationHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    // Simplified: use number of remaining bends as proxy for fragmentation risk
    // Real implementation would analyze grip region geometry
    auto remaining = getRemainingBends(state, allBends);

    if (remaining.size() <= 1) return 0.0;

    // Heuristic: more remaining bends = higher fragmentation risk
    int fragmentationRisk = static_cast<int>(remaining.size()) / 3;
    return m_weight * fragmentationRisk;
}
```

### Step 11.3: Commit

```bash
git add -A && git commit -m "feat(phase3): implement GraspFragmentationHeuristic (H3)"
```

---

## Task 12: CombinedHeuristic

**Goal:** Combine H1 + H2 + H3 into single admissible heuristic.

**Files:**
- Modify: `include/openpanelcam/phase3/heuristic.h`
- Modify: `src/phase3/heuristic.cpp`

### Step 12.1: Add CombinedHeuristic

```cpp
// Add to include/openpanelcam/phase3/heuristic.h

/**
 * @brief Combined heuristic: h = h1 + h2 + h3
 */
class CombinedHeuristic : public Heuristic {
public:
    CombinedHeuristic();

    double estimate(const SearchState& state,
                   const std::vector<phase1::BendFeature>& allBends) const override;

private:
    RotationalEntropyHeuristic m_h1;
    ToolingVarianceHeuristic m_h2;
    GraspFragmentationHeuristic m_h3;
};
```

### Step 12.2: Implement

```cpp
CombinedHeuristic::CombinedHeuristic() {}

double CombinedHeuristic::estimate(
    const SearchState& state,
    const std::vector<phase1::BendFeature>& allBends
) const {
    return m_h1.estimate(state, allBends)
         + m_h2.estimate(state, allBends)
         + m_h3.estimate(state, allBends);
}
```

### Step 12.3: Commit

```bash
git add -A && git commit -m "feat(phase3): implement CombinedHeuristic (H1+H2+H3)"
```

---

## Task 13: SuccessorGenerator Class

**Goal:** Generate valid successor states respecting precedence DAG.

**Files:**
- Create: `include/openpanelcam/phase3/successor_generator.h`
- Create: `src/phase3/successor_generator.cpp`
- Create: `tests/phase3/test_successor.cpp`

### Step 13.1: Create SuccessorGenerator header

```cpp
// include/openpanelcam/phase3/successor_generator.h
#pragma once

#include "types.h"
#include "cost_function.h"
#include "../phase2/precedence_dag.h"
#include "../phase2/constraint_solver.h"
#include <vector>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Generates valid successor states for A* search
 */
class SuccessorGenerator {
public:
    SuccessorGenerator(const phase2::PrecedenceDAG& dag,
                       const std::vector<phase1::BendFeature>& bends,
                       const std::vector<phase2::GraspConstraint>& graspConstraints);

    /**
     * @brief Generate all valid successors from current state
     */
    std::vector<SearchNode> generate(const SearchNode& current,
                                     const MaskedTimeCost& costFn,
                                     const Heuristic& heuristic);

    /**
     * @brief Check if bend can be performed in current state
     */
    bool canBend(int bendId, const SearchState& state) const;

    /**
     * @brief Get list of bendable bends in current state
     */
    std::vector<int> getBendableBends(const SearchState& state) const;

private:
    const phase2::PrecedenceDAG& m_dag;
    const std::vector<phase1::BendFeature>& m_bends;
    const std::vector<phase2::GraspConstraint>& m_graspConstraints;
    BendTimeEstimator m_estimator;

    SearchState applyBend(const SearchState& current, int bendId) const;
    bool checkGraspValid(const SearchState& state) const;
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 13.2: Implement SuccessorGenerator

```cpp
// src/phase3/successor_generator.cpp
#include "openpanelcam/phase3/successor_generator.h"

namespace openpanelcam {
namespace phase3 {

SuccessorGenerator::SuccessorGenerator(
    const phase2::PrecedenceDAG& dag,
    const std::vector<phase1::BendFeature>& bends,
    const std::vector<phase2::GraspConstraint>& graspConstraints
) : m_dag(dag), m_bends(bends), m_graspConstraints(graspConstraints) {}

bool SuccessorGenerator::canBend(int bendId, const SearchState& state) const {
    // Already bent?
    if (state.isBent(bendId)) return false;

    // Check precedence: all predecessors must be bent
    const auto* node = m_dag.getNodeByBendId(bendId);
    if (!node) return false;

    for (int predId : node->predecessors) {
        if (!state.isBent(predId)) {
            return false;  // Predecessor not yet bent
        }
    }

    return true;
}

std::vector<int> SuccessorGenerator::getBendableBends(const SearchState& state) const {
    std::vector<int> bendable;
    for (const auto& bend : m_bends) {
        if (canBend(bend.id, state)) {
            bendable.push_back(bend.id);
        }
    }
    return bendable;
}

SearchState SuccessorGenerator::applyBend(const SearchState& current, int bendId) const {
    SearchState next = current;
    next.markBent(bendId);

    // Update orientation and ABA config based on bend requirements
    for (const auto& bend : m_bends) {
        if (bend.id == bendId) {
            next.orientation = m_estimator.requiredOrientation(bend);
            next.abaConfig = m_estimator.requiredAbaConfig(bend);
            break;
        }
    }

    // Check if repo needed (simplified)
    next.needsRepo = false;  // Will be enhanced in Task 26

    return next;
}

bool SuccessorGenerator::checkGraspValid(const SearchState& state) const {
    // Simplified: always valid for now
    // Real implementation checks GraspConstraint for this state
    return true;
}

std::vector<SearchNode> SuccessorGenerator::generate(
    const SearchNode& current,
    const MaskedTimeCost& costFn,
    const Heuristic& heuristic
) {
    std::vector<SearchNode> successors;

    auto bendable = getBendableBends(current.state);

    for (int bendId : bendable) {
        SearchNode successor;

        // Apply bend
        successor.state = applyBend(current.state, bendId);

        // Check grasp validity
        if (!checkGraspValid(successor.state)) {
            continue;  // Prune invalid states
        }

        // Find bend feature
        const phase1::BendFeature* bend = nullptr;
        for (const auto& b : m_bends) {
            if (b.id == bendId) {
                bend = &b;
                break;
            }
        }
        if (!bend) continue;

        // Calculate costs
        successor.g = current.g + costFn.stepCost(current.state, bendId, successor.state, *bend);
        successor.h = heuristic.estimate(successor.state, m_bends);
        successor.parentId = -1;  // Will be set by search
        successor.lastBendId = bendId;
        successor.lastAction = ActionType::BEND;

        successors.push_back(successor);
    }

    return successors;
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 13.3: Add tests and commit

```bash
git add -A && git commit -m "feat(phase3): implement SuccessorGenerator respecting precedence DAG"
```

---

## Task 14-15: Priority Queue and State Deduplication

**Goal:** Implement open set with priority queue and closed set with Zobrist hashing.

*[Similar TDD pattern - creates OpenSet and ClosedSet classes]*

---

## Task 16-20: A* Search Engine

**Goal:** Implement the main A* search algorithm.

**Files:**
- Create: `include/openpanelcam/phase3/astar_search.h`
- Create: `src/phase3/astar_search.cpp`
- Create: `tests/phase3/test_astar.cpp`

### Step 16.1: Create AStarSearch header

```cpp
// include/openpanelcam/phase3/astar_search.h
#pragma once

#include "types.h"
#include "cost_function.h"
#include "heuristic.h"
#include "zobrist_table.h"
#include "successor_generator.h"
#include "../phase2/constraint_solver.h"
#include <queue>
#include <unordered_map>
#include <functional>

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Configuration for A* search
 */
struct SearchConfig {
    int maxNodes = 1000000;      // Maximum nodes to expand
    double timeoutSeconds = 30.0; // Search timeout
    bool useBeamSearch = false;  // Fall back to beam search if too slow
    int beamWidth = 100;         // Beam width for beam search
};

/**
 * @brief A* search engine for bend sequencing
 */
class AStarSearch {
public:
    AStarSearch(const phase2::Phase2Output& phase2Output,
                const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Find optimal bend sequence
     */
    Phase3Output search(const SearchConfig& config = SearchConfig());

    /**
     * @brief Get search statistics
     */
    const SequencerStatistics& getStatistics() const { return m_stats; }

private:
    // Input data
    const phase2::Phase2Output& m_phase2;
    const std::vector<phase1::BendFeature>& m_bends;

    // Search components
    ZobristTable m_zobrist;
    MaskedTimeCost m_costFn;
    CombinedHeuristic m_heuristic;
    std::unique_ptr<SuccessorGenerator> m_successor;

    // Statistics
    SequencerStatistics m_stats;

    // Internal methods
    Phase3Output reconstructPath(const std::vector<SearchNode>& closedSet,
                                  int goalNodeIndex);
    bool isGoal(const SearchState& state) const;
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 16.2: Implement A* search

```cpp
// src/phase3/astar_search.cpp
#include "openpanelcam/phase3/astar_search.h"
#include <chrono>
#include <algorithm>

namespace openpanelcam {
namespace phase3 {

AStarSearch::AStarSearch(const phase2::Phase2Output& phase2Output,
                         const std::vector<phase1::BendFeature>& bends)
    : m_phase2(phase2Output)
    , m_bends(bends)
    , m_zobrist(12345)  // Fixed seed for reproducibility
{
    m_successor = std::make_unique<SuccessorGenerator>(
        m_phase2.precedenceGraph,
        m_bends,
        m_phase2.graspConstraints
    );
}

bool AStarSearch::isGoal(const SearchState& state) const {
    return state.isGoal(static_cast<int>(m_bends.size()));
}

Phase3Output AStarSearch::search(const SearchConfig& config) {
    Phase3Output output;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Priority queue: min-heap by f-score
    auto cmp = [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<double, int>,
                        std::vector<std::pair<double, int>>,
                        decltype(cmp)> openQueue(cmp);

    // Closed set with all expanded nodes
    std::vector<SearchNode> closedSet;

    // Hash map for duplicate detection
    std::unordered_map<uint64_t, int> stateToNode;

    // Initialize with start node
    SearchNode startNode = SearchNode::createInitial();
    startNode.h = m_heuristic.estimate(startNode.state, m_bends);

    closedSet.push_back(startNode);
    int startIdx = 0;
    uint64_t startHash = m_zobrist.hash(startNode.state);
    stateToNode[startHash] = startIdx;
    openQueue.push({startNode.f(), startIdx});

    m_stats.nodesGenerated = 1;
    m_stats.maxOpenSetSize = 1;

    // Main A* loop
    while (!openQueue.empty()) {
        // Check limits
        if (m_stats.nodesExpanded >= config.maxNodes) {
            output.errorMessage = "Max nodes reached";
            break;
        }

        auto elapsed = std::chrono::high_resolution_clock::now() - startTime;
        double elapsedSec = std::chrono::duration<double>(elapsed).count();
        if (elapsedSec > config.timeoutSeconds) {
            output.errorMessage = "Timeout";
            break;
        }

        // Pop best node
        auto [fScore, nodeIdx] = openQueue.top();
        openQueue.pop();

        SearchNode& current = closedSet[nodeIdx];
        m_stats.nodesExpanded++;

        // Goal check
        if (isGoal(current.state)) {
            output = reconstructPath(closedSet, nodeIdx);
            output.success = true;
            output.optimal = true;
            break;
        }

        // Generate successors
        auto successors = m_successor->generate(current, m_costFn, m_heuristic);
        m_stats.nodesGenerated += successors.size();

        for (auto& succ : successors) {
            uint64_t hash = m_zobrist.hash(succ.state);

            // Check for duplicate
            auto it = stateToNode.find(hash);
            if (it != stateToNode.end()) {
                // Already visited - check if better path
                int existingIdx = it->second;
                if (succ.g < closedSet[existingIdx].g) {
                    closedSet[existingIdx] = succ;
                    closedSet[existingIdx].parentId = nodeIdx;
                    openQueue.push({succ.f(), existingIdx});
                } else {
                    m_stats.duplicatesSkipped++;
                }
                continue;
            }

            // New state
            succ.parentId = nodeIdx;
            int newIdx = static_cast<int>(closedSet.size());
            closedSet.push_back(succ);
            stateToNode[hash] = newIdx;
            openQueue.push({succ.f(), newIdx});
        }

        m_stats.maxOpenSetSize = std::max(m_stats.maxOpenSetSize,
                                           static_cast<int>(openQueue.size()));
    }

    // Record timing
    auto endTime = std::chrono::high_resolution_clock::now();
    m_stats.searchTimeMs = std::chrono::duration<double, std::milli>(
        endTime - startTime
    ).count();
    output.stats = m_stats;

    return output;
}

Phase3Output AStarSearch::reconstructPath(const std::vector<SearchNode>& closedSet,
                                           int goalNodeIndex) {
    Phase3Output output;

    // Trace back from goal to start
    std::vector<int> reversePath;
    int currentIdx = goalNodeIndex;

    while (currentIdx >= 0) {
        const SearchNode& node = closedSet[currentIdx];
        if (node.lastBendId >= 0) {
            reversePath.push_back(node.lastBendId);
        }
        currentIdx = node.parentId;
    }

    // Reverse to get forward sequence
    std::reverse(reversePath.begin(), reversePath.end());
    output.bendSequence = reversePath;

    // Calculate total cycle time
    output.totalCycleTime = closedSet[goalNodeIndex].g;

    // Generate detailed actions
    for (int bendId : output.bendSequence) {
        SequenceAction action;
        action.type = ActionType::BEND;
        action.bendId = bendId;
        // Find bend for duration
        for (const auto& bend : m_bends) {
            if (bend.id == bendId) {
                action.duration = m_costFn.bendTime(bend);
                action.description = "Bend " + std::to_string(bendId);
                break;
            }
        }
        output.actions.push_back(action);
    }

    output.stats.solutionDepth = static_cast<int>(output.bendSequence.size());

    return output;
}

} // namespace phase3
} // namespace openpanelcam
```

### Step 16.3-16.5: Add tests, update CMake, commit

*[Tests verify A* finds optimal solution for simple cases]*

---

## Task 21-25: Path Reconstruction and Beam Search Fallback

*[Implements path reconstruction with detailed actions and beam search for large state spaces]*

---

## Task 26-30: Repo Integration

**Goal:** Integrate repositioning as a search action.

### Step 26.1: Add RepoTriggerDetector

```cpp
// Add to include/openpanelcam/phase3/types.h or new file

class RepoTriggerDetector {
public:
    RepoTriggerDetector(double minGripArea = 100.0);

    RepoReason check(const SearchState& state,
                     const std::vector<phase2::GraspConstraint>& constraints) const;

private:
    double m_minGripArea;
};
```

### Step 26.2-30: Implement repo as action in successor generator

*[Adds REPOSITION as valid action type in SuccessorGenerator]*

---

## Task 31-35: Sequencer Main Class

**Goal:** Create the main Sequencer class that wraps everything.

**Files:**
- Create: `include/openpanelcam/phase3/sequencer.h`
- Create: `src/phase3/sequencer.cpp`
- Create: `tests/phase3/test_sequencer.cpp`

### Step 31.1: Create Sequencer header

```cpp
// include/openpanelcam/phase3/sequencer.h
#pragma once

#include "types.h"
#include "astar_search.h"
#include "../phase2/constraint_solver.h"

namespace openpanelcam {
namespace phase3 {

/**
 * @brief Main entry point for Phase 3 bend sequencing
 */
class Sequencer {
public:
    Sequencer();

    /**
     * @brief Find optimal bend sequence
     * @param phase2Output Output from Phase 2 constraint solver
     * @param bends Bend features from Phase 1
     * @return Optimal sequence with timing
     */
    Phase3Output sequence(const phase2::Phase2Output& phase2Output,
                          const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Configure search parameters
     */
    void setConfig(const SearchConfig& config);

    /**
     * @brief Get statistics from last search
     */
    const SequencerStatistics& getStatistics() const;

private:
    SearchConfig m_config;
    SequencerStatistics m_stats;
};

} // namespace phase3
} // namespace openpanelcam
```

### Step 31.2-35: Implement and test

*[Standard TDD pattern for main class]*

---

## Task 36-40: Testing and Documentation

### Task 36: Unit Tests

Create comprehensive unit tests for all components.

### Task 37: Integration Tests

Test Phase 2 → Phase 3 integration with sample parts.

### Task 38: Performance Benchmarks

```cpp
// tests/phase3/test_performance.cpp
TEST_CASE("Benchmark: 5 bends", "[phase3][benchmark]") {
    // ... create 5 bend scenario
    // Verify < 1 second
}

TEST_CASE("Benchmark: 10 bends", "[phase3][benchmark]") {
    // Verify < 5 seconds
}

TEST_CASE("Benchmark: 20 bends", "[phase3][benchmark]") {
    // Verify < 30 seconds
}
```

### Task 39: Edge Case Tests

- Empty input
- Single bend
- Cyclic dependencies (should fail gracefully)
- All same orientation (trivial)
- Box closing scenario

### Task 40: Documentation

Create `docs/Phase3_Architecture.md` and `docs/Phase3_API_Reference.md`

---

## Summary

### Total Tasks: 40

| Task Range | Component | Description |
|------------|-----------|-------------|
| 1-5 | Core Types | Enums, SearchState, SearchNode, Zobrist, Phase3Output |
| 6-10 | Cost Functions | MaskedTimeCost, BendTimeEstimator, RepoCostModel |
| 11-15 | Heuristics | H1 (Rotation), H2 (Tooling), H3 (Grasp), Combined |
| 16-20 | A* Core | PriorityQueue, Successor, Deduplication, GoalTest |
| 21-25 | Path | Reconstruction, Optimization, Beam Search fallback |
| 26-30 | Repo | TriggerDetector, RepoAction, RepoAsAction |
| 31-35 | Integration | Sequencer class, Phase2 bridge, Phase3Output |
| 36-40 | Quality | Unit tests, Integration tests, Benchmarks, Docs |

### Key Algorithms

1. **Zobrist Hashing** - O(1) state deduplication
2. **A* Search** - Optimal pathfinding with admissible heuristic
3. **Masked Time Cost** - Parallel operation modeling
4. **Three Research Heuristics** - Admissible estimates

### Performance Targets

- < 5 seconds for typical parts (< 10 bends)
- < 30 seconds for complex parts (< 20 bends)
- State deduplication eliminates 95%+ duplicates
- Heuristic reduces search space by 10x+

---

**Plan complete and saved to `docs/plans/2026-02-07-phase3-sequencer.md`**
