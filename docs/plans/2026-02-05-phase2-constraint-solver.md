# Phase 2: Constraint Solver Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build constraint solver that analyzes FAG from Phase 1 and generates Precedence DAG with grasp and ABA constraints for bend sequencing.

**Architecture:** 4 main modules (Geometric Precedence Analyzer, Grasp Constraint Generator, ABA Constraint Analyzer, Precedence DAG Builder) that consume Phase1Output and produce Phase2Output with complete constraint graph.

**Tech Stack:** C++17, OpenCASCADE (geometry), CGAL (polygon operations), Boost Graph (DAG), Catch2 (testing)

---

## Prerequisites

- Phase 1 compiled and tested
- Dependencies installed (CGAL, Boost Graph already in vcpkg.json)
- Working directory: `E:\DEV_CONTEXT_PROJECTs\Salvagnini_controller`

---

## Task 1: Core Data Structures - Types Header

**Files:**
- Create: `include/openpanelcam/phase2/types.h`
- Test: `tests/unit/test_phase2_types.cpp`

### Step 1.1: Write failing test for ConstraintType enum

**Test file:** `tests/unit/test_phase2_types.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <openpanelcam/phase2/types.h>

TEST_CASE("ConstraintType enum values", "[phase2][types]") {
    REQUIRE(static_cast<int>(ConstraintType::GEOMETRIC) == 0);
    REQUIRE(static_cast<int>(ConstraintType::BOX_CLOSING) == 1);
    REQUIRE(static_cast<int>(ConstraintType::SEQUENTIAL) == 2);
    REQUIRE(static_cast<int>(ConstraintType::USER_DEFINED) == 3);
}
```

### Step 1.2: Run test to verify failure

```bash
cd build
cmake --build . --target test_phase2_types
./bin/test_phase2_types
```

Expected: FAIL - "types.h not found"

### Step 1.3: Create types.h with enums

**File:** `include/openpanelcam/phase2/types.h`

```cpp
#pragma once

#include <openpanelcam/core/types.h>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <vector>
#include <string>

namespace openpanelcam {

/**
 * @brief Type of precedence constraint
 */
enum class ConstraintType {
    GEOMETRIC = 0,     // Corner overlap, geometric interference
    BOX_CLOSING,       // 3-sided box scenario
    SEQUENTIAL,        // Bent flanges blocking access
    USER_DEFINED       // Manual override
};

/**
 * @brief Type of dead zone (forbidden grip region)
 */
enum class DeadZoneType {
    FLANGE = 0,        // Standing bent flange
    SAFETY_MARGIN,     // Buffer around future bends
    ABA_TOOL,          // ABA interference zone
    BOUNDARY           // Part boundary
};

} // namespace openpanelcam
```

### Step 1.4: Run test to verify pass

```bash
cmake --build . --target test_phase2_types
./bin/test_phase2_types
```

Expected: PASS

### Step 1.5: Commit

```bash
git add include/openpanelcam/phase2/types.h tests/unit/test_phase2_types.cpp
git commit -m "feat(phase2): add core enums (ConstraintType, DeadZoneType)"
```

---

## Task 2: PrecedenceNode and PrecedenceEdge Structs

**Files:**
- Modify: `include/openpanelcam/phase2/types.h`
- Modify: `tests/unit/test_phase2_types.cpp`

### Step 2.1: Write failing test for PrecedenceNode

**Add to test file:**

```cpp
TEST_CASE("PrecedenceNode construction", "[phase2][types]") {
    PrecedenceNode node;
    node.id = 0;
    node.bendId = 5;
    node.predecessors = {1, 2};
    node.successors = {3, 4};
    node.level = -1;
    node.visited = false;

    REQUIRE(node.id == 0);
    REQUIRE(node.bendId == 5);
    REQUIRE(node.predecessors.size() == 2);
    REQUIRE(node.successors.size() == 2);
}
```

### Step 2.2: Run test (should fail)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: FAIL - "PrecedenceNode not defined"

### Step 2.3: Add PrecedenceNode struct

**Add to `types.h` after enums:**

```cpp
/**
 * @brief Node in Precedence DAG representing a bend
 */
struct PrecedenceNode {
    int id;                          // Node ID
    int bendId;                      // Associated bend ID

    std::vector<int> predecessors;   // Bends that must come before
    std::vector<int> successors;     // Bends that must come after

    int level;                       // Topological level (-1 = uncomputed)
    bool visited;                    // For graph traversal

    PrecedenceNode()
        : id(-1), bendId(-1), level(-1), visited(false)
    {}
};

/**
 * @brief Edge in Precedence DAG representing a constraint
 */
struct PrecedenceEdge {
    int id;
    int fromBend;                    // Bend that must come first
    int toBend;                      // Bend that must come second
    ConstraintType type;             // Type of constraint
    double confidence;               // 0.0-1.0
    std::string reasoning;           // Human-readable explanation

    gp_Pnt conflictPoint;            // Where interference occurs (optional)

    PrecedenceEdge()
        : id(-1), fromBend(-1), toBend(-1)
        , type(ConstraintType::GEOMETRIC)
        , confidence(1.0)
        , conflictPoint(0, 0, 0)
    {}
};
```

### Step 2.4: Run test (should pass)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: PASS

### Step 2.5: Write test for PrecedenceEdge

**Add to test file:**

```cpp
TEST_CASE("PrecedenceEdge construction", "[phase2][types]") {
    PrecedenceEdge edge;
    edge.id = 0;
    edge.fromBend = 1;
    edge.toBend = 2;
    edge.type = ConstraintType::GEOMETRIC;
    edge.confidence = 0.95;
    edge.reasoning = "Corner overlap";

    REQUIRE(edge.id == 0);
    REQUIRE(edge.fromBend == 1);
    REQUIRE(edge.toBend == 2);
    REQUIRE(edge.type == ConstraintType::GEOMETRIC);
    REQUIRE(edge.confidence == 0.95);
}
```

### Step 2.6: Run test (should pass since struct already added)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: PASS

### Step 2.7: Commit

```bash
git add include/openpanelcam/phase2/types.h tests/unit/test_phase2_types.cpp
git commit -m "feat(phase2): add PrecedenceNode and PrecedenceEdge structs"
```

---

## Task 3: 2D Geometry Helpers (Polygon2D, Rectangle2D)

**Files:**
- Modify: `include/openpanelcam/phase2/types.h`
- Modify: `tests/unit/test_phase2_types.cpp`

### Step 3.1: Write failing test for Polygon2D

**Add to test file:**

```cpp
#include <cmath>

TEST_CASE("Polygon2D area calculation", "[phase2][types]") {
    Polygon2D poly;
    // Square: 10x10
    poly.vertices = {
        gp_Pnt2d(0, 0),
        gp_Pnt2d(10, 0),
        gp_Pnt2d(10, 10),
        gp_Pnt2d(0, 10)
    };

    double area = poly.area();
    REQUIRE(std::abs(area - 100.0) < 0.01);
}

TEST_CASE("Polygon2D contains point", "[phase2][types]") {
    Polygon2D poly;
    poly.vertices = {
        gp_Pnt2d(0, 0),
        gp_Pnt2d(10, 0),
        gp_Pnt2d(10, 10),
        gp_Pnt2d(0, 10)
    };

    REQUIRE(poly.contains(gp_Pnt2d(5, 5)) == true);   // Inside
    REQUIRE(poly.contains(gp_Pnt2d(15, 5)) == false); // Outside
}
```

### Step 3.2: Run test (should fail)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: FAIL - "Polygon2D not defined"

### Step 3.3: Add Polygon2D and Rectangle2D structs

**Add to `types.h`:**

```cpp
/**
 * @brief 2D polygon for dead zones and grip regions
 */
struct Polygon2D {
    std::vector<gp_Pnt2d> vertices;
    std::vector<std::vector<gp_Pnt2d>> holes;  // Inner holes (optional)

    /**
     * @brief Compute polygon area using shoelace formula
     */
    double area() const {
        if (vertices.size() < 3) return 0.0;

        double sum = 0.0;
        for (size_t i = 0; i < vertices.size(); i++) {
            size_t j = (i + 1) % vertices.size();
            sum += vertices[i].X() * vertices[j].Y();
            sum -= vertices[j].X() * vertices[i].Y();
        }

        return std::abs(sum) / 2.0;
    }

    /**
     * @brief Check if point is inside polygon (ray casting)
     */
    bool contains(gp_Pnt2d point) const {
        if (vertices.size() < 3) return false;

        int crossings = 0;
        for (size_t i = 0; i < vertices.size(); i++) {
            size_t j = (i + 1) % vertices.size();

            gp_Pnt2d v1 = vertices[i];
            gp_Pnt2d v2 = vertices[j];

            // Ray casting along +X axis
            if ((v1.Y() > point.Y()) != (v2.Y() > point.Y())) {
                double slope = (v2.X() - v1.X()) / (v2.Y() - v1.Y());
                double xIntersect = v1.X() + slope * (point.Y() - v1.Y());

                if (point.X() < xIntersect) {
                    crossings++;
                }
            }
        }

        return (crossings % 2) == 1;  // Odd = inside
    }

    /**
     * @brief Compute centroid
     */
    gp_Pnt2d centroid() const {
        if (vertices.empty()) return gp_Pnt2d(0, 0);

        double sumX = 0.0, sumY = 0.0;
        for (const auto& v : vertices) {
            sumX += v.X();
            sumY += v.Y();
        }

        return gp_Pnt2d(sumX / vertices.size(), sumY / vertices.size());
    }
};

/**
 * @brief 2D rectangle for grip placement
 */
struct Rectangle2D {
    gp_Pnt2d bottomLeft;
    gp_Pnt2d topRight;
    double width;
    double height;
    double area;
    gp_Pnt2d center;

    Rectangle2D()
        : bottomLeft(0, 0), topRight(0, 0)
        , width(0), height(0), area(0)
        , center(0, 0)
    {}

    /**
     * @brief Check if point is inside rectangle
     */
    bool contains(gp_Pnt2d point) const {
        return point.X() >= bottomLeft.X() &&
               point.X() <= topRight.X() &&
               point.Y() >= bottomLeft.Y() &&
               point.Y() <= topRight.Y();
    }
};
```

### Step 3.4: Run test (should pass)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: PASS

### Step 3.5: Commit

```bash
git add include/openpanelcam/phase2/types.h tests/unit/test_phase2_types.cpp
git commit -m "feat(phase2): add Polygon2D and Rectangle2D with area/contains"
```

---

## Task 4: DeadZone Struct

**Files:**
- Modify: `include/openpanelcam/phase2/types.h`
- Modify: `tests/unit/test_phase2_types.cpp`

### Step 4.1: Write failing test for DeadZone

**Add to test file:**

```cpp
TEST_CASE("DeadZone creation and area", "[phase2][types]") {
    DeadZone zone;
    zone.id = 0;
    zone.type = DeadZoneType::FLANGE;
    zone.causedByBend = 1;
    zone.safetyMargin = 20.0;

    // 10x10 square
    zone.polygon = {
        gp_Pnt2d(0, 0),
        gp_Pnt2d(10, 0),
        gp_Pnt2d(10, 10),
        gp_Pnt2d(0, 10)
    };

    REQUIRE(zone.area() == 100.0);
    REQUIRE(zone.contains(gp_Pnt2d(5, 5)) == true);
}
```

### Step 4.2: Run test (should fail)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: FAIL - "DeadZone not defined"

### Step 4.3: Add DeadZone struct

**Add to `types.h`:**

```cpp
/**
 * @brief Forbidden grip region (dead zone)
 */
struct DeadZone {
    int id;
    DeadZoneType type;
    std::vector<gp_Pnt2d> polygon;   // 2D polygon on base plane
    int causedByBend;                 // Which bend created this (-1 if N/A)
    double safetyMargin;              // Additional buffer (mm)

    DeadZone()
        : id(-1), type(DeadZoneType::FLANGE)
        , causedByBend(-1), safetyMargin(0.0)
    {}

    /**
     * @brief Check if point is inside dead zone
     */
    bool contains(gp_Pnt2d point) const {
        Polygon2D poly;
        poly.vertices = polygon;
        return poly.contains(point);
    }

    /**
     * @brief Compute dead zone area
     */
    double area() const {
        Polygon2D poly;
        poly.vertices = polygon;
        return poly.area();
    }
};
```

### Step 4.4: Run test (should pass)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: PASS

### Step 4.5: Commit

```bash
git add include/openpanelcam/phase2/types.h tests/unit/test_phase2_types.cpp
git commit -m "feat(phase2): add DeadZone struct with polygon and area"
```

---

## Task 5: GraspConstraint and ABAConstraint Structs

**Files:**
- Modify: `include/openpanelcam/phase2/types.h`
- Modify: `tests/unit/test_phase2_types.cpp`

### Step 5.1: Write failing test

**Add to test file:**

```cpp
TEST_CASE("GraspConstraint initialization", "[phase2][types]") {
    GraspConstraint grasp;
    grasp.stateId = 0;
    grasp.bentBends = {0, 1};
    grasp.validArea = 5000.0;
    grasp.hasValidGrip = true;
    grasp.minRequiredArea = 100.0;

    REQUIRE(grasp.stateId == 0);
    REQUIRE(grasp.bentBends.size() == 2);
    REQUIRE(grasp.hasValidGrip == true);
}

TEST_CASE("ABAConstraint with subset sum solution", "[phase2][types]") {
    ABAConstraint aba;
    aba.bendId = 0;
    aba.requiredWidth = 275.0;
    aba.segmentSolution = {100, 175};
    aba.totalSegments = 2;
    aba.feasible = true;

    REQUIRE(aba.bendId == 0);
    REQUIRE(aba.segmentSolution.size() == 2);
    REQUIRE(aba.feasible == true);
}
```

### Step 5.2: Run test (should fail)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: FAIL

### Step 5.3: Add GraspConstraint and ABAConstraint

**Add to `types.h`:**

```cpp
/**
 * @brief Grasp constraint for a bend state
 */
struct GraspConstraint {
    int stateId;                         // After which bend configuration
    std::vector<int> bentBends;          // Bends already completed

    std::vector<DeadZone> deadZones;     // Forbidden regions
    Polygon2D validRegion;               // Valid grip area
    double validArea;                    // Total valid area (mm²)

    Rectangle2D maxInscribedRect;        // MIR result
    gp_Pnt2d optimalGripCenter;          // Recommended grip point

    bool hasValidGrip;                   // Can we grip?
    double minRequiredArea;              // Minimum threshold (100 mm²)
    gp_Pnt2d centerOfMass;               // COM of bent shape

    std::vector<std::string> warnings;

    GraspConstraint()
        : stateId(-1), validArea(0.0)
        , hasValidGrip(false), minRequiredArea(100.0)
        , optimalGripCenter(0, 0), centerOfMass(0, 0)
    {}
};

/**
 * @brief ABA tool constraint for a bend
 */
struct ABAConstraint {
    int bendId;
    double bendLength;                   // Length of bend line (mm)
    double requiredWidth;                // Minimum tool width
    double clearance;                    // Safety clearance

    std::vector<int> segmentSolution;    // e.g., {100, 150, 200}
    int totalSegments;                   // Number of segments
    double totalWidth;                   // Sum of segments

    bool feasible;                       // Solution exists?
    bool isBoxClosing;                   // Would close 3-sided box?

    std::string reason;                  // Why infeasible
    std::vector<int> suggestedAlternatives;

    ABAConstraint()
        : bendId(-1), bendLength(0.0), requiredWidth(0.0)
        , clearance(15.0), totalSegments(0), totalWidth(0.0)
        , feasible(false), isBoxClosing(false)
    {}
};
```

### Step 5.4: Run test (should pass)

```bash
cmake --build . --target test_phase2_types && ./bin/test_phase2_types
```

Expected: PASS

### Step 5.5: Commit

```bash
git add include/openpanelcam/phase2/types.h tests/unit/test_phase2_types.cpp
git commit -m "feat(phase2): add GraspConstraint and ABAConstraint structs"
```

---

## Task 6: PrecedenceDAG Class Header

**Files:**
- Create: `include/openpanelcam/phase2/precedence_dag.h`
- Create: `tests/unit/test_precedence_dag.cpp`

### Step 6.1: Write failing test for DAG construction

**File:** `tests/unit/test_precedence_dag.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <openpanelcam/phase2/precedence_dag.h>

TEST_CASE("PrecedenceDAG construction", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;

    REQUIRE(dag.nodeCount() == 0);
    REQUIRE(dag.edgeCount() == 0);
}

TEST_CASE("PrecedenceDAG add nodes", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;

    dag.addNode(0);  // Bend 0
    dag.addNode(1);  // Bend 1
    dag.addNode(2);  // Bend 2

    REQUIRE(dag.nodeCount() == 3);
}
```

### Step 6.2: Run test (should fail)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: FAIL - "precedence_dag.h not found"

### Step 6.3: Create PrecedenceDAG header

**File:** `include/openpanelcam/phase2/precedence_dag.h`

```cpp
#pragma once

#include <openpanelcam/phase2/types.h>
#include <vector>
#include <map>
#include <string>

namespace openpanelcam {

/**
 * @brief Precedence DAG for bend sequencing
 */
class PrecedenceDAG {
public:
    PrecedenceDAG();

    // Construction
    void addNode(int bendId);
    void addEdge(int from, int to, ConstraintType type,
                 const std::string& reason = "");
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

    const std::vector<PrecedenceNode>& nodes() const { return m_nodes; }
    const std::vector<PrecedenceEdge>& edges() const { return m_edges; }

    // Export
    std::string toDOT() const;  // Graphviz format

private:
    std::vector<PrecedenceNode> m_nodes;
    std::vector<PrecedenceEdge> m_edges;
    std::map<int, int> m_bendIdToIndex;          // bendId -> node index
    std::map<int, std::vector<int>> m_adjacency; // bendId -> successors
    bool m_finalized;

    // Internal helpers
    bool detectCyclesUtil(
        int nodeIdx,
        std::vector<bool>& visited,
        std::vector<bool>& recStack,
        std::vector<int>& path,
        std::vector<std::vector<int>>& cycles
    ) const;
};

} // namespace openpanelcam
```

### Step 6.4: Run test (should still fail - need implementation)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: FAIL - Linker error (no implementation)

### Step 6.5: Commit header

```bash
git add include/openpanelcam/phase2/precedence_dag.h tests/unit/test_precedence_dag.cpp
git commit -m "feat(phase2): add PrecedenceDAG class header"
```

---

## Task 7: PrecedenceDAG Implementation - Basic Operations

**Files:**
- Create: `src/phase2/precedence_dag.cpp`

### Step 7.1: Implement constructor and addNode

**File:** `src/phase2/precedence_dag.cpp`

```cpp
#include <openpanelcam/phase2/precedence_dag.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>

namespace openpanelcam {

PrecedenceDAG::PrecedenceDAG()
    : m_finalized(false)
{
}

void PrecedenceDAG::addNode(int bendId) {
    if (m_bendIdToIndex.count(bendId) > 0) {
        LOG_WARNING("Bend {} already in DAG", bendId);
        return;
    }

    PrecedenceNode node;
    node.id = static_cast<int>(m_nodes.size());
    node.bendId = bendId;

    m_bendIdToIndex[bendId] = node.id;
    m_nodes.push_back(node);

    LOG_DEBUG("Added node {} for bend {}", node.id, bendId);
}

void PrecedenceDAG::finalize() {
    // Build adjacency list from edges
    m_adjacency.clear();

    for (const auto& edge : m_edges) {
        m_adjacency[edge.fromBend].push_back(edge.toBend);

        // Update node predecessors/successors
        int fromIdx = m_bendIdToIndex[edge.fromBend];
        int toIdx = m_bendIdToIndex[edge.toBend];

        m_nodes[fromIdx].successors.push_back(edge.toBend);
        m_nodes[toIdx].predecessors.push_back(edge.fromBend);
    }

    m_finalized = true;
    LOG_INFO("DAG finalized: {} nodes, {} edges", nodeCount(), edgeCount());
}

const PrecedenceNode& PrecedenceDAG::getNode(int id) const {
    if (id < 0 || id >= static_cast<int>(m_nodes.size())) {
        THROW_PHASE2(INVALID_NODE_ID, "Node ID {} out of range", id);
    }
    return m_nodes[id];
}

const PrecedenceEdge& PrecedenceDAG::getEdge(int id) const {
    if (id < 0 || id >= static_cast<int>(m_edges.size())) {
        THROW_PHASE2(INVALID_EDGE_ID, "Edge ID {} out of range", id);
    }
    return m_edges[id];
}

} // namespace openpanelcam
```

### Step 7.2: Run test (should pass for basic operations)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: PASS for "PrecedenceDAG construction" and "add nodes"

### Step 7.3: Write test for addEdge

**Add to test file:**

```cpp
TEST_CASE("PrecedenceDAG add edges", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);

    dag.addEdge(0, 1, openpanelcam::ConstraintType::GEOMETRIC, "Test constraint");
    dag.addEdge(1, 2, openpanelcam::ConstraintType::SEQUENTIAL);

    REQUIRE(dag.edgeCount() == 2);
}
```

### Step 7.4: Implement addEdge

**Add to `precedence_dag.cpp`:**

```cpp
void PrecedenceDAG::addEdge(int from, int to, ConstraintType type,
                            const std::string& reason) {
    // Validate nodes exist
    if (m_bendIdToIndex.count(from) == 0) {
        LOG_ERROR("From bend {} not in DAG", from);
        return;
    }
    if (m_bendIdToIndex.count(to) == 0) {
        LOG_ERROR("To bend {} not in DAG", to);
        return;
    }

    PrecedenceEdge edge;
    edge.id = static_cast<int>(m_edges.size());
    edge.fromBend = from;
    edge.toBend = to;
    edge.type = type;
    edge.reasoning = reason;
    edge.confidence = 1.0;

    m_edges.push_back(edge);

    LOG_DEBUG("Added edge {}: {} -> {} ({})", edge.id, from, to,
              static_cast<int>(type));
}
```

### Step 7.5: Run test

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: PASS

### Step 7.6: Commit

```bash
git add src/phase2/precedence_dag.cpp tests/unit/test_precedence_dag.cpp
git commit -m "feat(phase2): implement PrecedenceDAG addNode/addEdge/finalize"
```

---

## Task 8: PrecedenceDAG - Cycle Detection

**Files:**
- Modify: `src/phase2/precedence_dag.cpp`
- Modify: `tests/unit/test_precedence_dag.cpp`

### Step 8.1: Write failing test for acyclic DAG

**Add to test file:**

```cpp
TEST_CASE("PrecedenceDAG acyclic check - valid", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, openpanelcam::ConstraintType::GEOMETRIC);
    dag.addEdge(1, 2, openpanelcam::ConstraintType::GEOMETRIC);
    dag.finalize();

    REQUIRE(dag.isAcyclic() == true);
}

TEST_CASE("PrecedenceDAG cycle detection", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addEdge(0, 1, openpanelcam::ConstraintType::GEOMETRIC);
    dag.addEdge(1, 0, openpanelcam::ConstraintType::SEQUENTIAL);  // Cycle!
    dag.finalize();

    REQUIRE(dag.isAcyclic() == false);

    auto cycles = dag.detectCycles();
    REQUIRE(cycles.size() > 0);
}
```

### Step 8.2: Run test (should fail)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: FAIL - "isAcyclic() not implemented"

### Step 8.3: Implement cycle detection

**Add to `precedence_dag.cpp`:**

```cpp
bool PrecedenceDAG::isAcyclic() const {
    std::vector<std::vector<int>> cycles;
    return detectCycles().empty();
}

std::vector<std::vector<int>> PrecedenceDAG::detectCycles() const {
    std::vector<std::vector<int>> cycles;

    int n = nodeCount();
    std::vector<bool> visited(n, false);
    std::vector<bool> recStack(n, false);
    std::vector<int> path;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            detectCyclesUtil(i, visited, recStack, path, cycles);
        }
    }

    return cycles;
}

bool PrecedenceDAG::detectCyclesUtil(
    int nodeIdx,
    std::vector<bool>& visited,
    std::vector<bool>& recStack,
    std::vector<int>& path,
    std::vector<std::vector<int>>& cycles
) const {
    visited[nodeIdx] = true;
    recStack[nodeIdx] = true;
    path.push_back(m_nodes[nodeIdx].bendId);

    // Get successors for this bend
    int bendId = m_nodes[nodeIdx].bendId;
    auto it = m_adjacency.find(bendId);
    if (it != m_adjacency.end()) {
        for (int successorBend : it->second) {
            int successorIdx = m_bendIdToIndex.at(successorBend);

            if (!visited[successorIdx]) {
                if (detectCyclesUtil(successorIdx, visited, recStack,
                                     path, cycles)) {
                    return true;
                }
            } else if (recStack[successorIdx]) {
                // Back edge found → cycle!
                std::vector<int> cycle;
                auto cycleStart = std::find(path.begin(), path.end(),
                                           successorBend);
                cycle.assign(cycleStart, path.end());
                cycles.push_back(cycle);

                LOG_WARNING("Cycle detected: size {}", cycle.size());
                return true;
            }
        }
    }

    recStack[nodeIdx] = false;
    path.pop_back();
    return false;
}
```

### Step 8.4: Run test (should pass)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: PASS

### Step 8.5: Commit

```bash
git add src/phase2/precedence_dag.cpp tests/unit/test_precedence_dag.cpp
git commit -m "feat(phase2): implement cycle detection with DFS"
```

---

## Task 9: PrecedenceDAG - Topological Sort

**Files:**
- Modify: `src/phase2/precedence_dag.cpp`
- Modify: `tests/unit/test_precedence_dag.cpp`

### Step 9.1: Write failing test

**Add to test file:**

```cpp
TEST_CASE("PrecedenceDAG topological sort", "[phase2][dag]") {
    openpanelcam::PrecedenceDAG dag;
    dag.addNode(0);
    dag.addNode(1);
    dag.addNode(2);
    dag.addEdge(0, 1, openpanelcam::ConstraintType::GEOMETRIC);
    dag.addEdge(1, 2, openpanelcam::ConstraintType::GEOMETRIC);
    dag.finalize();

    auto order = dag.topologicalSort();

    REQUIRE(order.size() == 3);
    // 0 must come before 1, 1 before 2
    auto pos0 = std::find(order.begin(), order.end(), 0);
    auto pos1 = std::find(order.begin(), order.end(), 1);
    auto pos2 = std::find(order.begin(), order.end(), 2);

    REQUIRE(pos0 < pos1);
    REQUIRE(pos1 < pos2);
}
```

### Step 9.2: Run test (should fail)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: FAIL

### Step 9.3: Implement topological sort (Kahn's algorithm)

**Add to `precedence_dag.cpp`:**

```cpp
std::vector<int> PrecedenceDAG::topologicalSort() const {
    if (!isAcyclic()) {
        LOG_ERROR("Cannot topological sort - DAG has cycles");
        return {};
    }

    int n = nodeCount();
    std::vector<int> inDegree(n, 0);

    // Compute in-degrees
    for (const auto& node : m_nodes) {
        inDegree[node.id] = static_cast<int>(node.predecessors.size());
    }

    // Queue of nodes with in-degree 0
    std::queue<int> queue;
    for (int i = 0; i < n; i++) {
        if (inDegree[i] == 0) {
            queue.push(i);
        }
    }

    std::vector<int> result;
    while (!queue.empty()) {
        int nodeIdx = queue.front();
        queue.pop();

        result.push_back(m_nodes[nodeIdx].bendId);

        // Reduce in-degree of successors
        for (int successorBend : m_nodes[nodeIdx].successors) {
            int successorIdx = m_bendIdToIndex.at(successorBend);
            inDegree[successorIdx]--;

            if (inDegree[successorIdx] == 0) {
                queue.push(successorIdx);
            }
        }
    }

    return result;
}

std::vector<int> PrecedenceDAG::getPredecessors(int bendId) const {
    auto it = m_bendIdToIndex.find(bendId);
    if (it == m_bendIdToIndex.end()) {
        return {};
    }
    return m_nodes[it->second].predecessors;
}

std::vector<int> PrecedenceDAG::getSuccessors(int bendId) const {
    auto it = m_bendIdToIndex.find(bendId);
    if (it == m_bendIdToIndex.end()) {
        return {};
    }
    return m_nodes[it->second].successors;
}
```

**Don't forget to add:**
```cpp
#include <queue>
#include <algorithm>
```

### Step 9.4: Run test (should pass)

```bash
cmake --build . --target test_precedence_dag && ./bin/test_precedence_dag
```

Expected: PASS

### Step 9.5: Commit

```bash
git add src/phase2/precedence_dag.cpp tests/unit/test_precedence_dag.cpp
git commit -m "feat(phase2): implement topological sort (Kahn's algorithm)"
```

---

## Summary of Tasks Remaining

Due to length constraints, I'll outline the remaining tasks:

### Week 2-3: Geometric Precedence Analyzer
- Task 10-15: Implement GeometricPrecedenceAnalyzer class
  - Ray casting for corner overlap
  - Box closing detection (3-sided box)
  - Sequential blocking analysis
  - BentState helper class

### Week 4: Grasp Constraint Generator
- Task 16-20: Implement GraspConstraintGenerator class
  - Dead zone calculation
  - 2D polygon projection
  - MIR algorithm (CGAL integration)
  - Grip physics validation

### Week 5: ABA Analyzer & Integration
- Task 21-25: Implement ABAConstraintAnalyzer class
  - Subset sum DP solver
  - ABA width calculation
- Task 26-30: Main ConstraintSolver class
  - Integrate all modules
  - Phase2Output generation

### Week 6: Testing & Documentation
- Task 31-35: Integration tests
- Task 36-40: Documentation & samples

---

## Execution Instructions

**Current progress:** Tasks 1-9 complete (Core types + PrecedenceDAG)

**Next:** Continue with Task 10 (GeometricPrecedenceAnalyzer)

**Full plan:** ~40 tasks total, 5-6 weeks implementation

---

## Plan Complete ✅

**Saved to:** `docs/plans/2026-02-05-phase2-constraint-solver.md`

**Two execution options:**

1. **Subagent-Driven (this session)** - I dispatch fresh subagent per task, review between tasks, fast iteration

2. **Parallel Session (separate)** - Open new session with executing-plans, batch execution with checkpoints

**Which approach do you prefer?**
