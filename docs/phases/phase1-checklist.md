# Phase 1 Implementation Checklist

> **Status**: IN DESIGN (Documentation Complete)
> **Last Updated**: 2026-02-04

---

## Documentation Status

### Completed
- [x] MASTER_CONTEXT.md - Project context and session state
- [x] ROADMAP.md - Development roadmap for all phases
- [x] phases/phase1-design.md - Detailed Phase 1 design
- [x] data-structures/cross-phase-types.md - Shared types
- [x] data-structures/phase1-types.md - Phase 1 specific types
- [x] architecture/system-overview.md - System architecture

### Pending
- [ ] API reference documentation
- [ ] Usage examples
- [ ] Test case specifications

---

## Implementation Checklist

### 1. Project Setup
- [ ] Create CMakeLists.txt (root)
- [ ] Create vcpkg.json
- [ ] Set up directory structure
- [ ] Configure CI/CD (optional)

### 2. Core Module (`src/core/`)
- [ ] `types.h` - Core type definitions
- [ ] `constants.h` - Global constants
- [ ] `geometry.h/.cpp` - Geometry utilities
- [ ] `logging.h/.cpp` - Logging setup

### 3. Phase 1 Components

#### 3.1 STEPReader (`src/phase1/step_reader.cpp`)
- [ ] Load STEP file using STEPControl_Reader
- [ ] Handle AP203, AP214, AP242 variants
- [ ] Unit conversion (inch to mm)
- [ ] Error handling for invalid files
- [ ] Extract metadata (part name, etc.)

#### 3.2 GeometryHealer (`src/phase1/geometry_healer.cpp`)
- [ ] ShapeFix_Shape for basic repairs
- [ ] ShapeUpgrade_UnifySameDomain for face merging
- [ ] BRepBuilderAPI_Sewing for gap closing
- [ ] Validation of healed result
- [ ] Logging of issues found

#### 3.3 FAGBuilder (`src/phase1/fag_builder.cpp`)
- [ ] TopExp::MapShapesAndAncestors for adjacency
- [ ] Face classification (planar, cylindrical, etc.)
- [ ] Edge classification (bend vs sharp)
- [ ] Compute geometric properties
- [ ] Build adjacency map

#### 3.4 BaseFaceIdentifier (`src/phase1/base_face_identifier.cpp`)
- [ ] Scoring algorithm implementation
- [ ] Area scoring
- [ ] Connectivity scoring
- [ ] Centrality scoring
- [ ] Orientation scoring
- [ ] Handle ambiguous cases

#### 3.5 BendClassifier (`src/phase1/bend_classifier.cpp`)
- [ ] GetMaterialOutwardNormal()
- [ ] Signed Distance Function (SDF)
- [ ] UP/DOWN/HEM classification
- [ ] Bend angle calculation
- [ ] Bend radius extraction
- [ ] Confidence scoring

### 4. Key Algorithms

#### GetMaterialOutwardNormal
- [ ] UV bounds computation
- [ ] Surface properties at midpoint
- [ ] Orientation correction (TopAbs_REVERSED)
- [ ] Unit tests with known cases

#### FAG Construction
- [ ] O(n) complexity using MapShapesAndAncestors
- [ ] Correct handling of shared edges
- [ ] Distinguish bends from sharp edges

#### Bend Classification
- [ ] SDF calculation
- [ ] Direction determination
- [ ] Hem detection (< 45°)

### 5. Unit Tests

#### STEPReader Tests
- [ ] test_loads_valid_file
- [ ] test_rejects_invalid_file
- [ ] test_handles_unicode_path
- [ ] test_converts_inches_to_mm

#### GeometryHealer Tests
- [ ] test_fixes_split_faces
- [ ] test_unifies_coplanar
- [ ] test_preserves_valid_geometry

#### FAGBuilder Tests
- [ ] test_simple_box
- [ ] test_identifies_bends
- [ ] test_handles_sharp_edges
- [ ] test_connected_graph

#### BaseFaceIdentifier Tests
- [ ] test_finds_largest
- [ ] test_prefers_horizontal
- [ ] test_handles_tie

#### BendClassifier Tests
- [ ] test_up_bend
- [ ] test_down_bend
- [ ] test_hem
- [ ] test_reversed_normal

### 6. Integration Tests
- [ ] test_simple_box_4_bends
- [ ] test_l_bracket_1_bend
- [ ] test_complex_panel_8_bends
- [ ] test_part_with_holes
- [ ] test_part_with_hems
- [ ] test_dirty_geometry

### 7. Test Data Files
- [ ] simple_box.step
- [ ] l_bracket.step
- [ ] u_channel.step
- [ ] z_bracket.step
- [ ] complex_panel.step
- [ ] hem_example.step
- [ ] dirty_geometry.step
- [ ] holes_cutouts.step

---

## Performance Benchmarks

| Operation | Target | Actual | Status |
|-----------|--------|--------|--------|
| STEP Load | < 200ms | - | Not tested |
| Geometry Healing | < 300ms | - | Not tested |
| FAG Construction | < 100ms | - | Not tested |
| Base Face ID | < 10ms | - | Not tested |
| Bend Classification | < 50ms | - | Not tested |
| **Total Phase 1** | < 700ms | - | Not tested |

---

## Quality Gates

### Before Implementation
- [x] Design document complete
- [x] Data structures defined
- [x] Algorithms specified
- [x] Test cases identified

### Before Phase 2
- [ ] All unit tests pass
- [ ] Integration tests pass
- [ ] Performance targets met
- [ ] Code review complete
- [ ] Documentation updated

---

## Open Issues / Questions

1. **Multiple Solids**: Decision made to process first solid only
2. **Freeform Surfaces**: Reject as unsupported
3. **Zero-Radius Bends**: Treat as sharp edge

---

## Next Steps

1. Set up project structure and build system
2. Implement `src/core/` foundation
3. Implement STEPReader
4. Implement GeometryHealer
5. Implement FAGBuilder
6. Implement BaseFaceIdentifier
7. Implement BendClassifier
8. Write unit tests
9. Write integration tests
10. Performance optimization

---

## Notes for Implementation

### OCCT Gotchas
1. Always check `face.Orientation()` before using normals
2. Use `Handle<>` for OCCT objects (reference counted)
3. `ShapeFix` may produce different topology - re-map references
4. `UnifySameDomain` may merge faces - expect face count to change

### Debugging Tips
1. Use BRepTools::Write() to dump shapes for visualization
2. Use DRAW test harness for interactive debugging
3. Log face IDs and properties at each step
4. Visualize FAG as graph for verification
