# OpenPanelCAM - Sample Applications

Các sample applications demo cách sử dụng OpenPanelCAM Phase 1 modules.

## Build Samples

```bash
mkdir build && cd build
cmake .. -DOPENPANELCAM_BUILD_SAMPLES=ON
cmake --build .
```

Executables sẽ được tạo trong `build/bin/`.

---

## Sample Programs

### 1. `test_core` - Test Core Utilities

Test các core utilities (Logger, Error handling, GeometryUtils).

**Usage:**
```bash
./test_core
```

---

### 2. `sample_parse_step` - Full Phase 1 Pipeline

Parse STEP file và extract tất cả bend features sử dụng full pipeline.

**Usage:**
```bash
./sample_parse_step <step_file> [options]

Options:
  --verbose           Enable verbose logging
  --thickness <mm>    Set sheet thickness (default: 2.0)
  --no-heal           Disable geometry healing
  --help              Show help message
```

**Examples:**
```bash
# Parse với default settings
./sample_parse_step part.step

# Parse với thickness tùy chỉnh và verbose logging
./sample_parse_step part.step --verbose --thickness 2.5

# Parse không healing
./sample_parse_step part.step --no-heal
```

**Output:**
- Parse time statistics
- Face counts (planar, cylindrical, other)
- Base face ID và score
- Bend counts (UP, DOWN, HEM)
- Detailed bend information (angle, radius, length)
- Warnings (if any)

---

### 3. `sample_phase1_modules` - Individual Module Demo

Demo cách sử dụng từng Phase 1 module riêng lẻ cho advanced use cases.

**Usage:**
```bash
./sample_phase1_modules <step_file>
```

**Example:**
```bash
./sample_phase1_modules part.step
```

**Shows:**
1. **STEPReader**: Load STEP file, unit conversion
2. **GeometryHealer**: Heal geometry với configurable options
3. **FAGBuilder**: Build Face-Adjacency Graph
4. **BaseFaceIdentifier**: Identify base face với scoring
5. **BendClassifier**: Classify all bends

**Use this when:**
- Bạn cần fine-grained control over mỗi step
- Debugging individual modules
- Custom pipeline với non-standard settings

---

### 4. `sample_visualize_fag` - Visualize FAG as JSON

Export Face-Adjacency Graph ra JSON format để visualization.

**Usage:**
```bash
./sample_visualize_fag <step_file> [output.json]
```

**Example:**
```bash
./sample_visualize_fag part.step fag.json
```

**Output JSON structure:**
```json
{
  "nodes": [
    {
      "id": 0,
      "type": "PLANAR",
      "isBaseFace": true,
      "area": 1234.5,
      "centroid": [0, 0, 0],
      "normal": [0, 0, 1]
    }
  ],
  "edges": [
    {
      "id": 0,
      "node1": 0,
      "node2": 1,
      "isBend": true,
      "angle": 90.0,
      "radius": 2.0
    }
  ],
  "bends": [
    {
      "id": 0,
      "direction": "UP",
      "angle": 90.0,
      "radius": 2.0,
      "length": 100.0
    }
  ]
}
```

---

## Test Data

Để test samples, bạn cần STEP files. Một số nguồn:

1. **GrabCAD**: https://grabcad.com/ (search "sheet metal")
2. **Thingiverse**: https://www.thingiverse.com/
3. **TraceParts**: https://www.traceparts.com/
4. **Tự tạo**: Sử dụng FreeCAD, SolidWorks, Inventor, etc.

**Recommended test parts:**
- Simple U-bracket (1-2 bends)
- L-bracket (1 bend)
- Box (4 bends)
- Complex enclosure (10+ bends)

---

## Expected Input

Phase 1 expects **sheet metal parts** với:
- Planar base face
- Cylindrical bend surfaces
- Clear bend edges (circular edges connecting faces)

**Good inputs:**
- Bent sheet metal parts
- Enclosures, brackets, panels
- Parts with clear bend lines

**Bad inputs:**
- Solid blocks (no bends)
- Organic shapes
- Parts với nhiều small features

---

## Troubleshooting

### "Failed to load STEP file"
- Check file path
- Verify STEP file is valid (AP203, AP214, AP242)
- Try opening in CAD software first

### "No planar faces found"
- Input không phải sheet metal part
- Geometry bị corrupt
- Try với healing enabled

### "Base face identification failed"
- No clear largest face
- Try adjusting weights trong config
- May need manual base face selection

### "No bends found"
- Part không có bends
- minBendRadius quá cao
- Circular edges không được detect

---

## Logging

Tất cả samples ghi log ra files:
- `parse_step.log` - Full pipeline logs
- `demo_phase1.log` - Individual module logs

Enable verbose logging với `--verbose` flag để chi tiết hơn.

---

## Next Steps

Sau khi test samples:
1. Verify kết quả với CAD software
2. Compare bend counts
3. Check bend directions (UP/DOWN)
4. Validate angles và radii

Nếu có issues, check logs và adjust config parameters.
