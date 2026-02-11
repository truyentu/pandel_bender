/**
 * @file generate_test_steps.cpp
 * @brief Generate STEP test files for Phase 1 testing
 *
 * Creates sheet metal parts using OpenCASCADE:
 * 1. L-bracket (1 bend)
 * 2. U-channel (2 bends)
 * 3. Z-shape (2 bends, opposite directions)
 * 4. Hat-profile (4 bends)
 *
 * Uses BRepOffsetAPI_MakeThickSolid to create realistic sheet metal
 * with proper cylindrical bend surfaces.
 */

#include <openpanelcam/core/logger.h>

// OCCT - Modeling
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeWedge.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAlgoAPI_Fuse.hxx>

// OCCT - Primitives
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <gp_Ax2.hxx>

// OCCT - Topology
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>

// OCCT - STEP Export
#include <STEPControl_Writer.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Interface_Static.hxx>

// OCCT - Sweep / Prism
#include <BRepPrimAPI_MakePrism.hxx>

// Standard
#include <iostream>
#include <string>
#include <filesystem>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fs = std::filesystem;

/**
 * Export a shape to STEP file
 */
bool exportSTEP(const TopoDS_Shape& shape, const std::string& filepath) {
    try {
        STEPControl_Writer writer;

        // Set units to MM
        Interface_Static::SetCVal("xstep.cascade.unit", "MM");
        Interface_Static::SetCVal("write.step.unit", "MM");

        // Transfer shape
        IFSelect_ReturnStatus status = writer.Transfer(shape, STEPControl_AsIs);
        if (status != IFSelect_RetDone) {
            std::cerr << "STEP transfer failed for: " << filepath << std::endl;
            return false;
        }

        // Write file
        status = writer.Write(filepath.c_str());
        if (status != IFSelect_RetDone) {
            std::cerr << "STEP write failed for: " << filepath << std::endl;
            return false;
        }

        std::cout << "  Exported: " << filepath << std::endl;
        return true;

    } catch (const Standard_Failure& e) {
        std::cerr << "OCCT exception: " << e.GetMessageString() << std::endl;
        return false;
    }
}

/**
 * Count faces by type in a shape
 */
void printShapeInfo(const TopoDS_Shape& shape, const std::string& name) {
    TopTools_IndexedMapOfShape faces, edges;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);

    int planar = 0, cylindrical = 0, other = 0;
    for (int i = 1; i <= faces.Extent(); i++) {
        TopoDS_Face face = TopoDS::Face(faces(i));
        BRepAdaptor_Surface adaptor(face);
        switch (adaptor.GetType()) {
            case GeomAbs_Plane: planar++; break;
            case GeomAbs_Cylinder: cylindrical++; break;
            default: other++; break;
        }
    }

    std::cout << "  " << name << ": "
              << faces.Extent() << " faces ("
              << planar << " planar, "
              << cylindrical << " cylindrical, "
              << other << " other), "
              << edges.Extent() << " edges" << std::endl;
}

/**
 * Create a 2D wire profile and extrude + add fillets to create sheet metal
 *
 * Strategy:
 *   1. Build a flat wire profile (L, U, Z, etc.)
 *   2. Extrude into a thin face (prism)
 *   3. Add fillets at the bends to create cylindrical surfaces
 *
 * Alternative (more robust):
 *   1. Create a thick solid from the profile
 *   2. Fillet the inner edges
 */

/**
 * Create L-bracket: flat base + one 90° bend up
 *
 *      |  flange
 *      |
 *      |___________  base
 *
 * Approach: Create a box, fillet one edge → cylindrical bend surface
 */
TopoDS_Shape createLBracket(
    double baseLength,   // 100mm
    double flangeHeight, // 60mm
    double width,        // 80mm
    double thickness,    // 2mm
    double bendRadius    // 3mm (internal)
) {
    std::cout << "\n--- Creating L-Bracket ---" << std::endl;

    // Build the L-shape profile as a wire (XZ plane)
    // Profile: start at (0,0) → (baseLength,0) → (baseLength,flangeHeight)
    // Then offset by thickness to create the solid

    // Outer profile
    gp_Pnt p1(0, 0, 0);
    gp_Pnt p2(baseLength, 0, 0);
    gp_Pnt p3(baseLength, 0, flangeHeight);
    gp_Pnt p4(baseLength - thickness, 0, flangeHeight);
    gp_Pnt p5(baseLength - thickness, 0, thickness);
    gp_Pnt p6(0, 0, thickness);

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p4, p5).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p5, p6).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p6, p1).Edge());

    TopoDS_Wire wire = wireMaker.Wire();

    // Create face from wire
    BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
    TopoDS_Face face = faceMaker.Face();

    // Extrude along Y-axis to create solid
    gp_Vec extrudeDir(0, width, 0);
    BRepPrimAPI_MakePrism prism(face, extrudeDir);
    TopoDS_Shape solid = prism.Shape();

    // Add fillets at the two inner corners (bend radii)
    // Find edges at the L-corner
    BRepFilletAPI_MakeFillet fillet(solid);

    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

    // Find edges at the corner points (p2 and p5)
    // These are the edges running along Y at the L-corner
    for (int i = 1; i <= edgeMap.Extent(); i++) {
        TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
        BRepAdaptor_Curve curve(edge);

        if (curve.GetType() != GeomAbs_Line) continue;

        // Check if edge is parallel to Y-axis
        gp_Dir lineDir = curve.Line().Direction();
        if (std::abs(std::abs(lineDir.Y()) - 1.0) > 0.01) continue;

        // Get start point
        gp_Pnt startPt = curve.Value(curve.FirstParameter());

        // Outer corner at p2 (baseLength, y, 0)
        double dx2 = std::abs(startPt.X() - baseLength);
        double dz2 = std::abs(startPt.Z());
        if (dx2 < 0.1 && dz2 < 0.1) {
            fillet.Add(bendRadius + thickness, edge);  // outer radius
            continue;
        }

        // Inner corner at p5 (baseLength-thickness, y, thickness)
        double dx5 = std::abs(startPt.X() - (baseLength - thickness));
        double dz5 = std::abs(startPt.Z() - thickness);
        if (dx5 < 0.1 && dz5 < 0.1) {
            fillet.Add(bendRadius, edge);  // inner radius
            continue;
        }
    }

    TopoDS_Shape result = fillet.Shape();

    printShapeInfo(result, "L-Bracket");
    return result;
}

/**
 * Create U-channel: base + two 90° bends up
 *
 *    |          |
 *    |          |  flanges
 *    |__________|  base
 */
TopoDS_Shape createUChannel(
    double baseLength,   // 120mm
    double flangeHeight, // 50mm
    double width,        // 80mm
    double thickness,    // 2mm
    double bendRadius    // 3mm
) {
    std::cout << "\n--- Creating U-Channel ---" << std::endl;

    // U-profile in XZ plane
    gp_Pnt p1(0, 0, flangeHeight);
    gp_Pnt p2(0, 0, 0);
    gp_Pnt p3(baseLength, 0, 0);
    gp_Pnt p4(baseLength, 0, flangeHeight);
    gp_Pnt p5(baseLength - thickness, 0, flangeHeight);
    gp_Pnt p6(baseLength - thickness, 0, thickness);
    gp_Pnt p7(thickness, 0, thickness);
    gp_Pnt p8(thickness, 0, flangeHeight);

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p4, p5).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p5, p6).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p6, p7).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p7, p8).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p8, p1).Edge());

    TopoDS_Wire wire = wireMaker.Wire();
    BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
    TopoDS_Face face = faceMaker.Face();

    // Extrude
    gp_Vec extrudeDir(0, width, 0);
    BRepPrimAPI_MakePrism prism(face, extrudeDir);
    TopoDS_Shape solid = prism.Shape();

    // Add fillets at 4 corners (2 outer + 2 inner)
    BRepFilletAPI_MakeFillet fillet(solid);

    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

    for (int i = 1; i <= edgeMap.Extent(); i++) {
        TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
        BRepAdaptor_Curve curve(edge);

        if (curve.GetType() != GeomAbs_Line) continue;

        gp_Dir lineDir = curve.Line().Direction();
        if (std::abs(std::abs(lineDir.Y()) - 1.0) > 0.01) continue;

        gp_Pnt startPt = curve.Value(curve.FirstParameter());

        // Left outer corner at p2 (0, y, 0)
        if (std::abs(startPt.X()) < 0.1 && std::abs(startPt.Z()) < 0.1) {
            fillet.Add(bendRadius + thickness, edge);
            continue;
        }
        // Left inner corner at p7 (thickness, y, thickness)
        if (std::abs(startPt.X() - thickness) < 0.1 &&
            std::abs(startPt.Z() - thickness) < 0.1) {
            fillet.Add(bendRadius, edge);
            continue;
        }
        // Right outer corner at p3 (baseLength, y, 0)
        if (std::abs(startPt.X() - baseLength) < 0.1 &&
            std::abs(startPt.Z()) < 0.1) {
            fillet.Add(bendRadius + thickness, edge);
            continue;
        }
        // Right inner corner at p6 (baseLength-thickness, y, thickness)
        if (std::abs(startPt.X() - (baseLength - thickness)) < 0.1 &&
            std::abs(startPt.Z() - thickness) < 0.1) {
            fillet.Add(bendRadius, edge);
            continue;
        }
    }

    TopoDS_Shape result = fillet.Shape();

    printShapeInfo(result, "U-Channel");
    return result;
}

/**
 * Create Z-shape: two bends in opposite directions
 *
 *              ___________
 *             |
 *             |   middle
 *    _________|
 */
TopoDS_Shape createZShape(
    double segLength,    // 80mm each segment
    double offset,       // 40mm vertical offset
    double width,        // 80mm
    double thickness,    // 2mm
    double bendRadius    // 3mm
) {
    std::cout << "\n--- Creating Z-Shape ---" << std::endl;

    // Z-profile in XZ plane
    gp_Pnt p1(0, 0, 0);
    gp_Pnt p2(segLength, 0, 0);
    gp_Pnt p3(segLength, 0, offset);
    gp_Pnt p4(segLength + segLength, 0, offset);
    gp_Pnt p5(segLength + segLength, 0, offset + thickness);
    gp_Pnt p6(segLength + thickness, 0, offset + thickness);
    gp_Pnt p7(segLength + thickness, 0, thickness);
    gp_Pnt p8(0, 0, thickness);

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p4, p5).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p5, p6).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p6, p7).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p7, p8).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p8, p1).Edge());

    TopoDS_Wire wire = wireMaker.Wire();
    BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
    TopoDS_Face face = faceMaker.Face();

    // Extrude
    gp_Vec extrudeDir(0, width, 0);
    BRepPrimAPI_MakePrism prism(face, extrudeDir);
    TopoDS_Shape solid = prism.Shape();

    // Add fillets at corners
    BRepFilletAPI_MakeFillet fillet(solid);

    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

    for (int i = 1; i <= edgeMap.Extent(); i++) {
        TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
        BRepAdaptor_Curve curve(edge);

        if (curve.GetType() != GeomAbs_Line) continue;

        gp_Dir lineDir = curve.Line().Direction();
        if (std::abs(std::abs(lineDir.Y()) - 1.0) > 0.01) continue;

        gp_Pnt startPt = curve.Value(curve.FirstParameter());

        // Bottom-right outer corner at p2 (segLength, y, 0)
        if (std::abs(startPt.X() - segLength) < 0.1 &&
            std::abs(startPt.Z()) < 0.1) {
            fillet.Add(bendRadius + thickness, edge);
            continue;
        }
        // Bottom-right inner corner at p7 (segLength+thickness, y, thickness)
        if (std::abs(startPt.X() - (segLength + thickness)) < 0.1 &&
            std::abs(startPt.Z() - thickness) < 0.1) {
            fillet.Add(bendRadius, edge);
            continue;
        }
        // Top-left outer corner at p3 (segLength, y, offset)
        if (std::abs(startPt.X() - segLength) < 0.1 &&
            std::abs(startPt.Z() - offset) < 0.1) {
            fillet.Add(bendRadius + thickness, edge);
            continue;
        }
        // Top-left inner corner at p6 (segLength+thickness, y, offset+thickness)
        if (std::abs(startPt.X() - (segLength + thickness)) < 0.1 &&
            std::abs(startPt.Z() - (offset + thickness)) < 0.1) {
            fillet.Add(bendRadius, edge);
            continue;
        }
    }

    TopoDS_Shape result = fillet.Shape();

    printShapeInfo(result, "Z-Shape");
    return result;
}

/**
 * Create Hat-profile: 4 bends
 *
 *   ___         ___
 *  |   |_______|   |
 *  |               |
 *  |_______________|  (flanges on bottom)
 *
 * Actually simpler: base + 2 up flanges + 2 outward lips
 *
 *        ___________
 *  _____|           |_____
 *       |           |
 *       |___________|
 */
TopoDS_Shape createHatProfile(
    double baseWidth,    // 100mm
    double flangeHeight, // 40mm
    double lipLength,    // 30mm
    double width,        // 80mm
    double thickness,    // 2mm
    double bendRadius    // 3mm
) {
    std::cout << "\n--- Creating Hat Profile ---" << std::endl;

    double totalWidth = lipLength + baseWidth + lipLength;

    // Hat-profile in XZ plane
    // Bottom-left lip → up → base → up → bottom-right lip
    gp_Pnt p1(0, 0, flangeHeight);                          // left lip start
    gp_Pnt p2(lipLength, 0, flangeHeight);                   // left corner
    gp_Pnt p3(lipLength, 0, 0);                              // bottom left
    gp_Pnt p4(lipLength + baseWidth, 0, 0);                  // bottom right
    gp_Pnt p5(lipLength + baseWidth, 0, flangeHeight);       // right corner
    gp_Pnt p6(totalWidth, 0, flangeHeight);                  // right lip end

    // Inner profile
    gp_Pnt p7(totalWidth, 0, flangeHeight - thickness);
    gp_Pnt p8(lipLength + baseWidth - thickness, 0, flangeHeight - thickness);
    gp_Pnt p9(lipLength + baseWidth - thickness, 0, thickness);
    gp_Pnt p10(lipLength + thickness, 0, thickness);
    gp_Pnt p11(lipLength + thickness, 0, flangeHeight - thickness);
    gp_Pnt p12(0, 0, flangeHeight - thickness);

    BRepBuilderAPI_MakeWire wireMaker;
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p3, p4).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p4, p5).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p5, p6).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p6, p7).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p7, p8).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p8, p9).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p9, p10).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p10, p11).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p11, p12).Edge());
    wireMaker.Add(BRepBuilderAPI_MakeEdge(p12, p1).Edge());

    TopoDS_Wire wire = wireMaker.Wire();
    BRepBuilderAPI_MakeFace faceMaker(wire, Standard_True);
    TopoDS_Face face = faceMaker.Face();

    // Extrude
    gp_Vec extrudeDir(0, width, 0);
    BRepPrimAPI_MakePrism prism(face, extrudeDir);
    TopoDS_Shape solid = prism.Shape();

    // Add fillets at 8 corner edges (4 outer + 4 inner)
    BRepFilletAPI_MakeFillet fillet(solid);

    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(solid, TopAbs_EDGE, edgeMap);

    struct CornerDef {
        double x, z;
        double radius;
    };

    std::vector<CornerDef> corners = {
        // Outer corners
        {lipLength, flangeHeight, bendRadius + thickness},        // p2
        {lipLength, 0, bendRadius + thickness},                   // p3
        {lipLength + baseWidth, 0, bendRadius + thickness},       // p4
        {lipLength + baseWidth, flangeHeight, bendRadius + thickness}, // p5
        // Inner corners
        {lipLength + thickness, flangeHeight - thickness, bendRadius}, // p11
        {lipLength + thickness, thickness, bendRadius},                // p10
        {lipLength + baseWidth - thickness, thickness, bendRadius},    // p9
        {lipLength + baseWidth - thickness, flangeHeight - thickness, bendRadius}, // p8
    };

    for (int i = 1; i <= edgeMap.Extent(); i++) {
        TopoDS_Edge edge = TopoDS::Edge(edgeMap(i));
        BRepAdaptor_Curve curve(edge);

        if (curve.GetType() != GeomAbs_Line) continue;

        gp_Dir lineDir = curve.Line().Direction();
        if (std::abs(std::abs(lineDir.Y()) - 1.0) > 0.01) continue;

        gp_Pnt startPt = curve.Value(curve.FirstParameter());

        for (const auto& corner : corners) {
            if (std::abs(startPt.X() - corner.x) < 0.1 &&
                std::abs(startPt.Z() - corner.z) < 0.1) {
                fillet.Add(corner.radius, edge);
                break;
            }
        }
    }

    TopoDS_Shape result = fillet.Shape();

    printShapeInfo(result, "Hat-Profile");
    return result;
}

/**
 * Create simple flat plate (no bends) for baseline testing
 */
TopoDS_Shape createFlatPlate(
    double length,   // 200mm
    double width,    // 100mm
    double thickness // 2mm
) {
    std::cout << "\n--- Creating Flat Plate ---" << std::endl;

    BRepPrimAPI_MakeBox box(length, width, thickness);
    TopoDS_Shape result = box.Shape();

    printShapeInfo(result, "Flat-Plate");
    return result;
}

int main(int argc, char** argv) {
    std::cout << "======================================" << std::endl;
    std::cout << " OpenPanelCAM STEP Test File Generator" << std::endl;
    std::cout << "======================================" << std::endl;

    // Determine output directory
    std::string outputDir = "step_files";
    if (argc > 1) {
        outputDir = argv[1];
    }

    // Create output directory if needed
    fs::create_directories(outputDir);

    std::cout << "Output directory: " << fs::absolute(outputDir) << std::endl;

    // Common parameters
    double thickness = 2.0;   // mm
    double bendRadius = 3.0;  // mm (internal)

    int success = 0;
    int total = 0;

    // 1. Flat plate (no bends - baseline)
    {
        total++;
        TopoDS_Shape shape = createFlatPlate(200, 100, thickness);
        if (exportSTEP(shape, outputDir + "/flat_plate.step")) success++;
    }

    // 2. L-bracket (1 bend)
    {
        total++;
        TopoDS_Shape shape = createLBracket(100, 60, 80, thickness, bendRadius);
        if (exportSTEP(shape, outputDir + "/l_bracket.step")) success++;
    }

    // 3. U-channel (2 bends)
    {
        total++;
        TopoDS_Shape shape = createUChannel(120, 50, 80, thickness, bendRadius);
        if (exportSTEP(shape, outputDir + "/u_channel.step")) success++;
    }

    // 4. Z-shape (2 bends, opposite directions)
    {
        total++;
        TopoDS_Shape shape = createZShape(80, 40, 80, thickness, bendRadius);
        if (exportSTEP(shape, outputDir + "/z_shape.step")) success++;
    }

    // 5. Hat profile (4 bends)
    {
        total++;
        TopoDS_Shape shape = createHatProfile(100, 40, 30, 80, thickness, bendRadius);
        if (exportSTEP(shape, outputDir + "/hat_profile.step")) success++;
    }

    std::cout << "\n======================================" << std::endl;
    std::cout << " Results: " << success << "/" << total << " files generated" << std::endl;
    std::cout << "======================================" << std::endl;

    return (success == total) ? 0 : 1;
}
