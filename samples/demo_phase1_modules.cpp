/**
 * @file demo_phase1_modules.cpp
 * @brief Sample: Demonstrate individual Phase 1 modules
 *
 * Shows how to use each Phase 1 module separately for advanced use cases.
 */

#include <openpanelcam/phase1/step_reader.h>
#include <openpanelcam/phase1/geometry_healer.h>
#include <openpanelcam/phase1/fag_builder.h>
#include <openpanelcam/phase1/base_face_identifier.h>
#include <openpanelcam/phase1/bend_classifier.h>
#include <openpanelcam/core/logger.h>

#include <iostream>

using namespace openpanelcam;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <step_file>\n";
        return 1;
    }

    std::string stepFile = argv[1];

    // Initialize logger
    Logger::initialize("demo_phase1.log", LogLevel::INFO);

    std::cout << "========================================\n";
    std::cout << "Demo: Phase 1 Individual Modules\n";
    std::cout << "========================================\n\n";

    try {
        // =====================================================================
        // 1. STEPReader - Load STEP file
        // =====================================================================
        std::cout << "1. Loading STEP file...\n";

        STEPReader reader;
        TopoDS_Shape shape = reader.read(stepFile);

        std::cout << "   - File loaded: " << stepFile << "\n";
        std::cout << "   - Units: " << reader.getUnits() << "\n";
        std::cout << "   - Unit converted: " << (reader.wasUnitConverted() ? "Yes" : "No") << "\n";
        std::cout << "   - Shape type: " << shape.ShapeType() << "\n\n";

        // =====================================================================
        // 2. GeometryHealer - Heal geometry
        // =====================================================================
        std::cout << "2. Healing geometry...\n";

        GeometryHealer healer;
        healer.setTolerance(0.01);

        GeometryHealer::HealingOptions options;
        options.fixSmallEdges = true;
        options.fixSmallFaces = true;
        options.sewFaces = true;
        options.fixOrientation = true;

        GeometryHealer::HealingResult healResult = healer.heal(shape, options);

        std::cout << "   - Success: " << (healResult.success ? "Yes" : "No") << "\n";
        std::cout << "   - Issues fixed: " << healResult.issuesFixed << "\n";
        std::cout << "   - Critical issues: " << (healResult.hadCriticalIssues ? "Yes" : "No") << "\n";
        std::cout << "   - Warnings: " << healResult.warnings.size() << "\n\n";

        TopoDS_Shape workingShape = healResult.success ? healResult.healedShape : shape;

        // =====================================================================
        // 3. FAGBuilder - Build Face-Adjacency Graph
        // =====================================================================
        std::cout << "3. Building Face-Adjacency Graph...\n";

        FAGBuilder fagBuilder;
        fagBuilder.setMinBendRadius(0.5);
        fagBuilder.setMinFaceArea(1.0);

        FaceAdjacencyGraph fag = fagBuilder.build(workingShape);

        std::cout << "   - Total faces: " << fag.nodeCount() << "\n";
        std::cout << "   - Bend edges: " << fag.bendCount() << "\n";
        std::cout << "   - FAG valid: " << (fag.isValid() ? "Yes" : "No") << "\n\n";

        // =====================================================================
        // 4. BaseFaceIdentifier - Identify base face
        // =====================================================================
        std::cout << "4. Identifying base face...\n";

        BaseFaceIdentifier baseFaceId;
        baseFaceId.setWeights(0.3, 0.3, 0.2, 0.2);  // area, connectivity, centrality, orientation
        baseFaceId.setPreferredNormal(gp_Dir(0, 0, 1));  // Z-axis (horizontal)

        int baseId = baseFaceId.identify(fag);

        auto candidates = baseFaceId.getCandidates();
        std::cout << "   - Base face ID: " << baseId << "\n";
        std::cout << "   - Score: " << (candidates.empty() ? 0.0 : candidates[0].score) << "\n";
        std::cout << "   - Total candidates: " << candidates.size() << "\n";

        // Show top 3 candidates
        std::cout << "   - Top 3 candidates:\n";
        for (size_t i = 0; i < std::min(size_t(3), candidates.size()); i++) {
            std::cout << "     " << (i+1) << ". Node " << candidates[i].nodeId
                     << " (score: " << candidates[i].score << ")\n";
        }
        std::cout << "\n";

        // Set base face in FAG
        fag.setBaseFace(baseId);

        // =====================================================================
        // 5. BendClassifier - Classify bends
        // =====================================================================
        std::cout << "5. Classifying bends...\n";

        BendClassifier classifier;
        classifier.setThickness(2.0);
        classifier.setHemAngleThreshold(45.0);
        classifier.setKFactor(0.5);

        std::vector<BendFeature> bends = classifier.classify(fag, baseId);

        std::cout << "   - Total bends: " << bends.size() << "\n";

        // Count bend types
        int upCount = 0, downCount = 0, hemCount = 0;
        for (const auto& bend : bends) {
            if (bend.direction == BendDirection::BEND_UP) upCount++;
            else if (bend.direction == BendDirection::BEND_DOWN) downCount++;
            else if (bend.direction == BendDirection::HEM) hemCount++;
        }

        std::cout << "   - UP bends: " << upCount << "\n";
        std::cout << "   - DOWN bends: " << downCount << "\n";
        std::cout << "   - HEMs: " << hemCount << "\n\n";

        // =====================================================================
        // Summary
        // =====================================================================
        std::cout << "========================================\n";
        std::cout << "SUMMARY\n";
        std::cout << "========================================\n";
        std::cout << "File:          " << stepFile << "\n";
        std::cout << "Faces:         " << fag.nodeCount() << "\n";
        std::cout << "Bends:         " << bends.size() << "\n";
        std::cout << "Base face:     " << baseId << "\n";
        std::cout << "========================================\n";
        std::cout << "\nSUCCESS\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
