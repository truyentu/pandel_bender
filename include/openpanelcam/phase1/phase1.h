#pragma once

/**
 * @file phase1.h
 * @brief Phase 1 integration - Main entry point for STEP parsing and feature extraction
 *
 * Integrates all Phase 1 modules:
 * 1. STEPReader - Load STEP file
 * 2. GeometryHealer - Heal/validate geometry
 * 3. FAGBuilder - Build Face-Adjacency Graph
 * 4. BaseFaceIdentifier - Identify base face
 * 5. BendClassifier - Classify all bends
 *
 * Usage:
 * @code
 *   Phase1Config config;
 *   config.thickness = 2.0;
 *   config.minBendRadius = 1.0;
 *   config.healGeometry = true;
 *
 *   Phase1Output output = parseSTEPFile("part.step", config);
 *
 *   if (output.success) {
 *       LOG_INFO("Parsed {} bends", output.bends.size());
 *       for (const auto& bend : output.bends) {
 *           LOG_INFO("  Bend {}: {} {:.1f}deg",
 *                    bend.id,
 *                    bend.direction == BendDirection::BEND_UP ? "UP" : "DOWN",
 *                    bend.targetAngle);
 *       }
 *   }
 * @endcode
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/phase1/fag.h>

#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

namespace openpanelcam {

/**
 * @brief Phase 1 configuration parameters
 */
struct Phase1Config {
    // Material properties
    double thickness;                    // Sheet thickness (mm)
    double kFactor;                      // Bend allowance K-factor (default: 0.5)

    // Geometry healing
    bool healGeometry;                   // Enable geometry healing (default: true)
    double healingTolerance;             // Healing tolerance (default: 0.01)
    bool fixSmallEdges;                  // Fix small edges (default: true)
    bool fixSmallFaces;                  // Fix small faces (default: true)
    bool sewFaces;                       // Sew faces (default: true)
    bool fixOrientation;                 // Fix face orientations (default: true)

    // FAG building
    double minBendRadius;                // Minimum bend radius (default: 0.5)
    double minFaceArea;                  // Minimum face area (default: 1.0)
    bool sdfValidation;                  // Enable SDF validation (default: false)

    // Base face identification
    double weightArea;                   // Area weight (default: 0.3)
    double weightConnectivity;           // Connectivity weight (default: 0.3)
    double weightCentrality;             // Centrality weight (default: 0.2)
    double weightOrientation;            // Orientation weight (default: 0.2)
    gp_Dir preferredNormal;              // Preferred base normal (default: Z-axis)

    // Bend classification
    double hemAngleThreshold;            // HEM angle threshold in degrees (default: 45.0)

    // Logging
    bool verbose;                        // Enable verbose logging (default: false)

    Phase1Config()
        : thickness(2.0)
        , kFactor(0.5)
        , healGeometry(true)
        , healingTolerance(0.01)
        , fixSmallEdges(true)
        , fixSmallFaces(true)
        , sewFaces(true)
        , fixOrientation(true)
        , minBendRadius(0.5)
        , minFaceArea(1.0)
        , sdfValidation(false)
        , weightArea(0.3)
        , weightConnectivity(0.3)
        , weightCentrality(0.2)
        , weightOrientation(0.2)
        , preferredNormal(0, 0, 1)  // Z-axis
        , hemAngleThreshold(45.0)
        , verbose(false)
    {}
};

/**
 * @brief Phase 1 output - Results of STEP parsing and feature extraction
 */
struct Phase1Output {
    // Status
    bool success;                        // Overall success flag
    std::string errorMessage;            // Error message if failed
    std::vector<std::string> warnings;   // Non-fatal warnings

    // Input info
    std::string filePath;                // Input STEP file path
    double parseTime;                    // Time spent parsing (ms)

    // Geometry
    TopoDS_Shape originalShape;          // Original shape from STEP
    TopoDS_Shape healedShape;            // Healed shape (if healing enabled)
    bool wasHealed;                      // True if healing was performed
    int healingIssuesFixed;              // Number of issues fixed during healing

    // FAG
    FaceAdjacencyGraph fag;              // Face-Adjacency Graph
    int baseFaceId;                      // ID of identified base face
    double baseFaceScore;                // Score of base face (0.0-1.0)

    // Bends
    std::vector<BendFeature> bends;      // All classified bends
    int bendCount;                       // Total number of bends
    int hemCount;                        // Number of HEMs
    int upBendCount;                     // Number of UP bends
    int downBendCount;                   // Number of DOWN bends

    // Statistics
    int totalFaces;                      // Total number of faces
    int planarFaces;                     // Number of planar faces
    int cylindricalFaces;                // Number of cylindrical faces
    int otherFaces;                      // Number of other faces

    Phase1Output()
        : success(false)
        , parseTime(0.0)
        , wasHealed(false)
        , healingIssuesFixed(0)
        , baseFaceId(-1)
        , baseFaceScore(0.0)
        , bendCount(0)
        , hemCount(0)
        , upBendCount(0)
        , downBendCount(0)
        , totalFaces(0)
        , planarFaces(0)
        , cylindricalFaces(0)
        , otherFaces(0)
    {}
};

/**
 * @brief Parse STEP file and extract Phase 1 features
 * @param filePath Path to STEP file
 * @param config Configuration parameters
 * @return Phase1Output with results
 *
 * Pipeline:
 * 1. Load STEP file (STEPReader)
 * 2. Heal geometry (GeometryHealer) - optional
 * 3. Build FAG (FAGBuilder)
 * 4. Identify base face (BaseFaceIdentifier)
 * 5. Classify bends (BendClassifier)
 *
 * @throws Phase1Exception on critical errors
 */
Phase1Output parseSTEPFile(const std::string& filePath, const Phase1Config& config = Phase1Config());

/**
 * @brief Validate Phase 1 output
 * @param output Output to validate
 * @return True if valid, false otherwise
 *
 * Checks:
 * - Base face exists and is valid
 * - All bends have valid base/flange references
 * - No orphaned faces (all planar faces are in FAG)
 * - Bend sequence constraints are not circular
 */
bool validatePhase1Output(const Phase1Output& output);

/**
 * @brief Print Phase 1 summary to log
 * @param output Output to summarize
 */
void printPhase1Summary(const Phase1Output& output);

} // namespace openpanelcam
