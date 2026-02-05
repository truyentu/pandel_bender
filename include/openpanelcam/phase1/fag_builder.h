#pragma once

/**
 * @file fag_builder.h
 * @brief FAG (Face-Adjacency Graph) builder
 *
 * Implements O(n) FAG construction algorithm from S1:
 * - Uses TopExp::MapShapesAndAncestors for efficient edge→face mapping
 * - Classifies edges as BEND or SHARP (from S2)
 * - Extracts bend axis from cylindrical faces
 * - Tracks orientation (isReversed, isInternal)
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/phase1/fag.h>

#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <string>
#include <vector>

namespace openpanelcam {

/**
 * @brief Edge classification result
 */
struct EdgeClassification {
    bool isBend;                     // True if bend, false if sharp
    double radius;                   // Bend radius (0 if sharp)
    double angle;                    // Dihedral angle between faces
    double confidence;               // Classification confidence (0-1)
    std::string reasoning;           // Debug info

    EdgeClassification()
        : isBend(false)
        , radius(0.0)
        , angle(0.0)
        , confidence(0.0)
    {}
};

/**
 * @brief FAG Builder class
 *
 * Usage:
 * @code
 *   FAGBuilder builder;
 *   builder.setMinBendRadius(0.1);
 *
 *   FaceAdjacencyGraph fag = builder.build(healedSolid);
 *
 *   LOG_INFO("Built FAG: {} nodes, {} bends",
 *            fag.nodeCount(), fag.bendCount());
 * @endcode
 */
class FAGBuilder {
public:
    FAGBuilder();

    /**
     * @brief Build FAG from a solid
     * @param shape Input shape (should be TopoDS_Solid)
     * @return Complete FAG with all properties computed
     *
     * Algorithm (from S1):
     * 1. Extract all faces
     * 2. Build edge→face mapping using TopExp::MapShapesAndAncestors (O(n))
     * 3. For each edge with 2 adjacent faces:
     *    - Classify as BEND or SHARP
     *    - Create FAG_Edge
     *    - Extract bend properties if bend
     * 4. Finalize FAG (compute geometric properties)
     */
    FaceAdjacencyGraph build(const TopoDS_Shape& shape);

    /**
     * @brief Set minimum bend radius threshold
     * @param radius Minimum radius in mm (default: 0.1)
     *
     * Edges with radius below this are classified as SHARP.
     */
    void setMinBendRadius(double radius);

    /**
     * @brief Set minimum face area threshold
     * @param area Minimum area in mm² (default: 0.01)
     *
     * Faces below this area are ignored (likely artifacts).
     */
    void setMinFaceArea(double area);

    /**
     * @brief Enable/disable SDF normal validation
     * @param enable If true, validate normals with SDF (slower but more robust)
     */
    void setSDFValidation(bool enable);

    /**
     * @brief Get number of ignored faces
     * @return Count of faces ignored due to min area threshold
     */
    int getIgnoredFaceCount() const;

    /**
     * @brief Get warnings generated during build
     * @return List of warning messages
     */
    std::vector<std::string> getWarnings() const;

    /**
     * @brief Get build statistics
     * @return Summary string
     */
    std::string getStatistics() const;

private:
    // Configuration
    double m_minBendRadius;
    double m_minFaceArea;
    bool m_sdfValidation;

    // Statistics
    int m_ignoredFaceCount;
    int m_totalFaces;
    int m_totalEdges;
    int m_bendEdges;
    int m_sharpEdges;
    std::vector<std::string> m_warnings;

    // Internal methods

    /**
     * @brief Classify an edge connecting two faces
     * @param edge The shared edge
     * @param face1 First adjacent face
     * @param face2 Second adjacent face
     * @return Classification result
     *
     * Algorithm (from S1, S2):
     * 1. Analyze edge geometry (linear vs circular)
     * 2. If circular with radius > MIN_BEND_RADIUS → BEND
     * 3. Compute dihedral angle between faces
     * 4. Return classification with confidence
     */
    EdgeClassification classifyEdge(
        const TopoDS_Edge& edge,
        const TopoDS_Face& face1,
        const TopoDS_Face& face2
    );

    /**
     * @brief Check if edge is on boundary (only 1 adjacent face)
     * @param edgeFaceCount Number of faces sharing this edge
     * @return True if boundary edge
     */
    bool isBoundaryEdge(int edgeFaceCount) const;

    /**
     * @brief Add warning message
     */
    void addWarning(const std::string& message);

    /**
     * @brief Clear statistics
     */
    void clearStatistics();
};

} // namespace openpanelcam

