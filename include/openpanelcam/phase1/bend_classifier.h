#pragma once

/**
 * @file bend_classifier.h
 * @brief Bend classification using SDF-based direction detection
 *
 * Classifies bends in FAG:
 * - Direction: UP, DOWN, HEM (using SDF method)
 * - Type: ACUTE, RIGHT, OBTUSE, HEM
 * - Extracts complete BendFeature objects
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/phase1/fag.h>

#include <TopoDS_Solid.hxx>
#include <vector>

namespace openpanelcam {

/**
 * @brief Bend classification result
 */
struct BendClassificationResult {
    BendDirection direction;         // UP, DOWN, HEM
    double confidence;               // 0.0-1.0
    double signedDistance;           // SDF value
    std::string reasoning;           // Debug info

    BendClassificationResult()
        : direction(BendDirection::UNKNOWN)
        , confidence(0.0)
        , signedDistance(0.0)
    {}
};

/**
 * @brief Bend classifier
 *
 * Usage:
 * @code
 *   BendClassifier classifier;
 *   classifier.setThickness(2.0);
 *   classifier.setSolid(solid);  // For SDF validation
 *
 *   std::vector<BendFeature> bends = classifier.classify(fag, baseFaceId);
 *
 *   for (const auto& bend : bends) {
 *       LOG_INFO("Bend {}: {} {:.1f}°",
 *                bend.id,
 *                bend.direction == BendDirection::BEND_UP ? "UP" : "DOWN",
 *                bend.targetAngle);
 *   }
 * @endcode
 */
class BendClassifier {
public:
    BendClassifier();

    /**
     * @brief Classify all bends in FAG
     * @param fag The Face-Adjacency Graph
     * @param baseFaceId ID of the base face
     * @return Vector of BendFeature objects
     *
     * Algorithm:
     * 1. Get all bend edges from FAG
     * 2. For each bend edge:
     *    a. Determine which face is base vs flange
     *    b. Classify direction using SDF
     *    c. Extract bend geometry
     *    d. Create BendFeature
     * 3. Return all BendFeature objects
     */
    std::vector<BendFeature> classify(
        const FaceAdjacencyGraph& fag,
        int baseFaceId
    );

    /**
     * @brief Classify a single bend edge
     * @param edge The FAG bend edge
     * @param baseNode The base face node
     * @param flangeNode The flange face node
     * @return Classification result
     *
     * SDF Algorithm (from design spec):
     * 1. Get centroids: P_base, P_flange
     * 2. Get base normal: N_base (material-outward)
     * 3. Compute signed distance: D = (P_flange - P_base) · N_base
     * 4. If D > 0 → BEND_UP, if D < 0 → BEND_DOWN
     * 5. If |D| < thickness → might be HEM
     */
    BendClassificationResult classifyBend(
        const FAG_Edge& edge,
        const FAG_Node& baseNode,
        const FAG_Node& flangeNode
    );

    /**
     * @brief Set sheet thickness
     * @param thickness Thickness in mm
     *
     * Used for coplanarity checks and HEM detection.
     */
    void setThickness(double thickness);

    /**
     * @brief Set HEM angle threshold
     * @param angle Angle in degrees (default: 45.0)
     *
     * Bends with angle < threshold are classified as HEM.
     */
    void setHemAngleThreshold(double angle);

    /**
     * @brief Set solid for SDF validation (optional)
     * @param solid The containing solid
     *
     * If set, enables SDF-based normal validation.
     */
    void setSolid(const TopoDS_Solid& solid);

    /**
     * @brief Set default K-factor
     * @param kFactor K-factor value (0.0-1.0, default: 0.5)
     */
    void setKFactor(double kFactor);

private:
    // Configuration
    double m_thickness;
    double m_hemAngleThreshold;
    double m_kFactor;
    TopoDS_Solid m_solid;
    bool m_hasSolid;

    // Internal methods

    /**
     * @brief Extract complete BendFeature from FAG_Edge
     */
    BendFeature extractBendFeature(
        int bendId,
        const FAG_Edge& edge,
        const FAG_Node& baseNode,
        const FAG_Node& flangeNode,
        const BendClassificationResult& classification
    );

    /**
     * @brief Determine which node is base vs flange
     * @return {baseNodeId, flangeNodeId}
     */
    std::pair<int, int> determineBaseAndFlange(
        const FAG_Edge& edge,
        int baseFaceId,
        const FaceAdjacencyGraph& fag
    );

    /**
     * @brief Compute bend allowance
     * @param radius Internal radius
     * @param angle Bend angle in degrees
     * @param thickness Sheet thickness
     * @param kFactor K-factor
     * @return Bend allowance in mm
     *
     * Formula: BA = (π/180) × angle × (radius + k×thickness)
     */
    double computeBendAllowance(
        double radius,
        double angle,
        double thickness,
        double kFactor
    );

    /**
     * @brief Compute flange dimensions
     */
    void computeFlangeDimensions(
        BendFeature& bend,
        const FAG_Node& flangeNode
    );
};

} // namespace openpanelcam
