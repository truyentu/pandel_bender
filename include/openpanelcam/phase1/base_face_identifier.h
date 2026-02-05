#pragma once

/**
 * @file base_face_identifier.h
 * @brief Base face identification using multi-criteria scoring
 *
 * Identifies the base face of a sheet metal part using:
 * - Area (larger is better)
 * - Connectivity (more bends = better)
 * - Centrality (closer to part center = better)
 * - Orientation (horizontal preferred)
 */

#include <openpanelcam/core/types.h>
#include <openpanelcam/phase1/fag.h>

#include <gp_Dir.hxx>
#include <vector>
#include <optional>

namespace openpanelcam {

/**
 * @brief Base face candidate with scoring details
 */
struct BaseFaceCandidate {
    int nodeId;                      // FAG node ID
    double score;                    // Total score

    // Scoring components
    double areaScore;                // Larger = higher
    double connectivityScore;        // More neighbors = higher
    double centralityScore;          // More central = higher
    double orientationScore;         // Horizontal preferred

    BaseFaceCandidate()
        : nodeId(-1)
        , score(0.0)
        , areaScore(0.0)
        , connectivityScore(0.0)
        , centralityScore(0.0)
        , orientationScore(0.0)
    {}
};

/**
 * @brief Base face identifier
 *
 * Usage:
 * @code
 *   BaseFaceIdentifier identifier;
 *   int baseFaceId = identifier.identify(fag);
 *
 *   // Get all candidates for debugging
 *   auto candidates = identifier.getCandidates();
 *   for (const auto& c : candidates) {
 *       LOG_DEBUG("Candidate {}: score={:.2f}", c.nodeId, c.score);
 *   }
 * @endcode
 */
class BaseFaceIdentifier {
public:
    BaseFaceIdentifier();

    /**
     * @brief Identify the base face in FAG
     * @param fag The Face-Adjacency Graph
     * @return Node ID of base face, or -1 if none found
     *
     * Algorithm:
     * 1. Collect all planar faces as candidates
     * 2. Score each candidate:
     *    - Area: normalized by max area
     *    - Connectivity: number of bend edges
     *    - Centrality: distance to part centroid
     *    - Orientation: dot product with Z-axis
     * 3. Total score = weighted sum
     * 4. Return highest scoring candidate
     */
    int identify(const FaceAdjacencyGraph& fag);

    /**
     * @brief Get all candidates with scores
     * @return Vector of candidates sorted by score (descending)
     */
    std::vector<BaseFaceCandidate> getCandidates() const;

    /**
     * @brief Set scoring weights
     * @param area Weight for area component (default: 0.3)
     * @param connectivity Weight for connectivity (default: 0.3)
     * @param centrality Weight for centrality (default: 0.2)
     * @param orientation Weight for orientation (default: 0.2)
     *
     * Weights should sum to 1.0 for normalized scores.
     */
    void setWeights(
        double area,
        double connectivity,
        double centrality,
        double orientation
    );

    /**
     * @brief Set preferred normal direction
     * @param normal Preferred normal (e.g., gp_Dir(0,0,1) for horizontal)
     *
     * Faces with normals aligned to this direction get higher orientation scores.
     */
    void setPreferredNormal(const gp_Dir& normal);

private:
    // Weights
    double m_weightArea;
    double m_weightConnectivity;
    double m_weightCentrality;
    double m_weightOrientation;

    // Preferred normal (optional)
    std::optional<gp_Dir> m_preferredNormal;

    // Results
    std::vector<BaseFaceCandidate> m_candidates;

    // Internal methods

    /**
     * @brief Score a single candidate
     */
    double scoreCandidate(
        const FAG_Node& node,
        const FaceAdjacencyGraph& fag,
        double maxArea,
        int maxBendCount,
        double maxDistToCentroid,
        const gp_Pnt& partCentroid
    );

    /**
     * @brief Count bend edges connected to node
     */
    int countBendEdges(int nodeId, const FaceAdjacencyGraph& fag) const;
};

} // namespace openpanelcam

