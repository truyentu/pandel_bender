/**
 * @file base_face_identifier.cpp
 * @brief Base face identifier implementation
 */

#include <openpanelcam/phase1/base_face_identifier.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>

#include <algorithm>
#include <cmath>

namespace openpanelcam {

BaseFaceIdentifier::BaseFaceIdentifier()
    : m_weightArea(0.3)
    , m_weightConnectivity(0.3)
    , m_weightCentrality(0.2)
    , m_weightOrientation(0.2)
{
    // Default preferred normal: Z-axis (horizontal face)
    m_preferredNormal = gp_Dir(0, 0, 1);
}

void BaseFaceIdentifier::setWeights(
    double area,
    double connectivity,
    double centrality,
    double orientation
) {
    m_weightArea = area;
    m_weightConnectivity = connectivity;
    m_weightCentrality = centrality;
    m_weightOrientation = orientation;

    // Warn if weights don't sum to 1.0
    double sum = area + connectivity + centrality + orientation;
    if (std::abs(sum - 1.0) > 0.01) {
        LOG_WARNING("Base face weights sum to {:.2f} (expected 1.0)", sum);
    }
}

void BaseFaceIdentifier::setPreferredNormal(const gp_Dir& normal) {
    m_preferredNormal = normal;
}

int BaseFaceIdentifier::identify(const FaceAdjacencyGraph& fag) {
    m_candidates.clear();

    if (fag.nodeCount() == 0) {
        LOG_ERROR("Cannot identify base face - FAG is empty");
        return -1;
    }

    LOG_INFO("Identifying base face from {} nodes", fag.nodeCount());

    // Collect all planar faces as candidates
    std::vector<int> planarNodes;

    for (const auto& node : fag.nodes()) {
        if (node.type == FaceType::PLANAR) {
            planarNodes.push_back(node.id);
        }
    }

    if (planarNodes.empty()) {
        LOG_ERROR("No planar faces found in FAG");
        return -1;
    }

    LOG_DEBUG("Found {} planar face candidates", planarNodes.size());

    // Compute part centroid for centrality scoring
    Bnd_Box partBox = fag.getPartBoundingBox();
    double xmin, ymin, zmin, xmax, ymax, zmax;
    partBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    gp_Pnt partCentroid(
        (xmin + xmax) / 2.0,
        (ymin + ymax) / 2.0,
        (zmin + zmax) / 2.0
    );

    // Find max values for normalization
    double maxArea = 0.0;
    int maxBendCount = 0;
    double maxDistToCentroid = 0.0;

    for (int nodeId : planarNodes) {
        const FAG_Node& node = fag.getNode(nodeId);

        maxArea = std::max(maxArea, node.area);

        int bendCount = countBendEdges(nodeId, fag);
        maxBendCount = std::max(maxBendCount, bendCount);

        double dist = node.centroid.Distance(partCentroid);
        maxDistToCentroid = std::max(maxDistToCentroid, dist);
    }

    LOG_DEBUG("Normalization: maxArea={:.2f}, maxBendCount={}, maxDist={:.2f}",
              maxArea, maxBendCount, maxDistToCentroid);

    // Score each candidate
    for (int nodeId : planarNodes) {
        const FAG_Node& node = fag.getNode(nodeId);

        BaseFaceCandidate candidate;
        candidate.nodeId = nodeId;

        // Area score (normalized)
        if (maxArea > 0.0) {
            candidate.areaScore = node.area / maxArea;
        }

        // Connectivity score (more bends = better)
        int bendCount = countBendEdges(nodeId, fag);
        if (maxBendCount > 0) {
            candidate.connectivityScore = static_cast<double>(bendCount) / maxBendCount;
        }

        // Centrality score (closer to center = better)
        double dist = node.centroid.Distance(partCentroid);
        if (maxDistToCentroid > 0.0) {
            candidate.centralityScore = 1.0 - (dist / maxDistToCentroid);
        } else {
            candidate.centralityScore = 1.0;  // All at same distance
        }

        // Orientation score (aligned with preferred normal = better)
        if (m_preferredNormal.has_value()) {
            double dot = std::abs(node.normal.Dot(m_preferredNormal.value()));
            candidate.orientationScore = dot;  // 1.0 if parallel, 0.0 if perpendicular
        } else {
            candidate.orientationScore = 0.5;  // Neutral if no preference
        }

        // Total score (weighted sum)
        candidate.score =
            m_weightArea * candidate.areaScore +
            m_weightConnectivity * candidate.connectivityScore +
            m_weightCentrality * candidate.centralityScore +
            m_weightOrientation * candidate.orientationScore;

        m_candidates.push_back(candidate);

        LOG_DEBUG("Node {}: area={:.2f}, conn={:.2f}, cent={:.2f}, orient={:.2f} -> score={:.3f}",
                  nodeId,
                  candidate.areaScore,
                  candidate.connectivityScore,
                  candidate.centralityScore,
                  candidate.orientationScore,
                  candidate.score);
    }

    // Sort by score (descending)
    std::sort(m_candidates.begin(), m_candidates.end(),
              [](const BaseFaceCandidate& a, const BaseFaceCandidate& b) {
                  return a.score > b.score;
              });

    // Return best candidate
    if (!m_candidates.empty()) {
        int baseFaceId = m_candidates[0].nodeId;
        LOG_INFO("Identified base face: node {} (score: {:.3f})",
                 baseFaceId, m_candidates[0].score);

        // Log top 3 candidates for debugging
        int topN = std::min(3, static_cast<int>(m_candidates.size()));
        LOG_DEBUG("Top {} candidates:", topN);
        for (int i = 0; i < topN; i++) {
            LOG_DEBUG("  {}. Node {} - score {:.3f}",
                     i + 1, m_candidates[i].nodeId, m_candidates[i].score);
        }

        return baseFaceId;
    }

    return -1;
}

std::vector<BaseFaceCandidate> BaseFaceIdentifier::getCandidates() const {
    return m_candidates;
}

int BaseFaceIdentifier::countBendEdges(int nodeId, const FaceAdjacencyGraph& fag) const {
    std::vector<int> bendAdjacentNodes = fag.getBendAdjacentNodes(nodeId);
    return static_cast<int>(bendAdjacentNodes.size());
}

} // namespace openpanelcam
