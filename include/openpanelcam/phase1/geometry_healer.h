#pragma once

/**
 * @file geometry_healer.h
 * @brief Geometry healing pipeline for STEP-imported shapes
 *
 * Implements healing pipeline from S4 research paper:
 * 1. ShapeFix_Shape - Basic shape fixing
 * 2. ShapeUpgrade_UnifySameDomain - Merge coplanar faces
 * 3. BRepBuilderAPI_Sewing - Close gaps
 *
 * Critical for sheet metal parts exported from CAD systems.
 */

#include <openpanelcam/core/types.h>

#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

namespace openpanelcam {

/**
 * @brief Geometry healing class
 *
 * Usage:
 * @code
 *   GeometryHealer healer;
 *   healer.setTolerance(1e-6);
 *   TopoDS_Shape healed = healer.heal(rawShape);
 *
 *   if (healer.hadCriticalIssues()) {
 *       LOG_WARNING("Healing had issues: {}", healer.getIssuesFound());
 *   }
 * @endcode
 */
class GeometryHealer {
public:
    GeometryHealer();

    /**
     * @brief Set linear tolerance
     * @param tol Tolerance in mm (default: 1e-6)
     */
    void setTolerance(double tol);

    /**
     * @brief Set angular tolerance
     * @param tol Angular tolerance in radians (default: 0.01)
     */
    void setAngularTolerance(double tol);

    /**
     * @brief Enable/disable UnifySameDomain step
     * @param enable If false, skip coplanar face merging
     */
    void setUnifySameDomain(bool enable);

    /**
     * @brief Enable/disable sewing step
     * @param enable If false, skip gap closing
     */
    void setSewing(bool enable);

    /**
     * @brief Full healing pipeline
     * @param input Raw shape from STEP
     * @return Healed shape
     *
     * Pipeline (from S4):
     * 1. fixShape() - Fix invalid vertices, edges, faces
     * 2. unifySameDomain() - Merge coplanar faces (critical for sheet metal!)
     * 3. sewFaces() - Close small gaps
     */
    TopoDS_Shape heal(const TopoDS_Shape& input);

    /**
     * @brief Step 1: Fix shape issues
     * @param input Shape to fix
     * @return Fixed shape
     *
     * Uses ShapeFix_Shape to fix:
     * - Invalid vertices
     * - Self-intersecting edges
     * - Missing faces
     */
    TopoDS_Shape fixShape(const TopoDS_Shape& input);

    /**
     * @brief Step 2: Unify coplanar faces
     * @param input Shape with possibly split faces
     * @return Shape with merged faces
     *
     * CRITICAL for sheet metal:
     * CAD exports often split planar faces unnecessarily.
     * This merges adjacent coplanar faces into single faces.
     *
     * Uses ShapeUpgrade_UnifySameDomain.
     */
    TopoDS_Shape unifySameDomain(const TopoDS_Shape& input);

    /**
     * @brief Step 3: Sew faces (close gaps)
     * @param input Shape with potential gaps
     * @return Sewn shape
     *
     * Closes small gaps between faces.
     * Uses BRepBuilderAPI_Sewing.
     */
    TopoDS_Shape sewFaces(const TopoDS_Shape& input);

    /**
     * @brief Get issues found during healing
     * @return List of issue descriptions
     */
    std::vector<std::string> getIssuesFound() const;

    /**
     * @brief Check if critical issues were found
     * @return true if healing encountered critical problems
     */
    bool hadCriticalIssues() const;

    /**
     * @brief Get healing statistics
     * @return Summary string
     */
    std::string getStatistics() const;

private:
    // Configuration
    double m_tolerance;
    double m_angularTolerance;
    bool m_enableUnifySameDomain;
    bool m_enableSewing;

    // Statistics
    std::vector<std::string> m_issues;
    bool m_hadCriticalIssues;
    int m_facesBeforeUnify;
    int m_facesAfterUnify;
    int m_edgesBeforeSew;
    int m_edgesAfterSew;

    // Internal helpers
    void clearStatistics();
    void addIssue(const std::string& issue, bool critical = false);
    int countFaces(const TopoDS_Shape& shape) const;
    int countEdges(const TopoDS_Shape& shape) const;
};

} // namespace openpanelcam

