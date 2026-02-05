/**
 * @file geometry_healer.cpp
 * @brief Geometry healing implementation
 */

#include <openpanelcam/phase1/geometry_healer.h>
#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/constants.h>

// OCCT Shape Healing
#include <ShapeFix_Shape.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>

// Topology
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <sstream>

namespace openpanelcam {

GeometryHealer::GeometryHealer()
    : m_tolerance(constants::LINEAR_TOLERANCE)
    , m_angularTolerance(constants::ANGULAR_TOLERANCE)
    , m_enableUnifySameDomain(true)
    , m_enableSewing(true)
    , m_hadCriticalIssues(false)
    , m_facesBeforeUnify(0)
    , m_facesAfterUnify(0)
    , m_edgesBeforeSew(0)
    , m_edgesAfterSew(0)
{
}

void GeometryHealer::setTolerance(double tol) {
    m_tolerance = tol;
}

void GeometryHealer::setAngularTolerance(double tol) {
    m_angularTolerance = tol;
}

void GeometryHealer::setUnifySameDomain(bool enable) {
    m_enableUnifySameDomain = enable;
}

void GeometryHealer::setSewing(bool enable) {
    m_enableSewing = enable;
}

void GeometryHealer::clearStatistics() {
    m_issues.clear();
    m_hadCriticalIssues = false;
    m_facesBeforeUnify = 0;
    m_facesAfterUnify = 0;
    m_edgesBeforeSew = 0;
    m_edgesAfterSew = 0;
}

void GeometryHealer::addIssue(const std::string& issue, bool critical) {
    m_issues.push_back(issue);
    if (critical) {
        m_hadCriticalIssues = true;
    }
    LOG_DEBUG("Healing issue (critical={}): {}", critical, issue);
}

int GeometryHealer::countFaces(const TopoDS_Shape& shape) const {
    if (shape.IsNull()) return 0;

    int count = 0;
    TopExp_Explorer exp(shape, TopAbs_FACE);
    for (; exp.More(); exp.Next()) {
        count++;
    }
    return count;
}

int GeometryHealer::countEdges(const TopoDS_Shape& shape) const {
    if (shape.IsNull()) return 0;

    int count = 0;
    TopExp_Explorer exp(shape, TopAbs_EDGE);
    for (; exp.More(); exp.Next()) {
        count++;
    }
    return count;
}

TopoDS_Shape GeometryHealer::heal(const TopoDS_Shape& input) {
    clearStatistics();

    if (input.IsNull()) {
        addIssue("Input shape is null", true);
        THROW_PHASE1(ErrorCode::GEOMETRY_INVALID,
                    "Cannot heal null shape",
                    "Input shape is null");
        return input;
    }

    LOG_INFO("Starting geometry healing pipeline");
    LOG_DEBUG("Initial shape: {} faces, {} edges",
              countFaces(input), countEdges(input));

    TopoDS_Shape shape = input;

    try {
        // Step 1: Fix shape
        LOG_DEBUG("Step 1: Fixing shape issues");
        shape = fixShape(shape);

        // Step 2: Unify same domain (merge coplanar faces)
        if (m_enableUnifySameDomain) {
            LOG_DEBUG("Step 2: Unifying coplanar faces");
            m_facesBeforeUnify = countFaces(shape);
            shape = unifySameDomain(shape);
            m_facesAfterUnify = countFaces(shape);

            int merged = m_facesBeforeUnify - m_facesAfterUnify;
            if (merged > 0) {
                LOG_INFO("Merged {} coplanar faces ({} -> {})",
                         merged, m_facesBeforeUnify, m_facesAfterUnify);
            }
        } else {
            LOG_DEBUG("Step 2: Skipped (UnifySameDomain disabled)");
        }

        // Step 3: Sew faces
        if (m_enableSewing) {
            LOG_DEBUG("Step 3: Sewing faces");
            m_edgesBeforeSew = countEdges(shape);
            shape = sewFaces(shape);
            m_edgesAfterSew = countEdges(shape);

            int sewn = m_edgesBeforeSew - m_edgesAfterSew;
            if (sewn > 0) {
                LOG_INFO("Sewn {} gaps ({} -> {} edges)",
                         sewn, m_edgesBeforeSew, m_edgesAfterSew);
            }
        } else {
            LOG_DEBUG("Step 3: Skipped (Sewing disabled)");
        }

        // Final validation
        BRepCheck_Analyzer analyzer(shape);
        if (!analyzer.IsValid()) {
            addIssue("Shape still has validation issues after healing", false);
            LOG_WARNING("Shape has remaining issues after healing");
        } else {
            LOG_INFO("Healing complete - shape is valid");
        }

        LOG_DEBUG("Final shape: {} faces, {} edges",
                  countFaces(shape), countEdges(shape));

    } catch (const Standard_Failure& e) {
        std::string msg = "OCCT healing error: " + std::string(e.GetMessageString());
        addIssue(msg, true);
        LOG_ERROR("{}", msg);
        THROW_OCCT("Geometry healing failed", msg);
    } catch (const std::exception& e) {
        std::string msg = "Healing error: " + std::string(e.what());
        addIssue(msg, true);
        LOG_ERROR("{}", msg);
        throw;
    }

    return shape;
}

TopoDS_Shape GeometryHealer::fixShape(const TopoDS_Shape& input) {
    if (input.IsNull()) {
        return input;
    }

    try {
        // Create shape fixer
        ShapeFix_Shape fixer(input);

        // Set tolerance
        fixer.SetPrecision(m_tolerance);
        fixer.SetMaxTolerance(m_tolerance * 10.0);

        // Perform fixing
        fixer.Perform();

        // Get result
        TopoDS_Shape result = fixer.Shape();

        // Check if anything was fixed
        if (result.IsSame(input)) {
            LOG_DEBUG("fixShape: No issues found");
        } else {
            LOG_DEBUG("fixShape: Shape was modified");
            addIssue("Shape required basic fixing", false);
        }

        return result;

    } catch (const Standard_Failure& e) {
        std::string msg = "Shape fixing failed: " + std::string(e.GetMessageString());
        addIssue(msg, false);
        LOG_WARNING("{}", msg);
        return input;  // Return original if fixing fails
    }
}

TopoDS_Shape GeometryHealer::unifySameDomain(const TopoDS_Shape& input) {
    if (input.IsNull()) {
        return input;
    }

    try {
        // Create unifier
        ShapeUpgrade_UnifySameDomain unifier(input);

        // Set tolerances
        unifier.SetLinearTolerance(m_tolerance);
        unifier.SetAngularTolerance(m_angularTolerance);

        // CRITICAL: Enable face unification
        // This merges adjacent coplanar faces (essential for sheet metal!)
        unifier.AllowInternalEdges(Standard_False);

        // Build result
        unifier.Build();

        // Get result
        TopoDS_Shape result = unifier.Shape();

        if (result.IsNull()) {
            addIssue("UnifySameDomain produced null shape", false);
            LOG_WARNING("UnifySameDomain failed, returning original");
            return input;
        }

        return result;

    } catch (const Standard_Failure& e) {
        std::string msg = "UnifySameDomain failed: " + std::string(e.GetMessageString());
        addIssue(msg, false);
        LOG_WARNING("{}", msg);
        return input;
    }
}

TopoDS_Shape GeometryHealer::sewFaces(const TopoDS_Shape& input) {
    if (input.IsNull()) {
        return input;
    }

    try {
        // Create sewer
        BRepBuilderAPI_Sewing sewer(m_tolerance);

        // Add shape
        sewer.Add(input);

        // Perform sewing
        sewer.Perform();

        // Get result
        TopoDS_Shape result = sewer.SewedShape();

        if (result.IsNull()) {
            addIssue("Sewing produced null shape", false);
            LOG_WARNING("Sewing failed, returning original");
            return input;
        }

        // Check if sewing did anything
        int nbFree = sewer.NbFreeEdges();
        int nbContig = sewer.NbContigousEdges();

        if (nbFree > 0) {
            LOG_DEBUG("Sewing: {} free edges remain", nbFree);
        }
        if (nbContig > 0) {
            LOG_DEBUG("Sewing: {} contiguous edges found", nbContig);
        }

        return result;

    } catch (const Standard_Failure& e) {
        std::string msg = "Sewing failed: " + std::string(e.GetMessageString());
        addIssue(msg, false);
        LOG_WARNING("{}", msg);
        return input;
    }
}

std::vector<std::string> GeometryHealer::getIssuesFound() const {
    return m_issues;
}

bool GeometryHealer::hadCriticalIssues() const {
    return m_hadCriticalIssues;
}

std::string GeometryHealer::getStatistics() const {
    std::ostringstream oss;

    oss << "Geometry Healing Statistics:\n";
    oss << "  Issues found: " << m_issues.size();
    if (m_hadCriticalIssues) {
        oss << " (CRITICAL)";
    }
    oss << "\n";

    if (m_enableUnifySameDomain && m_facesBeforeUnify > 0) {
        int merged = m_facesBeforeUnify - m_facesAfterUnify;
        oss << "  Faces unified: " << m_facesBeforeUnify
            << " -> " << m_facesAfterUnify
            << " (merged " << merged << ")\n";
    }

    if (m_enableSewing && m_edgesBeforeSew > 0) {
        int sewn = m_edgesBeforeSew - m_edgesAfterSew;
        oss << "  Edges sewn: " << m_edgesBeforeSew
            << " -> " << m_edgesAfterSew
            << " (sewn " << sewn << ")\n";
    }

    if (!m_issues.empty()) {
        oss << "  Issues:\n";
        for (const auto& issue : m_issues) {
            oss << "    - " << issue << "\n";
        }
    }

    return oss.str();
}

} // namespace openpanelcam

