#pragma once

#include <vector>
#include <string>
#include <array>
#include <cmath>

namespace openpanelcam {
namespace phase2 {

enum class ConstraintType {
    GEOMETRIC,
    BOX_CLOSING,
    SEQUENTIAL
};

enum class DeadZoneType {
    STANDING_FLANGE,
    SAFETY_MARGIN,
    ABA_INTERFERENCE
};

// Simple 2D point representation
struct Point2D {
    double x = 0.0;
    double y = 0.0;

    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

// Simple 3D point representation to avoid OpenCASCADE dependency in Phase 2
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Point3D() = default;
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

// 2D Polygon with geometric operations
struct Polygon2D {
    std::vector<Point2D> vertices;
    std::vector<std::vector<Point2D>> holes;

    // Calculate polygon area using shoelace formula
    double area() const {
        if (vertices.size() < 3) return 0.0;

        double sum = 0.0;
        for (size_t i = 0; i < vertices.size(); i++) {
            size_t j = (i + 1) % vertices.size();
            sum += vertices[i].x * vertices[j].y;
            sum -= vertices[j].x * vertices[i].y;
        }

        return std::abs(sum) / 2.0;
    }

    // Check if point is inside polygon (ray casting algorithm)
    bool contains(const Point2D& point) const {
        if (vertices.size() < 3) return false;

        int intersections = 0;
        for (size_t i = 0; i < vertices.size(); i++) {
            size_t j = (i + 1) % vertices.size();

            const Point2D& vi = vertices[i];
            const Point2D& vj = vertices[j];

            // Check if ray crosses edge
            if ((vi.y > point.y) != (vj.y > point.y)) {
                double x_intersection = (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x;
                if (point.x < x_intersection) {
                    intersections++;
                }
            }
        }

        return (intersections % 2) == 1;
    }

    // Calculate centroid
    Point2D centroid() const {
        if (vertices.empty()) return Point2D(0, 0);

        double cx = 0.0, cy = 0.0;
        double signedArea = 0.0;

        for (size_t i = 0; i < vertices.size(); i++) {
            size_t j = (i + 1) % vertices.size();
            double cross = vertices[i].x * vertices[j].y - vertices[j].x * vertices[i].y;
            signedArea += cross;
            cx += (vertices[i].x + vertices[j].x) * cross;
            cy += (vertices[i].y + vertices[j].y) * cross;
        }

        signedArea *= 0.5;
        cx /= (6.0 * signedArea);
        cy /= (6.0 * signedArea);

        return Point2D(cx, cy);
    }
};

// 2D Rectangle (axis-aligned bounding box)
struct Rectangle2D {
    Point2D bottomLeft;
    Point2D topRight;
    double width = 0.0;
    double height = 0.0;
    double area = 0.0;
    Point2D center;

    Rectangle2D() = default;

    Rectangle2D(const Point2D& bl, const Point2D& tr)
        : bottomLeft(bl), topRight(tr) {
        width = tr.x - bl.x;
        height = tr.y - bl.y;
        area = width * height;
        center = Point2D((bl.x + tr.x) / 2.0, (bl.y + tr.y) / 2.0);
    }

    // Check if point is inside rectangle
    bool contains(const Point2D& point) const {
        return point.x >= bottomLeft.x && point.x <= topRight.x &&
               point.y >= bottomLeft.y && point.y <= topRight.y;
    }
};

// Dead zone on base plane (forbidden grip region)
struct DeadZone {
    int id = -1;
    DeadZoneType type = DeadZoneType::STANDING_FLANGE;
    Polygon2D polygon;
    int causedByBend = -1;
    double safetyMargin = 0.0;

    // Calculate dead zone area
    double area() const {
        return polygon.area();
    }

    // Check if point is in dead zone
    bool contains(const Point2D& point) const {
        return polygon.contains(point);
    }
};

// Grasp constraint for a specific bend state
struct GraspConstraint {
    int stateId = -1;
    std::vector<int> bentBends;
    std::vector<DeadZone> deadZones;
    Polygon2D validRegion;
    double validArea = 0.0;
    Rectangle2D maxInscribedRect;
    Point2D optimalGripCenter;
    bool hasValidGrip = false;
    double minRequiredArea = 100.0;
    Point2D centerOfMass;
    std::vector<std::string> warnings;
};

// ABA tool constraint for a bend
struct ABAConstraint {
    int bendId = -1;
    double bendLength = 0.0;
    double requiredWidth = 0.0;
    double clearance = 0.0;
    std::vector<int> segmentSolution;
    int totalSegments = 0;
    double totalWidth = 0.0;
    bool feasible = false;
    bool isBoxClosing = false;
    std::string reason;
    std::vector<int> suggestedAlternatives;
};

struct PrecedenceNode {
    int id = -1;
    int bendId = -1;
    std::vector<int> predecessors;
    std::vector<int> successors;
    int level = 0;
    bool visited = false;
};

struct PrecedenceEdge {
    int id = -1;
    int fromBend = -1;
    int toBend = -1;
    ConstraintType type = ConstraintType::GEOMETRIC;
    double confidence = 1.0;
    std::string reasoning;
    Point3D conflictPoint;
};

} // namespace phase2
} // namespace openpanelcam
