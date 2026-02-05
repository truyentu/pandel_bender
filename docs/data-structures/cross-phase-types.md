# Cross-Phase Data Structures

> **Purpose**: Định nghĩa các types được chia sẻ giữa các phases để đảm bảo compatibility.
> **Version**: 1.0

---

## 1. Core Identifier Types

```cpp
//=============================================================================
// Type Aliases for Clarity
//=============================================================================

using FaceId = int;
using EdgeId = int;
using BendId = int;
using NodeId = int;
using StepId = int;

// Invalid sentinel values
constexpr FaceId INVALID_FACE = -1;
constexpr BendId INVALID_BEND = -1;
constexpr NodeId INVALID_NODE = -1;
```

---

## 2. Geometry Primitives

```cpp
//=============================================================================
// Common Geometry Types (wrapping OCCT)
//=============================================================================

// 3D Point
struct Point3D {
    double x, y, z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    Point3D(const gp_Pnt& p) : x(p.X()), y(p.Y()), z(p.Z()) {}

    gp_Pnt toOCCT() const { return gp_Pnt(x, y, z); }

    double distanceTo(const Point3D& other) const;
    Point3D operator+(const Point3D& other) const;
    Point3D operator-(const Point3D& other) const;
    Point3D operator*(double scalar) const;
};

// 3D Vector/Direction
struct Vector3D {
    double x, y, z;

    Vector3D() : x(0), y(0), z(1) {}
    Vector3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    Vector3D(const gp_Dir& d) : x(d.X()), y(d.Y()), z(d.Z()) {}
    Vector3D(const gp_Vec& v) : x(v.X()), y(v.Y()), z(v.Z()) {}

    gp_Dir toDir() const { return gp_Dir(x, y, z); }
    gp_Vec toVec() const { return gp_Vec(x, y, z); }

    double dot(const Vector3D& other) const;
    Vector3D cross(const Vector3D& other) const;
    Vector3D normalized() const;
    double magnitude() const;
};

// Axis (point + direction)
struct Axis3D {
    Point3D origin;
    Vector3D direction;

    Axis3D() = default;
    Axis3D(const Point3D& o, const Vector3D& d) : origin(o), direction(d) {}
    Axis3D(const gp_Ax1& ax);

    gp_Ax1 toOCCT() const;
};

// Line (infinite)
struct Line3D {
    Point3D point;
    Vector3D direction;

    gp_Lin toOCCT() const;
    double distanceToPoint(const Point3D& p) const;
    Point3D projectPoint(const Point3D& p) const;
};

// Bounding Box
struct BoundingBox3D {
    Point3D min;
    Point3D max;

    BoundingBox3D() = default;
    BoundingBox3D(const Bnd_Box& box);

    Point3D center() const;
    Vector3D dimensions() const;
    double volume() const;
    bool intersects(const BoundingBox3D& other) const;
    void expand(const Point3D& point);
    void expand(const BoundingBox3D& other);
};
```

---

## 3. Material Properties

```cpp
//=============================================================================
// Material Definition (Used by Phase 1, 4, 5)
//=============================================================================

struct MaterialProperties {
    // Identification
    std::string materialId;        // e.g., "SS304", "AL5052-H32"
    std::string materialName;      // e.g., "Stainless Steel 304"

    // Physical properties
    double thickness;              // mm (nominal)
    double thicknessTolerance;     // mm (±)
    double density;                // g/cm³

    // Mechanical properties
    double elasticModulus;         // GPa (Young's modulus)
    double yieldStrength;          // MPa
    double tensileStrength;        // MPa
    double poissonRatio;           // dimensionless (typically 0.3)

    // Bending properties
    double kFactor;                // K-factor for bend allowance
    double springbackFactor;       // Springback compensation factor

    // Processing
    enum class GrainDirection { LONGITUDINAL, TRANSVERSE, UNKNOWN };
    GrainDirection grainDirection;

    // Default constructor with typical steel values
    MaterialProperties()
        : materialId("UNKNOWN")
        , thickness(2.0)
        , thicknessTolerance(0.1)
        , density(7.85)
        , elasticModulus(200.0)
        , yieldStrength(250.0)
        , tensileStrength(400.0)
        , poissonRatio(0.3)
        , kFactor(0.44)
        , springbackFactor(1.0)
        , grainDirection(GrainDirection::UNKNOWN)
    {}
};

// Predefined materials
namespace Materials {
    MaterialProperties MildSteel();
    MaterialProperties StainlessSteel304();
    MaterialProperties Aluminum5052H32();
    MaterialProperties Aluminum6061T6();
}
```

---

## 4. Bend Representation (Cross-Phase)

```cpp
//=============================================================================
// Bend Types (Used by all phases)
//=============================================================================

enum class BendType {
    STANDARD,       // Normal bend (< 120°)
    ACUTE,          // Sharp bend (> 120°)
    HEM,            // Fold-back (flattened)
    JOGGLE,         // Offset bend (Z-shape)
    UNKNOWN
};

enum class BendDirection {
    UP,             // Positive bend (flange goes up)
    DOWN,           // Negative bend (flange goes down)
    UNKNOWN
};

//=============================================================================
// Unified Bend Descriptor (Cross-Phase)
//=============================================================================

struct BendDescriptor {
    // Identity
    BendId id;
    std::string name;              // e.g., "Bend_1", "Flange_A"

    // Classification
    BendType type;
    BendDirection direction;

    // Geometry - Bend Line
    Line3D bendLine;
    double bendLineLength;         // mm

    // Geometry - Angles
    double nominalAngle;           // Target angle (degrees, 0-180)
    double actualAngle;            // Measured/computed angle
    double springbackAngle;        // Predicted springback (degrees)

    // Geometry - Dimensions
    double internalRadius;         // mm
    double flangeLength;           // mm (from bend line to edge)
    double flangeWidth;            // mm (along bend line)

    // Relationships
    FaceId baseFaceId;
    FaceId flangeFaceId;
    FaceId bendFaceId;             // Cylindrical surface (may be INVALID)

    // Computed in Phase 2
    std::vector<BendId> mustBendBefore;
    std::vector<BendId> mustBendAfter;
    bool isClosingBend;            // True if closes a box

    // Computed in Phase 3
    int sequencePosition;          // Order in final sequence
    int rotationBefore;            // Rotation needed before this bend

    // Computed in Phase 4
    bool isValidated;
    std::string validationNotes;
};
```

---

## 5. Sequence Step (Phase 3 → Phase 4 → Phase 5)

```cpp
//=============================================================================
// Machine Actions
//=============================================================================

enum class ActionType {
    BEND,           // Perform a bend
    ROTATE,         // Rotate part (0, 90, 180, 270)
    FLIP,           // Flip part over (rare for panel bender)
    REPO,           // Reposition grip
    ABA_ADJUST,     // Adjust tool width
    REFERENCE,      // Push against reference stop
    IDLE            // Wait / no-op
};

//=============================================================================
// Bend Step (Output of Phase 3)
//=============================================================================

struct BendStep {
    StepId id;
    ActionType action;

    // For BEND action
    BendId bendId;
    double targetAngle;
    BendDirection direction;

    // For ROTATE action
    int rotationDegrees;           // 0, 90, 180, 270

    // For ABA_ADJUST action
    double abaTargetWidth;         // mm

    // For REPO action
    Point3D newGripPosition;

    // Cost metrics (computed by sequencer)
    double actionTime;             // seconds
    double maskedTime;             // time hidden by parallel operation

    // Validation (set by Phase 4)
    bool isValid;
    std::string validationResult;
};

//=============================================================================
// Complete Sequence (Output of Phase 3/4)
//=============================================================================

struct BendSequence {
    // Metadata
    std::string partId;
    std::string sequenceId;
    int version;

    // Steps
    std::vector<BendStep> steps;

    // Summary
    int totalBends;
    int totalRotations;
    int totalRepos;
    double totalCycleTime;         // seconds

    // Validation status
    bool isFullyValidated;
    std::vector<std::string> validationWarnings;
};
```

---

## 6. Machine State

```cpp
//=============================================================================
// Machine Configuration (Used by Phase 3, 4, 5)
//=============================================================================

struct MachineConfig {
    // Machine identity
    std::string model;             // e.g., "P4-2520"

    // Work envelope
    double maxSheetLength;         // mm (X direction)
    double maxSheetWidth;          // mm (Y direction)
    double minSheetLength;         // mm
    double minSheetWidth;          // mm

    // Thickness range
    double minThickness;           // mm
    double maxThickness;           // mm

    // Bend capabilities
    double minBendAngle;           // degrees (negative)
    double maxBendAngle;           // degrees (positive)
    double minBendRadius;          // mm
    double minFlangeLength;        // mm

    // ABA configuration
    double abaMinWidth;            // mm (central segment)
    double abaMaxWidth;            // mm
    double abaSegmentStep;         // mm (typically 5)
    std::vector<double> abaSegments; // Available segment widths

    // Timing (for cost calculation)
    double rotationSpeed;          // degrees/second
    double abaAdjustSpeed;         // mm/second
    double bendingSpeed;           // degrees/second
};

//=============================================================================
// Runtime Machine State
//=============================================================================

struct MachineState {
    // Current orientation
    int partRotation;              // 0, 90, 180, 270 degrees

    // Tool configuration
    double currentABAWidth;        // mm
    std::vector<int> activeSegments; // Which segments are engaged

    // Grip position
    Point3D gripPosition;          // Where manipulator holds part
    bool gripActive;

    // Part state
    std::vector<BendId> completedBends;
    BendId currentBend;            // INVALID_BEND if none

    // For simulation
    double elapsedTime;            // seconds since start
};
```

---

## 7. Validation Results

```cpp
//=============================================================================
// Validation Types (Phase 4 Output)
//=============================================================================

enum class ValidationStatus {
    VALID,
    INVALID,
    WARNING,
    NOT_CHECKED
};

enum class FailureType {
    NONE,
    COLLISION_FLANGE_TOOL,
    COLLISION_FLANGE_TABLE,
    COLLISION_FLANGE_FLANGE,
    TRAP_CANNOT_EXTRACT,
    GRASP_INSUFFICIENT,
    GRASP_UNSTABLE,
    KINEMATIC_OUT_OF_REACH,
    MATERIAL_SPRINGBACK_EXCESSIVE,
    UNKNOWN
};

struct CollisionInfo {
    FailureType type;
    Point3D collisionPoint;
    Vector3D collisionNormal;
    double penetrationDepth;       // mm
    std::string description;
};

struct ValidationResult {
    StepId stepId;
    ValidationStatus status;

    // If invalid
    FailureType failureType;
    CollisionInfo collision;

    // Suggested corrections
    std::vector<std::string> corrections;

    // Metrics
    double clearance;              // mm (minimum)
    double graspQuality;           // 0-1
    double stabilityMargin;        // 0-1
};

struct SequenceValidation {
    bool isValid;
    std::vector<ValidationResult> stepResults;

    // Summary
    int validSteps;
    int invalidSteps;
    int warningSteps;

    // Critical failures
    std::vector<StepId> criticalFailures;
};
```

---

## 8. Output Formats (Phase 5)

```cpp
//=============================================================================
// Machine Instruction Types
//=============================================================================

struct MachineInstruction {
    StepId stepId;
    ActionType action;

    // Action-specific parameters (union-like)
    struct BendParams {
        double yPosition;          // Axis position
        double targetAngle;
        double crowning;           // Crowning compensation
        bool adaptiveEnabled;      // Use MAC 2.0
    } bendParams;

    struct RotateParams {
        int angle;
        double liftHeight;
    } rotateParams;

    struct ABAParams {
        double targetWidth;
        std::vector<int> segmentPattern;
    } abaParams;

    // Motion profile
    double approachSpeed;
    double actionSpeed;
    double returnSpeed;
    double dwellTime;              // ms
};

struct MachineProgram {
    // Header
    std::string jobId;
    std::string partName;
    std::string createdBy;
    std::string timestamp;
    std::string machineModel;

    // Material
    MaterialProperties material;

    // Instructions
    std::vector<MachineInstruction> instructions;

    // Estimated cycle time
    double estimatedCycleTime;     // seconds
};

//=============================================================================
// HMI Visualization Data
//=============================================================================

struct VisualizationKeyframe {
    StepId stepId;
    int boneIndex;
    double rotationAngle;          // degrees
};

struct VisualizationData {
    // Mesh reference
    std::string meshFile;          // GLB file path

    // Bone structure
    struct Bone {
        int index;
        std::string name;
        Point3D origin;
        Vector3D axis;
        int parentIndex;           // -1 for root
    };
    std::vector<Bone> bones;

    // Animation
    std::vector<VisualizationKeyframe> keyframes;

    // Metadata
    BoundingBox3D partBounds;
    Point3D cameraTarget;
};
```

---

## 9. Error Types (Shared)

```cpp
//=============================================================================
// Common Error Handling
//=============================================================================

enum class ErrorSeverity {
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

struct ProcessingError {
    ErrorSeverity severity;
    std::string phase;             // "Phase1", "Phase2", etc.
    std::string component;         // "STEPReader", "BendClassifier", etc.
    std::string code;              // Machine-readable code
    std::string message;           // Human-readable message
    std::string details;           // Additional context

    // Optional context
    std::optional<BendId> bendId;
    std::optional<StepId> stepId;
    std::optional<Point3D> location;
};

class ProcessingResult {
public:
    bool isSuccess() const { return m_errors.empty() || !hasCritical(); }
    bool hasWarnings() const;
    bool hasCritical() const;

    void addError(const ProcessingError& error);
    void addWarning(const std::string& message);

    const std::vector<ProcessingError>& errors() const { return m_errors; }
    std::string getSummary() const;

private:
    std::vector<ProcessingError> m_errors;
};
```

---

## 10. Constants and Tolerances

```cpp
//=============================================================================
// Global Constants
//=============================================================================

namespace Constants {
    // Geometric tolerances
    constexpr double POSITION_TOLERANCE = 1e-6;    // mm
    constexpr double ANGULAR_TOLERANCE = 1e-4;     // radians
    constexpr double AREA_TOLERANCE = 1e-4;        // mm²

    // Bend classification
    constexpr double HEM_ANGLE_THRESHOLD = 45.0;   // degrees
    constexpr double ACUTE_ANGLE_THRESHOLD = 120.0;// degrees

    // Machine defaults
    constexpr double DEFAULT_BEND_RADIUS = 2.0;    // mm
    constexpr double DEFAULT_THICKNESS = 2.0;      // mm

    // Safety margins
    constexpr double COLLISION_CLEARANCE = 0.5;    // mm
    constexpr double GRASP_MARGIN = 5.0;           // mm

    // Rotation angles
    constexpr int ROTATION_STEP = 90;              // degrees
    constexpr int ROTATION_OPTIONS[] = {0, 90, 180, 270};
}

//=============================================================================
// Unit Conversions
//=============================================================================

namespace Units {
    constexpr double MM_PER_INCH = 25.4;
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;

    inline double inchToMM(double inches) { return inches * MM_PER_INCH; }
    inline double mmToInch(double mm) { return mm / MM_PER_INCH; }
    inline double degToRad(double deg) { return deg * DEG_TO_RAD; }
    inline double radToDeg(double rad) { return rad * RAD_TO_DEG; }
}
```

---

## 11. Serialization Support

```cpp
//=============================================================================
// JSON Serialization (for debugging, HMI)
//=============================================================================

// Using nlohmann/json or similar
namespace Serialization {
    // To JSON
    nlohmann::json toJson(const BendDescriptor& bend);
    nlohmann::json toJson(const BendSequence& sequence);
    nlohmann::json toJson(const ValidationResult& result);

    // From JSON
    BendDescriptor bendFromJson(const nlohmann::json& j);
    BendSequence sequenceFromJson(const nlohmann::json& j);

    // To/From binary (for performance)
    std::vector<uint8_t> toBinary(const BendSequence& sequence);
    BendSequence sequenceFromBinary(const std::vector<uint8_t>& data);
}
```

---

## 12. Phase Interface Contract

```cpp
//=============================================================================
// Interface that all phases implement
//=============================================================================

template<typename InputT, typename OutputT>
class IPhase {
public:
    virtual ~IPhase() = default;

    // Main processing
    virtual OutputT process(const InputT& input) = 0;

    // Validation
    virtual bool validateInput(const InputT& input) = 0;
    virtual bool validateOutput(const OutputT& output) = 0;

    // Error handling
    virtual ProcessingResult getResult() const = 0;
    virtual std::string getPhaseName() const = 0;

    // Progress (for long operations)
    virtual double getProgress() const { return 1.0; }
    virtual void cancel() {}
};

// Type aliases for each phase
using Phase1 = IPhase<std::string, Phase1Output>;           // filepath → FAG
using Phase2 = IPhase<Phase1Output, Phase2Output>;          // FAG → DAG
using Phase3 = IPhase<Phase2Output, BendSequence>;          // DAG → Sequence
using Phase4 = IPhase<BendSequence, SequenceValidation>;    // Sequence → Validated
using Phase5 = IPhase<BendSequence, MachineProgram>;        // Sequence → Code
```
