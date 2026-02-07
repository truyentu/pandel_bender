#pragma once

#include <string>
#include <vector>
#include <cstdint>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Type of machine instruction
 */
enum class InstructionType {
    BEND,
    ROTATE,
    ABA_SETUP,
    REPOSITION,
    GRIP,
    RELEASE
};

/**
 * @brief Single machine instruction for Salvagnini P4
 */
struct MachineInstruction {
    int stepId = -1;
    InstructionType type = InstructionType::BEND;

    // Bend parameters
    int bendId = -1;
    double targetAngle = 0.0;
    double compensatedAngle = 0.0;
    double springbackAngle = 0.0;
    double bendForce = 0.0;         // kN
    double bendLength = 0.0;        // mm

    // Bend line position
    double startX = 0.0, startY = 0.0;
    double endX = 0.0, endY = 0.0;

    // Rotation parameters
    int rotationAngle = 0;          // 0, 90, 180, 270

    // ABA parameters
    uint16_t abaConfig = 0;
    std::vector<double> segmentPositions;
    std::vector<double> segmentWidths;

    // Motion profile
    double duration = 0.0;          // seconds
    std::string description;
};

/**
 * @brief Complete machine program for PB-XML output
 */
struct MachineProgram {
    // Header
    std::string jobId;
    std::string partName = "UNNAMED";
    std::string createdBy = "OpenPanelCAM";
    std::string timestamp;
    std::string machineModel = "P4";

    // Material
    double materialThickness = 1.5;
    std::string materialType = "AISI304";
    double yieldStrength = 200.0;

    // Tooling
    std::string upperTool = "Standard";
    std::string lowerTool = "ABA";

    // Instructions
    std::vector<MachineInstruction> instructions;

    // Metadata
    double totalCycleTime = 0.0;
    int bendCount = 0;
    int repositionCount = 0;
    int rotationCount = 0;

    // Adaptive control
    bool adaptiveEnabled = true;
    double angleTolerance = 0.5;     // degrees
    double forceLimit = 25.0;        // kN
    bool speedAdaptive = true;
};

/**
 * @brief Bone for skeletal animation in HMI
 */
struct AnimationBone {
    int index = -1;
    std::string name;
    int parentIndex = -1;  // -1 for root
    double originX = 0, originY = 0, originZ = 0;
    double axisX = 0, axisY = 0, axisZ = 1;
};

/**
 * @brief Keyframe for animation timeline
 */
struct AnimationKeyframe {
    double time = 0.0;       // seconds
    int boneIndex = -1;
    double rotationAngle = 0.0;
    double translationX = 0, translationY = 0, translationZ = 0;
    std::string actionDescription;
};

/**
 * @brief Visualization data for HMI (VDM format)
 */
struct VisualizationData {
    // Scene
    double cameraX = 0, cameraY = -500, cameraZ = 300;
    double lookAtX = 0, lookAtY = 0, lookAtZ = 0;
    double fov = 45.0;

    // Bones
    std::vector<AnimationBone> bones;

    // Keyframes
    std::vector<AnimationKeyframe> keyframes;

    // Metadata
    std::string partName;
    double cycleTime = 0.0;
    int bendCount = 0;
};

/**
 * @brief Complete Phase 5 output
 */
struct Phase5Output {
    bool success = false;

    MachineProgram program;
    VisualizationData visualization;

    std::string xmlOutput;
    std::string jsonOutput;

    bool xmlValid = false;
    bool jsonValid = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    double generationTimeMs = 0.0;
};

/**
 * @brief Configuration for post-processor
 */
struct PostProcessorConfig {
    // Machine limits
    double maxBendAngle = 135.0;    // degrees
    double minBendAngle = -135.0;   // degrees
    double maxForce = 25.0;         // kN
    double maxPartLength = 2500.0;  // mm
    double maxPartWidth = 1250.0;   // mm

    // Output options
    bool generateXML = true;
    bool generateJSON = true;
    bool includeAdaptiveControl = true;
    bool includeComments = true;

    // Material defaults
    double defaultThickness = 1.5;
    std::string defaultMaterial = "AISI304";
    double defaultYieldStrength = 200.0; // MPa

    // Animation
    double animationFPS = 30.0;
};

} // namespace phase5
} // namespace openpanelcam
