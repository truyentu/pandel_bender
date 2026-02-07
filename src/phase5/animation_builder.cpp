#include "openpanelcam/phase5/animation_builder.h"

namespace openpanelcam {
namespace phase5 {

AnimationBuilder::AnimationBuilder(const PostProcessorConfig& config)
    : m_config(config) {}

VisualizationData AnimationBuilder::build(
    const MachineProgram& program,
    const std::vector<phase1::BendFeature>& bends
) {
    VisualizationData viz;
    viz.partName = program.partName;
    viz.cycleTime = program.totalCycleTime;
    viz.bendCount = program.bendCount;

    viz.bones = buildBoneHierarchy(bends);
    viz.keyframes = buildKeyframes(program);

    return viz;
}

std::vector<AnimationBone> AnimationBuilder::buildBoneHierarchy(
    const std::vector<phase1::BendFeature>& bends
) {
    std::vector<AnimationBone> bones;

    // Root bone = base face
    AnimationBone root;
    root.index = 0;
    root.name = "base";
    root.parentIndex = -1;
    root.originX = 0; root.originY = 0; root.originZ = 0;
    root.axisX = 0; root.axisY = 0; root.axisZ = 1;
    bones.push_back(root);

    // Each bend creates a child bone
    for (const auto& bend : bends) {
        AnimationBone bone;
        bone.index = static_cast<int>(bones.size());
        bone.name = "flange_" + std::to_string(bend.id);
        bone.parentIndex = 0; // All flanges parented to base
        bone.originX = bend.position.x;
        bone.originY = bend.position.y;
        bone.originZ = bend.position.z;
        // Rotation axis = bend direction
        bone.axisX = bend.direction.x;
        bone.axisY = bend.direction.y;
        bone.axisZ = bend.direction.z;
        bones.push_back(bone);
    }

    return bones;
}

std::vector<AnimationKeyframe> AnimationBuilder::buildKeyframes(
    const MachineProgram& program
) {
    std::vector<AnimationKeyframe> keyframes;

    // Initial keyframe: all bones at rest
    AnimationKeyframe initial;
    initial.time = 0.0;
    initial.boneIndex = 0;
    initial.rotationAngle = 0.0;
    initial.actionDescription = "Start";
    keyframes.push_back(initial);

    double cumulativeTime = 0.0;

    for (const auto& instr : program.instructions) {
        cumulativeTime += instr.duration;

        AnimationKeyframe kf;
        kf.time = cumulativeTime;

        switch (instr.type) {
            case InstructionType::BEND:
                // Bend → rotation keyframe on the flange bone
                // Bone index = bendId + 1 (0 is root)
                kf.boneIndex = instr.bendId + 1;
                kf.rotationAngle = instr.targetAngle;
                kf.actionDescription = instr.description;
                break;

            case InstructionType::ROTATE:
                // Part rotation → keyframe on root bone
                kf.boneIndex = 0;
                kf.rotationAngle = static_cast<double>(instr.rotationAngle);
                kf.actionDescription = instr.description;
                break;

            case InstructionType::REPOSITION:
                // Reposition → translation on root bone
                kf.boneIndex = 0;
                kf.rotationAngle = 0.0;
                kf.actionDescription = instr.description;
                break;

            default:
                // ABA setup, grip, release → no visual keyframe needed
                continue;
        }

        keyframes.push_back(kf);
    }

    return keyframes;
}

} // namespace phase5
} // namespace openpanelcam
