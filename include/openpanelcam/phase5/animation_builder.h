#pragma once

#include "types.h"
#include "../phase2/phase1_mock.h"
#include <vector>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Builds bone hierarchy and animation keyframes for HMI
 *
 * Creates a skeletal animation representation:
 * - Root bone = base face of the part
 * - Child bones = each flange at its bend position
 * - Keyframes from machine instruction timeline
 */
class AnimationBuilder {
public:
    explicit AnimationBuilder(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Build complete visualization data
     */
    VisualizationData build(const MachineProgram& program,
                            const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Build bone hierarchy from bend features
     * Root bone = base, each bend = child bone
     */
    std::vector<AnimationBone> buildBoneHierarchy(
        const std::vector<phase1::BendFeature>& bends);

    /**
     * @brief Build keyframes from machine instructions
     * Cumulative time from instruction durations
     */
    std::vector<AnimationKeyframe> buildKeyframes(
        const MachineProgram& program);

private:
    PostProcessorConfig m_config;
};

} // namespace phase5
} // namespace openpanelcam
