#pragma once

#include "types.h"
#include <string>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Generates JSON Visualization Data Model (VDM) for HMI
 *
 * JSON Schema:
 * {
 *   "scene": { camera, lights },
 *   "skeleton": { bones[] },
 *   "animation": { duration, fps, keyframes[] },
 *   "metadata": { partName, cycleTime, bendCount }
 * }
 */
class VDMGenerator {
public:
    explicit VDMGenerator(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Generate JSON string from visualization data
     */
    std::string generate(const VisualizationData& data);

    const PostProcessorConfig& config() const { return m_config; }

private:
    PostProcessorConfig m_config;
};

} // namespace phase5
} // namespace openpanelcam
