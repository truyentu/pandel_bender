#pragma once

#include "openpanelcam/phase5/types.h"
#include "openpanelcam/phase5/instruction_builder.h"
#include "openpanelcam/phase5/animation_builder.h"
#include "openpanelcam/phase5/output_validator.h"
#include "openpanelcam/phase5/xml_generator.h"
#include "openpanelcam/phase5/json_generator.h"
#include <chrono>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Main PostProcessor pipeline
 *
 * Orchestrates: Phase4Output → InstructionBuilder → AnimationBuilder
 *               → OutputValidator → PBXMLGenerator → VDMGenerator → Phase5Output
 */
class PostProcessor {
public:
    explicit PostProcessor(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Run full post-processing pipeline
     */
    Phase5Output process(const phase4::Phase4Output& validated,
                         const phase3::Phase3Output& sequence,
                         const std::vector<phase1::BendFeature>& bends);

    void setConfig(const PostProcessorConfig& config);
    const PostProcessorConfig& getConfig() const { return config_; }

private:
    PostProcessorConfig config_;
};

} // namespace phase5
} // namespace openpanelcam
