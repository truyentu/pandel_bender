#include "openpanelcam/phase5/post_processor.h"
#include <chrono>

namespace openpanelcam {
namespace phase5 {

PostProcessor::PostProcessor(const PostProcessorConfig& config)
    : config_(config) {}

void PostProcessor::setConfig(const PostProcessorConfig& config) {
    config_ = config;
}

Phase5Output PostProcessor::process(const phase4::Phase4Output& validated,
                                     const phase3::Phase3Output& sequence,
                                     const std::vector<phase1::BendFeature>& bends) {
    auto startTime = std::chrono::high_resolution_clock::now();

    Phase5Output output;

    // Step 1: Build machine instructions
    InstructionBuilder instrBuilder(config_);
    output.program = instrBuilder.build(validated, sequence, bends);

    // Step 2: Build animation data
    AnimationBuilder animBuilder(config_);
    output.visualization = animBuilder.build(output.program, bends);

    // Step 3: Validate machine program
    OutputValidator validator(config_);
    auto validationResult = validator.validate(output.program);
    output.errors = validationResult.errors;
    output.warnings = validationResult.warnings;

    // Step 4: Generate XML (if configured and valid)
    if (config_.generateXML) {
        PBXMLGenerator xmlGen(config_);
        output.xmlOutput = xmlGen.generate(output.program);
        output.xmlValid = !output.xmlOutput.empty() && validationResult.valid;
    }

    // Step 5: Generate JSON (if configured)
    if (config_.generateJSON) {
        VDMGenerator jsonGen(config_);
        output.jsonOutput = jsonGen.generate(output.visualization);
        output.jsonValid = !output.jsonOutput.empty();
    }

    // Step 6: Determine overall success
    output.success = validationResult.valid &&
                     (!config_.generateXML || output.xmlValid) &&
                     (!config_.generateJSON || output.jsonValid);

    auto endTime = std::chrono::high_resolution_clock::now();
    output.generationTimeMs = std::chrono::duration<double, std::milli>(
        endTime - startTime).count();

    return output;
}

} // namespace phase5
} // namespace openpanelcam
