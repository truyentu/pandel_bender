#pragma once

#include "types.h"
#include <string>

namespace openpanelcam {
namespace phase5 {

/**
 * @brief Generates PB-XML format output for Salvagnini P4 Panel Bender
 *
 * XML Schema:
 * <Job>
 *   <Header> (Material, Tooling, JobInfo)
 *   <ProcessSequence> (Steps: Bend, Rotate, ABASetup, Reposition)
 *   <AdaptiveControl> (AngleFeedback, ForceLimit, SpeedControl)
 *   <Metadata> (TotalCycleTime, BendCount, RepositionCount)
 * </Job>
 */
class PBXMLGenerator {
public:
    explicit PBXMLGenerator(const PostProcessorConfig& config = PostProcessorConfig());

    /**
     * @brief Generate PB-XML string from machine program
     */
    std::string generate(const MachineProgram& program);

    const PostProcessorConfig& config() const { return m_config; }

private:
    PostProcessorConfig m_config;
};

} // namespace phase5
} // namespace openpanelcam
