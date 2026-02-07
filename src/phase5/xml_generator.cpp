#include "openpanelcam/phase5/xml_generator.h"
#include <sstream>
#include <iomanip>

// Use PugiXML if available, otherwise manual XML string building
#if defined(HAS_PUGIXML) && HAS_PUGIXML
#include <pugixml.hpp>
#endif

namespace openpanelcam {
namespace phase5 {

PBXMLGenerator::PBXMLGenerator(const PostProcessorConfig& config)
    : m_config(config) {}

std::string PBXMLGenerator::generate(const MachineProgram& program) {
#if defined(HAS_PUGIXML) && HAS_PUGIXML
    pugi::xml_document doc;
    auto decl = doc.prepend_child(pugi::node_declaration);
    decl.append_attribute("version") = "1.0";
    decl.append_attribute("encoding") = "UTF-8";

    auto job = doc.append_child("Job");

    // Header
    auto header = job.append_child("Header");

    auto material = header.append_child("Material");
    material.append_attribute("thickness").set_value(program.materialThickness);
    material.append_attribute("type").set_value(program.materialType.c_str());
    material.append_attribute("yieldStrength").set_value(program.yieldStrength);

    auto tooling = header.append_child("Tooling");
    tooling.append_attribute("upper").set_value(program.upperTool.c_str());
    tooling.append_attribute("lower").set_value(program.lowerTool.c_str());

    auto jobInfo = header.append_child("JobInfo");
    jobInfo.append_attribute("partName").set_value(program.partName.c_str());
    jobInfo.append_attribute("program").set_value(program.createdBy.c_str());
    jobInfo.append_attribute("timestamp").set_value(program.timestamp.c_str());
    jobInfo.append_attribute("machine").set_value(program.machineModel.c_str());

    // ProcessSequence
    auto procSeq = job.append_child("ProcessSequence");

    for (const auto& instr : program.instructions) {
        auto step = procSeq.append_child("Step");
        step.append_attribute("id").set_value(instr.stepId);

        switch (instr.type) {
            case InstructionType::BEND: {
                step.append_attribute("type").set_value("Bend");

                auto bendLine = step.append_child("BendLine");
                auto start = bendLine.append_child("Start");
                start.append_attribute("x").set_value(instr.startX);
                start.append_attribute("y").set_value(instr.startY);
                auto end = bendLine.append_child("End");
                end.append_attribute("x").set_value(instr.endX);
                end.append_attribute("y").set_value(instr.endY);

                auto angle = step.append_child("Angle");
                angle.append_attribute("target").set_value(instr.targetAngle);
                angle.append_attribute("compensated").set_value(instr.compensatedAngle);

                step.append_child("Force").text().set(instr.bendForce);
                step.append_child("Springback").text().set(instr.springbackAngle);
                step.append_child("Length").text().set(instr.bendLength);
                step.append_child("Duration").text().set(instr.duration);
                break;
            }

            case InstructionType::ROTATE:
                step.append_attribute("type").set_value("Rotation");
                step.append_child("Angle").text().set(instr.rotationAngle);
                step.append_child("Duration").text().set(instr.duration);
                break;

            case InstructionType::ABA_SETUP: {
                step.append_attribute("type").set_value("ABASetup");
                auto segs = step.append_child("Segments");
                for (size_t i = 0; i < instr.segmentPositions.size(); i++) {
                    auto seg = segs.append_child("Seg");
                    seg.append_attribute("pos").set_value(instr.segmentPositions[i]);
                    if (i < instr.segmentWidths.size()) {
                        seg.append_attribute("width").set_value(instr.segmentWidths[i]);
                    }
                }
                step.append_child("Duration").text().set(instr.duration);
                break;
            }

            case InstructionType::REPOSITION:
                step.append_attribute("type").set_value("Reposition");
                step.append_child("Duration").text().set(instr.duration);
                if (!instr.description.empty()) {
                    step.append_child("Description").text().set(instr.description.c_str());
                }
                break;

            default:
                step.append_attribute("type").set_value("Other");
                break;
        }
    }

    // AdaptiveControl
    if (m_config.includeAdaptiveControl) {
        auto adaptive = job.append_child("AdaptiveControl");

        auto angleFeedback = adaptive.append_child("AngleFeedback");
        angleFeedback.append_attribute("enabled").set_value(program.adaptiveEnabled);
        angleFeedback.append_attribute("tolerance").set_value(program.angleTolerance);

        auto forceLimit = adaptive.append_child("ForceLimit");
        forceLimit.append_attribute("max").set_value(program.forceLimit);

        auto speedCtrl = adaptive.append_child("SpeedControl");
        speedCtrl.append_attribute("adaptive").set_value(program.speedAdaptive);
    }

    // Metadata
    auto metadata = job.append_child("Metadata");
    metadata.append_child("TotalCycleTime").text().set(program.totalCycleTime);
    metadata.append_child("BendCount").text().set(program.bendCount);
    metadata.append_child("RepositionCount").text().set(program.repositionCount);
    metadata.append_child("RotationCount").text().set(program.rotationCount);

    // Serialize to string
    std::ostringstream oss;
    doc.save(oss, "  ");
    return oss.str();

#else
    // Fallback: manual XML string building
    std::ostringstream xml;
    xml << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    xml << "<Job>\n";

    xml << "  <Header>\n";
    xml << "    <Material thickness=\"" << program.materialThickness
        << "\" type=\"" << program.materialType
        << "\" yieldStrength=\"" << program.yieldStrength << "\"/>\n";
    xml << "    <Tooling upper=\"" << program.upperTool
        << "\" lower=\"" << program.lowerTool << "\"/>\n";
    xml << "    <JobInfo partName=\"" << program.partName
        << "\" program=\"" << program.createdBy
        << "\" timestamp=\"" << program.timestamp
        << "\" machine=\"" << program.machineModel << "\"/>\n";
    xml << "  </Header>\n";

    xml << "  <ProcessSequence>\n";
    for (const auto& instr : program.instructions) {
        xml << "    <Step id=\"" << instr.stepId << "\" type=\"";
        switch (instr.type) {
            case InstructionType::BEND:
                xml << "Bend\">\n";
                xml << "      <Angle target=\"" << instr.targetAngle
                    << "\" compensated=\"" << instr.compensatedAngle << "\"/>\n";
                xml << "      <Force>" << instr.bendForce << "</Force>\n";
                xml << "      <Duration>" << instr.duration << "</Duration>\n";
                xml << "    </Step>\n";
                break;
            case InstructionType::ROTATE:
                xml << "Rotation\">\n";
                xml << "      <Angle>" << instr.rotationAngle << "</Angle>\n";
                xml << "      <Duration>" << instr.duration << "</Duration>\n";
                xml << "    </Step>\n";
                break;
            case InstructionType::ABA_SETUP:
                xml << "ABASetup\">\n";
                xml << "      <Duration>" << instr.duration << "</Duration>\n";
                xml << "    </Step>\n";
                break;
            case InstructionType::REPOSITION:
                xml << "Reposition\">\n";
                xml << "      <Duration>" << instr.duration << "</Duration>\n";
                xml << "    </Step>\n";
                break;
            default:
                xml << "Other\"/>\n";
                break;
        }
    }
    xml << "  </ProcessSequence>\n";

    xml << "  <Metadata>\n";
    xml << "    <TotalCycleTime>" << program.totalCycleTime << "</TotalCycleTime>\n";
    xml << "    <BendCount>" << program.bendCount << "</BendCount>\n";
    xml << "    <RepositionCount>" << program.repositionCount << "</RepositionCount>\n";
    xml << "  </Metadata>\n";

    xml << "</Job>\n";
    return xml.str();
#endif
}

} // namespace phase5
} // namespace openpanelcam
