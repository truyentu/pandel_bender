#include "openpanelcam/phase5/json_generator.h"
#include <sstream>
#include <iomanip>

#if defined(HAS_NLOHMANN_JSON) && HAS_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

namespace openpanelcam {
namespace phase5 {

VDMGenerator::VDMGenerator(const PostProcessorConfig& config)
    : m_config(config) {}

std::string VDMGenerator::generate(const VisualizationData& data) {
#if defined(HAS_NLOHMANN_JSON) && HAS_NLOHMANN_JSON
    nlohmann::json root;

    // Scene
    root["scene"]["camera"]["position"] = {data.cameraX, data.cameraY, data.cameraZ};
    root["scene"]["camera"]["lookAt"] = {data.lookAtX, data.lookAtY, data.lookAtZ};
    root["scene"]["camera"]["fov"] = data.fov;
    root["scene"]["lights"] = nlohmann::json::array({
        {{"type", "directional"}, {"direction", {1, -1, -1}}},
        {{"type", "ambient"}, {"intensity", 0.4}}
    });

    // Skeleton
    auto& bonesArr = root["skeleton"]["bones"];
    bonesArr = nlohmann::json::array();
    for (const auto& bone : data.bones) {
        nlohmann::json b;
        b["index"] = bone.index;
        b["name"] = bone.name;
        b["parent"] = bone.parentIndex;
        b["origin"] = {bone.originX, bone.originY, bone.originZ};
        b["axis"] = {bone.axisX, bone.axisY, bone.axisZ};
        bonesArr.push_back(b);
    }

    // Animation
    root["animation"]["duration"] = data.cycleTime;
    root["animation"]["fps"] = m_config.animationFPS;
    auto& kfArr = root["animation"]["keyframes"];
    kfArr = nlohmann::json::array();
    for (const auto& kf : data.keyframes) {
        nlohmann::json k;
        k["time"] = kf.time;
        k["bone"] = kf.boneIndex;
        k["rotation"] = kf.rotationAngle;
        if (kf.translationX != 0 || kf.translationY != 0 || kf.translationZ != 0) {
            k["translation"] = {kf.translationX, kf.translationY, kf.translationZ};
        }
        if (!kf.actionDescription.empty()) {
            k["action"] = kf.actionDescription;
        }
        kfArr.push_back(k);
    }

    // Metadata
    root["metadata"]["partName"] = data.partName;
    root["metadata"]["cycleTime"] = data.cycleTime;
    root["metadata"]["bendCount"] = data.bendCount;

    return root.dump(2);

#else
    // Fallback: manual JSON string building
    std::ostringstream json;
    json << std::fixed << std::setprecision(1);
    json << "{\n";
    json << "  \"scene\": {\n";
    json << "    \"camera\": {\n";
    json << "      \"position\": [" << data.cameraX << ", " << data.cameraY << ", " << data.cameraZ << "],\n";
    json << "      \"lookAt\": [" << data.lookAtX << ", " << data.lookAtY << ", " << data.lookAtZ << "],\n";
    json << "      \"fov\": " << data.fov << "\n";
    json << "    }\n";
    json << "  },\n";

    json << "  \"skeleton\": {\n";
    json << "    \"bones\": [\n";
    for (size_t i = 0; i < data.bones.size(); i++) {
        const auto& b = data.bones[i];
        json << "      {\"index\": " << b.index << ", \"name\": \"" << b.name
             << "\", \"parent\": " << b.parentIndex << "}";
        if (i < data.bones.size() - 1) json << ",";
        json << "\n";
    }
    json << "    ]\n";
    json << "  },\n";

    json << "  \"animation\": {\n";
    json << "    \"duration\": " << data.cycleTime << ",\n";
    json << "    \"fps\": " << m_config.animationFPS << ",\n";
    json << "    \"keyframes\": [\n";
    for (size_t i = 0; i < data.keyframes.size(); i++) {
        const auto& kf = data.keyframes[i];
        json << "      {\"time\": " << kf.time << ", \"bone\": " << kf.boneIndex
             << ", \"rotation\": " << kf.rotationAngle << "}";
        if (i < data.keyframes.size() - 1) json << ",";
        json << "\n";
    }
    json << "    ]\n";
    json << "  },\n";

    json << "  \"metadata\": {\n";
    json << "    \"partName\": \"" << data.partName << "\",\n";
    json << "    \"cycleTime\": " << data.cycleTime << ",\n";
    json << "    \"bendCount\": " << data.bendCount << "\n";
    json << "  }\n";
    json << "}\n";
    return json.str();
#endif
}

} // namespace phase5
} // namespace openpanelcam
