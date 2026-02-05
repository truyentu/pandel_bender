/**
 * @file error.cpp
 * @brief Error handling implementation
 */

#include <openpanelcam/core/error.h>
#include <sstream>

namespace openpanelcam {

const char* errorCodeToString(ErrorCode code) {
    switch (code) {
        // Generic
        case ErrorCode::SUCCESS: return "SUCCESS";
        case ErrorCode::UNKNOWN_ERROR: return "UNKNOWN_ERROR";
        case ErrorCode::NOT_IMPLEMENTED: return "NOT_IMPLEMENTED";
        case ErrorCode::INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case ErrorCode::OUT_OF_MEMORY: return "OUT_OF_MEMORY";

        // Phase 1
        case ErrorCode::STEP_LOAD_FAILED: return "STEP_LOAD_FAILED";
        case ErrorCode::STEP_PARSE_ERROR: return "STEP_PARSE_ERROR";
        case ErrorCode::INVALID_STEP_FORMAT: return "INVALID_STEP_FORMAT";
        case ErrorCode::GEOMETRY_HEALING_FAILED: return "GEOMETRY_HEALING_FAILED";
        case ErrorCode::NO_SOLID_FOUND: return "NO_SOLID_FOUND";
        case ErrorCode::EMPTY_SHAPE: return "EMPTY_SHAPE";
        case ErrorCode::FAG_CONSTRUCTION_FAILED: return "FAG_CONSTRUCTION_FAILED";
        case ErrorCode::NO_PLANAR_FACES: return "NO_PLANAR_FACES";
        case ErrorCode::NO_BENDS_FOUND: return "NO_BENDS_FOUND";
        case ErrorCode::BASE_FACE_AMBIGUOUS: return "BASE_FACE_AMBIGUOUS";
        case ErrorCode::INCONSISTENT_NORMALS: return "INCONSISTENT_NORMALS";

        // Phase 2
        case ErrorCode::PRECEDENCE_CYCLE_DETECTED: return "PRECEDENCE_CYCLE_DETECTED";
        case ErrorCode::INVALID_CONSTRAINT: return "INVALID_CONSTRAINT";
        case ErrorCode::NO_VALID_GRIP: return "NO_VALID_GRIP";
        case ErrorCode::ABA_CONSTRAINT_VIOLATION: return "ABA_CONSTRAINT_VIOLATION";

        // Phase 3
        case ErrorCode::NO_VALID_SEQUENCE: return "NO_VALID_SEQUENCE";
        case ErrorCode::SEARCH_TIMEOUT: return "SEARCH_TIMEOUT";
        case ErrorCode::STATE_SPACE_TOO_LARGE: return "STATE_SPACE_TOO_LARGE";

        // Phase 4
        case ErrorCode::COLLISION_DETECTED: return "COLLISION_DETECTED";
        case ErrorCode::GRIP_VALIDATION_FAILED: return "GRIP_VALIDATION_FAILED";
        case ErrorCode::TOOL_EXTRACTION_FAILED: return "TOOL_EXTRACTION_FAILED";

        // Phase 5
        case ErrorCode::XML_GENERATION_FAILED: return "XML_GENERATION_FAILED";
        case ErrorCode::JSON_GENERATION_FAILED: return "JSON_GENERATION_FAILED";
        case ErrorCode::INVALID_MACHINE_PARAMETERS: return "INVALID_MACHINE_PARAMETERS";

        // OCCT
        case ErrorCode::OCCT_EXCEPTION: return "OCCT_EXCEPTION";
        case ErrorCode::OCCT_INVALID_SHAPE: return "OCCT_INVALID_SHAPE";
        case ErrorCode::OCCT_OPERATION_FAILED: return "OCCT_OPERATION_FAILED";

        default: return "UNKNOWN_ERROR_CODE";
    }
}

OpenPanelCAMException::OpenPanelCAMException(
    ErrorCode code,
    const std::string& message,
    const std::string& details
)
    : std::runtime_error(message)
    , m_code(code)
    , m_details(details)
{
    // Build full message
    std::ostringstream oss;
    oss << "[" << errorCodeToString(code) << "] " << message;
    if (!details.empty()) {
        oss << "\nDetails: " << details;
    }
    m_fullMessage = oss.str();
}

std::string OpenPanelCAMException::toString() const {
    return m_fullMessage;
}

} // namespace openpanelcam
