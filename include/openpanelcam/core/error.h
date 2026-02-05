#pragma once

/**
 * @file error.h
 * @brief Error handling and exception classes
 *
 * Provides structured error handling for OpenPanelCAM with:
 * - Custom exception hierarchy
 * - Error codes for programmatic handling
 * - Rich error context (file, phase, details)
 *
 * Usage:
 * @code
 * throw Phase1Exception(
 *     ErrorCode::STEP_LOAD_FAILED,
 *     "Failed to load STEP file",
 *     "File not found: example.step"
 * );
 * @endcode
 */

#include <string>
#include <exception>
#include <stdexcept>

namespace openpanelcam {

/**
 * @brief Error codes for programmatic error handling
 */
enum class ErrorCode {
    // Generic errors (0-99)
    SUCCESS = 0,
    UNKNOWN_ERROR = 1,
    NOT_IMPLEMENTED = 2,
    INVALID_ARGUMENT = 3,
    OUT_OF_MEMORY = 4,

    // Phase 1 errors (100-199)
    STEP_LOAD_FAILED = 100,
    STEP_PARSE_ERROR = 101,
    INVALID_STEP_FORMAT = 102,
    GEOMETRY_HEALING_FAILED = 103,
    NO_SOLID_FOUND = 104,
    EMPTY_SHAPE = 105,
    FAG_CONSTRUCTION_FAILED = 106,
    NO_PLANAR_FACES = 107,
    NO_BENDS_FOUND = 108,
    BASE_FACE_AMBIGUOUS = 109,
    INCONSISTENT_NORMALS = 110,

    // Phase 2 errors (200-299)
    PRECEDENCE_CYCLE_DETECTED = 200,
    INVALID_CONSTRAINT = 201,
    NO_VALID_GRIP = 202,
    ABA_CONSTRAINT_VIOLATION = 203,

    // Phase 3 errors (300-399)
    NO_VALID_SEQUENCE = 300,
    SEARCH_TIMEOUT = 301,
    STATE_SPACE_TOO_LARGE = 302,

    // Phase 4 errors (400-499)
    COLLISION_DETECTED = 400,
    GRIP_VALIDATION_FAILED = 401,
    TOOL_EXTRACTION_FAILED = 402,

    // Phase 5 errors (500-599)
    XML_GENERATION_FAILED = 500,
    JSON_GENERATION_FAILED = 501,
    INVALID_MACHINE_PARAMETERS = 502,

    // OCCT errors (900-999)
    OCCT_EXCEPTION = 900,
    OCCT_INVALID_SHAPE = 901,
    OCCT_OPERATION_FAILED = 902
};

/**
 * @brief Convert error code to string
 * @param code Error code
 * @return Human-readable string
 */
const char* errorCodeToString(ErrorCode code);

/**
 * @brief Base exception class for OpenPanelCAM
 */
class OpenPanelCAMException : public std::runtime_error {
public:
    /**
     * @brief Construct exception with error code and message
     * @param code Error code
     * @param message Error message
     * @param details Additional details (optional)
     */
    OpenPanelCAMException(
        ErrorCode code,
        const std::string& message,
        const std::string& details = ""
    );

    /**
     * @brief Get error code
     * @return Error code
     */
    ErrorCode code() const noexcept { return m_code; }

    /**
     * @brief Get detailed error message
     * @return Full error message with details
     */
    const std::string& details() const noexcept { return m_details; }

    /**
     * @brief Get formatted error string
     * @return Complete error description
     */
    std::string toString() const;

private:
    ErrorCode m_code;
    std::string m_details;
    std::string m_fullMessage;
};

/**
 * @brief Phase 1 specific exception
 */
class Phase1Exception : public OpenPanelCAMException {
public:
    using OpenPanelCAMException::OpenPanelCAMException;
};

/**
 * @brief Phase 2 specific exception
 */
class Phase2Exception : public OpenPanelCAMException {
public:
    using OpenPanelCAMException::OpenPanelCAMException;
};

/**
 * @brief Phase 3 specific exception
 */
class Phase3Exception : public OpenPanelCAMException {
public:
    using OpenPanelCAMException::OpenPanelCAMException;
};

/**
 * @brief Phase 4 specific exception
 */
class Phase4Exception : public OpenPanelCAMException {
public:
    using OpenPanelCAMException::OpenPanelCAMException;
};

/**
 * @brief Phase 5 specific exception
 */
class Phase5Exception : public OpenPanelCAMException {
public:
    using OpenPanelCAMException::OpenPanelCAMException;
};

/**
 * @brief OCCT-related exception wrapper
 */
class OCCTException : public OpenPanelCAMException {
public:
    OCCTException(const std::string& message, const std::string& details = "")
        : OpenPanelCAMException(ErrorCode::OCCT_EXCEPTION, message, details) {}
};

// Convenience macros for throwing exceptions
#define THROW_PHASE1(code, msg, ...) \
    throw ::openpanelcam::Phase1Exception(code, msg, ##__VA_ARGS__)

#define THROW_PHASE2(code, msg, ...) \
    throw ::openpanelcam::Phase2Exception(code, msg, ##__VA_ARGS__)

#define THROW_PHASE3(code, msg, ...) \
    throw ::openpanelcam::Phase3Exception(code, msg, ##__VA_ARGS__)

#define THROW_PHASE4(code, msg, ...) \
    throw ::openpanelcam::Phase4Exception(code, msg, ##__VA_ARGS__)

#define THROW_PHASE5(code, msg, ...) \
    throw ::openpanelcam::Phase5Exception(code, msg, ##__VA_ARGS__)

} // namespace openpanelcam
