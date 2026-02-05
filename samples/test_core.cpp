/**
 * @file test_core_utilities.cpp
 * @brief Basic test to verify core utilities compile
 */

#include <openpanelcam/core/logger.h>
#include <openpanelcam/core/error.h>
#include <openpanelcam/core/types.h>
#include <openpanelcam/core/constants.h>

#include <iostream>

using namespace openpanelcam;

int main() {
    // Test logger initialization
    LoggerConfig config;
    config.level = LogLevel::DEBUG;
    config.consoleOutput = true;
    config.fileOutput = false;

    Logger::initialize(config);

    LOG_INFO("Testing OpenPanelCAM core utilities");
    LOG_DEBUG("Debug logging enabled");

    // Test error codes
    std::cout << "Error code test: "
              << errorCodeToString(ErrorCode::STEP_LOAD_FAILED)
              << std::endl;

    // Test exception
    try {
        throw Phase1Exception(
            ErrorCode::NO_SOLID_FOUND,
            "Test exception",
            "This is just a test"
        );
    } catch (const OpenPanelCAMException& ex) {
        LOG_ERROR("Caught exception: {}", ex.what());
        LOG_ERROR("Details: {}", ex.details());
        std::cout << "Exception test passed" << std::endl;
    }

    // Test constants
    std::cout << "MIN_BEND_RADIUS: "
              << constants::MIN_BEND_RADIUS << " mm" << std::endl;
    std::cout << "LINEAR_TOLERANCE: "
              << constants::LINEAR_TOLERANCE << " mm" << std::endl;

    LOG_INFO("All core utility tests passed");

    Logger::shutdown();

    return 0;
}
