/**
 * @file parse_step.cpp
 * @brief Sample application: Parse STEP file and extract bend features
 *
 * Usage:
 *   sample_parse_step <step_file> [--verbose] [--thickness <mm>]
 *
 * Example:
 *   sample_parse_step part.step
 *   sample_parse_step part.step --verbose --thickness 2.0
 */

#include <openpanelcam/phase1/phase1.h>
#include <openpanelcam/core/logger.h>
#include <iostream>
#include <string>

using namespace openpanelcam;

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <step_file> [options]\n";
    std::cout << "\nOptions:\n";
    std::cout << "  --verbose           Enable verbose logging\n";
    std::cout << "  --thickness <mm>    Set sheet thickness (default: 2.0)\n";
    std::cout << "  --no-heal           Disable geometry healing\n";
    std::cout << "  --help              Show this help message\n";
    std::cout << "\nExample:\n";
    std::cout << "  " << programName << " part.step\n";
    std::cout << "  " << programName << " part.step --verbose --thickness 2.0\n";
}

int main(int argc, char** argv) {
    // Parse arguments
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string stepFile;
    Phase1Config config;
    config.verbose = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else if (arg == "--no-heal") {
            config.healGeometry = false;
        } else if (arg == "--thickness") {
            if (i + 1 < argc) {
                config.thickness = std::stod(argv[++i]);
            } else {
                std::cerr << "Error: --thickness requires a value\n";
                return 1;
            }
        } else if (arg[0] != '-') {
            stepFile = arg;
        } else {
            std::cerr << "Error: Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (stepFile.empty()) {
        std::cerr << "Error: No STEP file specified\n";
        printUsage(argv[0]);
        return 1;
    }

    // Initialize logger
    Logger::initialize("parse_step.log", config.verbose ? LogLevel::DEBUG : LogLevel::INFO);

    std::cout << "========================================\n";
    std::cout << "OpenPanelCAM - STEP File Parser\n";
    std::cout << "========================================\n";
    std::cout << "Input file: " << stepFile << "\n";
    std::cout << "Thickness:  " << config.thickness << " mm\n";
    std::cout << "Healing:    " << (config.healGeometry ? "Enabled" : "Disabled") << "\n";
    std::cout << "Verbose:    " << (config.verbose ? "Yes" : "No") << "\n";
    std::cout << "========================================\n\n";

    try {
        // Parse STEP file
        Phase1Output output = parseSTEPFile(stepFile, config);

        if (!output.success) {
            std::cerr << "ERROR: Failed to parse STEP file\n";
            std::cerr << "Reason: " << output.errorMessage << "\n";
            return 1;
        }

        // Print summary
        std::cout << "\n========================================\n";
        std::cout << "RESULTS\n";
        std::cout << "========================================\n";
        std::cout << "Parse time:    " << output.parseTime << " ms\n";
        std::cout << "Total faces:   " << output.totalFaces << "\n";
        std::cout << "  - Planar:    " << output.planarFaces << "\n";
        std::cout << "  - Cylindrical: " << output.cylindricalFaces << "\n";
        std::cout << "  - Other:     " << output.otherFaces << "\n";
        std::cout << "\nBase face ID:  " << output.baseFaceId << "\n";
        std::cout << "Base score:    " << output.baseFaceScore << "\n";
        std::cout << "\nTotal bends:   " << output.bendCount << "\n";
        std::cout << "  - UP bends:  " << output.upBendCount << "\n";
        std::cout << "  - DOWN bends: " << output.downBendCount << "\n";
        std::cout << "  - HEMs:      " << output.hemCount << "\n";

        // List all bends
        if (output.bendCount > 0) {
            std::cout << "\n--- Bend Details ---\n";
            for (const auto& bend : output.bends) {
                std::string dirStr;
                if (bend.direction == BendDirection::BEND_UP) {
                    dirStr = "UP";
                } else if (bend.direction == BendDirection::BEND_DOWN) {
                    dirStr = "DOWN";
                } else if (bend.direction == BendDirection::HEM) {
                    dirStr = "HEM";
                } else {
                    dirStr = "UNKNOWN";
                }

                std::cout << "  Bend " << bend.id << ": "
                         << dirStr << " "
                         << bend.targetAngle << "° "
                         << "(R=" << bend.internalRadius << " mm, "
                         << "L=" << bend.bendLineLength << " mm)\n";
            }
        }

        // Warnings
        if (!output.warnings.empty()) {
            std::cout << "\n--- Warnings ---\n";
            for (const auto& warning : output.warnings) {
                std::cout << "  ! " << warning << "\n";
            }
        }

        std::cout << "========================================\n";
        std::cout << "\nSUCCESS: STEP file parsed successfully\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    }
}
