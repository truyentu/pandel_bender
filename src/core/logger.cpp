/**
 * @file logger.cpp
 * @brief Logging system implementation
 */

#include <openpanelcam/core/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <vector>

namespace openpanelcam {

// Static member initialization
std::shared_ptr<spdlog::logger> Logger::s_logger = nullptr;
bool Logger::s_initialized = false;

void Logger::initialize(const LoggerConfig& config) {
    if (s_initialized) {
        return; // Already initialized
    }

    try {
        std::vector<spdlog::sink_ptr> sinks;

        // Console sink (with colors)
        if (config.consoleOutput) {
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_pattern(config.pattern);
            sinks.push_back(console_sink);
        }

        // File sink
        if (config.fileOutput) {
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                config.logFilePath, true // truncate
            );
            file_sink->set_pattern(config.pattern);
            sinks.push_back(file_sink);
        }

        // Create logger with sinks
        if (!sinks.empty()) {
            s_logger = std::make_shared<spdlog::logger>("openpanelcam", sinks.begin(), sinks.end());
        } else {
            // Fallback to console only
            s_logger = spdlog::stdout_color_mt("openpanelcam");
        }

        // Set log level
        setLevel(config.level);

        // Set flush policy
        s_logger->flush_on(spdlog::level::err);

        s_initialized = true;

        LOG_INFO("OpenPanelCAM Logger initialized");
        LOG_DEBUG("Log level: {}", spdlog::level::to_string_view(s_logger->level()));

    } catch (const spdlog::spdlog_ex& ex) {
        // Fallback: create basic console logger
        s_logger = spdlog::stdout_color_mt("openpanelcam");
        s_initialized = true;
        LOG_ERROR("Logger initialization failed: {}", ex.what());
    }
}

void Logger::shutdown() {
    if (s_initialized && s_logger) {
        LOG_INFO("OpenPanelCAM Logger shutting down");
        s_logger->flush();
        spdlog::drop("openpanelcam");
        s_logger = nullptr;
        s_initialized = false;
    }
}

void Logger::setLevel(LogLevel level) {
    if (!s_initialized || !s_logger) {
        return;
    }

    // Convert our LogLevel to spdlog::level
    spdlog::level::level_enum spdlog_level;
    switch (level) {
        case LogLevel::TRACE:    spdlog_level = spdlog::level::trace; break;
        case LogLevel::DEBUG:    spdlog_level = spdlog::level::debug; break;
        case LogLevel::INFO:     spdlog_level = spdlog::level::info; break;
        case LogLevel::WARNING:  spdlog_level = spdlog::level::warn; break;
        case LogLevel::ERROR:    spdlog_level = spdlog::level::err; break;
        case LogLevel::CRITICAL: spdlog_level = spdlog::level::critical; break;
        default:                 spdlog_level = spdlog::level::info; break;
    }

    s_logger->set_level(spdlog_level);
}

std::shared_ptr<spdlog::logger> Logger::get() {
    if (!s_initialized) {
        // Auto-initialize with defaults
        initialize();
    }
    return s_logger;
}

bool Logger::isInitialized() {
    return s_initialized && s_logger != nullptr;
}

} // namespace openpanelcam
