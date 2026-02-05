#pragma once

/**
 * @file logger.h
 * @brief Logging system wrapper around spdlog
 *
 * Provides a simple, consistent logging interface for OpenPanelCAM.
 * Supports multiple log levels, file output, and console output.
 *
 * Usage:
 * @code
 * LOG_INFO("Processing STEP file: {}", filename);
 * LOG_ERROR("Failed to load file: {}", error);
 * LOG_DEBUG("FAG has {} nodes, {} edges", nodeCount, edgeCount);
 * @endcode
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <memory>
#include <string>

namespace openpanelcam {

/**
 * @brief Log levels matching spdlog
 */
enum class LogLevel {
    TRACE,      ///< Very detailed information, typically only for diagnosis
    DEBUG,      ///< Detailed information useful during development
    INFO,       ///< General informational messages
    WARNING,    ///< Warning messages for potentially harmful situations
    ERROR,      ///< Error messages for failures
    CRITICAL    ///< Critical errors that may cause termination
};

/**
 * @brief Logger configuration options
 */
struct LoggerConfig {
    LogLevel level = LogLevel::INFO;           ///< Minimum log level
    bool consoleOutput = true;                 ///< Enable console output
    bool fileOutput = false;                   ///< Enable file output
    std::string logFilePath = "openpanelcam.log"; ///< Log file path
    std::string pattern = "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v"; ///< Log format
};

/**
 * @brief Logger wrapper class
 *
 * Singleton wrapper around spdlog providing a simple interface.
 * Thread-safe and supports multiple output sinks.
 */
class Logger {
public:
    /**
     * @brief Initialize the logger with configuration
     * @param config Logger configuration
     */
    static void initialize(const LoggerConfig& config = LoggerConfig{});

    /**
     * @brief Shutdown the logger (flushes buffers)
     */
    static void shutdown();

    /**
     * @brief Set minimum log level
     * @param level New log level
     */
    static void setLevel(LogLevel level);

    /**
     * @brief Get the underlying spdlog logger
     * @return Shared pointer to spdlog logger
     */
    static std::shared_ptr<spdlog::logger> get();

    /**
     * @brief Check if logger is initialized
     * @return True if initialized
     */
    static bool isInitialized();

private:
    static std::shared_ptr<spdlog::logger> s_logger;
    static bool s_initialized;
};

// Convenience macros for logging
#define LOG_TRACE(...)    if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->trace(__VA_ARGS__)
#define LOG_DEBUG(...)    if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->debug(__VA_ARGS__)
#define LOG_INFO(...)     if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->info(__VA_ARGS__)
#define LOG_WARNING(...)  if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->warn(__VA_ARGS__)
#define LOG_ERROR(...)    if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->error(__VA_ARGS__)
#define LOG_CRITICAL(...) if (::openpanelcam::Logger::isInitialized()) \
                              ::openpanelcam::Logger::get()->critical(__VA_ARGS__)

} // namespace openpanelcam
