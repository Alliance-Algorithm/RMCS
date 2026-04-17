#pragma once

#include <cstdint>
#include <cstdio>
#include <format>
#include <iostream>
#include <print>
#include <string_view>
#include <utility>

#include "core/src/utility/assert.hpp"

namespace librmcs::host::logging {

enum class Level : std::uint8_t {
    kTrace = 0,
    kDebug = 1,
    kInfo = 2,
    kWarn = 3,
    kErr = 4,
    kCritical = 5,
    kOff = 6,
};

#ifndef LIBRMCS_LOGGING_LEVEL
# define LIBRMCS_LOGGING_LEVEL kInfo
#endif

class Logger {
public:
    static constexpr Level kLoggingLevel = Level::LIBRMCS_LOGGING_LEVEL;

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = delete;
    Logger& operator=(Logger&&) = delete;
    ~Logger() = default;

public: // Singleton
    static Logger& get_instance() noexcept {
        static Logger logger{};
        return logger;
    }

public: // Logging
    static constexpr bool should_log(Level level) { return level >= kLoggingLevel; }

public: // Logging.Formatted
    template <typename... Args>
    void trace(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kTrace, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void debug(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kDebug, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kInfo, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void warn(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kWarn, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kErr, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(std::format_string<Args...> fmt, Args&&... args) {
        log_internal(Level::kCritical, fmt, std::forward<Args>(args)...);
    }

    template <typename... Args>
    void log(Level level, std::format_string<Args...> fmt, Args&&... args) {
        log_internal(level, fmt, std::forward<Args>(args)...);
    }

public: // Logging.Raw
    template <typename T>
    void trace(const T& msg) {
        log_internal(Level::kTrace, msg);
    }

    template <typename T>
    void debug(const T& msg) {
        log_internal(Level::kDebug, msg);
    }

    template <typename T>
    void info(const T& msg) {
        log_internal(Level::kInfo, msg);
    }

    template <typename T>
    void warn(const T& msg) {
        log_internal(Level::kWarn, msg);
    }

    template <typename T>
    void error(const T& msg) {
        log_internal(Level::kErr, msg);
    }

    template <typename T>
    void critical(const T& msg) {
        log_internal(Level::kCritical, msg);
    }

    template <typename T>
    void log(Level level, const T& msg) {
        log_internal(level, msg);
    }

private:
    constexpr Logger() noexcept = default;

    template <typename... Args>
    void log_internal(Level level, std::format_string<Args...> fmt, Args&&... args) {
        if (!should_log(level))
            return;

        print_prefix(level);
        std::println(std::cerr, fmt, std::forward<Args>(args)...);
    }

    template <typename T>
    void log_internal(Level level, const T& msg) {
        if (!should_log(level))
            return;

        print_prefix(level);
        std::cerr << msg << '\n';
    }

    static void print_prefix(Level level) {
        std::string_view level_text = [level]() constexpr -> std::string_view {
            if (level == Level::kTrace)
                return "trace";
            if (level == Level::kDebug)
                return "debug";
            if (level == Level::kInfo)
                return "info";
            if (level == Level::kWarn)
                return "warn";
            if (level == Level::kErr)
                return "error";
            if (level == Level::kCritical)
                return "critical";
            core::utility::assert_failed_debug();
        }();
        std::print(std::cerr, "[librmcs] [{}] ", level_text);
    }
};

inline Logger& get_logger() { return Logger::get_instance(); }

} // namespace librmcs::host::logging
