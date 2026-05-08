#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>

namespace rmcs_utility {

enum class PowerCycleInitStatus {
    INITIALIZED,
    SKIPPED,
    FAILED,
};

struct PowerCycleInitResult {
    PowerCycleInitStatus status{PowerCycleInitStatus::FAILED};
    std::string message;

    bool ok() const { return status != PowerCycleInitStatus::FAILED; }
    bool initialized() const { return status == PowerCycleInitStatus::INITIALIZED; }
    bool skipped() const { return status == PowerCycleInitStatus::SKIPPED; }

    explicit operator bool() const { return ok(); }
};

namespace detail {

inline std::string sanitize_power_cycle_init_name(std::string_view name) {
    std::string sanitized;
    sanitized.reserve(name.size());

    for (const char ch : name) {
        const bool is_lower = ch >= 'a' && ch <= 'z';
        const bool is_upper = ch >= 'A' && ch <= 'Z';
        const bool is_digit = ch >= '0' && ch <= '9';
        if (is_lower || is_upper || is_digit || ch == '_' || ch == '-' || ch == '.') {
            sanitized.push_back(ch);
        } else {
            sanitized.push_back('_');
        }
    }

    if (sanitized.empty()) {
        sanitized = "power_cycle_init";
    }

    return sanitized;
}

inline bool path_exists(const std::filesystem::path& path) {
    std::error_code ec;
    return std::filesystem::exists(path, ec) && !ec;
}

template <typename InitFunction>
bool invoke_power_cycle_initializer(InitFunction&& init_function) {
    using Result = std::invoke_result_t<InitFunction&>;

    if constexpr (std::is_void_v<Result>) {
        std::forward<InitFunction>(init_function)();
        return true;
    } else {
        return static_cast<bool>(std::forward<InitFunction>(init_function)());
    }
}

struct DirectoryLockGuard {
    std::filesystem::path path;

    ~DirectoryLockGuard() {
        if (path.empty()) {
            return;
        }

        std::error_code ignore_ec;
        std::filesystem::remove(path, ignore_ec);
    }
};

} // namespace detail

template <typename InitFunction>
PowerCycleInitResult run_once_after_power_cycle(
    std::string_view init_name, InitFunction&& init_function,
    const std::filesystem::path& state_directory = "/dev/shm/rmcs_power_cycle_init") {

    if (init_name.empty()) {
        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = "init name is empty",
        };
    }

    std::error_code ec;
    std::filesystem::create_directories(state_directory, ec);
    if (ec) {
        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = "failed to create state directory: " + ec.message(),
        };
    }

    const std::string safe_name = detail::sanitize_power_cycle_init_name(init_name);
    const auto marker_path      = state_directory / (safe_name + ".done");
    if (detail::path_exists(marker_path)) {
        return {
            .status = PowerCycleInitStatus::SKIPPED,
            .message = "already initialized in current power cycle",
        };
    }

    const auto lock_path = state_directory / (safe_name + ".lock");
    ec.clear();
    if (!std::filesystem::create_directory(lock_path, ec)) {
        if (detail::path_exists(marker_path)) {
            return {
                .status = PowerCycleInitStatus::SKIPPED,
                .message = "already initialized in current power cycle",
            };
        }

        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = ec ? "initializer lock error: " + ec.message()
                          : "initializer is already running",
        };
    }

    const detail::DirectoryLockGuard lock_guard{lock_path};

    if (!detail::invoke_power_cycle_initializer(std::forward<InitFunction>(init_function))) {
        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = "initializer returned failure",
        };
    }

    std::ofstream marker_file(marker_path, std::ios::trunc);
    if (!marker_file.is_open()) {
        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = "failed to open state marker",
        };
    }

    marker_file << "initialized\n";
    marker_file.close();
    if (marker_file.fail()) {
        return {
            .status = PowerCycleInitStatus::FAILED,
            .message = "failed to write state marker",
        };
    }

    return {
        .status = PowerCycleInitStatus::INITIALIZED,
        .message = "initializer executed",
    };
}

inline bool clear_power_cycle_init_marker(
    std::string_view init_name,
    const std::filesystem::path& state_directory = "/dev/shm/rmcs_power_cycle_init") {

    if (init_name.empty()) {
        return false;
    }

    std::error_code ec;
    const auto marker_path =
        state_directory / (detail::sanitize_power_cycle_init_name(init_name) + ".done");
    return std::filesystem::remove(marker_path, ec) || !detail::path_exists(marker_path);
}

} // namespace rmcs_utility
