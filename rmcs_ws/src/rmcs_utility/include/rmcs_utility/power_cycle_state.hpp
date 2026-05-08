#pragma once

#include "rmcs_utility/power_cycle_init.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>

namespace rmcs_utility {

enum class PowerCycleStateStatus {
    AVAILABLE,
    NOT_FOUND,
    FAILED,
};

struct PowerCycleDoubleState {
    PowerCycleStateStatus status{PowerCycleStateStatus::FAILED};
    std::optional<double> value;
    std::string message;

    bool available() const { return status == PowerCycleStateStatus::AVAILABLE; }
    bool missing() const { return status == PowerCycleStateStatus::NOT_FOUND; }

    explicit operator bool() const { return available(); }
};

inline PowerCycleDoubleState load_power_cycle_double(
    std::string_view state_name,
    const std::filesystem::path& state_directory = "/dev/shm/rmcs_power_cycle_state") {

    if (state_name.empty()) {
        return {
            .status = PowerCycleStateStatus::FAILED,
            .value = std::nullopt,
            .message = "state name is empty",
        };
    }

    const std::string safe_name = detail::sanitize_power_cycle_init_name(state_name);
    const auto state_path       = state_directory / (safe_name + ".value");
    if (!detail::path_exists(state_path)) {
        return {
            .status = PowerCycleStateStatus::NOT_FOUND,
            .value = std::nullopt,
            .message = "state not found",
        };
    }

    std::ifstream state_file(state_path);
    if (!state_file.is_open()) {
        return {
            .status = PowerCycleStateStatus::FAILED,
            .value = std::nullopt,
            .message = "failed to open state file",
        };
    }

    double value = 0.0;
    state_file >> value >> std::ws;
    if (!state_file || !state_file.eof()) {
        return {
            .status = PowerCycleStateStatus::FAILED,
            .value = std::nullopt,
            .message = "failed to parse state file",
        };
    }

    return {
        .status = PowerCycleStateStatus::AVAILABLE,
        .value = value,
        .message = "state loaded",
    };
}

inline bool store_power_cycle_double(
    std::string_view state_name, double value, std::string* error_message = nullptr,
    const std::filesystem::path& state_directory = "/dev/shm/rmcs_power_cycle_state") {

    if (state_name.empty()) {
        if (error_message != nullptr) {
            *error_message = "state name is empty";
        }
        return false;
    }

    std::error_code ec;
    std::filesystem::create_directories(state_directory, ec);
    if (ec) {
        if (error_message != nullptr) {
            *error_message = "failed to create state directory: " + ec.message();
        }
        return false;
    }

    const std::string safe_name = detail::sanitize_power_cycle_init_name(state_name);
    const auto state_path       = state_directory / (safe_name + ".value");
    const auto temp_path        = state_directory / (safe_name + ".tmp");

    std::ofstream temp_file(temp_path, std::ios::trunc);
    if (!temp_file.is_open()) {
        if (error_message != nullptr) {
            *error_message = "failed to open temp state file";
        }
        return false;
    }

    temp_file << std::setprecision(std::numeric_limits<double>::max_digits10) << value << '\n';
    temp_file.close();
    if (temp_file.fail()) {
        if (error_message != nullptr) {
            *error_message = "failed to write temp state file";
        }

        std::error_code ignore_ec;
        std::filesystem::remove(temp_path, ignore_ec);
        return false;
    }

    ec.clear();
    std::filesystem::rename(temp_path, state_path, ec);
    if (ec) {
        if (error_message != nullptr) {
            *error_message = "failed to publish state file: " + ec.message();
        }

        std::error_code ignore_ec;
        std::filesystem::remove(temp_path, ignore_ec);
        return false;
    }

    return true;
}

inline bool clear_power_cycle_double(
    std::string_view state_name, std::string* error_message = nullptr,
    const std::filesystem::path& state_directory = "/dev/shm/rmcs_power_cycle_state") {

    if (state_name.empty()) {
        if (error_message != nullptr) {
            *error_message = "state name is empty";
        }
        return false;
    }

    std::error_code ec;
    const auto state_path =
        state_directory / (detail::sanitize_power_cycle_init_name(state_name) + ".value");
    if (std::filesystem::remove(state_path, ec) || !detail::path_exists(state_path)) {
        return true;
    }

    if (error_message != nullptr) {
        *error_message = ec ? "failed to clear state file: " + ec.message()
                            : "failed to clear state file";
    }
    return false;
}

} // namespace rmcs_utility
