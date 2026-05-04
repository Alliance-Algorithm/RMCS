#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <limits>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

/// @brief GY-614V3BAA, a kind of temperature sensor
class Gy614 {
public:
    explicit Gy614(Component& status_component, const std::string& name) {
        status_component.register_output(
            name, target_temperature_, std::numeric_limits<double>::quiet_NaN());
    };

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != package_length)
            return;

        // TODO: CRC check

        Package package;
        std::memcpy(&package, uart_data + 4, sizeof(Package));
        package_.store(package, std::memory_order::relaxed);
    }

    void update_status() {
        const auto package = package_.load(std::memory_order::relaxed);

        emissivity_                    = package.calculate_emissivity();
        *target_temperature_           = package.calculate_target_temperature();
        environment_temperature_       = package.calculate_environment_temperature();
        body_temperature_from_forhead_ = package.calculate_body_temperature_from_forhead();
    }

    double emissivity() const { return emissivity_; }
    double target_temperature() const { return *target_temperature_; }
    double environment_temperature() const { return environment_temperature_; }
    double body_temperature_from_forhead() const { return body_temperature_from_forhead_; }

    static constexpr size_t package_length = 12;

private:
    struct __attribute__((packed)) Package {
        uint8_t e;
        uint8_t to[2];
        uint8_t ta[2];
        uint8_t bo[2];
        uint8_t crc;

        double calculate_emissivity() const { //
            return static_cast<double>(e) / 100;
        }
        double calculate_target_temperature() const {
            return static_cast<double>(to[0] * 256 + to[1]) / 100;
        }
        double calculate_environment_temperature() const {
            return static_cast<double>(ta[0] * 256 + ta[1]) / 100;
        }
        double calculate_body_temperature_from_forhead() const {
            return static_cast<double>(bo[0] * 256 + bo[1]) / 100;
        }
    };

    std::atomic<Package> package_;
    static_assert(decltype(package_)::is_always_lock_free);

    double emissivity_ = 0;
    Component::OutputInterface<double> target_temperature_;
    double environment_temperature_       = 0;
    double body_temperature_from_forhead_ = 0;
};

} // namespace rmcs_core::hardware::device