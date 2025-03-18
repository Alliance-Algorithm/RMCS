#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <limits>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Benewake {
public:
    explicit Benewake(Component& status_component, const std::string& name) {
        status_component.register_output(name, distance_, std::numeric_limits<double>::quiet_NaN());
    }

    bool store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != sizeof(Package) + sizeof(uint8_t)) {
            return false;
        }

        Package package;
        uint8_t checksum;
        std::memcpy(&package, uart_data, sizeof(Package));
        std::memcpy(&checksum, uart_data + sizeof(Package), sizeof(uint8_t));

        if (package.header[0] != 0x59 || package.header[1] != 0x59) {
            return false;
        }

        if (checksum != package.calculate_checksum()) {
            return false;
        }
        package_.store(package, std::memory_order::relaxed);
        return true;
    }

    void update_status() {
        const auto package = package_.load(std::memory_order::relaxed);

        *distance_       = package.calculate_distance();
        signal_strength_ = package.calculate_signal_strength();
    }

    double get_distance() const { return *distance_; }
    uint16_t get_signal_strength() const { return signal_strength_; }

private:
    static constexpr uint16_t max_ = std::numeric_limits<uint16_t>::max();

    struct __attribute__((packed)) Package {
        uint8_t header[2];
        uint8_t distance[2];
        uint8_t signal_strength[2];
        uint8_t reserved[2];

        double calculate_distance() const { return (distance[1] << 8 | distance[0]) / 100.0; }

        uint16_t calculate_signal_strength() const {
            return signal_strength[1] << 8 | signal_strength[0];
        }

        uint8_t calculate_checksum() const {
            return header[0] ^ header[1] ^ distance[0] ^ distance[1] ^ signal_strength[0]
                 ^ signal_strength[1] ^ reserved[0] ^ reserved[1];
        }
    };

    std::atomic<Package> package_;
    static_assert(decltype(package_)::is_always_lock_free);

    Component::OutputInterface<double> distance_;
    /*
     * Strength value is between 0 and 3500. Threshold of strength is 40, when
     * strength is lower than 40, distance will output maximum value. When
     * strength is between 40 and 1200, distance is more reliable. When there is a
     * high reflectivity object, strength will be over 1500.
     */
    uint16_t signal_strength_ = max_;
};
} // namespace rmcs_core::hardware::device