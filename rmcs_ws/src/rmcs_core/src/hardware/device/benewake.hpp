#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <atomic>
#include <limits>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Benewake {
public:
    explicit Benewake(Component& status_component, const std::string& name) {
        status_component.register_output(name, distance_, nan_);
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != sizeof(Package)) {
            return;
        }

        Package package;
        uint8_t checksum;
        std::memcpy(&package, uart_data, sizeof(Package));
        std::memcpy(&checksum, uart_data + sizeof(Package), sizeof(uint8_t));

        if (checksum != package.calculate_checksum()) {
            return;
        }

        if (package.header[0] != 0x59 || package.header[1] != 0x59) {
            return;
        }
        package_.store(package, std::memory_order::relaxed);
    }

    void update_status() {
        const auto package = package_.load(std::memory_order::relaxed);

        *distance_       = package.calculate_distance();
        signal_strength_ = package.calculate_signal_strength();
    }

    double get_distance() const { return *distance_; }
    double get_signal_strength() const { return signal_strength_; }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    struct __attribute__((packed)) Package {
        uint8_t header[2];
        uint8_t distance[2];
        uint8_t signal_strength[2];
        uint8_t reserved[2];

        double calculate_distance() const {
            return static_cast<double>(distance[0] * 256 + distance[1]);
        }

        double calculate_signal_strength() const {
            return static_cast<double>(signal_strength[0] * 256 + signal_strength[1]);
        }

        double calculate_reserved() const {
            return static_cast<double>(reserved[0] * 256 + reserved[1]);
        }

        uint8_t calculate_checksum() const {
            return header[0] ^ header[1] ^ distance[0] ^ distance[1] ^ signal_strength[0]
                 ^ signal_strength[1] ^ reserved[0] ^ reserved[1];
        }
    };

    std::atomic<Package> package_;
    static_assert(decltype(package_)::is_always_lock_free);

    Component::OutputInterface<double> distance_;
    double signal_strength_ = nan_;
};

} // namespace rmcs_core::hardware::device