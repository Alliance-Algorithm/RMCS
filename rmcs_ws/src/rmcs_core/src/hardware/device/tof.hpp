#pragma once

#include <atomic>

#include <cmath>
#include <cstdint>
#include <cstring>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
class Tof : rclcpp::Node {
public:
    struct Config {
        explicit Config() {
            this->reserved      = 1.0;
            this->mode_inquired = false;
        }
        Config& set_reversed() { return reserved = -1.0, *this; }
        Config& mode_to_inquired() { return mode_inquired = true, *this; }

        double reserved;
        bool mode_inquired;
    };

    explicit Tof(Component& status_component, const std::string& name_prefix)
        : Node{"bbb"} {
        status_component.register_output(
            name_prefix + "/distance", distance_, std::numeric_limits<double>::quiet_NaN());

        status_component.register_output(name_prefix + "/tof", tof_, this);
    };

    explicit Tof(
        rmcs_executor::Component& status_component, const std::string& name_prefix,
        const Config& config)
        : Tof(status_component, name_prefix) {
        configure(config);
    }

    Tof(const Tof&)            = delete;
    Tof& operator=(const Tof&) = delete;

    void configure(const Config& config) {
        reserve      = config.reserved;
        mode_inquire = config.mode_inquired;
    }

    void store_status(const std::byte* uart_data, size_t uart_data_length) {
        if (uart_data_length != package_length)
            return;
        Package package;
        std::memcpy(&package, uart_data + 2, sizeof(Package));
        package_.store(package, std::memory_order::relaxed);
    }

    void update_status() {
        const Package package = package_.load(std::memory_order::relaxed);

        distance_valid = package.calculate_valid();

        if (distance_valid) {

            *distance_ = package.calculate_distance();

            RCLCPP_INFO(this->get_logger(),"距离为：%f",*distance_);
        } else {
            //RCLCPP_INFO(this->get_logger(),"failed invalid data");
            *distance_ = std::numeric_limits<double>::quiet_NaN();
        }
    }

    double get_reserved() const { return reserve; }
    bool is_mode_inquire() const { return mode_inquire; }

    double get_distance() { return *distance_; }
    bool get_valid() { return distance_valid; }
    static constexpr size_t package_length = 14;

private:
    struct __attribute__((packed)) Package {
        uint8_t reserved;           // 0xFF
        uint8_t id;                 // 0x00
        uint8_t system_time[4];     // 0x9E
        uint8_t raw_distance[3];    // 0x8F
        uint8_t dis_status;         // 0x00
        uint8_t signal_strength[2]; // 0x00
        uint8_t range_precision;    // 0xAD
        uint8_t crc;                // 0x41

        double calculate_distance() const {
            uint32_t raw = raw_distance[0] | (raw_distance[1] << 8) | (raw_distance[2] << 16);
            return static_cast<double>(raw) / 1000.0;
        }

        bool calculate_valid() const { return static_cast<bool>(dis_status); }
    };

    std::atomic<Package> package_;

    Component::OutputInterface<double> distance_;
    Component::OutputInterface<Tof*> tof_;

    double reserve;
    bool mode_inquire;
    bool distance_valid;
};

} // namespace rmcs_core::hardware::device