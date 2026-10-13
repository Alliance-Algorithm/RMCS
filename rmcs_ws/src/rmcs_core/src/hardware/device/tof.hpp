#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

class Tof {
public:
    using Clock = std::chrono::steady_clock;

    struct Config {
        Config& set_reversed() { return reverse_factor = -1.0, *this; }
        Config& mode_to_inquired() { return mode_inquired = true, *this; }

        double reverse_factor = 1.0;
        bool mode_inquired = false;
        std::chrono::milliseconds timeout{200};
    };

    explicit Tof(Component& status_component, const std::string& name_prefix) {
        status_component.register_output(
            name_prefix + "/distance", distance_, std::numeric_limits<double>::quiet_NaN());
    }

    explicit Tof(Component& status_component, const std::string& name_prefix, const Config& config)
        : Tof(status_component, name_prefix) {
        configure(config);
    }

    Tof(const Tof&) = delete;
    Tof& operator=(const Tof&) = delete;

    void configure(const Config& config) {
        if (config.timeout.count() <= 0)
            throw std::invalid_argument("TOF timeout must be positive");
        config_ = config;
    }

    // UART Frame0: 16 bytes, little-endian uint24 distance, trailing sum check.
    bool store_status(
        const std::byte* uart_data, size_t uart_data_length, Clock::time_point now = Clock::now()) {
        if (uart_data_length != package_length || uart_data == nullptr)
            return false;
        if (uart_data[0] != std::byte{0x57} || uart_data[1] != std::byte{0x00})
            return false;

        uint8_t checksum = 0;
        for (size_t i = 0; i < package_length - 1; ++i)
            checksum += std::to_integer<uint8_t>(uart_data[i]);
        if (checksum != std::to_integer<uint8_t>(uart_data[package_length - 1]))
            return false;

        Package package{};
        std::memcpy(&package, uart_data + 8, 6);
        store_package(package, now);
        return true;
    }

    // CAN Frame0: distance[3], dis_status, signal_strength[2], reserved[2].
    bool store_status(std::span<const std::byte> can_data, Clock::time_point now = Clock::now()) {
        if (can_data.size() != can_package_length)
            return false;

        Package package{};
        std::memcpy(&package, can_data.data(), 6);
        store_package(package, now);
        return true;
    }

    void update_status(Clock::time_point now = Clock::now()) {
        const auto package = package_.load(std::memory_order::relaxed);
        const auto received = Clock::time_point{
            std::chrono::nanoseconds{last_received_ns_.load(std::memory_order::relaxed)}};
        distance_valid_ =
            package.received && package.dis_status == 0 && now - received <= config_.timeout;
        *distance_ = distance_valid_ ? package.calculate_distance() * config_.reverse_factor
                                     : std::numeric_limits<double>::quiet_NaN();
    }

    void update() { update_status(); }

    double get_reverse_factor() const { return config_.reverse_factor; }
    bool is_mode_inquire() const { return config_.mode_inquired; }
    double get_distance() const { return *distance_; }
    bool get_valid() const { return distance_valid_; }

    static std::array<std::byte, 8> generate_can_query(uint8_t id) {
        return {std::byte{0xff}, std::byte{0xff}, std::byte{0xff}, std::byte{id},
                std::byte{0xff}, std::byte{0xff}, std::byte{0xff}, std::byte{0xff}};
    }

    static constexpr size_t package_length = 16;
    static constexpr size_t can_package_length = 8;
    static constexpr uint32_t can_feedback_base_id = 0x200;
    static constexpr uint32_t can_query_id = 0x402;

private:
    struct alignas(uint64_t) Package {
        uint8_t raw_distance[3];
        uint8_t dis_status;
        uint8_t signal_strength[2];
        uint8_t reserved;
        uint8_t received;

        double calculate_distance() const {
            const uint32_t raw = raw_distance[0] | (raw_distance[1] << 8) | (raw_distance[2] << 16);
            return static_cast<double>(raw) / 1000.0;
        }
    };
    static_assert(sizeof(Package) == can_package_length);
    static_assert(std::atomic<Package>::is_always_lock_free);

    void store_package(Package package, Clock::time_point now) {
        package.received = 1;
        last_received_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(),
            std::memory_order::relaxed);
        package_.store(package, std::memory_order::relaxed);
    }

    Component::OutputInterface<double> distance_;

    Config config_;
    std::atomic<Package> package_{Package{}};
    std::atomic<int64_t> last_received_ns_{0};
    bool distance_valid_ = false;
};

} // namespace rmcs_core::hardware::device
