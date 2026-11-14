#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class Tof {
public:
    struct Config {
        Config& set_id(uint8_t value) { return id = value, *this; }
        uint8_t id = 0;
    };

    explicit Tof(rmcs_executor::Component& status_component, const std::string& name_prefix) {
        status_component.register_output(
            name_prefix + "/distance", distance_, std::numeric_limits<double>::quiet_NaN());
    }

    void configure(const Config& config) { id_ = config.id; }
    uint32_t recv_id() const { return 0x200 + id_; }

    // ACTIVE CAN Frame0: distance[3], status, signal strength[2], reserved[2].
    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;
        Package package{};
        std::memcpy(&package, can_data.data(), 6);
        package.received = true;
        package_.store(package, std::memory_order_relaxed);
    }

    void update_status() {
        const auto package = package_.load(std::memory_order_relaxed);
        const uint32_t raw =
            package.distance[0] | (package.distance[1] << 8) | (package.distance[2] << 16);
        *distance_ = package.received && package.status == 0
                       ? static_cast<double>(raw) / 1000.
                       : std::numeric_limits<double>::quiet_NaN();
    }

private:
    struct alignas(uint64_t) Package {
        uint8_t distance[3], status, signal_strength[2], reserved;
        bool received;
    };
    static_assert(sizeof(Package) == 8);
    static_assert(std::atomic<Package>::is_always_lock_free);

    rmcs_executor::Component::OutputInterface<double> distance_;
    std::atomic<Package> package_{Package{}};
    uint8_t id_ = 0;
};

} // namespace rmcs_core::hardware::device
