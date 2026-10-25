#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/endian_promise.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class TofSense {
public:
    using Clock = std::chrono::steady_clock;

    struct Config {
        uint8_t id = 0;
        Clock::duration feedback_timeout = std::chrono::milliseconds{200};
    };

    TofSense(rmcs_executor::Component& component, const std::string& name, const Config& config)
        : config_(config) {
        if (config.feedback_timeout <= Clock::duration::zero())
            throw std::invalid_argument("TOFSense timeout must be positive");

        component.register_output(name + "/distance", distance_, kUnknownDistance);
        component.register_output(name + "/valid", valid_, false);
        component.register_output(name + "/status", status_, uint8_t{0xff});
        component.register_output(name + "/signal_strength", signal_strength_, 0);
        component.register_output(name + "/age", age_, std::numeric_limits<double>::infinity());
        component.register_output(name + "/frames", frames_, 0);
    }

    TofSense(const TofSense&) = delete;
    TofSense& operator=(const TofSense&) = delete;
    TofSense(TofSense&&) = delete;
    TofSense& operator=(TofSense&&) = delete;

    static constexpr uint32_t recv_id(uint8_t id) noexcept { return 0x200 + id; }
    static constexpr uint32_t send_id() noexcept { return 0x402; }

    uint8_t id() const noexcept { return config_.id; }
    uint32_t recv_id() const noexcept { return recv_id(id()); }

    bool matches_feedback(uint32_t can_id, std::span<const std::byte> can_data) const noexcept {
        return can_id == recv_id() && can_data.size() == sizeof(CanPacket8);
    }

    bool match_then_store_status(
        uint32_t can_id, std::span<const std::byte> can_data,
        Clock::time_point now = Clock::now()) noexcept {
        if (!matches_feedback(can_id, can_data))
            return false;
        store_status(can_data, now);
        return true;
    }

    // Keep the frame and its receive time together across the SDK/executor thread boundary.
    void store_status(
        std::span<const std::byte> can_data, Clock::time_point now = Clock::now()) noexcept {
        if (can_data.size() != sizeof(CanPacket8)) [[unlikely]]
            return;

        if (!samples_.emplace_back(Sample{CanPacket8{can_data}, now})) {
            const auto dropped = dropped_frames_.load(std::memory_order_relaxed);
            if (dropped < std::numeric_limits<int>::max())
                dropped_frames_.store(dropped + 1, std::memory_order_relaxed);
        }
    }

    void update_status(Clock::time_point now = Clock::now()) noexcept {
        samples_.pop_front_n([this](const Sample& sample) noexcept {
            const auto feedback = std::bit_cast<Feedback>(sample.packet);
            received_at_ = sample.received_at;
            distance_m_ = feedback.distance();
            *status_ = feedback.status();
            *signal_strength_ = feedback.signal_strength;
            if (*frames_ < std::numeric_limits<int>::max())
                ++*frames_;
        });

        if (*frames_ == 0)
            return;
        // A callback may arrive after the executor samples its clock, before queue draining.
        const auto elapsed = std::max(now - received_at_, Clock::duration::zero());
        *age_ = std::chrono::duration<double>(elapsed).count();
        *valid_ = elapsed < config_.feedback_timeout && *status_ == 0;
        *distance_ = *valid_ ? distance_m_ : kUnknownDistance;
    }

    CanPacket8 generate_status_request() const noexcept {
        const struct alignas(CanPacket8) {
            uint8_t reserved_before[3] = {0xff, 0xff, 0xff};
            uint8_t id;
            uint8_t reserved_after[4] = {0xff, 0xff, 0xff, 0xff};
        } request{.id = id()};
        return std::bit_cast<CanPacket8>(request);
    }

    double distance() const noexcept { return *distance_; }
    bool valid() const noexcept { return *valid_; }
    uint8_t status() const noexcept { return *status_; }
    int signal_strength() const noexcept { return *signal_strength_; }
    double age() const noexcept { return *age_; }
    int frames() const noexcept { return *frames_; }
    int dropped_frames() const noexcept { return dropped_frames_.load(std::memory_order_relaxed); }

private:
    static constexpr double kUnknownDistance = std::numeric_limits<double>::quiet_NaN();

    struct alignas(CanPacket8) Feedback {
        // The first little-endian word holds a 24-bit distance followed by an 8-bit status.
        rmcs_utility::le_uint32_t distance_and_status;
        rmcs_utility::le_uint16_t signal_strength;
        std::array<std::byte, 2> reserved;

        double distance() const noexcept {
            return (static_cast<uint32_t>(distance_and_status) & 0x00ffffff) / 1000.0;
        }
        uint8_t status() const noexcept { return static_cast<uint32_t>(distance_and_status) >> 24; }
    };
    static_assert(sizeof(Feedback) == sizeof(CanPacket8));

    struct Sample {
        CanPacket8 packet;
        Clock::time_point received_at;
    };

    const Config config_;
    Clock::time_point received_at_{};
    double distance_m_ = kUnknownDistance;
    rmcs_utility::RingBuffer<Sample> samples_{64};
    std::atomic<int> dropped_frames_{0};

    rmcs_executor::Component::OutputInterface<double> distance_;
    rmcs_executor::Component::OutputInterface<bool> valid_;
    rmcs_executor::Component::OutputInterface<uint8_t> status_;
    rmcs_executor::Component::OutputInterface<int> signal_strength_;
    rmcs_executor::Component::OutputInterface<double> age_;
    rmcs_executor::Component::OutputInterface<int> frames_;
};

} // namespace rmcs_core::hardware::device
