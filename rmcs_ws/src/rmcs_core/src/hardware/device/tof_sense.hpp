#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/ring_buffer.hpp>

namespace rmcs_core::hardware::device {

class TofSense {
public:
    using Clock = std::chrono::steady_clock;

    static constexpr uint16_t kQueryId = 0x402;

    TofSense(
        rmcs_executor::Component& component, const std::string& prefix,
        const std::vector<uint8_t>& ids, Clock::duration timeout)
        : timeout_(timeout) {
        if (ids.empty())
            throw std::invalid_argument("TOFSense requires at least one module ID");
        if (timeout <= Clock::duration::zero())
            throw std::invalid_argument("TOFSense timeout must be positive");

        std::array<bool, 256> registered{};
        for (const auto id : ids) {
            if (registered[id])
                throw std::invalid_argument("TOFSense module IDs must be unique");
            registered[id] = true;
        }

        sensors_.reserve(ids.size());
        for (const auto id : ids) {
            auto sensor = std::make_unique<Sensor>();
            const auto name = prefix + "/sensor_" + std::to_string(id);
            component.register_output(name + "/distance", sensor->distance, kUnknownDistance);
            component.register_output(name + "/valid", sensor->valid, false);
            component.register_output(name + "/status", sensor->status, uint8_t{0xff});
            component.register_output(name + "/signal_strength", sensor->signal_strength, 0);
            component.register_output(
                name + "/age", sensor->age, std::numeric_limits<double>::infinity());
            component.register_output(name + "/frames", sensor->frames, 0);
            sensor_by_id_[id] = sensor.get();
            sensors_.push_back(std::move(sensor));
        }
        component.register_output(prefix + "/dropped_frames", dropped_frames_output_, 0);
    }

    // The SDK receive thread is the sole producer; the executor consumes complete samples.
    void store_can_status(
        uint32_t can_id, std::span<const std::byte> data, bool extended = false,
        bool remote = false, Clock::time_point now = Clock::now()) noexcept {
        if (extended || remote || data.size() != 8 || can_id < 0x200 || can_id > 0x2ff)
            return;
        const auto id = static_cast<uint8_t>(can_id - 0x200);
        if (!sensor_by_id_[id])
            return;

        const Sample sample{
            .received_at = now,
            .distance_mm = std::to_integer<uint32_t>(data[0])
                         | (std::to_integer<uint32_t>(data[1]) << 8)
                         | (std::to_integer<uint32_t>(data[2]) << 16),
            .signal_strength = static_cast<uint16_t>(
                std::to_integer<uint16_t>(data[4]) | (std::to_integer<uint16_t>(data[5]) << 8)),
            .id = id,
            .status = std::to_integer<uint8_t>(data[3]),
        };
        if (!samples_.emplace_back(sample)) {
            const auto dropped = dropped_frames_.load(std::memory_order::relaxed);
            if (dropped < std::numeric_limits<int>::max())
                dropped_frames_.store(dropped + 1, std::memory_order::relaxed);
        }
    }

    void update_status(Clock::time_point now = Clock::now()) noexcept {
        samples_.pop_front_n([this](const Sample& sample) noexcept {
            auto& sensor = *sensor_by_id_[sample.id];
            sensor.received = true;
            sensor.received_at = sample.received_at;
            sensor.distance_m = sample.distance_mm / 1000.0;
            *sensor.status = sample.status;
            *sensor.signal_strength = sample.signal_strength;
            if (*sensor.frames < std::numeric_limits<int>::max())
                ++*sensor.frames;
        });

        for (const auto& sensor : sensors_) {
            if (!sensor->received)
                continue;
            // A callback can arrive after the executor sampled its clock, but before queue
            // draining.
            const auto age = std::max(now - sensor->received_at, Clock::duration::zero());
            *sensor->age = std::chrono::duration<double>(age).count();
            *sensor->valid = age < timeout_ && *sensor->status == 0;
            *sensor->distance = *sensor->valid ? sensor->distance_m : kUnknownDistance;
        }
        *dropped_frames_output_ = dropped_frames_.load(std::memory_order::relaxed);
    }

    static constexpr std::array<std::byte, 8> query(uint8_t id) noexcept {
        return {std::byte{0xff}, std::byte{0xff}, std::byte{0xff}, std::byte{id},
                std::byte{0xff}, std::byte{0xff}, std::byte{0xff}, std::byte{0xff}};
    }

private:
    static constexpr double kUnknownDistance = std::numeric_limits<double>::quiet_NaN();

    struct Sample {
        Clock::time_point received_at;
        uint32_t distance_mm;
        uint16_t signal_strength;
        uint8_t id;
        uint8_t status;
    };

    struct Sensor {
        Clock::time_point received_at{};
        double distance_m = kUnknownDistance;
        bool received = false;

        rmcs_executor::Component::OutputInterface<double> distance;
        rmcs_executor::Component::OutputInterface<bool> valid;
        rmcs_executor::Component::OutputInterface<uint8_t> status;
        rmcs_executor::Component::OutputInterface<int> signal_strength;
        rmcs_executor::Component::OutputInterface<double> age;
        rmcs_executor::Component::OutputInterface<int> frames;
    };

    const Clock::duration timeout_;
    std::vector<std::unique_ptr<Sensor>> sensors_;
    std::array<Sensor*, 256> sensor_by_id_{};
    rmcs_utility::RingBuffer<Sample> samples_{64};
    std::atomic<int> dropped_frames_{0};
    rmcs_executor::Component::OutputInterface<int> dropped_frames_output_;
};

} // namespace rmcs_core::hardware::device
