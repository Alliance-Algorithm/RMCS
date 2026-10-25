#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <rmcs_executor/component.hpp>

#include "hardware/device/tof_sense.hpp"

namespace rmcs_core::hardware {

class TofSensors {
public:
    using Clock = device::TofSense::Clock;

    TofSensors(
        rmcs_executor::Component& component, const std::string& prefix,
        std::span<const int64_t> ids, double timeout_seconds) {
        const auto timeout = std::chrono::duration<double>{timeout_seconds};
        if (!std::isfinite(timeout_seconds) || timeout < Clock::duration{1}
            || timeout >= Clock::duration::max())
            throw std::invalid_argument("TOF feedback timeout must be positive and finite");
        if (ids.empty())
            throw std::invalid_argument("TOF sensor IDs must not be empty");

        std::array<bool, 256> registered{};
        for (const auto id : ids) {
            if (id < 0 || id > 255)
                throw std::invalid_argument("TOF sensor IDs must be between 0 and 255");
            if (registered[id])
                throw std::invalid_argument("TOF sensor IDs must be unique");
            registered[id] = true;
        }

        sensors_.reserve(ids.size());
        for (const auto id : ids) {
            sensors_.push_back(
                std::make_unique<device::TofSense>(
                    component, prefix + "/sensor_" + std::to_string(id),
                    device::TofSense::Config{
                        .id = static_cast<uint8_t>(id),
                        .feedback_timeout = std::chrono::duration_cast<Clock::duration>(timeout)}));
        }
        component.register_output(prefix + "/dropped_frames", dropped_frames_, 0);
    }

    // The board callback filters the bus and rejects extended, remote and FD frames.
    void store_status(
        uint32_t can_id, std::span<const std::byte> can_data,
        Clock::time_point now = Clock::now()) noexcept {
        for (const auto& sensor : sensors_) {
            if (sensor->match_then_store_status(can_id, can_data, now))
                break;
        }
    }

    void update_status(Clock::time_point now = Clock::now()) noexcept {
        int64_t dropped = 0;
        for (const auto& sensor : sensors_) {
            sensor->update_status(now);
            dropped += sensor->dropped_frames();
        }
        *dropped_frames_ =
            static_cast<int>(std::min<int64_t>(dropped, std::numeric_limits<int>::max()));
    }

private:
    std::vector<std::unique_ptr<device::TofSense>> sensors_;
    rmcs_executor::Component::OutputInterface<int> dropped_frames_;
};

} // namespace rmcs_core::hardware
