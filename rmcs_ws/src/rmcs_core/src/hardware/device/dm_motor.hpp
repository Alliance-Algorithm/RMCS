#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <span>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class DmMotor {
public:
    enum class Type : uint8_t { kDM8009 };

    struct Config {
        explicit Config(Type type, uint32_t send_id, uint32_t master_id)
            : type(type)
            , send_id(send_id)
            , master_id(master_id) {}

        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle = true, *this; }

        Type type;
        uint32_t send_id, master_id;
        bool reversed = false, multi_turn_angle = false;
    };

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.);
        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    void configure(const Config& config) {
        send_id_ = config.send_id;
        master_id_ = config.master_id;
        reverse_factor_ = config.reversed ? -1. : 1.;
        multi_turn_angle_ = config.multi_turn_angle;
        switch (config.type) {
        case Type::kDM8009:
            // MIT ranges match the drive's P_MAX / V_MAX / T_MAX registers.
            position_max_ = std::numbers::pi;
            velocity_max_ = 45.;
            torque_max_ = 54.;
            break;
        }
        initialized_ = false;
        state_ = State::kDisabled;
    }

    void store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8) [[unlikely]]
            return;
        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
    }

    void update_status() {
        auto packet = can_data_.load(std::memory_order_relaxed);
        const auto bytes = packet.as_bytes();
        const auto id_state = std::to_integer<uint8_t>(bytes[0]);
        if ((id_state & 0x0F) != (send_id_ & 0x0F))
            return;

        state_ = static_cast<State>(id_state >> 4);
        const auto position = static_cast<uint16_t>(
            (std::to_integer<uint16_t>(bytes[1]) << 8) | std::to_integer<uint16_t>(bytes[2]));
        const auto velocity = static_cast<uint16_t>(
            (std::to_integer<uint16_t>(bytes[3]) << 4)
            | (std::to_integer<uint16_t>(bytes[4]) >> 4));
        const auto torque = static_cast<uint16_t>(
            ((std::to_integer<uint16_t>(bytes[4]) & 0x0F) << 8)
            | std::to_integer<uint16_t>(bytes[5]));

        const double raw = reverse_factor_ * uint_to_float(position, position_max_, 16);
        if (multi_turn_angle_) {
            const double step = std::remainder(raw - last_raw_angle_, 2. * position_max_);
            const bool anchor = !initialized_ || !(std::abs(step) <= std::numbers::pi / 2.);
            *angle_output_ = anchor ? raw : *angle_output_ + step;
        } else {
            *angle_output_ = raw;
        }
        last_raw_angle_ = raw;
        initialized_ = true;
        *velocity_output_ = reverse_factor_ * uint_to_float(velocity, velocity_max_, 12);
        *torque_output_ = reverse_factor_ * uint_to_float(torque, torque_max_, 12);
    }

    uint32_t send_id() const { return send_id_; }
    uint32_t master_id() const { return master_id_; }
    bool enabled() const { return state_ == State::kEnabled; }
    double angle() const { return *angle_output_; }
    double velocity() const { return *velocity_output_; }
    double torque() const { return *torque_output_; }
    double control_torque() const {
        return control_torque_.ready() ? *control_torque_
                                       : std::numeric_limits<double>::quiet_NaN();
    }

    CanPacket8 generate_command() const {
        const double torque = control_torque();
        if (std::isnan(torque))
            return generate_disable_command();
        if (state_ == State::kDisabled)
            return generate_enable_command();
        if (state_ != State::kEnabled)
            return generate_clear_error_command();
        return generate_torque_command(torque);
    }

    CanPacket8 generate_enable_command() const { return system_command(0xFC); }
    CanPacket8 generate_disable_command() const { return system_command(0xFD); }
    CanPacket8 generate_clear_error_command() const { return system_command(0xFB); }

    CanPacket8 generate_torque_command(double control_torque) const {
        constexpr double kControlTorqueMax = 40.;
        control_torque = std::clamp(control_torque, -kControlTorqueMax, kControlTorqueMax);
        auto torque = float_to_uint(reverse_factor_ * control_torque, torque_max_, 12);
        // Keep the quantized command inside the software torque limit.
        if (std::abs(uint_to_float(torque, torque_max_, 12)) > kControlTorqueMax)
            torque = float_to_uint(
                reverse_factor_
                    * std::copysign(kControlTorqueMax - 2. * torque_max_ / 4095., control_torque),
                torque_max_, 12);
        const auto position = float_to_uint(0., position_max_, 16);
        const auto velocity = float_to_uint(0., velocity_max_, 12);
        const std::array<std::byte, 8> bytes{
            std::byte(position >> 8),
            std::byte(position & 0xFF),
            std::byte(velocity >> 4),
            std::byte((velocity & 0x0F) << 4),
            std::byte{0},
            std::byte{0},
            std::byte(torque >> 8),
            std::byte(torque & 0xFF)};
        return CanPacket8{std::span<const std::byte>{bytes}};
    }

private:
    enum class State : uint8_t { kDisabled = 0x0, kEnabled = 0x1 };

    static uint16_t float_to_uint(double value, double max, int bits) {
        const double limit = static_cast<double>((uint32_t{1} << bits) - 1);
        return static_cast<uint16_t>(
            std::round(std::clamp((value + max) / (2. * max) * limit, 0., limit)));
    }
    static double uint_to_float(uint16_t value, double max, int bits) {
        const double limit = static_cast<double>((uint32_t{1} << bits) - 1);
        return -max + static_cast<double>(value) / limit * (2. * max);
    }
    static CanPacket8 system_command(uint8_t command) {
        std::array<std::byte, 8> bytes;
        bytes.fill(std::byte{0xFF});
        bytes.back() = std::byte{command};
        return CanPacket8{std::span<const std::byte>{bytes}};
    }

    rmcs_executor::Component::OutputInterface<double> angle_output_, velocity_output_,
        torque_output_;
    rmcs_executor::Component::InputInterface<double> control_torque_;

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};
    uint32_t send_id_ = 1, master_id_ = 1;
    double position_max_ = std::numbers::pi, velocity_max_ = 45., torque_max_ = 54.;
    double reverse_factor_ = 1., last_raw_angle_ = 0.;
    bool multi_turn_angle_ = false, initialized_ = false;
    State state_ = State::kDisabled;
};

} // namespace rmcs_core::hardware::device
