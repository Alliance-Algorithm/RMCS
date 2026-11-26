#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>

#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class DmMotor {
public:
    using Clock = std::chrono::steady_clock;
    enum class Type : uint8_t { kDM8009 };

    struct Config {
        explicit Config(Type type, uint32_t send_id, uint32_t master_id)
            : type(type)
            , send_id(send_id)
            , master_id(master_id) {}

        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle = true, *this; }
        Config& set_arming_timeout(std::chrono::milliseconds value) {
            arming_timeout = value;
            return *this;
        }

        Type type;
        uint32_t send_id, master_id;
        bool reversed = false, multi_turn_angle = false;
        std::chrono::milliseconds arming_timeout{100};
    };

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.);
        status_component.register_output(name_prefix + "/status", status_output_, 0);
        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    void configure(const Config& config) {
        if (config.arming_timeout.count() <= 0)
            throw std::invalid_argument("DM arming timeout must be positive");
        arming_timeout_ = config.arming_timeout;
        send_id_ = config.send_id;
        master_id_ = config.master_id;
        reverse_factor_ = config.reversed ? -1. : 1.;
        multi_turn_angle_ = config.multi_turn_angle;
        switch (config.type) {
        case Type::kDM8009:
            // MIT ranges match the drive's P_MAX / V_MAX / T_MAX registers; 40 N*m is the peak
            // torque.
            position_max_ = std::numbers::pi;
            velocity_max_ = 45.;
            torque_max_ = 54.;
            max_torque_ = 40.;
            break;
        }
        initialized_ = false;
        state_ = State::kDisabled;
        arming_ = true;
        error_cleared_ = fault_latched_ = false;
        arming_deadline_.reset();
    }

    bool store_status(std::span<const std::byte> can_data) {
        if (can_data.size() != 8
            || (std::to_integer<uint8_t>(can_data[0]) & 0x0F) != (send_id_ & 0x0F)) [[unlikely]]
            return false;
        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
        received_sequence_.fetch_add(1, std::memory_order_release);
        return true;
    }

    void update_status() {
        const auto sequence = received_sequence_.load(std::memory_order_acquire);
        if (sequence == status_sequence_)
            return;
        auto packet = can_data_.load(std::memory_order_relaxed);
        const auto bytes = packet.as_bytes();
        const auto id_state = std::to_integer<uint8_t>(bytes[0]);
        if ((id_state & 0x0F) != (send_id_ & 0x0F))
            return;

        state_ = static_cast<State>(id_state >> 4);
        *status_output_ = id_state >> 4;
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
        status_sequence_ = sequence;
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

    // A new enabled feedback frame must acknowledge arming before torque is transmitted.
    CanPacket8 generate_command(Clock::time_point now = Clock::now()) {
        const double torque = control_torque();
        if (!std::isfinite(torque)) {
            arming_ = true;
            error_cleared_ = fault_latched_ = false;
            arming_deadline_.reset();
            return generate_disable_command();
        }
        if (fault_latched_)
            return generate_disable_command();
        if (!arming_) {
            if (state_ == State::kEnabled)
                return generate_torque_command(torque);
            fault_latched_ = true;
            return generate_disable_command();
        }
        if (!arming_deadline_) {
            arming_deadline_ = now + arming_timeout_;
            arming_sequence_ = status_sequence_;
        }
        if (now >= *arming_deadline_) {
            fault_latched_ = true;
            return generate_disable_command();
        }
        if (state_ == State::kEnabled && status_sequence_ != arming_sequence_) {
            arming_ = false;
            return generate_torque_command(torque);
        }
        if (state_ == State::kEnabled)
            return generate_enable_command();
        if (state_ == State::kDisabled)
            return generate_enable_command();
        if (!error_cleared_) {
            error_cleared_ = true;
            arming_sequence_ = status_sequence_;
            return generate_clear_error_command();
        }
        return generate_disable_command();
    }

    CanPacket8 generate_enable_command() const { return system_command(0xFC); }
    CanPacket8 generate_disable_command() const { return system_command(0xFD); }
    CanPacket8 generate_clear_error_command() const { return system_command(0xFB); }

    CanPacket8 generate_torque_command(double control_torque) const {
        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        const auto torque = float_to_uint(reverse_factor_ * control_torque, torque_max_, 12);
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
    rmcs_executor::Component::OutputInterface<int> status_output_;
    rmcs_executor::Component::InputInterface<double> control_torque_;

    std::atomic<CanPacket8> can_data_{CanPacket8{0}};
    std::atomic<std::uint64_t> received_sequence_{0};
    std::uint64_t status_sequence_ = 0, arming_sequence_ = 0;
    std::chrono::milliseconds arming_timeout_{100};
    std::optional<Clock::time_point> arming_deadline_;
    uint32_t send_id_ = 1, master_id_ = 1;
    double position_max_ = std::numbers::pi, velocity_max_ = 45., torque_max_ = 54.,
           max_torque_ = 40.;
    double reverse_factor_ = 1., last_raw_angle_ = 0.;
    bool multi_turn_angle_ = false, initialized_ = false;
    bool arming_ = true, error_cleared_ = false, fault_latched_ = false;
    State state_ = State::kDisabled;
};

} // namespace rmcs_core::hardware::device
