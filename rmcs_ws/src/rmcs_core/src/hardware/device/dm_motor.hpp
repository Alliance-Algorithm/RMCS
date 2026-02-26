#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
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
        explicit Config(Type motor_type)
            : motor_type(motor_type) {}

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_gear_ratio(double value) { return gear_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point = 0;
        double gear_ratio = 1.0;
        bool reversed = false;
        bool multi_turn_angle_enabled = false;
    };

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {

        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);
        status_component.register_output(name_prefix + "/state", state_output_, uint8_t{0});
        status_component.register_output(name_prefix + "/temperature_mos", temperature_mos_output_, 0.0);
        status_component.register_output(
            name_prefix + "/temperature_coil", temperature_coil_output_, 0.0);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DmMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DmMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DmMotor(const DmMotor&) = delete;
    DmMotor& operator=(const DmMotor&) = delete;
    DmMotor(DmMotor&&) = delete;
    DmMotor& operator=(DmMotor&&) = delete;

    ~DmMotor() = default;

    void configure(const Config& config) {
        switch (config.motor_type) {
        case Type::kDM8009:
            velocity_max_ = 25.0;
            max_torque_ = 40.0;
            break;
        }

        encoder_zero_point_ = config.encoder_zero_point % static_cast<int>(kRawAngleMax);
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += static_cast<int>(kRawAngleMax);

        const double sign = config.reversed ? -1.0 : 1.0;

        raw_angle_to_angle_coefficient_ =
            sign * 2.0 * std::numbers::pi / (static_cast<double>(kRawAngleMax) * config.gear_ratio);

        raw_velocity_to_velocity_coefficient_ = sign;
        raw_torque_to_torque_coefficient_ = sign;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;
        last_raw_angle_ = 0;

        *max_torque_output_ = max_torque();
    }

    void store_status(std::span<const std::byte> can_data) {
        can_packet_.store(CanPacket8{can_data}, std::memory_order::relaxed);
    }

    void update_status() {
        const auto feedback =
            std::bit_cast<Feedback>(can_packet_.load(std::memory_order::relaxed));

        state_ = static_cast<uint8_t>((feedback.byte0 >> 4) & 0x0F);
        const uint16_t raw_angle =
            static_cast<uint16_t>((static_cast<uint16_t>(feedback.byte1) << 8) | feedback.byte2);
        const uint16_t raw_velocity = static_cast<uint16_t>(
            (static_cast<uint16_t>(feedback.byte3) << 4) | (feedback.byte4 >> 4));
        const uint16_t raw_torque = static_cast<uint16_t>(
            ((static_cast<uint16_t>(feedback.byte4) & 0x0F) << 8) | feedback.byte5);

        int calibrated_raw_angle = static_cast<int>(raw_angle) - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += static_cast<int>(kRawAngleMax);

        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ > std::numbers::pi)
                angle_ -= 2.0 * std::numbers::pi;
            else if (angle_ < -std::numbers::pi)
                angle_ += 2.0 * std::numbers::pi;
        } else {
            auto diff =
                (calibrated_raw_angle - angle_multi_turn_) % static_cast<int64_t>(kRawAngleMax);
            if (diff <= -static_cast<int64_t>(kRawAngleMax / 2))
                diff += static_cast<int64_t>(kRawAngleMax);
            else if (diff > static_cast<int64_t>(kRawAngleMax / 2))
                diff -= static_cast<int64_t>(kRawAngleMax);

            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        last_raw_angle_ = raw_angle;

        velocity_ =
            raw_velocity_to_velocity_coefficient_ * uint_to_double(raw_velocity, -velocity_max_, velocity_max_, 12);
        torque_ = raw_torque_to_torque_coefficient_ * uint_to_double(raw_torque, -max_torque_, max_torque_, 12);

        temperature_mos_ = static_cast<double>(feedback.byte6);
        temperature_coil_ = static_cast<double>(feedback.byte7);

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
        *state_output_ = state();
        *temperature_mos_output_ = temperature_mos();
        *temperature_coil_output_ = temperature_coil();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        return std::numeric_limits<double>::quiet_NaN();
    }

    CanPacket8 generate_torque_command() const { return generate_torque_command(control_torque()); }

    CanPacket8 generate_torque_command(double control_torque) const {
        if (std::isnan(control_torque))
            control_torque = 0.0;

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        const uint16_t command_torque =
            static_cast<uint16_t>(double_to_uint(control_torque, -max_torque_, max_torque_, 12));

        std::array<uint8_t, 8> command{};
        command[6] = static_cast<uint8_t>((command_torque >> 8) & 0x0F);
        command[7] = static_cast<uint8_t>(command_torque & 0xFF);
        return std::bit_cast<CanPacket8>(command);
    }

    static constexpr CanPacket8 generate_enable_command() {
        return std::bit_cast<CanPacket8>(kEnableCommand);
    }

    static constexpr CanPacket8 generate_close_command() {
        return std::bit_cast<CanPacket8>(kCloseCommand);
    }

    static constexpr CanPacket8 generate_clear_error_command() {
        return std::bit_cast<CanPacket8>(kClearErrorCommand);
    }

    int calibrate_zero_point() {
        angle_multi_turn_ = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    double angle() const { return angle_; }
    double velocity() const { return velocity_; }
    double torque() const { return torque_; }
    double max_torque() const { return max_torque_; }
    uint8_t state() const { return state_; }
    double temperature_mos() const { return temperature_mos_; }
    double temperature_coil() const { return temperature_coil_; }

private:
    struct alignas(uint64_t) Feedback {
        uint8_t byte0;
        uint8_t byte1;
        uint8_t byte2;
        uint8_t byte3;
        uint8_t byte4;
        uint8_t byte5;
        uint8_t byte6;
        uint8_t byte7;
    };
    static_assert(sizeof(Feedback) == sizeof(CanPacket8));

    static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        return static_cast<double>(x_int) * span / static_cast<double>((1 << bits) - 1) + x_min;
    }

    static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
        const double span = x_max - x_min;
        return static_cast<int>(
            (x_double - x_min) * static_cast<double>((1 << bits) - 1) / span);
    }

    std::atomic<CanPacket8> can_packet_{CanPacket8{uint64_t{0}}};

    static constexpr uint16_t kRawAngleMax = 65535;

    int encoder_zero_point_ = 0;
    int last_raw_angle_ = 0;

    bool multi_turn_angle_enabled_ = false;
    int64_t angle_multi_turn_ = 0;

    double velocity_max_ = 0.0;
    double max_torque_ = 0.0;

    double raw_angle_to_angle_coefficient_ = 0.0;
    double raw_velocity_to_velocity_coefficient_ = 0.0;
    double raw_torque_to_torque_coefficient_ = 0.0;

    double angle_ = 0.0;
    double velocity_ = 0.0;
    double torque_ = 0.0;
    uint8_t state_ = 0;
    double temperature_mos_ = 0.0;
    double temperature_coil_ = 0.0;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;
    rmcs_executor::Component::OutputInterface<uint8_t> state_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_mos_output_;
    rmcs_executor::Component::OutputInterface<double> temperature_coil_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;

    static constexpr std::array<uint8_t, 8> kEnableCommand{
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static constexpr std::array<uint8_t, 8> kCloseCommand{
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    static constexpr std::array<uint8_t, 8> kClearErrorCommand{
        0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
};

} // namespace rmcs_core::hardware::device
