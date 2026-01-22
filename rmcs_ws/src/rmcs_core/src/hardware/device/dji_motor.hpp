#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class DjiMotor {
public:
    enum class Type : uint8_t { GM6020, GM6020_VOLTAGE, M3508, M2006 };

    struct Config {
        explicit Config(Type motor_type) {
            this->encoder_zero_point = 0;
            this->motor_type = motor_type;
            switch (motor_type) {
            case Type::GM6020:
            case Type::GM6020_VOLTAGE: reduction_ratio = 1.0; break;
            case Type::M3508: reduction_ratio = 3591.0 / 187.0; break;
            case Type::M2006: reduction_ratio = 36.0; break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point;
        double reduction_ratio;
        bool reversed;
        bool multi_turn_angle_enabled;
    };

    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix)
        : angle_(0.0)
        , velocity_(0.0)
        , torque_(0.0) {
        status_component.register_output(name_prefix + "/angle", angle_output_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_output_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_output_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_output_, 0.0);

        command_component.register_input(name_prefix + "/control_torque", control_torque_, false);
    }

    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix, const Config& config)
        : DjiMotor(status_component, command_component, name_prefix) {
        configure(config);
    }

    DjiMotor(const DjiMotor&) = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;

    void configure(const Config& config) {
        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += raw_angle_max_;

        double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ =
            sign / config.reduction_ratio / raw_angle_max_ * 2 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / config.reduction_ratio / 60 * 2 * std::numbers::pi;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        double torque_constant, raw_current_max, current_max;
        switch (config.motor_type) {
        case Type::GM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max = 3.0;
            break;
        case Type::GM6020_VOLTAGE:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max = 3.0;
            break;
        case Type::M3508:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max = 20.0;
            break;
        case Type::M2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max = 10.0;
            break;
        }

        raw_current_to_torque_coefficient_ =
            sign * config.reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        max_torque_ = 1 * config.reduction_ratio * torque_constant * current_max;

        last_raw_angle_ = 0;
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_ = 0;

        *max_torque_output_ = max_torque();
    }

    void store_status(uint64_t can_data) { can_data_.store(can_data, std::memory_order_relaxed); }

    void update_status() {
        const auto feedback =
            std::bit_cast<DjiMotorFeedback>(can_data_.load(std::memory_order::relaxed));

        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temperature);

        // Angle unit: rad
        const int raw_angle = feedback.angle;
        int calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -raw_angle_max_ / 2)
                diff += raw_angle_max_;
            else if (diff > raw_angle_max_ / 2)
                diff -= raw_angle_max_;
            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        last_raw_angle_ = raw_angle;

        // Velocity unit: rad/s
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        *angle_output_    = angle();
        *velocity_output_ = velocity();
        *torque_output_   = torque();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    uint16_t generate_command() { return generate_command(control_torque()); }

    uint16_t generate_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return 0;
        }

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        double current = std::round(torque_to_raw_current_coefficient_ * control_torque);
        BeInt16 control_current = static_cast<int16_t>(current);

        return std::bit_cast<uint16_t>(control_current);
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
    double temperature() const { return temperature_; }

private:
    struct BeInt16 {
        int16_t value_buffer;

        static constexpr int16_t transform(int16_t value) noexcept {
            if constexpr (std::endian::native == std::endian::big)
                return value;

            uint16_t temp = static_cast<uint16_t>(value);
            temp = static_cast<uint16_t>((temp >> 8) | (temp << 8));
            return static_cast<int16_t>(temp);
        }

        constexpr BeInt16() = default;
        constexpr BeInt16(int16_t value) noexcept
            : value_buffer(transform(value)) {}

        constexpr operator int16_t() const noexcept { return transform(value_buffer); }
    };
    static_assert(sizeof(BeInt16) == sizeof(int16_t));

    struct alignas(uint64_t) DjiMotorFeedback {
        BeInt16 angle;
        BeInt16 velocity;
        BeInt16 current;
        uint8_t temperature;
        uint8_t unused;
    };

    std::atomic<uint64_t> can_data_ = 0;

    static constexpr int raw_angle_max_ = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double angle_;
    double velocity_;
    double torque_;
    double max_torque_;
    double temperature_;

    rmcs_executor::Component::OutputInterface<double> angle_output_;
    rmcs_executor::Component::OutputInterface<double> velocity_output_;
    rmcs_executor::Component::OutputInterface<double> torque_output_;
    rmcs_executor::Component::OutputInterface<double> max_torque_output_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device
