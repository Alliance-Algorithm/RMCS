#pragma once

#include <bit>
#include <cmath>
#include <cstring>
#include <numbers>
#include <stdexcept>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/endian_promise.hpp"

namespace rmcs_core::hardware::device {

enum class DjiMotorType : uint8_t {
    UNKNOWN        = 0,
    GM6020         = 1,
    GM6020_VOLTAGE = 2,
    M3508          = 3,
    M2006          = 4
};

struct DjiMotorConfig {
    DjiMotorType motor_type;
    int encoder_zero_point;
    double reduction_ratio;
    bool reversed;
    bool multi_turn_angle_enabled;

    explicit DjiMotorConfig(DjiMotorType motor_type) {
        this->encoder_zero_point = 0.0;
        this->motor_type         = motor_type;
        switch (motor_type) {
        case DjiMotorType::UNKNOWN:
        case DjiMotorType::GM6020:
        case DjiMotorType::GM6020_VOLTAGE: reduction_ratio = 1.0; break;
        case DjiMotorType::M3508: reduction_ratio = 3591.0 / 187.0; break;
        case DjiMotorType::M2006: reduction_ratio = 36.0; break;
        }
        this->reversed                 = false;
        this->multi_turn_angle_enabled = false;
    }

    DjiMotorConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    DjiMotorConfig& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
    DjiMotorConfig& reverse() { return reversed = true, *this; }
    DjiMotorConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};

class DjiMotor {
public:
    DjiMotor(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix) {
        encoder_zero_point_       = 0;
        last_raw_angle_           = 0;
        multi_turn_angle_enabled_ = false;

        raw_angle_to_angle_coefficient_ = angle_to_raw_angle_coefficient_ = 0.0;
        raw_current_to_torque_coefficient_ = torque_to_raw_current_coefficient_ = 0.0;

        status_component.register_output(name_prefix + "/reduction_ratio", reduction_ratio_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);

        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);

        status_component.register_output(name_prefix + "/motor", motor_, this);

        command_component.register_input(name_prefix + "/control_torque", control_torque_);
    }
    DjiMotor(const DjiMotor&)            = delete;
    DjiMotor& operator=(const DjiMotor&) = delete;

    void configure(const DjiMotorConfig& config) {
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
        case DjiMotorType::GM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max     = 3.0;
            break;
        case DjiMotorType::GM6020_VOLTAGE:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max     = 3.0;
            break;
        case DjiMotorType::M3508:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max     = 20.0;
            break;
        case DjiMotorType::M2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max     = 10.0;
            break;
        default: throw std::runtime_error{"Unknown motor type"};
        }

        raw_current_to_torque_coefficient_ =
            sign * config.reduction_ratio * torque_constant / raw_current_max * current_max;
        torque_to_raw_current_coefficient_ = 1 / raw_current_to_torque_coefficient_;

        *reduction_ratio_ = config.reduction_ratio;
        *max_torque_      = 1 * config.reduction_ratio * torque_constant * current_max;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
    }

    void read_status(uint64_t can_data) {
        auto feedback = std::bit_cast<DjiMotorFeedback>(can_data);
        int raw_angle = feedback.angle;

        // Angle unit: rad
        int angle = raw_angle - encoder_zero_point_;
        if (angle < 0)
            angle += raw_angle_max_;
        if (!multi_turn_angle_enabled_) {
            *angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle);
        } else {
            auto diff = (angle - angle_multi_turn_) % raw_angle_max_;
            if (diff <= -raw_angle_max_ / 2)
                diff += raw_angle_max_;
            else if (diff > raw_angle_max_ / 2)
                diff -= raw_angle_max_;
            angle_multi_turn_ += diff;
            *angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }

        // Velocity unit: rad/s
        *velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        *torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        last_raw_angle_ = raw_angle;
    }

    uint16_t generate_command() const {
        double torque = *control_torque_;
        if (std::isnan(torque)) {
            return 0;
        }

        be_int16_t control_current;

        double max_torque = (*motor_)->get_max_torque();
        torque            = std::clamp(torque, -max_torque, max_torque);

        double current  = std::round((*motor_)->torque_to_raw_current_coefficient_ * torque);
        control_current = static_cast<int16_t>(current);

        return std::bit_cast<uint16_t>(control_current);
    }

    int calibrate_zero_point() {
        angle_multi_turn_   = 0;
        encoder_zero_point_ = last_raw_angle_;
        return encoder_zero_point_;
    }

    double get_angle() { return *angle_; }
    double get_velocity() { return *velocity_; }
    double get_torque() { return *torque_; }
    double get_max_torque() { return *max_torque_; }

private:
    struct alignas(uint64_t) DjiMotorFeedback {
        be_int16_t angle;
        be_int16_t velocity;
        be_int16_t current;
        uint8_t temperature;
        uint8_t unused;
    };

    struct alignas(uint64_t) DjiMotorCommand {
        be_int16_t control_current[4];
    };

    static constexpr int raw_angle_max_ = 8192;
    int encoder_zero_point_, last_raw_angle_;

    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    rmcs_executor::Component::OutputInterface<double> reduction_ratio_;
    rmcs_executor::Component::OutputInterface<double> max_torque_;

    rmcs_executor::Component::OutputInterface<double> angle_;
    rmcs_executor::Component::OutputInterface<double> velocity_;
    rmcs_executor::Component::OutputInterface<double> torque_;

    rmcs_executor::Component::OutputInterface<DjiMotor*> motor_;

    rmcs_executor::Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device