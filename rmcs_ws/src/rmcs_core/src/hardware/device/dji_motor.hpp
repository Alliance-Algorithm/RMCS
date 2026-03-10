#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <span>
#include <string>
#include <utility>

#include <rmcs_executor/component.hpp>
#include <rmcs_utility/endian_promise.hpp>

#include "hardware/device/can_packet.hpp"

namespace rmcs_core::hardware::device {

class DjiMotor {
public:
    enum class Type : uint8_t { kGM6020, kGM6020Voltage, kM3508, kM2006 };

    struct Config {
        explicit Config(Type motor_type)
            : motor_type(motor_type) {
            switch (motor_type) {
            case Type::kGM6020:
            case Type::kGM6020Voltage: reduction_ratio = 1.0; break;
            case Type::kM3508: reduction_ratio = 3591.0 / 187.0; break;
            case Type::kM2006: reduction_ratio = 36.0; break;
            }
            this->reversed = false;
            this->multi_turn_angle_enabled = false;
        }

        Config& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
        Config& set_reduction_ratio(double value) { return reduction_ratio = value, *this; }
        Config& set_reversed() { return reversed = true, *this; }
        Config& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }

        Type motor_type;
        int encoder_zero_point = 0;
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
    DjiMotor(DjiMotor&&) = delete;
    DjiMotor& operator=(DjiMotor&&) = delete;

    ~DjiMotor() = default;

    void configure(const Config& config) {
        encoder_zero_point_ = config.encoder_zero_point % kRawAngleMax;
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += kRawAngleMax;

        const double sign = config.reversed ? -1 : 1;

        raw_angle_to_angle_coefficient_ =
            sign / config.reduction_ratio / kRawAngleMax * 2 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            sign / config.reduction_ratio / 60 * 2 * std::numbers::pi;
        velocity_to_raw_velocity_coefficient_ = 1 / raw_velocity_to_velocity_coefficient_;

        double torque_constant, raw_current_max, current_max;
        switch (config.motor_type) {
        case Type::kGM6020:
            torque_constant = 0.741;
            raw_current_max = 16384.0;
            current_max = 3.0;
            break;
        case Type::kGM6020Voltage:
            torque_constant = 0.741;
            raw_current_max = 25000.0;
            current_max = 3.0;
            break;
        case Type::kM3508:
            torque_constant = 0.3 * 187.0 / 3591.0;
            raw_current_max = 16384.0;
            current_max = 20.0;
            break;
        case Type::kM2006:
            torque_constant = 0.18 * 1.0 / 36.0;
            raw_current_max = 16384.0;
            current_max = 10.0;
            break;
        default: std::unreachable();
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

    void store_status(std::span<const std::byte> can_data) {
        can_data_.store(CanPacket8{can_data}, std::memory_order_relaxed);
    }

    void update_status() {
        const auto feedback =
            std::bit_cast<DjiMotorFeedback>(can_data_.load(std::memory_order::relaxed));

        // Temperature unit: celsius
        temperature_ = static_cast<double>(feedback.temperature);

        // Angle unit: rad
        const int raw_angle = feedback.angle;
        int calibrated_raw_angle = raw_angle - encoder_zero_point_;
        if (calibrated_raw_angle < 0)
            calibrated_raw_angle += kRawAngleMax;
        if (!multi_turn_angle_enabled_) {
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(calibrated_raw_angle);
            if (angle_ < 0)
                angle_ += 2 * std::numbers::pi;
        } else {
            auto diff = (calibrated_raw_angle - angle_multi_turn_) % kRawAngleMax;
            if (diff <= -kRawAngleMax / 2)
                diff += kRawAngleMax;
            else if (diff > kRawAngleMax / 2)
                diff -= kRawAngleMax;
            angle_multi_turn_ += diff;
            angle_ = raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }
        last_raw_angle_ = raw_angle;

        // Velocity unit: rad/s
        velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        *angle_output_ = angle();
        *velocity_output_ = velocity();
        *torque_output_ = torque();
    }

    double control_torque() const {
        if (control_torque_.ready()) [[likely]]
            return *control_torque_;
        else
            return 0.0;
    }

    CanPacket8::Quarter generate_command() const { return generate_command(control_torque()); }

    CanPacket8::Quarter generate_command(double control_torque) const {
        if (std::isnan(control_torque)) {
            return CanPacket8::Quarter{0};
        }

        control_torque = std::clamp(control_torque, -max_torque_, max_torque_);
        const double current = std::round(torque_to_raw_current_coefficient_ * control_torque);
        const rmcs_utility::be_int16_t control_current = static_cast<int16_t>(current);

        return std::bit_cast<CanPacket8::Quarter>(control_current);
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
    struct alignas(uint64_t) DjiMotorFeedback {
        rmcs_utility::be_int16_t angle;
        rmcs_utility::be_int16_t velocity;
        rmcs_utility::be_int16_t current;
        uint8_t temperature;
        uint8_t unused;
    };

    std::atomic<CanPacket8> can_data_;

    static constexpr int kRawAngleMax = 8192;
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