#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>
#include <stdexcept>

#include "hardware/endian_promise.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

enum class DMMotorType : uint8_t { 
    UNKNOWN = 0,
    DM8009=1,
    DM6006=2,
    DM4310=3 };

struct DMMotorConfig {
    DMMotorType motor_type;
    int encoder_zero_point; // 0~65536
    double gear_ratio;
    double reversed;
    bool multi_turn_angle_enabled;
    double iq;

    explicit DMMotorConfig(DMMotorType motor_type) {
        this->encoder_zero_point       = 0;
        this->motor_type               = motor_type;
        this->reversed                 = 1.0;
        this->multi_turn_angle_enabled = false;
        this->gear_ratio               = 1.0;
    }
    DMMotorConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    DMMotorConfig& set_gear_ratio(double value) { return gear_ratio = value, *this; }
    DMMotorConfig& reverse() { return reversed = -1.0, *this; }
    DMMotorConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};

class DMMotor : rclcpp::Node {
public:
    DMMotor(
        Component& status_component, Component& command_component, const std::string& name_prefix)
        : rclcpp::Node("agfdhfhf")

    {
        encoder_zero_point_       = 0;
        last_raw_angle_           = 0;
        multi_turn_angle_enabled_ = false;

        raw_angle_to_angle_coefficient_ = angle_to_raw_angle_coefficient_ = 0.0;
        raw_current_to_torque_coefficient_ = torque_to_raw_current_coefficient_ = 0.0;

        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);

        status_component.register_output(name_prefix + "/motor", motor_, this);

        command_component.register_input(name_prefix + "/control_torque", control_torque_);
        status_component.register_output(name_prefix + "/raw_encoder", raw_encoder_, (uint16_t)0);
    }
    DMMotor(const DMMotor&)            = delete;
    DMMotor& operator=(const DMMotor&) = delete;

    void configure(const DMMotorConfig& config) {

        switch (config.motor_type) {
        case DMMotorType::DM8009:
            VMAX = 25.0;
            TMAX = 40.0;
            break;
        case DMMotorType::DM6006:
            VMAX = 25.0;
            TMAX = 11.0;
            break;
        case DMMotorType::DM4310:
            VMAX = 15.0;
            TMAX = 11.0;
            break;
        default: throw std::runtime_error{"Unknown motor type"};
        }
        reverse     = config.reversed;
        gear_ratio_ = config.gear_ratio;

        encoder_zero_point_ = config.encoder_zero_point % (raw_angle_max_);
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += (raw_angle_max_);

        raw_angle_to_angle_coefficient_ =
            1.0 * reverse * 2 * std::numbers::pi / (raw_angle_max_ * gear_ratio_);
        angle_to_raw_angle_coefficient_ = 1.0 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            1.0 * (reverse * 60.0) / (2 * std::numbers::pi * gear_ratio_);
        velocity_to_raw_velocity_coefficient_ = 1.0 / raw_velocity_to_velocity_coefficient_;

        raw_current_to_torque_coefficient_ = 1.0 * config.gear_ratio;
        torque_to_raw_current_coefficient_ = 1.0 / raw_current_to_torque_coefficient_;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
    }

    void store_status(uint64_t can_result) {

        can_result_.store(can_result, std::memory_order::relaxed);
    }
    void update() {
        uint64_t value = can_result_.load(std::memory_order_relaxed);

        uint8_t rx_buff[8];
        std::memcpy(rx_buff, &value, sizeof(value));
        *id            = (rx_buff[0]) & 0xff;
        *state         = (rx_buff[0]) >> 4;
        uint16_t p_int = (rx_buff[1] << 8) | rx_buff[2];
        *raw_encoder_  = p_int;
        uint16_t v_int = (rx_buff[3] << 4) | (rx_buff[4] >> 4);
        uint16_t t_int = ((rx_buff[4] & 0xF) << 8) | rx_buff[5];
        uint16_t angle = p_int - encoder_zero_point_;
        if (angle < 0)
            angle += raw_angle_max_;
        *angle_    = angle < raw_angle_max_ / 2
                       ? raw_angle_to_angle_coefficient_ * angle
                       : -2 * std::numbers::pi + raw_angle_to_angle_coefficient_ * angle;
        *velocity_ = uint_to_double(v_int, -VMAX, VMAX, 12);
        *torque_   = uint_to_double(t_int, -TMAX, TMAX, 12);

        *T_mos  = (double)(rx_buff[6]);
        *T_coil = (double)(rx_buff[7]);
    }

    uint64_t generate_torque_command() {
        uint64_t result;
        double torque = *control_torque_;
        if (std::isnan(torque)) {
            torque = 0.0;
        }
        torque           = std::clamp(torque, -TMAX, TMAX);
        uint16_t tor_tmp = double_to_uint(torque, -TMAX, TMAX, 12);
        uint8_t tx_buff[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        tx_buff[6]         = ((0 & 0xf) << 4) | (tor_tmp >> 8);
        tx_buff[7]         = tor_tmp;
        std::copy(tx_buff, tx_buff + 8, reinterpret_cast<uint8_t*>(&result));
        return result;
    }

    static uint64_t dm_enable_command() { return std::bit_cast<uint64_t>(DM_ENABLE); }
    static uint64_t dm_close_command() { return std::bit_cast<uint64_t>(DM_CLOSE); }
    static uint64_t dm_clear_error_command() { return std::bit_cast<uint64_t>(DM_CLEAR_ERROR); }

    double get_angle() { return *angle_; }
    double get_velocity() { return *velocity_; }
    double get_torque() { return *torque_; }
    double get_state() { return *state; }
    double get_T_coil() { return *T_coil; }
    uint16_t get_raw_encoder() { return *raw_encoder_; }

private:
    std::atomic<uint64_t> can_result_ = 0;

    static double uint_to_double(int x_int, double x_min, double x_max, int bits) {
        double span   = x_max - x_min;
        double offset = x_min;
        return ((double)x_int) * span / ((double)((1 << bits) - 1)) + offset;
    }
    static int double_to_uint(double x_double, double x_min, double x_max, int bits) {
        double span   = x_max - x_min;
        double offset = x_min;
        return (int)((x_double - offset) * ((double)((1 << bits) - 1)) / span);
    }

    int last_raw_angle_;
    Component::OutputInterface<uint16_t> raw_encoder_;

    double reverse     = 1.0;
    double gear_ratio_ = 1.0;

    int encoder_zero_point_;
    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    double VMAX, TMAX;
    static constexpr uint8_t DM_ENABLE[8]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    static constexpr uint8_t DM_CLOSE[8]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    static constexpr uint8_t DM_CLEAR_ERROR[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

    static constexpr uint16_t raw_angle_max_ = 65535;
    Component::OutputInterface<uint8_t> id;
    Component::OutputInterface<uint8_t> state;
    Component::OutputInterface<double> angle_;
    Component::OutputInterface<double> velocity_;
    Component::OutputInterface<double> torque_;
    Component::OutputInterface<double> T_mos;
    Component::OutputInterface<double> T_coil;

    Component::OutputInterface<DMMotor*> motor_;

    Component::InputInterface<double> control_torque_;
};

} // namespace rmcs_core::hardware::device