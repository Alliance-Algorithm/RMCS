#pragma once

#include <algorithm>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <stdexcept>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/endian_promise.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;

enum class LKMotorType : uint8_t {
    UNKNOWN        = 0,
    MF7015V210T    = 1,
    MG4010E_i10V3  = 2,
    MG4010E_i36V3  = 3,
    MG8010E_i36    = 4
};

struct LKMotorConfig {
    LKMotorType motor_type;
    int encoder_zero_point;//0~65536
    double gear_ratio;
    double reversed;
    bool multi_turn_angle_enabled;
    double iq;

    explicit LKMotorConfig(LKMotorType motor_type) {
        this->encoder_zero_point = 0;
        this->motor_type         = motor_type;
        this->reversed                 = 1.0;
        this->multi_turn_angle_enabled = false;
        this->gear_ratio = 1.0;
        switch (motor_type) {
            case LKMotorType::UNKNOWN:
            case LKMotorType::MG4010E_i10V3:
            case LKMotorType::MG4010E_i36V3:
            case LKMotorType::MG8010E_i36:iq = 66.0/4096;break;
            case LKMotorType::MF7015V210T:iq = 33.0/4096;break;
        }
    }
    LKMotorConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    LKMotorConfig& set_gear_ratio(double value) { return gear_ratio = value, *this; }
    LKMotorConfig& reverse() { return reversed = -1.0, *this; }
    LKMotorConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};

class LKMotor
      {
public:
    LKMotor(Component& status_component, Component& command_component, const std::string& name_prefix)

        {
        encoder_zero_point_       = 0;
        last_raw_angle_           = 0;
        multi_turn_angle_enabled_ = false;

        raw_angle_to_angle_coefficient_ = angle_to_raw_angle_coefficient_ = 0.0;
        raw_current_to_torque_coefficient_ = torque_to_raw_current_coefficient_ = 0.0;

        status_component.register_output(name_prefix + "/gear_ratio", gear_ratio_, 0.0);
        status_component.register_output(name_prefix + "/max_torque", max_torque_, 0.0);
        status_component.register_output(name_prefix + "/rate_torque", rated_torque_, 0.0);

        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/velocity", velocity_, 0.0);
        status_component.register_output(name_prefix + "/torque", torque_, 0.0);
        status_component.register_output(name_prefix + "/raw_angle",raw_angle_,0);

        status_component.register_output(name_prefix + "/motor", motor_, this);

        command_component.register_input(name_prefix + "/control_torque", control_torque_);
    }
    LKMotor(const LKMotor&)            = delete;
    LKMotor& operator=(const LKMotor&) = delete;

    void configure(const LKMotorConfig& config) {
        
        double torque_constant, rated_current,rated_torque,max_torque;
        switch (config.motor_type) {
        case LKMotorType::MF7015V210T:
            torque_constant = 0.12;
            rated_current     = 8.3; 
            rated_torque    = 1.0;
            max_torque      = 2.0;
            LSB             =18000;
            break;
        case LKMotorType::MG4010E_i10V3:
            torque_constant = 0.07 * 10;
            rated_current     = 3.5;
            rated_torque    = 2.5;
            max_torque      = 4.5;
            LSB             =180000;
            break;
        case LKMotorType::MG4010E_i36V3:
            torque_constant = 2.58;
            rated_current     = 3.5;
            rated_torque    = 9;
            max_torque      = 18;
            LSB             =648000;
            break;
        case LKMotorType::MG8010E_i36:
            torque_constant = 0.15 * 36;
            rated_current     = 6.9;
            rated_torque    = 35.0;
            max_torque      = 45.0;
            LSB             =648000;
            break;
        default: throw std::runtime_error{"Unknown motor type"};
        }
        
        encoder_zero_point_ = config.encoder_zero_point % (raw_angle_max_);
        if (encoder_zero_point_ < 0)
            encoder_zero_point_ += (raw_angle_max_);
        

        reverse = config.reversed;

        
        raw_angle_to_angle_coefficient_ =
            1.0*(1.0/ (raw_angle_max_)/config.gear_ratio)* 2.0 * std::numbers::pi;

        angle_to_raw_angle_coefficient_ = 1.0 / raw_angle_to_angle_coefficient_;

        raw_velocity_to_velocity_coefficient_ =
            1.0 / (config.gear_ratio * 6);
        velocity_to_raw_velocity_coefficient_ = 1.0 / raw_velocity_to_velocity_coefficient_;

        
        raw_current_to_torque_coefficient_ = //含有传动比
            1.0 * config.gear_ratio * torque_constant * config.iq;
        torque_to_raw_current_coefficient_ = 1.0 / raw_current_to_torque_coefficient_;

        *gear_ratio_ = config.gear_ratio;

        *max_torque_      = config.gear_ratio * max_torque;
        *rated_torque_    = config.gear_ratio * rated_torque;
        
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
    }

    void store_status(uint64_t can_result) { 
        uint8_t command_byte = static_cast<uint8_t>(can_result);  
        if (command_byte == 0x9C || command_byte == 0xA4 || command_byte == 0xA1 || command_byte == 0xAD) {
            can_result_.store(can_result, std::memory_order::relaxed);
        }
    }
    void update() {
        auto feedback = std::bit_cast<LKMotorFeedback>(can_result_.load(std::memory_order::relaxed));
        int raw_angle = feedback.encoder;
        *raw_angle_ = feedback.encoder;
        int angle = raw_angle - encoder_zero_point_;
        if(angle < 0)angle+=raw_angle_max_;
        if (!multi_turn_angle_enabled_) { 
            *angle_ = this->reverse*(((double)angle <= (raw_angle_max_)/2.0 )? (raw_angle_to_angle_coefficient_ * angle): (-2*std::numbers::pi )+ (raw_angle_to_angle_coefficient_ * angle));
        } else {
            auto diff = (angle - angle_multi_turn_) % (raw_angle_max_);
            if (diff <= -(raw_angle_max_)/ 2)
                diff += (raw_angle_max_);
            else if (diff > (raw_angle_max_)/ 2)
                diff -= (raw_angle_max_);
            angle_multi_turn_ += diff;
            *angle_ = this->reverse * raw_angle_to_angle_coefficient_ * static_cast<double>(angle_multi_turn_);
        }

        // Velocity unit: rpm   
        *velocity_ = raw_velocity_to_velocity_coefficient_ * static_cast<double>(feedback.velocity);

        // Torque unit: N*m
        *torque_ = raw_current_to_torque_coefficient_ * static_cast<double>(feedback.current);

        last_raw_angle_ = raw_angle;
    }
   

    uint64_t generate_torque_command(){
        std::array<uint8_t, 8> result = {0};  
        result[0] = 0xA1; 
        double torque = *control_torque_;
        if (std::isnan(torque)) {
            result[1] = 0X00;
            result[2] = 0X00;
            result[3] = 0X00;
            result[4] = 0X00;
            result[5] = 0X00;
            result[6] = 0X00;
            result[7] = 0X00;

            return std::bit_cast<uint64_t>(result);;
        }
        double max_torque = (*motor_)->get_max_torque();
        torque            = std::clamp(torque, -max_torque, max_torque);
        double current  = std::round((*motor_)->torque_to_raw_current_coefficient_ * torque);
        int16_t control_current = static_cast<int16_t>(current);
        auto control_current_bits = std::bit_cast<std::array<uint8_t, 2>>(control_current);
        std::copy(control_current_bits.begin(),control_current_bits.end(),result.begin()+4);
        return std::bit_cast<uint64_t>(result);
    }
    
    // uint64_t generate_velocity_command(double radians, double max_speed_rpm) {
    //     std::array<uint8_t, 8> result = {0};
    //     radians = this->reverse * radians;
    //     result[0] = 0xAD;   
        
    //     uint16_t max_speed = static_cast<uint16_t>(max_speed_rpm * 6);
    //     auto maxSpeedBytes = std::bit_cast<std::array<uint8_t, 2>>(max_speed);
    //     //std::copy(maxSpeedBytes.begin(), maxSpeedBytes.end(), result.begin() + 2);

    //     double real_radians = radians + this->encoder_zero_point_ * raw_angle_to_angle_coefficient_;
    //     real_radians = real_radians >= 0 ? real_radians : 2 * std::numbers::pi + real_radians;
    //     double degrees = real_radians * this->LSB * (* motor_)->get_gear_ratio() / std::numbers::pi;
    //     uint32_t angle_control = static_cast<uint32_t>(degrees);
    //     auto angleControlBytes = std::bit_cast<std::array<uint8_t, 4>>(angle_control);
    //     std::copy(angleControlBytes.begin(), angleControlBytes.end(), result.begin() + 4);
        
        
    //     return std::bit_cast<uint64_t>(result);
    // }   
    
    static uint64_t lk_stop_command() {
        return std::bit_cast<uint64_t>(uint64_t{0x81});
    }
    static uint64_t lk_quest_command() {
        return std::bit_cast<uint64_t>(uint64_t{0x9C});
    }
    static uint64_t lk_enable_command(){
        return std::bit_cast<uint64_t>(uint64_t{0x88});
    }
    static uint64_t lk_close_command(){
        return std::bit_cast<uint64_t>(uint64_t{0x80});
    }

    double get_angle() { return *angle_; }
    double get_velocity() { return *velocity_; }
    double get_torque() { return *torque_; }
    double get_max_torque() { return *max_torque_; }



private:
    struct alignas(uint64_t) LKMotorFeedback {
        uint8_t command;
        uint8_t temperature;
        int16_t current;
        int16_t velocity;
        uint16_t encoder;
    };

    std::atomic<uint64_t> can_result_ = 0;

    static constexpr uint16_t raw_angle_max_ = 65535;
    int last_raw_angle_;
    uint32_t LSB;
    double reverse;
    int encoder_zero_point_;
    bool multi_turn_angle_enabled_;
    int64_t angle_multi_turn_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    double raw_velocity_to_velocity_coefficient_, velocity_to_raw_velocity_coefficient_;
    double raw_current_to_torque_coefficient_, torque_to_raw_current_coefficient_;

    Component::OutputInterface<double> gear_ratio_;
    Component::OutputInterface<double> max_torque_;
    Component::OutputInterface<double> rated_torque_;

    Component::OutputInterface<int> raw_angle_;
    Component::OutputInterface<double> angle_;
    Component::OutputInterface<double> velocity_;
    Component::OutputInterface<double> torque_;

    Component::OutputInterface<LKMotor*>    motor_;

    Component::InputInterface<double> control_torque_;
    Component::InputInterface<double> control_velocity;


};

} // namespace rmcs_core::hardware::device