#pragma once

#include <atomic>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <bitset>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/endian_promise.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
struct EncoderConfig {
    
    int encoder_zero_point;
    double reversed;
    bool multi_turn_angle_enabled;

    explicit EncoderConfig() {
        this->encoder_zero_point = 0;
        this->reversed                 = 1.0;
        this->multi_turn_angle_enabled = false;
        
    }
    EncoderConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    EncoderConfig& reverse() { return reversed = -1.0, *this; }
    EncoderConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};
class Encoder:rclcpp::Node
{
public:
    Encoder(Component& status_component, Component& command_component, const std::string& name_prefix)
    :Node{"aaa"}
    {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/motor", encoder_, this);
    }

    Encoder(const Encoder&)            = delete;
    Encoder& operator=(const Encoder&) = delete;


    void configure(const EncoderConfig& config){
        encoder_zero_point_ = config.encoder_zero_point % raw_angle_max_;
        reverse = config.reversed;
        raw_angle_to_angle_coefficient_ =
            (1.0/ (raw_angle_max_))* 2.0 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1.0 / raw_angle_to_angle_coefficient_;
        
        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
    }
    void store_status(uint64_t can_result) { 
        raw_angle = std::bitset<18>(can_result);
    }
    void update() {
        int raw_angle_ = static_cast<int>(raw_angle.to_ulong());
        
        int angle = raw_angle_ - encoder_zero_point_;
        if(angle < 0) angle+=raw_angle_max_;
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
            last_raw_angle_ = raw_angle_;
        }
    }
    double get_angle() { return *angle_; }
    double get_raw_angle() { return static_cast<int>(raw_angle.to_ulong());}

private:

    std::bitset<18> raw_angle;
    int64_t angle_multi_turn_;
    Component::OutputInterface<double> angle_;
    Component::OutputInterface<Encoder*> encoder_;

    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    int encoder_zero_point_;
    static constexpr int raw_angle_max_ = 262144;
    int last_raw_angle_;
    double reverse;
    Component::OutputInterface<double> gear_ratio_;
    bool multi_turn_angle_enabled_;};


  
}