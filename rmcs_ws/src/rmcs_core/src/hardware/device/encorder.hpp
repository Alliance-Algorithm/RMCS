#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <rclcpp/logger.hpp>
#include <rmcs_executor/component.hpp>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_core::hardware::device {
using rmcs_executor::Component;
enum class EncoderType : uint8_t {
    UNKNOWN = 0,
    Old_    = 1,
    KTH7823 = 2,
};
struct EncoderConfig {
    EncoderType encoder_type;
    int encoder_zero_point;
    double reversed;
    bool multi_turn_angle_enabled;
    int raw_angle_max_;
    explicit EncoderConfig(EncoderType encoder_type) {
        this->encoder_type             = encoder_type;
        this->encoder_zero_point       = 0;
        this->reversed                 = 1.0;
        this->multi_turn_angle_enabled = false;
        switch (encoder_type) {
        case EncoderType::Old_: {
            raw_angle_max_ = 262144;
            break;
        }
        case EncoderType::KTH7823: {
            raw_angle_max_ = 65536;
            break;
        }
        default: {
            throw std::runtime_error{"Unknown motor type"};
            break;
        }
        }
    }
    EncoderConfig& set_encoder_zero_point(int value) { return encoder_zero_point = value, *this; }
    EncoderConfig& reverse() { return reversed = -1.0, *this; }
    EncoderConfig& enable_multi_turn_angle() { return multi_turn_angle_enabled = true, *this; }
};
class Encoder //: public rclcpp::Node
{
public:
    Encoder(Component& status_component, const std::string& name_prefix) {
        status_component.register_output(name_prefix + "/angle", angle_, 0.0);
        status_component.register_output(name_prefix + "/encoder", encoder_, this);
    }

    Encoder(const Encoder&)            = delete;
    Encoder& operator=(const Encoder&) = delete;

    void configure(const EncoderConfig& config) {
        raw_angle_max                   = config.raw_angle_max_;
        type_                           = config.encoder_type;
        encoder_zero_point_             = config.encoder_zero_point % raw_angle_max;
        reverse                         = config.reversed;
        raw_angle_to_angle_coefficient_ = (1.0 / (raw_angle_max)) * 2.0 * std::numbers::pi;
        angle_to_raw_angle_coefficient_ = 1.0 / raw_angle_to_angle_coefficient_;

        multi_turn_angle_enabled_ = config.multi_turn_angle_enabled;
        angle_multi_turn_         = 0;
    }
    void store_status(uint64_t can_result) {
        if (type_ == EncoderType::Old_) {
            raw_angle = static_cast<uint32_t>((can_result) & 0x3FFFF);
        } else if (type_ == EncoderType::KTH7823) {
            raw_angle = static_cast<uint32_t>(can_result & 0xFFFFUL);
        }
    }
    void update() {
        int32_t relative_angle = static_cast<int32_t>(raw_angle) - encoder_zero_point_;
        if (relative_angle < 0) {
            relative_angle += raw_angle_max;
        }

        if (!multi_turn_angle_enabled_) {
            *angle_ = this->reverse
                    * (((double)relative_angle <= (raw_angle_max) / 2.0)
                           ? (raw_angle_to_angle_coefficient_ * relative_angle)
                           : (-2 * std::numbers::pi)
                                 + (raw_angle_to_angle_coefficient_ * relative_angle));
        } else {
            auto diff = (relative_angle - angle_multi_turn_) % (raw_angle_max);
            if (diff <= -(raw_angle_max) / 2)
                diff += (raw_angle_max);
            else if (diff > (raw_angle_max) / 2)
                diff -= (raw_angle_max);
            angle_multi_turn_ += diff;
            *angle_ = this->reverse * raw_angle_to_angle_coefficient_
                    * static_cast<double>(angle_multi_turn_);
            last_raw_angle_ = raw_angle;
        }
    }
    double get_angle() { return *angle_; }
    uint32_t get_raw_angle() const { return raw_angle; }

private:
    EncoderType type_;
    uint32_t raw_angle;
    int64_t angle_multi_turn_;
    Component::OutputInterface<double> angle_;
    Component::OutputInterface<Encoder*> encoder_;
    int raw_angle_max;
    double raw_angle_to_angle_coefficient_, angle_to_raw_angle_coefficient_;
    int encoder_zero_point_;
    uint32_t last_raw_angle_;
    double reverse;
    bool multi_turn_angle_enabled_;
};

} // namespace rmcs_core::hardware::device