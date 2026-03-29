#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware::device {

class PWMServo {
public:
    PWMServo(
        const std::string& name, rmcs_executor::Component& command_component, double pwm_cycle_time,
        double min_position_high_time, double max_position_high_time)
        : duty_offset_(
              (min_position_high_time / pwm_cycle_time) * std::numeric_limits<uint16_t>::max())
        , duty_scale_(
              ((max_position_high_time - min_position_high_time) / pwm_cycle_time)
              * std::numeric_limits<uint16_t>::max()) {
        command_component.register_input(name + "/control_angle", value_);
    }
    constexpr uint16_t generate_duty_cycle() const {
        double value = std::clamp(*value_, 0.0, 1.0);
        double duty = duty_offset_ + (value * duty_scale_);
        return static_cast<uint16_t>(std::round(duty));
    }

private:
    const double duty_offset_;
    const double duty_scale_;

    rmcs_executor::Component::InputInterface<double> value_;
};

} // namespace rmcs_core::hardware::device