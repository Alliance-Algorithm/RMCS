#pragma once

#include "rmcs_msgs/wheel_leg_mode.hpp"
#include "rmcs_msgs/wheel_leg_state.hpp"
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/eigen_structured_bindings.hpp>

namespace rmcs_core::controller {

class DesireStateSolver {
public:
    explicit DesireStateSolver(double desire_leg_length)
        : initial_leg_length_(desire_leg_length) {
        reset();
    }

    void reset() {
        desire_leg_length_ = initial_leg_length_;
        desire_roll_angle_ = 0.0;
    }

    rmcs_msgs::WheelLegState update(Eigen::Vector3d control_velocity) {
        // x-axis translational velocity, z-axis vertical velocity, z-axis angular velocity
        auto& [vx, vz, wz] = control_velocity;

        // desire_leg_length_ += vz * leg_length_sensitivity_;

        constexpr double power_kp_x = 0.26;
        // power_limit_velocity_x = power_kp_x * std::sqrt(referee_->chassis_power_limit_);

        constexpr double power_kp_w = 0.40;
        // power_limit_velocity_w = power_kp_w * std::sqrt(referee_->chassis_power_limit_);

        // switch (chassis_mode_) {
        // case rmcs_msgs::WheelLegMode::STOP: break;
        // case rmcs_msgs::WheelLegMode::SPIN: break;
        // case rmcs_msgs::WheelLegMode::FOLLOW: break;
        // case rmcs_msgs::WheelLegMode::LAUNCH_RAMP: break;
        // case rmcs_msgs::WheelLegMode::BALANCELESS: break;
        // }

        if (std::abs(measure_state_.velocity) < 1e-2) {
            park_mode_ = true;
        } else {
            park_mode_ = false;
        }

        // distance :always 0 during velocity control
        desire_state_.distance = 0.0; // else measure.distance
        desire_state_.velocity = 0.0;

        // yaw angle:
        desire_state_.yaw_angle = 0.0; // else measure.yaw
        desire_state_.yaw_velocity = 0.0;

        // leg tilt angle:
        desire_state_.left_tilt_angle = 0.0;
        desire_state_.right_tilt_angle = 0.0;

        desire_state_.body_pitch_angle = 0.0;

        return desire_state_;
    }

    double update_desire_leg_length(const double min, const double max) {
        desire_leg_length_ = std::clamp(desire_leg_length_, min, max);

        return desire_leg_length_;
    }

    double update_desire_roll_angle() const { return desire_roll_angle_; }

    void update_measure_state(rmcs_msgs::WheelLegState& measure_state) {
        measure_state_ = measure_state;
    }

    void update_power() {}

private:
    constexpr static double leg_length_sensitivity_ = 0.00001;

    constexpr static double dt_ = 0.001;

    // Maximum control velocities
    static constexpr double translational_velocity_max_ = 2.0;
    static constexpr double angular_velocity_max_ = 2.0;

    const double initial_leg_length_;

    bool park_mode_ = false;

    double desire_leg_length_;
    double desire_roll_angle_;

    rmcs_msgs::WheelLegState measure_state_;
    rmcs_msgs::WheelLegState desire_state_;
};
} // namespace rmcs_core::controller