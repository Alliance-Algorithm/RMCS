#pragma once

#include "rmcs_msgs/wheel_leg_state.hpp"
#include <rmcs_utility/eigen_structured_bindings.hpp>

namespace rmcs_core::controller {

class DesireStateSolver {
public:
    explicit DesireStateSolver(double desire_leg_length)
        : initial_leg_length_(desire_leg_length) {
        reset();
    }

    void reset() {
        desire_state_ = reset_state();

        desire_leg_length_ = initial_leg_length_;
        desire_roll_angle_ = 0.0;

        jump_state_ = JumpState::IDLE;
    }

    rmcs_msgs::WheelLegState update(Eigen::Vector3d control_velocity) {
        // desire_leg_length_ += vz * leg_length_sensitivity_;

        if (std::abs(measure_state_.velocity) < 1e-2) {
            park_mode_ = true;
        } else {
            park_mode_ = false;
        }

        // distance :always 0 during velocity control
        desire_state_.distance = 0.0;
        desire_state_.velocity = 0.0;

        // yaw angle:
        desire_state_.yaw_angle = 0.0; // else measure.yaw
        desire_state_.yaw_velocity = 0.0;

        // leg tilt angle:
        desire_state_.left_tilt_angle = 0.0;
        desire_state_.left_tilt_velocity = 0.0;

        desire_state_.right_tilt_angle = 0.0;
        desire_state_.right_tilt_velocity = 0.0;

        desire_state_.body_pitch_angle = 0.0;
        desire_state_.body_pitch_velocity = 0.0;

        return desire_state_;
    }

    void update_measure_state(rmcs_msgs::WheelLegState& measure_state) {
        measure_state_ = measure_state;
    }

    double
        update_desire_leg_length(const bool is_leg_extended, const double min, const double max) {
        if (is_leg_extended) {
            desire_leg_length_ = 0.25;
        } else {
            desire_leg_length_ = 0.16;
        }
        desire_leg_length_ = std::clamp(desire_leg_length_, min, max);

        return desire_leg_length_;
    }

    double update_desire_roll_angle() const { return desire_roll_angle_; }

    void update_jump_state(bool jump_active, double leg_length) {
        switch (jump_state_) {
        case JumpState::IDLE: {
            if (jump_active) {
                jump_state_ = JumpState::EXTEND_LEGS;
            }
            break;
        }
        case JumpState::EXTEND_LEGS: {
            desire_leg_length_ = 0.35;
            if (leg_length >= 0.33) {
                jump_state_ = JumpState::CONTRACT_LEGS;
            }
            break;
        }
        case JumpState::CONTRACT_LEGS: {
            desire_leg_length_ = 0.14;
            if (leg_length <= 0.15) {
                jump_state_ = JumpState::LEVITATE;
            }
            break;
        }
        case JumpState::LEVITATE: {
            desire_leg_length_ = 0.17;
            if (leg_length <= 0.18) {
                jump_state_ = JumpState::LAND;
            }
            break;
        }
        case JumpState::LAND: {
            jump_state_ = JumpState::IDLE;
            break;
        }
        }
    }

    void update_climb_state(bool climb_active, double leg_length) {}

    double desire_leg_length() const { return desire_leg_length_; }
    double desire_roll_angle() const { return desire_roll_angle_; }

private:
    enum class JumpState {
        IDLE = 0,
        EXTEND_LEGS = 1,
        CONTRACT_LEGS = 2,
        LEVITATE = 3,
        LAND = 4,
    };

    enum class ClimbState {
        IDLE = 0,
        DETECT = 1,
        CONTRACT_LEGS = 2,
        EXTEND_LEGS = 3,
        LAND = 4,
    };

    void update_desire_state(Eigen::Vector3d control_velocity, double control_angle) {
        // Update desire state based on control velocity and measure state

        // x-axis translational velocity, z-axis vertical velocity, z-axis angular velocity
        auto& [vx, vz, wz] = control_velocity;

        // distance :always 0 during velocity control
        desire_state_.distance = 0.0;
        desire_state_.velocity = vx;

        desire_state_.yaw_angle = 0.0; // or
        desire_state_.yaw_velocity = wz;

        desire_state_.left_tilt_angle = 0.0;
        desire_state_.left_tilt_velocity = 0.0;
        desire_state_.right_tilt_angle = 0.0;
        desire_state_.right_tilt_velocity = 0.0;
        desire_state_.body_pitch_angle = 0.0;
        desire_state_.body_pitch_velocity = 0.0;
    }

    constexpr static rmcs_msgs::WheelLegState reset_state() {
        return rmcs_msgs::WheelLegState{nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_, nan_};
    }

    constexpr static double nan_ = std::numeric_limits<double>::quiet_NaN();

    constexpr static double leg_length_sensitivity_ = 0.00001;

    constexpr static double dt_ = 0.001;

    // Maximum control velocities
    static constexpr double translational_velocity_max_ = 2.0;
    static constexpr double angular_velocity_max_ = 2.0;

    const double initial_leg_length_;

    bool park_mode_ = false;

    double desire_leg_length_;
    double desire_roll_angle_;

    JumpState jump_state_ = JumpState::IDLE;
    // ClimbState climb_state_ = ClimbState::IDLE;

    rmcs_msgs::WheelLegState measure_state_;
    rmcs_msgs::WheelLegState desire_state_;
};
} // namespace rmcs_core::controller