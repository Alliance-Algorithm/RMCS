#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

class DeformableChassisJointManager {
public:
    static constexpr size_t kJointCount = 4;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    DeformableChassisJointManager() = default;

    void configure(rclcpp::Node& node) {
        target_vel_limit_ = std::max(
            deg_to_rad_(std::abs(
                node.get_parameter_or("target_physical_velocity_limit", 180.0))),
            1e-6);
        target_acc_limit_ = std::max(
            deg_to_rad_(std::abs(
                node.get_parameter_or("target_physical_acceleration_limit", 720.0))),
            1e-6);
        suspension_vel_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "active_suspension_target_velocity_limit_deg",
                node.get_parameter_or("target_physical_velocity_limit", 180.0)))),
            1e-6);
        suspension_acc_limit_ = std::max(
            deg_to_rad_(std::abs(node.get_parameter_or(
                "active_suspension_target_acceleration_limit_deg",
                node.get_parameter_or("target_physical_acceleration_limit", 720.0)))),
            1e-6);
    }

    void reset() {
        joint_target_active_.fill(false);
        target_angle_state_rad_.fill(nan_);
        target_velocity_state_rad_.fill(0.0);
        target_acceleration_state_rad_.fill(0.0);
    }

    bool init_from_feedback(
        const std::array<double, kJointCount>& physical_angles) {

        bool any_active = false;
        for (size_t i = 0; i < kJointCount; ++i) {
            if (std::isfinite(physical_angles[i]) && !joint_target_active_[i]) {
                target_angle_state_rad_[i]        = physical_angles[i];
                target_velocity_state_rad_[i]     = 0.0;
                target_acceleration_state_rad_[i] = 0.0;
                joint_target_active_[i]            = true;
            }
            any_active = any_active || joint_target_active_[i];
        }
        return any_active;
    }

    bool any_active() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_target_active_[i])
                return true;
        return false;
    }

    void run_trajectory(
        const std::array<double, kJointCount>& target_angles_rad,
        bool suspension_active,
        double dt) {

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i])
                continue;

            double& angle_st = target_angle_state_rad_[i];
            double& vel_st   = target_velocity_state_rad_[i];
            double& acc_st   = target_acceleration_state_rad_[i];
            double target    = target_angles_rad[i];

            double vel_limit = suspension_active
                                   ? suspension_vel_limit_
                                   : target_vel_limit_;
            double acc_limit = suspension_active
                                   ? suspension_acc_limit_
                                   : target_acc_limit_;

            if (!std::isfinite(target) || !std::isfinite(angle_st))
                continue;

            double position_error    = target - angle_st;
            double stopping_distance =
                vel_st * vel_st / (2.0 * acc_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6
                && std::abs(position_error) > stopping_distance) {
                desired_velocity =
                    std::copysign(vel_limit, position_error);
            }

            double velocity_error = desired_velocity - vel_st;
            acc_st = std::clamp(
                velocity_error / dt, -acc_limit, acc_limit);

            vel_st += acc_st * dt;
            vel_st = std::clamp(vel_st, -vel_limit, vel_limit);
            angle_st += vel_st * dt;

            double next_error = target - angle_st;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5
                    && std::abs(vel_st) < 1e-3)) {
                angle_st = target;
                vel_st   = 0.0;
                acc_st   = 0.0;
            }
        }
    }

    double angle_state(size_t i) const {
        return target_angle_state_rad_[i];
    }
    double velocity_state(size_t i) const {
        return target_velocity_state_rad_[i];
    }
    double acceleration_state(size_t i) const {
        return target_acceleration_state_rad_[i];
    }
    bool joint_active(size_t i) const { return joint_target_active_[i]; }

    std::array<double, kJointCount> angle_states() const {
        return target_angle_state_rad_;
    }

private:
    static double deg_to_rad_(double deg) {
        return deg * std::numbers::pi / 180.0;
    }

    std::array<bool, kJointCount> joint_target_active_ = {
        false, false, false, false};
    std::array<double, kJointCount> target_angle_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> target_velocity_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> target_acceleration_state_rad_ = {
        0.0, 0.0, 0.0, 0.0};

    double target_vel_limit_      = 0.0;
    double target_acc_limit_      = 0.0;
    double suspension_vel_limit_  = 0.0;
    double suspension_acc_limit_  = 0.0;
};

} // namespace rmcs_core::controller::chassis
