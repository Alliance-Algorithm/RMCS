#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::chassis {

enum class JointFeedbackSource : uint8_t { kLegacyEncoderAngle, kMotorAngle };

enum JointIndex : size_t {
    kLeftFront = 0,
    kLeftBack = 1,
    kRightBack = 2,
    kRightFront = 3,
    kJointCount = 4
};

inline constexpr std::array<const char*, kJointCount> kJointNames{
    "left_front", "left_back", "right_back", "right_front"};

struct JointFeedbackFrame {
    std::array<double, kJointCount> motor_angles{};
    std::array<double, kJointCount> physical_angles{};
    std::array<double, kJointCount> physical_velocities{};
    std::array<double, kJointCount> joint_torques{};
    std::array<double, kJointCount> eso_z2{};
    std::array<double, kJointCount> eso_z3{};
};

struct LegFeedback {
    double motor_angle = std::numeric_limits<double>::quiet_NaN();
    double physical_angle = std::numeric_limits<double>::quiet_NaN();
    double physical_velocity = std::numeric_limits<double>::quiet_NaN();
    double joint_torque = std::numeric_limits<double>::quiet_NaN();
    double eso_z2 = std::numeric_limits<double>::quiet_NaN();
    double eso_z3 = std::numeric_limits<double>::quiet_NaN();
};

struct JointIO {
    using In = rmcs_executor::Component::InputInterface<double>;
    using Out = rmcs_executor::Component::OutputInterface<double>;
    In angle, physical_angle, physical_velocity, torque, encoder_angle, eso_z2, eso_z3;
    Out target_angle, target_physical_angle, target_physical_velocity, target_physical_acceleration;
    Out suspension_torque;
};

struct JointTrajectoryPlanner {
    static constexpr double kJointZeroPhysicalAngleRad = 1.090830782496456;

    void init(double min_angle, double max_angle, double velocity_limit, double acceleration_limit) {
        min_angle_ = min_angle;
        max_angle_ = max_angle;
        velocity_limit_ = velocity_limit;
        acceleration_limit_ = acceleration_limit;
    }

    void set_target_angle(double angle) { current_target_angle_ = angle; }
    double target_angle() const { return current_target_angle_; }

    bool initialize_from_feedback(
        const std::array<double, kJointCount>& motor_angles,
        const std::array<double, kJointCount>& physical_angles) {
        for (size_t i = 0; i < kJointCount; ++i)
            if (!std::isfinite(motor_angles[i]) || !std::isfinite(physical_angles[i]))
                return false;
        target_motor_state_ = motor_angles;
        target_physical_state_ = physical_angles;
        target_velocity_state_.fill(0.0);
        target_acceleration_state_.fill(0.0);
        requested_physical_ = physical_angles;
        current_physical_ = physical_angles;
        active_ = true;
        return true;
    }

    void sync_from_feedback(size_t index, double motor_angle, double physical_angle) {
        target_motor_state_[index] = motor_angle;
        target_physical_state_[index] = physical_angle;
        target_velocity_state_[index] = 0.0;
        target_acceleration_state_[index] = 0.0;
    }

    bool active() const { return active_; }
    void set_active(bool value) { active_ = value; }

    void fill_symmetric_targets() { per_joint_targets_.fill(current_target_angle_); }

    bool symmetric_requested() const {
        for (size_t i = 1; i < kJointCount; ++i)
            if (std::abs(per_joint_targets_[0] - per_joint_targets_[i]) > 1e-6)
                return false;
        return true;
    }

    void refresh_deploy_targets(bool deploy_requested, bool /*prone_override*/, double deploy_angle) {
        for (size_t i = 0; i < kJointCount; ++i)
            requested_physical_[i] = per_joint_targets_[i] * std::numbers::pi / 180.0;
        if (deploy_requested)
            requested_physical_.fill(deploy_angle * std::numbers::pi / 180.0);
        current_physical_ = requested_physical_;
    }

    void update_trajectory(
        double delta_time, bool use_suspension_limits,
        double suspension_velocity_limit, double suspension_acceleration_limit) {
        double velocity_limit = use_suspension_limits ? suspension_velocity_limit : velocity_limit_;
        double acceleration_limit =
            use_suspension_limits ? suspension_acceleration_limit : acceleration_limit_;
        for (size_t i = 0; i < kJointCount; ++i) {
            double target = current_physical_[i];
            double current_position = target_physical_state_[i];
            double current_velocity = target_velocity_state_[i];
            double error = target - current_position;
            double max_velocity = std::sqrt(2.0 * acceleration_limit * std::abs(error));
            double command_velocity = std::copysign(std::min(max_velocity, velocity_limit), error);
            double delta_velocity = command_velocity - current_velocity;
            double command_acceleration = std::clamp(delta_velocity / delta_time, -acceleration_limit, acceleration_limit);
            target_acceleration_state_[i] = command_acceleration;
            target_velocity_state_[i] =
                std::clamp(current_velocity + command_acceleration * delta_time, -velocity_limit, velocity_limit);
            target_physical_state_[i] += target_velocity_state_[i] * delta_time;
            target_motor_state_[i] = kJointZeroPhysicalAngleRad - target_physical_state_[i];
        }
    }

    const std::array<double, kJointCount>& target_angles() const { return target_motor_state_; }
    const std::array<double, kJointCount>& target_physical_angles() const {
        return target_physical_state_;
    }
    const std::array<double, kJointCount>& target_velocities() const {
        return target_velocity_state_;
    }
    const std::array<double, kJointCount>& target_accelerations() const {
        return target_acceleration_state_;
    }
    const std::array<double, kJointCount>& current_physical() const { return current_physical_; }

    void reset(double angle) {
        current_target_angle_ = angle;
        per_joint_targets_.fill(angle);
        active_ = false;
        target_motor_state_.fill(0.0);
        target_physical_state_.fill(0.0);
        target_velocity_state_.fill(0.0);
        target_acceleration_state_.fill(0.0);
    }

    double min_angle() const { return min_angle_; }
    double max_angle() const { return max_angle_; }

private:
    double min_angle_ = 15.0;
    double max_angle_ = 55.0;
    double velocity_limit_ = 1.0;
    double acceleration_limit_ = 1.0;
    double current_target_angle_ = 55.0;
    std::array<double, kJointCount> per_joint_targets_{55.0, 55.0, 55.0, 55.0};
    bool active_ = false;
    std::array<double, kJointCount> target_motor_state_{};
    std::array<double, kJointCount> target_physical_state_{};
    std::array<double, kJointCount> target_velocity_state_{};
    std::array<double, kJointCount> target_acceleration_state_{};
    std::array<double, kJointCount> requested_physical_{};
    std::array<double, kJointCount> current_physical_{};
};

} // namespace rmcs_core::controller::chassis
