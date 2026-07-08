#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <fmt/format.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

#include "deformable_chassis_joint_manager.hpp"
#include "deformable_chassis_mode_manager.hpp"
#include "deformable_chassis_suspension.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , spin_ratio_(std::clamp(get_parameter_or("spin_ratio", 0.6), 0.0, 1.0))
        , joint_mode_mgr_(*this) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        suspension_.configure(*this);
        joint_mgr_.configure(*this);

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);

        for (size_t i = 0; i < kJointCount; ++i) {
            register_input(
                fmt::format("/chassis/{}_joint/physical_angle", kJointName[i]),
                joint_physical_angle_[i], false);
            register_output(
                fmt::format("/chassis/{}_joint/target_physical_angle", kJointName[i]),
                joint_target_angle_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/target_physical_velocity", kJointName[i]),
                joint_target_velocity_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/target_physical_acceleration", kJointName[i]),
                joint_target_acceleration_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/control_angle_error", kJointName[i]),
                joint_angle_error_[i], nan_);
        }

        register_output("/gimbal/scope/control_torque", scope_motor_control_torque, nan_);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);
        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/pitch_lock_active", pitch_lock_active_, false);
        register_output("/chassis/active_suspension/active", active_suspension_active_, false);

        *mode_             = rmcs_msgs::ChassisMode::AUTO;
        *pitch_lock_active_ = false;
        chassis_control_velocity_->vector << nan_, nan_, nan_;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". "
                              "Set to 0.0.");
        }
        if (!chassis_imu_pitch_.ready())
            chassis_imu_pitch_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_.ready())
            chassis_imu_roll_.make_and_bind_directly(0.0);
        if (!chassis_imu_pitch_rate_.ready())
            chassis_imu_pitch_rate_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_rate_.ready())
            chassis_imu_roll_rate_.make_and_bind_directly(0.0);
        validate_joint_feedback_inputs();
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;
        const auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            double rotary_knob = rotary_knob_.ready() ? *rotary_knob_ : 0.0;

            joint_mode_mgr_.update(switch_left, switch_right, keyboard, rotary_knob, update_dt());

            const auto& joint_posture_state = joint_mode_mgr_.joint_posture_state();
            *mode_                     = joint_posture_state.mode;
            *pitch_lock_active_        = joint_posture_state.pitch_lock_active;
            *active_suspension_active_ = joint_posture_state.suspension_active;

            update_velocity_control();
            run_joint_intent_pipeline_();
        } while (false);
    }

private:
    static constexpr size_t kJointCount                 = 4;
    static constexpr double nan_                        = std::numeric_limits<double>::quiet_NaN();
    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_       = 30.0;
    static constexpr double default_dt_                 = 1e-3;

    void validate_joint_feedback_inputs() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (!joint_physical_angle_[i].ready())
                throw std::runtime_error(
                    "missing deformable chassis feedback "
                    "interfaces: expected "
                    "/chassis/*_joint/physical_angle");
    }

    void reset_all_controls() {
        *mode_             = rmcs_msgs::ChassisMode::AUTO;
        *pitch_lock_active_ = false;

        joint_mode_mgr_.reset();
        suspension_.reset();
        joint_mgr_.reset();

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_         = nan_;
        *chassis_control_angle_ = nan_;

        *scope_motor_control_torque = nan_;

        publish_nan_joint_targets();
    }

    double update_dt() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return default_dt_;
    }

    void update_velocity_control() {
        Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        double angular_velocity                = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move);

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max_;
        return translational_velocity;
    }

    double update_angular_velocity_control() {
        double angular_velocity      = 0.0;
        double chassis_control_angle = nan_;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;

        case rmcs_msgs::ChassisMode::SPIN: {
            const auto& joint_posture_state = joint_mode_mgr_.joint_posture_state();
            bool forward = joint_posture_state.spinning_forward;
            angular_velocity =
                spin_ratio_ * (forward ? angular_velocity_max_ : -angular_velocity_max_);
            angular_velocity =
                std::clamp(angular_velocity, -angular_velocity_max_, angular_velocity_max_);
        } break;

        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double chassis_angle_error =
                calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = std::numbers::pi;
            while (chassis_angle_error > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                chassis_angle_error -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(chassis_angle_error);
        } break;

        default: break;
        }

        *chassis_angle_         = 2 * std::numbers::pi - *gimbal_yaw_angle_;
        *chassis_control_angle_ = chassis_control_angle;

        return angular_velocity;
    }

    double calculate_unsigned_chassis_angle_error(double& chassis_control_angle) {
        chassis_control_angle = *gimbal_yaw_angle_error_;
        if (chassis_control_angle < 0)
            chassis_control_angle += 2 * std::numbers::pi;

        double unsigned_angle_error = chassis_control_angle + *gimbal_yaw_angle_;
        if (unsigned_angle_error >= 2 * std::numbers::pi)
            unsigned_angle_error -= 2 * std::numbers::pi;

        return unsigned_angle_error;
    }

    static double deg_to_rad(double deg) { return deg * std::numbers::pi / 180.0; }

    std::array<double, kJointCount> read_feedback_() const {
        std::array<double, kJointCount> angles;
        angles.fill(nan_);

        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_physical_angle_[i].ready() && std::isfinite(*joint_physical_angle_[i]))
                angles[i] = *joint_physical_angle_[i];

        return angles;
    }

    std::array<double, kJointCount> compute_target_angles_rad_() const {
        const auto& joint_posture_state = joint_mode_mgr_.joint_posture_state();
        std::array<double, kJointCount> targets_deg = joint_posture_state.joint_posture_target_deg;

        std::array<double, kJointCount> targets_rad;
        for (size_t i = 0; i < kJointCount; ++i)
            targets_rad[i] = deg_to_rad(targets_deg[i]);

        return targets_rad;
    }

    void run_joint_intent_pipeline_() {
        const auto current_physical_angles = read_feedback_();
        const auto& joint_posture_state = joint_mode_mgr_.joint_posture_state();

        if (!joint_mgr_.init_from_feedback(current_physical_angles)) {
            publish_nan_joint_targets();
            return;
        }

        std::array<double, kJointCount> target_angles_rad = compute_target_angles_rad_();

        suspension_.calibrate(
            *chassis_imu_pitch_, *chassis_imu_roll_, joint_posture_state.symmetric_posture_target,
            update_dt());

        *scope_motor_control_torque = suspension_.scope_torque(
            joint_posture_state.suspension_active,
            joint_posture_state.mode == rmcs_msgs::ChassisMode::SPIN);

        auto corrections = suspension_.update(
            *chassis_imu_pitch_ - suspension_.pitch_offset(),
            *chassis_imu_roll_ - suspension_.roll_offset(), *chassis_imu_pitch_rate_,
            *chassis_imu_roll_rate_, joint_posture_state.suspension_active,
            joint_posture_state.ctrl_low_prone_active, joint_mode_mgr_.min_angle(),
            joint_mode_mgr_.max_angle(), joint_posture_state.suspension_reference_angle_deg,
            joint_mode_mgr_.correction_inverted(), joint_mgr_.angle_states(), update_dt());

        joint_mgr_.run_trajectory(
            target_angles_rad, joint_posture_state.suspension_active, update_dt());

        publish_joint_targets_(current_physical_angles, corrections);
    }

    void publish_joint_targets_(
        const std::array<double, kJointCount>& feedback_angles,
        const DeformableChassisActiveSuspension::Corrections& corrections) {

        double min_angle_rad = joint_mode_mgr_.active_suspension_min_angle_rad();
        double max_angle_rad = joint_mode_mgr_.max_angle_rad();

        if (!joint_mgr_.any_active()) {
            publish_nan_joint_targets();
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_mgr_.joint_active(i)) {
                *joint_target_angle_[i] = nan_;
                *joint_target_velocity_[i] = nan_;
                *joint_target_acceleration_[i] = nan_;
                *joint_angle_error_[i] = nan_;
                continue;
            }

            double target = joint_mgr_.angle_state(i) + corrections.joint_angle_correction[i];
            *joint_target_angle_[i] = std::clamp(target, min_angle_rad, max_angle_rad);
            *joint_target_velocity_[i] =
                joint_mgr_.velocity_state(i) + corrections.joint_velocity_correction[i];
            *joint_target_acceleration_[i] =
                joint_mgr_.acceleration_state(i) + corrections.joint_acceleration_correction[i];
            *joint_angle_error_[i] = std::isfinite(feedback_angles[i])
                                       ? feedback_angles[i] - *joint_target_angle_[i]
                                       : nan_;
        }
    }

    void publish_nan_joint_targets() {
        suspension_.reset();
        joint_mgr_.reset();

        for (size_t i = 0; i < kJointCount; ++i) {
            *joint_target_angle_[i]        = nan_;
            *joint_target_velocity_[i]     = nan_;
            *joint_target_acceleration_[i] = nan_;
            *joint_angle_error_[i]         = nan_;
        }
    }

    static constexpr const char* kJointName[] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> update_rate_;

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;
    OutputInterface<bool> pitch_lock_active_;
    OutputInterface<bool> active_suspension_active_;

    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;

    OutputInterface<double> scope_motor_control_torque;

    std::array<InputInterface<double>, kJointCount> joint_physical_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_velocity_;
    std::array<OutputInterface<double>, kJointCount> joint_target_acceleration_;
    std::array<OutputInterface<double>, kJointCount> joint_angle_error_;

    pid::PidCalculator following_velocity_controller_;
    const double spin_ratio_;

    DeformableChassisModeManager joint_mode_mgr_;
    DeformableChassisActiveSuspension suspension_;
    DeformableChassisJointManager joint_mgr_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
