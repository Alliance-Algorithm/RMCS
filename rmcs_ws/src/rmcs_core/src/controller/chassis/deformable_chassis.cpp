#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
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

#include "controller/chassis/deformable_mode.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)
        , wireless_charging_offset_rad_(
              deg_to_rad(get_parameter_or("wireless_charging_offset_deg", 135.0)))
        , joint_mode_mgr_(*this) {

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/predefined/update_rate", update_rate_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);
        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);
        register_output("/chassis/pitch_lock_active", pitch_lock_active_, false);
        register_output("/chassis/active_suspension/active", active_suspension_active_, false);
        register_output("/chassis/deformable/low_prone_active", low_prone_active_, false);
        register_output(
            "/chassis/deformable/symmetric_posture_target", symmetric_posture_target_, true);
        register_output("/chassis/deformable/correction_inverted", correction_inverted_, false);
        register_output(
            "/chassis/deformable/min_angle_deg", min_angle_deg_, joint_mode_mgr_.min_angle());
        register_output(
            "/chassis/deformable/max_angle_deg", max_angle_deg_, joint_mode_mgr_.max_angle());
        register_output(
            "/chassis/deformable/suspension_reference_angle_deg", suspension_reference_angle_deg_,
            joint_mode_mgr_.suspension_reference_angle_deg());
        register_output(
            "/chassis/deformable/reset_count", deformable_reset_count_, static_cast<size_t>(0));
        for (size_t i = 0; i < kJointCount; ++i) {
            register_output(
                fmt::format("/chassis/deformable/{}_joint/posture_target_angle", kJointName[i]),
                joint_posture_target_angle_rad_[i], deg_to_rad(joint_mode_mgr_.max_angle()));
        }

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *pitch_lock_active_ = false;
        *active_suspension_active_ = false;
        *low_prone_active_ = false;
        *symmetric_posture_target_ = true;
        *correction_inverted_ = false;
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
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;
        const auto keyboard = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            double rotary_knob = rotary_knob_.ready() ? *rotary_knob_ : 0.0;

            joint_mode_mgr_.update(switch_left, switch_right, keyboard, rotary_knob, update_dt());

            *mode_ = joint_mode_mgr_.mode();
            *pitch_lock_active_ = joint_mode_mgr_.pitch_lock_active();
            *active_suspension_active_ = joint_mode_mgr_.suspension_active();
            *low_prone_active_ = joint_mode_mgr_.low_prone_active();
            *symmetric_posture_target_ = joint_mode_mgr_.symmetric_posture_target();
            *correction_inverted_ = joint_mode_mgr_.correction_inverted();
            *min_angle_deg_ = joint_mode_mgr_.min_angle();
            *max_angle_deg_ = joint_mode_mgr_.max_angle();
            *suspension_reference_angle_deg_ = joint_mode_mgr_.suspension_reference_angle_deg();
            publish_joint_posture_targets_();

            update_velocity_control();
        } while (false);
    }

private:
    static constexpr size_t kJointCount = 4;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 30.0;
    static constexpr double default_dt_ = 1e-3;

    void reset_all_controls() {
        joint_mode_mgr_.reset();
        *deformable_reset_count_ += 1;

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        *pitch_lock_active_ = false;
        *active_suspension_active_ = false;
        *low_prone_active_ = false;
        *symmetric_posture_target_ = true;
        *correction_inverted_ = false;
        *min_angle_deg_ = joint_mode_mgr_.min_angle();
        *max_angle_deg_ = joint_mode_mgr_.max_angle();
        *suspension_reference_angle_deg_ = joint_mode_mgr_.suspension_reference_angle_deg();
        publish_joint_posture_targets_();

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;
    }

    double update_dt() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return default_dt_;
    }

    void update_velocity_control() {
        Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        double angular_velocity = update_angular_velocity_control();
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
        double angular_velocity = 0.0;
        double chassis_control_angle = nan_;

        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: break;

        case rmcs_msgs::ChassisMode::SPIN_FAST: {
            bool forward = joint_mode_mgr_.spinning_forward();
            angular_velocity =
                forward ? angular_velocity_max_ : -angular_velocity_max_;
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

        case rmcs_msgs::ChassisMode::WIRELESS_CHARGING: {
            double chassis_angle_error =
                calculate_unsigned_chassis_angle_error(chassis_control_angle);

            chassis_control_angle = normalize_positive_angle(
                chassis_control_angle + wireless_charging_offset_rad_);
            chassis_angle_error = normalize_positive_angle(
                chassis_angle_error + wireless_charging_offset_rad_);
            chassis_angle_error = normalize_signed_angle(chassis_angle_error);

            angular_velocity = following_velocity_controller_.update(chassis_angle_error);
        } break;

        default: break;
        }

        *chassis_angle_ = 2 * std::numbers::pi - *gimbal_yaw_angle_;
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

    static double normalize_positive_angle(double angle) {
        constexpr double full_turn = 2 * std::numbers::pi;
        while (angle >= full_turn)
            angle -= full_turn;
        while (angle < 0.0)
            angle += full_turn;
        return angle;
    }

    static double normalize_signed_angle(double angle) {
        angle = normalize_positive_angle(angle);
        if (angle > std::numbers::pi)
            angle -= 2 * std::numbers::pi;
        return angle;
    }

    void publish_joint_posture_targets_() {
        std::array<double, kJointCount> targets_deg{};
        joint_mode_mgr_.copy_joint_posture_target_deg(targets_deg);

        for (size_t i = 0; i < kJointCount; ++i)
            *joint_posture_target_angle_rad_[i] = deg_to_rad(targets_deg[i]);
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
    OutputInterface<bool> low_prone_active_;
    OutputInterface<bool> symmetric_posture_target_;
    OutputInterface<bool> correction_inverted_;
    OutputInterface<double> min_angle_deg_;
    OutputInterface<double> max_angle_deg_;
    OutputInterface<double> suspension_reference_angle_deg_;
    OutputInterface<size_t> deformable_reset_count_;
    std::array<OutputInterface<double>, kJointCount> joint_posture_target_angle_rad_;

    pid::PidCalculator following_velocity_controller_;
    const double wireless_charging_offset_rad_;

    DeformableChassisModeManager joint_mode_mgr_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
