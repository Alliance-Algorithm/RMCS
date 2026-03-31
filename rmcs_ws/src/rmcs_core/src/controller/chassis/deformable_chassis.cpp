#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableChassis
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class JointFeedbackSource : uint8_t { kLegacyEncoderAngle, kMotorAngle };

    DeformableChassis()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , following_velocity_controller_(10.0, 0.0, 0.0)

        , min_angle_(get_parameter_or("min_angle", 15.0))
        , max_angle_(get_parameter_or("max_angle", 55.0))
        , left_front_joint_offset_(get_parameter_or("left_front_joint_offset", 0.0))
        , left_back_joint_offset_(get_parameter_or("left_back_joint_offset", 0.0))
        , right_front_joint_offset_(get_parameter_or("right_front_joint_offset", 0.0))
        , right_back_joint_offset_(get_parameter_or("right_back_joint_offset", 0.0))
        , Bx_(get_parameter_or("Rod_relative_horizontal_coordinate", 0.0))
        , By_(get_parameter_or("Rod_relative_longitudinal_coordinate", 0.0))
        , L_(get_parameter_or("Rod_length", 0.0))

        , joint_velocity_limit_(get_parameter_or("joint_velocity_limit", 500.0))
        , joint_torque_limit_(get_parameter_or("joint_torque_limit", 10.0)) {

        double joint_angle_kp = get_parameter_or("joint_angle_kp", 2.0);
        double joint_angle_ki = get_parameter_or("joint_angle_ki", 0.0);
        double joint_angle_kd = get_parameter_or("joint_angle_kd", 0.0);
        double joint_velocity_kp = get_parameter_or("joint_velocity_kp", 1.0);
        double joint_velocity_ki = get_parameter_or("joint_velocity_ki", 0.0);
        double joint_velocity_kd = get_parameter_or("joint_velocity_kd", 0.0);

        lf_angle_pid_ = pid::PidCalculator(joint_angle_kp, joint_angle_ki, joint_angle_kd);
        lb_angle_pid_ = pid::PidCalculator(joint_angle_kp, joint_angle_ki, joint_angle_kd);
        rf_angle_pid_ = pid::PidCalculator(joint_angle_kp, joint_angle_ki, joint_angle_kd);
        rb_angle_pid_ = pid::PidCalculator(joint_angle_kp, joint_angle_ki, joint_angle_kd);

        lf_velocity_pid_ =
            pid::PidCalculator(joint_velocity_kp, joint_velocity_ki, joint_velocity_kd);
        lb_velocity_pid_ =
            pid::PidCalculator(joint_velocity_kp, joint_velocity_ki, joint_velocity_kd);
        rf_velocity_pid_ =
            pid::PidCalculator(joint_velocity_kp, joint_velocity_ki, joint_velocity_kd);
        rb_velocity_pid_ =
            pid::PidCalculator(joint_velocity_kp, joint_velocity_ki, joint_velocity_kd);

        lf_velocity_pid_.output_max = joint_torque_limit_;
        lf_velocity_pid_.output_min = -joint_torque_limit_;
        lb_velocity_pid_.output_max = joint_torque_limit_;
        lb_velocity_pid_.output_min = -joint_torque_limit_;
        rf_velocity_pid_.output_max = joint_torque_limit_;
        rf_velocity_pid_.output_min = -joint_torque_limit_;
        rb_velocity_pid_.output_max = joint_torque_limit_;
        rb_velocity_pid_.output_min = -joint_torque_limit_;

        following_velocity_controller_.output_max = angular_velocity_max_;
        following_velocity_controller_.output_min = -angular_velocity_max_;

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_, false);
        register_input("/gimbal/yaw/control_angle_error", gimbal_yaw_angle_error_, false);

        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_, false);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_, false);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_, false);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_, false);

        register_input("/chassis/left_front_joint/velocity", left_front_joint_velocity_, false);
        register_input("/chassis/left_back_joint/velocity", left_back_joint_velocity_, false);
        register_input("/chassis/right_front_joint/velocity", right_front_joint_velocity_, false);
        register_input("/chassis/right_back_joint/velocity", right_back_joint_velocity_, false);

        register_input(
            "/chassis/left_front_joint/encoder_angle", left_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/left_back_joint/encoder_angle", left_back_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_front_joint/encoder_angle", right_front_joint_encoder_angle_, false);
        register_input(
            "/chassis/right_back_joint/encoder_angle", right_back_joint_encoder_angle_, false);

        register_output("/chassis/angle", chassis_angle_, nan_);
        register_output("/chassis/control_angle", chassis_control_angle_, nan_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/control_angle_error", lf_angle_error_, nan_);
        register_output("/chassis/left_back_joint/control_angle_error", lb_angle_error_, nan_);
        register_output("/chassis/right_front_joint/control_angle_error", rf_angle_error_, nan_);
        register_output("/chassis/right_back_joint/control_angle_error", rb_angle_error_, nan_);

        register_output("/chassis/left_front_joint/control_torque", lf_control_torque_, nan_);
        register_output("/chassis/left_back_joint/control_torque", lb_control_torque_, nan_);
        register_output("/chassis/right_front_joint/control_torque", rf_control_torque_, nan_);
        register_output("/chassis/right_back_joint/control_torque", rb_control_torque_, nan_);

        register_output("/chassis/processed_encoder/angle", processed_encoder_angle_, nan_);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        current_target_angle_ = max_angle_;
        lf_current_target_angle_ = max_angle_;
        lb_current_target_angle_ = max_angle_;
        rf_current_target_angle_ = max_angle_;
        rb_current_target_angle_ = max_angle_;

        const bool left_front_joint_offset = has_parameter("left_front_joint_offset");
        const bool left_back_joint_offset = has_parameter("left_back_joint_offset");
        const bool right_front_joint_offset = has_parameter("right_front_joint_offset");
        const bool right_back_joint_offset = has_parameter("right_back_joint_offset");

        const bool has_any_joint_offset = left_front_joint_offset || left_back_joint_offset
                                       || right_front_joint_offset || right_back_joint_offset;
        const bool has_all_joint_offsets = left_front_joint_offset && left_back_joint_offset
                                        && right_front_joint_offset && right_back_joint_offset;
        if (has_any_joint_offset && !has_all_joint_offsets)
            throw std::runtime_error(
                "deformable chassis joint offsets must be configured for all four joints or "
                "removed entirely");

        joint_feedback_source_ = has_all_joint_offsets ? JointFeedbackSource::kLegacyEncoderAngle
                                                       : JointFeedbackSource::kMotorAngle;
    }

    void before_updating() override {
        if (!gimbal_yaw_angle_.ready()) {
            gimbal_yaw_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/gimbal/yaw/angle\". Set to 0.0.");
        }
        if (!gimbal_yaw_angle_error_.ready()) {
            gimbal_yaw_angle_error_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/gimbal/yaw/control_angle_error\". Set to 0.0.");
        }
        validate_joint_feedback_inputs();
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

            auto mode = *mode_;
            if (switch_left != Switch::DOWN) {
                if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::STEP_DOWN;
                    } else {
                        mode = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.c && keyboard.c) {
                    if (mode == rmcs_msgs::ChassisMode::SPIN) {
                        mode = rmcs_msgs::ChassisMode::AUTO;
                    } else {
                        mode = rmcs_msgs::ChassisMode::SPIN;
                        spinning_forward_ = !spinning_forward_;
                    }
                } else if (!last_keyboard_.x && keyboard.x) {
                    mode = mode == rmcs_msgs::ChassisMode::LAUNCH_RAMP
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::LAUNCH_RAMP;
                } else if (!last_keyboard_.z && keyboard.z) {
                    mode = mode == rmcs_msgs::ChassisMode::STEP_DOWN
                             ? rmcs_msgs::ChassisMode::AUTO
                             : rmcs_msgs::ChassisMode::STEP_DOWN;
                }
                *mode_ = mode;
            }

            update_velocity_control();
            update_lift_target_toggle(switch_left, switch_right, keyboard);
            update_lift_angle_error();
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_ = switch_left;
        last_keyboard_ = keyboard;
    }

private:
    static constexpr double inf_ = std::numeric_limits<double>::infinity();
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 20.0;
    static constexpr double rad_to_deg_ = 180.0 / std::numbers::pi;

    static double wrap_deg(double deg) {
        deg = std::fmod(deg, 360.0);
        if (deg >= 180.0)
            deg -= 360.0;
        if (deg < -180.0)
            deg += 360.0;
        return deg;
    }

    void validate_joint_feedback_inputs() const {
        const bool ready =
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? left_front_joint_angle_.ready() && left_back_joint_angle_.ready()
                      && right_front_joint_angle_.ready() && right_back_joint_angle_.ready()
                : left_front_joint_encoder_angle_.ready() && left_back_joint_encoder_angle_.ready()
                      && right_front_joint_encoder_angle_.ready()
                      && right_back_joint_encoder_angle_.ready();

        if (ready)
            return;

        throw std::runtime_error(
            joint_feedback_source_ == JointFeedbackSource::kMotorAngle
                ? "missing V2 joint feedback interfaces: expected /chassis/*_joint/angle"
                : "missing legacy joint feedback interfaces: expected /chassis/*_joint/"
                  "encoder_angle");
    }

    double joint_angle_deg(
        const InputInterface<double>& joint_angle,
        const InputInterface<double>& joint_encoder_angle, double joint_offset,
        double legacy_fixed_compensation) const {
        if (joint_feedback_source_ == JointFeedbackSource::kMotorAngle)
            return wrap_deg(*joint_angle * rad_to_deg_);

        return wrap_deg(joint_offset) - wrap_deg(*joint_encoder_angle) + legacy_fixed_compensation;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;

        chassis_control_velocity_->vector << nan_, nan_, nan_;
        *chassis_angle_ = nan_;
        *chassis_control_angle_ = nan_;

        current_target_angle_ = max_angle_;
        test_init_ = false;

        *lf_angle_error_ = nan_;
        *lb_angle_error_ = nan_;
        *rf_angle_error_ = nan_;
        *rb_angle_error_ = nan_;

        *processed_encoder_angle_ = nan_;
        // RCLCPP_INFO(get_logger(), "%f", *scope_motor_velocity);
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity =
            Eigen::Rotation2Dd{*gimbal_yaw_angle_} * ((*joystick_right_) + keyboard_move);

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

        case rmcs_msgs::ChassisMode::SPIN: {
            angular_velocity =
                0.6 * (spinning_forward_ ? angular_velocity_max_ : -angular_velocity_max_);
        } break;

        case rmcs_msgs::ChassisMode::STEP_DOWN: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = std::numbers::pi;
            while (err > alignment / 2) {
                chassis_control_angle -= alignment;
                if (chassis_control_angle < 0)
                    chassis_control_angle += 2 * std::numbers::pi;
                err -= alignment;
            }

            angular_velocity = following_velocity_controller_.update(err);
        } break;

        case rmcs_msgs::ChassisMode::LAUNCH_RAMP: {
            double err = calculate_unsigned_chassis_angle_error(chassis_control_angle);

            constexpr double alignment = 2 * std::numbers::pi;
            if (err > alignment / 2)
                err -= alignment;

            angular_velocity = following_velocity_controller_.update(err);
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

        double err = chassis_control_angle + *gimbal_yaw_angle_;
        if (err >= 2 * std::numbers::pi)
            err -= 2 * std::numbers::pi;

        return err;
    }

    void update_lift_target_toggle(
        rmcs_msgs::Switch left_switch, rmcs_msgs::Switch right_switch,
        rmcs_msgs::Keyboard keyboard) {
        const bool switch_toggle_condition =
            (left_switch == rmcs_msgs::Switch::MIDDLE) && (right_switch == rmcs_msgs::Switch::UP);
        const bool keyboard_toggle_condition = !last_keyboard_.r && keyboard.r;

        const bool last_switch_toggle_condition = (last_switch_left_ == rmcs_msgs::Switch::MIDDLE)
                                               && (last_switch_right_ == rmcs_msgs::Switch::UP);
        const bool front_high_rear_low = !last_keyboard_.b && keyboard.b;
        const bool front_low_rear_high = !last_keyboard_.g && keyboard.g;
        const bool uphill = !last_keyboard_.ctrl && keyboard.ctrl;

        if ((switch_toggle_condition && !last_switch_toggle_condition)
            || keyboard_toggle_condition) {
            current_target_angle_ =
                (std::abs(current_target_angle_ - max_angle_) < 1e-6) ? min_angle_ : max_angle_;
        } else if (uphill) {
            lf_current_target_angle_ = min_angle_;
            rf_current_target_angle_ = min_angle_;
            lb_current_target_angle_ = min_angle_ + 10.0;
            rb_current_target_angle_ = min_angle_ + 10.0;
        } else if (front_high_rear_low) {
            lf_current_target_angle_ = max_angle_;
            rf_current_target_angle_ = max_angle_;
            lb_current_target_angle_ = min_angle_;
            rb_current_target_angle_ = min_angle_;
        } else if (front_low_rear_high) {
            lf_current_target_angle_ = min_angle_;
            rf_current_target_angle_ = min_angle_;
            lb_current_target_angle_ = max_angle_;
            rb_current_target_angle_ = max_angle_;
        }
        lf_current_target_angle_ = current_target_angle_;
        lb_current_target_angle_ = current_target_angle_;
        rb_current_target_angle_ = current_target_angle_;
        rf_current_target_angle_ = current_target_angle_;
    }

    void update_lift_angle_error() {
        double lf_s_target_ = trapezoidal_calculator(lf_current_target_angle_);
        double lb_s_target_ = trapezoidal_calculator(lb_current_target_angle_);
        double rb_s_target_ = trapezoidal_calculator(rb_current_target_angle_);
        double rf_s_target_ = trapezoidal_calculator(rf_current_target_angle_);

        const double alpha_lf = joint_angle_deg(
            left_front_joint_angle_, left_front_joint_encoder_angle_, left_front_joint_offset_,
            10.81767);
        const double alpha_lb = joint_angle_deg(
            left_back_joint_angle_, left_back_joint_encoder_angle_, left_back_joint_offset_,
            10.38748);
        const double alpha_rf = joint_angle_deg(
            right_front_joint_angle_, right_front_joint_encoder_angle_, right_front_joint_offset_,
            10.57454);
        const double alpha_rb = joint_angle_deg(
            right_back_joint_angle_, right_back_joint_encoder_angle_, right_back_joint_offset_,
            10.326716);

        *processed_encoder_angle_ = (alpha_lb + alpha_rb + alpha_lf + alpha_rf) / 4.0;

        s_lf_ = trapezoidal_calculator(alpha_lf);
        s_lb_ = trapezoidal_calculator(alpha_lb);
        s_rf_ = trapezoidal_calculator(alpha_rf);
        s_rb_ = trapezoidal_calculator(alpha_rb);

        *lf_angle_error_ = s_lf_ - lf_s_target_;
        *lb_angle_error_ = s_lb_ - lb_s_target_;
        *rf_angle_error_ = s_rf_ - rf_s_target_;
        *rb_angle_error_ = s_rb_ - rb_s_target_;

        double lf_velocity_ref = lf_angle_pid_.update(*lf_angle_error_);
        double lb_velocity_ref = lb_angle_pid_.update(*lb_angle_error_);
        double rf_velocity_ref = rf_angle_pid_.update(*rf_angle_error_);
        double rb_velocity_ref = rb_angle_pid_.update(*rb_angle_error_);

        lf_velocity_ref =
            std::clamp(lf_velocity_ref, -joint_velocity_limit_, joint_velocity_limit_);
        lb_velocity_ref =
            std::clamp(lb_velocity_ref, -joint_velocity_limit_, joint_velocity_limit_);
        rf_velocity_ref =
            std::clamp(rf_velocity_ref, -joint_velocity_limit_, joint_velocity_limit_);
        rb_velocity_ref =
            std::clamp(rb_velocity_ref, -joint_velocity_limit_, joint_velocity_limit_);

        double lf_torque = lf_velocity_pid_.update(lf_velocity_ref - *left_front_joint_velocity_);
        double lb_torque = lb_velocity_pid_.update(lb_velocity_ref - *left_back_joint_velocity_);
        double rf_torque = rf_velocity_pid_.update(rf_velocity_ref - *right_front_joint_velocity_);
        double rb_torque = rb_velocity_pid_.update(rb_velocity_ref - *right_back_joint_velocity_);

        *lf_control_torque_ = lf_torque;
        *lb_control_torque_ = lb_torque;
        *rf_control_torque_ = rf_torque;
        *rb_control_torque_ = rb_torque;
    }

    double trapezoidal_calculator(double alpha_deg) const {
        const double rad = alpha_deg * pi_ / 180.0;

        const double term = Bx_ * std::cos(rad) + By_ * std::sin(rad);
        const double t = (Bx_ * std::sin(rad) - By_ * std::cos(rad) + 15.0);

        const double inside = L_ * L_ - t * t;
        return term + std::sqrt(std::max(0.0, inside));
    }

private:
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;

    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_switch_left_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<double> gimbal_yaw_angle_, gimbal_yaw_angle_error_;
    OutputInterface<double> chassis_angle_, chassis_control_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    bool spinning_forward_ = true;
    pid::PidCalculator following_velocity_controller_;

    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> right_back_joint_angle_;

    InputInterface<double> left_front_joint_velocity_;
    InputInterface<double> left_back_joint_velocity_;
    InputInterface<double> right_front_joint_velocity_;
    InputInterface<double> right_back_joint_velocity_;

    InputInterface<double> left_front_joint_encoder_angle_;
    InputInterface<double> left_back_joint_encoder_angle_;
    InputInterface<double> right_front_joint_encoder_angle_;
    InputInterface<double> right_back_joint_encoder_angle_;

    OutputInterface<double> lf_angle_error_;
    OutputInterface<double> lb_angle_error_;
    OutputInterface<double> rf_angle_error_;
    OutputInterface<double> rb_angle_error_;

    OutputInterface<double> lf_control_torque_;
    OutputInterface<double> lb_control_torque_;
    OutputInterface<double> rf_control_torque_;
    OutputInterface<double> rb_control_torque_;

    OutputInterface<double> processed_encoder_angle_;

    double min_angle_;
    double max_angle_;
    double left_front_joint_offset_;
    double left_back_joint_offset_;
    double right_front_joint_offset_;
    double right_back_joint_offset_;
    JointFeedbackSource joint_feedback_source_ = JointFeedbackSource::kLegacyEncoderAngle;

    double current_target_angle_;
    double lf_current_target_angle_, lb_current_target_angle_, rb_current_target_angle_,
        rf_current_target_angle_;

    double s_lf_ = 0.0, s_lb_ = 0.0, s_rf_ = 0.0, s_rb_ = 0.0;

    double Bx_ = 0.0;
    double By_ = 0.0;
    double L_ = 0.0;

    pid::PidCalculator lf_angle_pid_, lb_angle_pid_, rf_angle_pid_, rb_angle_pid_;
    pid::PidCalculator lf_velocity_pid_, lb_velocity_pid_, rf_velocity_pid_, rb_velocity_pid_;

    double joint_velocity_limit_ = 500.0;
    double joint_torque_limit_ = 10.0;

    bool test_init_ = false;

    static constexpr double pi_ = std::numbers::pi;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::DeformableChassis, rmcs_executor::Component)
