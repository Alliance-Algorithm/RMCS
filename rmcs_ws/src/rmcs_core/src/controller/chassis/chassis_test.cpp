#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include <rclcpp/node.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::chassis {

class ChassisTestV2Controller
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisTestV2Controller()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , min_angle_rad_(deg_to_rad_(get_parameter_or("min_angle", 15.0)))
        , max_angle_rad_(deg_to_rad_(get_parameter_or("max_angle", 35.0)))
        , target_physical_velocity_limit_(
              deg_to_rad_(get_parameter_or("target_physical_velocity_limit", 180.0)))
        , target_physical_acceleration_limit_(
              deg_to_rad_(get_parameter_or("target_physical_acceleration_limit", 720.0))) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", right_switch_);
        register_input("/remote/switch/left", left_switch_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob", rotary_knob_);
        register_input("/chassis/left_front_joint/angle", left_front_joint_angle_);
        register_input("/chassis/left_back_joint/angle", left_back_joint_angle_);
        register_input("/chassis/right_back_joint/angle", right_back_joint_angle_);
        register_input("/chassis/right_front_joint/angle", right_front_joint_angle_);
        register_input(
            "/chassis/left_front_joint/physical_angle", left_front_joint_physical_angle_);
        register_input("/chassis/left_back_joint/physical_angle", left_back_joint_physical_angle_);
        register_input(
            "/chassis/right_back_joint/physical_angle", right_back_joint_physical_angle_);
        register_input(
            "/chassis/right_front_joint/physical_angle", right_front_joint_physical_angle_);

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/target_angle", left_front_target_angle_, nan_);
        register_output("/chassis/left_back_joint/target_angle", left_back_target_angle_, nan_);
        register_output("/chassis/right_back_joint/target_angle", right_back_target_angle_, nan_);
        register_output("/chassis/right_front_joint/target_angle", right_front_target_angle_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_angle", left_front_target_physical_angle_,
            nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_angle", left_back_target_physical_angle_,
            nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_angle", right_back_target_physical_angle_,
            nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_angle", right_front_target_physical_angle_,
            nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_velocity",
            left_front_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_velocity",
            left_back_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_velocity",
            right_back_target_physical_velocity_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_velocity",
            right_front_target_physical_velocity_, nan_);
        register_output(
            "/chassis/left_front_joint/target_physical_acceleration",
            left_front_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/left_back_joint/target_physical_acceleration",
            left_back_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_back_joint/target_physical_acceleration",
            right_back_target_physical_acceleration_, nan_);
        register_output(
            "/chassis/right_front_joint/target_physical_acceleration",
            right_front_target_physical_acceleration_, nan_);

        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;
    }

    void update() override {
        using rmcs_msgs::Switch;

        const auto right_switch = *right_switch_;
        const auto left_switch = *left_switch_;
        const auto keyboard = *keyboard_;

        if ((left_switch == Switch::UNKNOWN || right_switch == Switch::UNKNOWN)
            || (left_switch == Switch::DOWN && right_switch == Switch::DOWN)) {
            reset_all_controls();

            last_right_switch_ = right_switch;
            last_left_switch_ = left_switch;
            last_keyboard_ = keyboard;

            last_lift_left_switch_ = left_switch;
            last_lift_right_switch_ = right_switch;
            return;
        }

        update_chassis_mode(right_switch, left_switch, keyboard);
        update_velocity_control();

        if (!joint_target_active_) {
            publish_current_joint_target_angles();
        }

        if (joint_target_active_) {
            update_lift_target_toggle(left_switch, right_switch);
            update_joint_target_trajectory();
        }

        publish_joint_target_angles();

        last_right_switch_ = right_switch;
        last_left_switch_ = left_switch;
        last_keyboard_ = keyboard;

        last_lift_left_switch_ = left_switch;
        last_lift_right_switch_ = right_switch;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    static constexpr double translational_velocity_max_ = 10.0;
    static constexpr double angular_velocity_max_ = 16.0;
    static constexpr double spin_ratio_default_ = 0.6;
    static constexpr double joint_zero_physical_angle_rad_ = 1.090830782496456;

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    static double physical_to_motor_angle_(double physical_angle_rad) {
        return joint_zero_physical_angle_rad_ - physical_angle_rad;
    }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        *left_front_target_angle_ = nan_;
        *left_back_target_angle_ = nan_;
        *right_back_target_angle_ = nan_;
        *right_front_target_angle_ = nan_;
        *left_front_target_physical_angle_ = nan_;
        *left_back_target_physical_angle_ = nan_;
        *right_back_target_physical_angle_ = nan_;
        *right_front_target_physical_angle_ = nan_;
        *left_front_target_physical_velocity_ = nan_;
        *left_back_target_physical_velocity_ = nan_;
        *right_back_target_physical_velocity_ = nan_;
        *right_front_target_physical_velocity_ = nan_;
        *left_front_target_physical_acceleration_ = nan_;
        *left_back_target_physical_acceleration_ = nan_;
        *right_back_target_physical_acceleration_ = nan_;
        *right_front_target_physical_acceleration_ = nan_;

        joint_target_active_ = false;
        target_physical_angle_state_ = nan_;
        target_physical_velocity_state_ = 0.0;
        target_physical_acceleration_state_ = 0.0;
    }

    bool publish_current_joint_target_angles() {
        const Eigen::Vector4d current_motor_angles{
            *left_front_joint_angle_, *left_back_joint_angle_, *right_back_joint_angle_,
            *right_front_joint_angle_};
        const Eigen::Vector4d current_physical_angles{
            *left_front_joint_physical_angle_, *left_back_joint_physical_angle_,
            *right_back_joint_physical_angle_, *right_front_joint_physical_angle_};
        if (!current_motor_angles.array().isFinite().all()
            || !current_physical_angles.array().isFinite().all()) {
            return false;
        }

        *left_front_target_angle_ = current_motor_angles[0];
        *left_back_target_angle_ = current_motor_angles[1];
        *right_back_target_angle_ = current_motor_angles[2];
        *right_front_target_angle_ = current_motor_angles[3];
        *left_front_target_physical_angle_ = current_physical_angles[0];
        *left_back_target_physical_angle_ = current_physical_angles[1];
        *right_back_target_physical_angle_ = current_physical_angles[2];
        *right_front_target_physical_angle_ = current_physical_angles[3];
        *left_front_target_physical_velocity_ = 0.0;
        *left_back_target_physical_velocity_ = 0.0;
        *right_back_target_physical_velocity_ = 0.0;
        *right_front_target_physical_velocity_ = 0.0;
        *left_front_target_physical_acceleration_ = 0.0;
        *left_back_target_physical_acceleration_ = 0.0;
        *right_back_target_physical_acceleration_ = 0.0;
        *right_front_target_physical_acceleration_ = 0.0;
        current_target_angle_rad_ = current_physical_angles.mean();
        target_physical_angle_state_ = current_target_angle_rad_;
        target_physical_velocity_state_ = 0.0;
        target_physical_acceleration_state_ = 0.0;
        joint_target_active_ = true;
        return true;
    }

    void update_joint_target_trajectory() {
        if (!joint_target_active_ || !std::isfinite(current_target_angle_rad_)
            || !std::isfinite(target_physical_angle_state_)) {
            return;
        }

        const double position_error = current_target_angle_rad_ - target_physical_angle_state_;
        const double stopping_distance = target_physical_velocity_state_
                                       * target_physical_velocity_state_
                                       / (2.0 * target_physical_acceleration_limit_);

        double desired_velocity = 0.0;
        if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance) {
            desired_velocity = std::copysign(target_physical_velocity_limit_, position_error);
        }

        const double velocity_error = desired_velocity - target_physical_velocity_state_;
        target_physical_acceleration_state_ = std::clamp(
            velocity_error / dt_, -target_physical_acceleration_limit_,
            target_physical_acceleration_limit_);

        target_physical_velocity_state_ += target_physical_acceleration_state_ * dt_;
        target_physical_velocity_state_ = std::clamp(
            target_physical_velocity_state_, -target_physical_velocity_limit_,
            target_physical_velocity_limit_);
        target_physical_angle_state_ += target_physical_velocity_state_ * dt_;

        const double next_error = current_target_angle_rad_ - target_physical_angle_state_;
        if ((position_error > 0.0 && next_error < 0.0) || (position_error < 0.0 && next_error > 0.0)
            || (std::abs(next_error) < 1e-5 && std::abs(target_physical_velocity_state_) < 1e-3)) {
            target_physical_angle_state_ = current_target_angle_rad_;
            target_physical_velocity_state_ = 0.0;
            target_physical_acceleration_state_ = 0.0;
        }
    }

    void publish_joint_target_angles() {
        if (!joint_target_active_) {
            *left_front_target_angle_ = nan_;
            *left_back_target_angle_ = nan_;
            *right_back_target_angle_ = nan_;
            *right_front_target_angle_ = nan_;
            *left_front_target_physical_angle_ = nan_;
            *left_back_target_physical_angle_ = nan_;
            *right_back_target_physical_angle_ = nan_;
            *right_front_target_physical_angle_ = nan_;
            *left_front_target_physical_velocity_ = nan_;
            *left_back_target_physical_velocity_ = nan_;
            *right_back_target_physical_velocity_ = nan_;
            *right_front_target_physical_velocity_ = nan_;
            *left_front_target_physical_acceleration_ = nan_;
            *left_back_target_physical_acceleration_ = nan_;
            *right_back_target_physical_acceleration_ = nan_;
            *right_front_target_physical_acceleration_ = nan_;
            return;
        }

        const double current_target_motor_angle =
            physical_to_motor_angle_(target_physical_angle_state_);

        *left_front_target_angle_ = current_target_motor_angle;
        *left_back_target_angle_ = current_target_motor_angle;
        *right_back_target_angle_ = current_target_motor_angle;
        *right_front_target_angle_ = current_target_motor_angle;
        *left_front_target_physical_angle_ = target_physical_angle_state_;
        *left_back_target_physical_angle_ = target_physical_angle_state_;
        *right_back_target_physical_angle_ = target_physical_angle_state_;
        *right_front_target_physical_angle_ = target_physical_angle_state_;
        *left_front_target_physical_velocity_ = target_physical_velocity_state_;
        *left_back_target_physical_velocity_ = target_physical_velocity_state_;
        *right_back_target_physical_velocity_ = target_physical_velocity_state_;
        *right_front_target_physical_velocity_ = target_physical_velocity_state_;
        *left_front_target_physical_acceleration_ = target_physical_acceleration_state_;
        *left_back_target_physical_acceleration_ = target_physical_acceleration_state_;
        *right_back_target_physical_acceleration_ = target_physical_acceleration_state_;
        *right_front_target_physical_acceleration_ = target_physical_acceleration_state_;
    }

    void update_chassis_mode(
        rmcs_msgs::Switch right_switch, rmcs_msgs::Switch left_switch,
        const rmcs_msgs::Keyboard& keyboard) {

        auto mode = *mode_;
        if (left_switch != rmcs_msgs::Switch::DOWN) {
            if (last_right_switch_ == rmcs_msgs::Switch::MIDDLE
                && right_switch == rmcs_msgs::Switch::DOWN) {
                mode = (mode == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::AUTO
                                                              : rmcs_msgs::ChassisMode::SPIN;
                if (mode == rmcs_msgs::ChassisMode::SPIN)
                    spinning_forward_ = !spinning_forward_;
            } else if (!last_keyboard_.c && keyboard.c) {
                mode = (mode == rmcs_msgs::ChassisMode::SPIN) ? rmcs_msgs::ChassisMode::AUTO
                                                              : rmcs_msgs::ChassisMode::SPIN;
                if (mode == rmcs_msgs::ChassisMode::SPIN)
                    spinning_forward_ = !spinning_forward_;
            }
        }
        *mode_ = mode;
    }

    void update_velocity_control() {
        const Eigen::Vector2d translational_velocity = update_translational_velocity_control();
        const double angular_velocity = update_angular_velocity_control();
        chassis_control_velocity_->vector << translational_velocity, angular_velocity;
    }

    Eigen::Vector2d update_translational_velocity_control() {
        const auto keyboard = *keyboard_;
        const Eigen::Vector2d keyboard_move{keyboard.w - keyboard.s, keyboard.a - keyboard.d};

        Eigen::Vector2d translational_velocity = (*joystick_right_) + keyboard_move;

        if (translational_velocity.norm() > 1.0)
            translational_velocity.normalize();

        translational_velocity *= translational_velocity_max_;
        return translational_velocity;
    }

    double update_angular_velocity_control() {
        switch (*mode_) {
        case rmcs_msgs::ChassisMode::AUTO: return 0.0;

        case rmcs_msgs::ChassisMode::SPIN: {
            double ratio = spin_ratio_default_;

            const double rotary_knob = *rotary_knob_;
            if (std::isfinite(rotary_knob)) {
                if (rotary_knob >= -1.0 && rotary_knob <= 1.0)
                    ratio = std::abs(rotary_knob);
                else
                    ratio = std::clamp(std::abs(rotary_knob), 0.0, 1.0);
            }

            const double angular_velocity = ratio * angular_velocity_max_;
            return spinning_forward_ ? angular_velocity : -angular_velocity;
        }

        default: return 0.0;
        }
    }

    void update_lift_target_toggle(rmcs_msgs::Switch left_switch, rmcs_msgs::Switch right_switch) {
        const bool toggle_condition =
            (left_switch == rmcs_msgs::Switch::MIDDLE) && (right_switch == rmcs_msgs::Switch::UP);

        const bool last_toggle_condition = (last_lift_left_switch_ == rmcs_msgs::Switch::MIDDLE)
                                        && (last_lift_right_switch_ == rmcs_msgs::Switch::UP);

        if (toggle_condition && !last_toggle_condition) {
            current_target_angle_rad_ =
                (std::abs(current_target_angle_rad_ - max_angle_rad_) < 1e-6) ? min_angle_rad_
                                                                              : max_angle_rad_;
        }
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> right_switch_;
    InputInterface<rmcs_msgs::Switch> left_switch_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<double> rotary_knob_;
    InputInterface<double> left_front_joint_angle_;
    InputInterface<double> left_back_joint_angle_;
    InputInterface<double> right_back_joint_angle_;
    InputInterface<double> right_front_joint_angle_;
    InputInterface<double> left_front_joint_physical_angle_;
    InputInterface<double> left_back_joint_physical_angle_;
    InputInterface<double> right_back_joint_physical_angle_;
    InputInterface<double> right_front_joint_physical_angle_;

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    OutputInterface<double> left_front_target_angle_;
    OutputInterface<double> left_back_target_angle_;
    OutputInterface<double> right_back_target_angle_;
    OutputInterface<double> right_front_target_angle_;
    OutputInterface<double> left_front_target_physical_angle_;
    OutputInterface<double> left_back_target_physical_angle_;
    OutputInterface<double> right_back_target_physical_angle_;
    OutputInterface<double> right_front_target_physical_angle_;
    OutputInterface<double> left_front_target_physical_velocity_;
    OutputInterface<double> left_back_target_physical_velocity_;
    OutputInterface<double> right_back_target_physical_velocity_;
    OutputInterface<double> right_front_target_physical_velocity_;
    OutputInterface<double> left_front_target_physical_acceleration_;
    OutputInterface<double> left_back_target_physical_acceleration_;
    OutputInterface<double> right_back_target_physical_acceleration_;
    OutputInterface<double> right_front_target_physical_acceleration_;

    rmcs_msgs::Switch last_right_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_left_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool spinning_forward_ = true;
    bool joint_target_active_ = false;

    const double min_angle_rad_;
    const double max_angle_rad_;
    const double target_physical_velocity_limit_;
    const double target_physical_acceleration_limit_;
    double current_target_angle_rad_ = nan_;
    double target_physical_angle_state_ = nan_;
    double target_physical_velocity_state_ = 0.0;
    double target_physical_acceleration_state_ = 0.0;

    rmcs_msgs::Switch last_lift_left_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_lift_right_switch_ = rmcs_msgs::Switch::UNKNOWN;

    static constexpr double dt_ = 0.001;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisTestV2Controller, rmcs_executor::Component)
