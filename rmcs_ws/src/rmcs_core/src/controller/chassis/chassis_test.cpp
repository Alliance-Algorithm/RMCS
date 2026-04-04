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
        , max_angle_rad_(deg_to_rad_(get_parameter_or("max_angle", 35.0))) {

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

        register_output("/chassis/control_mode", mode_);
        register_output("/chassis/control_velocity", chassis_control_velocity_);

        register_output("/chassis/left_front_joint/target_angle", left_front_target_angle_, nan_);
        register_output("/chassis/left_back_joint/target_angle", left_back_target_angle_, nan_);
        register_output("/chassis/right_back_joint/target_angle", right_back_target_angle_, nan_);
        register_output("/chassis/right_front_joint/target_angle", right_front_target_angle_, nan_);

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

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    void reset_all_controls() {
        *mode_ = rmcs_msgs::ChassisMode::AUTO;
        chassis_control_velocity_->vector << nan_, nan_, nan_;

        *left_front_target_angle_ = nan_;
        *left_back_target_angle_ = nan_;
        *right_back_target_angle_ = nan_;
        *right_front_target_angle_ = nan_;

        joint_target_active_ = false;
    }

    bool publish_current_joint_target_angles() {
        const Eigen::Vector4d current_angles{
            *left_front_joint_angle_, *left_back_joint_angle_, *right_back_joint_angle_,
            *right_front_joint_angle_};
        if (!current_angles.array().isFinite().all()) {
            return false;
        }

        *left_front_target_angle_ = current_angles[0];
        *left_back_target_angle_ = current_angles[1];
        *right_back_target_angle_ = current_angles[2];
        *right_front_target_angle_ = current_angles[3];
        current_target_angle_rad_ = current_angles.mean();
        joint_target_active_ = true;
        return true;
    }

    void publish_joint_target_angles() {
        if (!joint_target_active_) {
            *left_front_target_angle_ = nan_;
            *left_back_target_angle_ = nan_;
            *right_back_target_angle_ = nan_;
            *right_front_target_angle_ = nan_;
            return;
        }

        *left_front_target_angle_ = current_target_angle_rad_;
        *left_back_target_angle_ = current_target_angle_rad_;
        *right_back_target_angle_ = current_target_angle_rad_;
        *right_front_target_angle_ = current_target_angle_rad_;
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

    OutputInterface<rmcs_msgs::ChassisMode> mode_;
    OutputInterface<rmcs_description::BaseLink::DirectionVector> chassis_control_velocity_;

    OutputInterface<double> left_front_target_angle_;
    OutputInterface<double> left_back_target_angle_;
    OutputInterface<double> right_back_target_angle_;
    OutputInterface<double> right_front_target_angle_;

    rmcs_msgs::Switch last_right_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_left_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    bool spinning_forward_ = true;
    bool joint_target_active_ = false;

    const double min_angle_rad_;
    const double max_angle_rad_;
    double current_target_angle_rad_ = nan_;

    rmcs_msgs::Switch last_lift_left_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Switch last_lift_right_switch_ = rmcs_msgs::Switch::UNKNOWN;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ChassisTestV2Controller, rmcs_executor::Component)
