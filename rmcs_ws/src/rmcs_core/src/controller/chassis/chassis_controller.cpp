#include <limits>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "rmcs_core/msgs.hpp"

namespace rmcs_core::controller::chassis {

class ChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/yaw/angle", gimbal_yaw_angle_);

        register_output(
            "/chassis/left_front_wheel/control_velocity", left_front_control_velocity_, nan);
        register_output(
            "/chassis/left_back_wheel/control_velocity", left_back_control_velocity_, nan);
        register_output(
            "/chassis/right_back_wheel/control_velocity", right_back_control_velocity_, nan);
        register_output(
            "/chassis/right_front_wheel/control_velocity", right_front_control_velocity_, nan);
    }

    void update() override {
        using namespace rmcs_core::msgs;

        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;

        do {
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_controls();
                break;
            }

            if (switch_left != Switch::DOWN) {
                if (switch_right == Switch::MIDDLE) {
                    spinning_mode_ |= keyboard.c;
                    spinning_mode_ &= !(keyboard.ctrl && keyboard.c);
                } else if (last_switch_right_ == Switch::MIDDLE && switch_right == Switch::DOWN) {
                    spinning_mode_ = !spinning_mode_;
                }
            }

            auto keyboard_move =
                Eigen::Vector2d{0.5 * (keyboard.w - keyboard.s), 0.5 * (keyboard.a - keyboard.d)};
            update_wheel_velocities(
                Eigen::Rotation2Dd{*gimbal_yaw_angle_} * (*joystick_right_ + keyboard_move));
        } while (false);

        last_switch_right_ = switch_right;
        last_switch_left_  = switch_left;
    }

    void reset_all_controls() {
        spinning_mode_ = false;

        *left_front_control_velocity_  = nan;
        *left_back_control_velocity_   = nan;
        *right_back_control_velocity_  = nan;
        *right_front_control_velocity_ = nan;
    }

    void update_wheel_velocities(Eigen::Vector2d move) {
        constexpr double velocity_limit = 60;

        double right_oblique = velocity_limit * (-move.y() * cos_45 + move.x() * sin_45);
        double left_oblique  = velocity_limit * (move.x() * cos_45 + move.y() * sin_45);

        double spinning_velocity = spinning_mode_ ? 0.8 : 0.0;

        double velocities[4] = {right_oblique, left_oblique, -right_oblique, -left_oblique};
        double max_velocity  = 0;

        for (auto& velocity : velocities) {
            velocity += 0.4 * velocity_limit * spinning_velocity;
            max_velocity = std::max(std::abs(velocity), max_velocity);
        }
        if (max_velocity > velocity_limit) {
            double scale = velocity_limit / max_velocity;
            for (auto& velocity : velocities)
                velocity *= scale;
        }

        *left_front_control_velocity_  = velocities[0];
        *left_back_control_velocity_   = velocities[1];
        *right_back_control_velocity_  = velocities[2];
        *right_front_control_velocity_ = velocities[3];
    }

private:
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    // Since sine and cosine function are not constexpr, we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_core::msgs::Switch> switch_right_;
    InputInterface<rmcs_core::msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_core::msgs::Mouse> mouse_;
    InputInterface<rmcs_core::msgs::Keyboard> keyboard_;

    InputInterface<double> gimbal_yaw_angle_;

    rmcs_core::msgs::Switch last_switch_right_ = rmcs_core::msgs::Switch::UNKNOWN;
    rmcs_core::msgs::Switch last_switch_left_  = rmcs_core::msgs::Switch::UNKNOWN;

    bool spinning_mode_;

    OutputInterface<double> left_front_control_velocity_;
    OutputInterface<double> left_back_control_velocity_;
    OutputInterface<double> right_back_control_velocity_;
    OutputInterface<double> right_front_control_velocity_;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::ChassisController, rmcs_executor::Component)