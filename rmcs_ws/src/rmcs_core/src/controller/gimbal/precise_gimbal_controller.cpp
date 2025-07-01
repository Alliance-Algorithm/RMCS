#include <limits>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class PreciseGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PreciseGimbalController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_output("/gimbal/top_yaw/control_angle_shift", control_top_yaw_angle_shift_, nan_);
        register_output(
            "/gimbal/bottom_yaw/control_angle_shift", control_bottom_yaw_angle_shift_, nan_);
        register_output("/gimbal/pitch/control_angle_shift", control_pitch_angle_shift_, nan_);
    }

    void update() override {
        const auto joystick_left = *joystick_left_;
        const auto switch_right  = *switch_right_;
        const auto switch_left   = *switch_left_;
        // const auto mouse         = *mouse_;
        // const auto keyboard     = *keyboard_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        auto joystick_sensitivity = 0.006 / 16;
        auto mouse_sensitivity    = 0.5 / 4;

        *control_top_yaw_angle_shift_ = 0.0;
        *control_bottom_yaw_angle_shift_ =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        *control_pitch_angle_shift_ =
            -joystick_sensitivity * joystick_left_->x() - mouse_sensitivity * mouse_velocity_->x();
    }

private:
    void reset_all_controls() {
        *control_top_yaw_angle_shift_    = nan_;
        *control_bottom_yaw_angle_shift_ = nan_;
        *control_pitch_angle_shift_      = nan_;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<double> control_top_yaw_angle_shift_;
    OutputInterface<double> control_bottom_yaw_angle_shift_;
    OutputInterface<double> control_pitch_angle_shift_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::PreciseGimbalController, rmcs_executor::Component)