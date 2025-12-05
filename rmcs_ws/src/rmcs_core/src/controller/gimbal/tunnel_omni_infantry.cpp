#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node.hpp>

#include <eigen3/Eigen/Geometry>

namespace rmcs_core {

class TunnelOmniInfantryGimbalController : rmcs_executor::Component {
public:
    TunnelOmniInfantryGimbalController() {
        {
            register_input("/remote/joystick/left", input.joystick_left);
            register_input("/remote/switch/right", input.switch_right);
            register_input("/remote/switch/left", input.switch_left);
            register_input("/remote/mouse/velocity", input.mouse_velocity);
            register_input("/remote/mouse", input.mouse_event);

            register_input("/gimbal/pitch/angle", input.gimbal_pitch_angle);

            register_input("/gimbal/shooter/mode", input.shoot_mode);
        }
    }

    auto update() -> void override {}

private:
    constexpr static auto kNan = std::numeric_limits<double>::quiet_NaN();

    rmcs_util::RclcppNode rclcpp{Component::get_component_name()};

    struct Input {
        InputInterface<double> gimbal_pitch_angle;
        InputInterface<Eigen::Vector2d> joystick_left;

        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse_event;

        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;

        InputInterface<rmcs_msgs::ShootMode> shoot_mode;

        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse_event);

            component.register_input("/gimbal/pitch/angle", gimbal_pitch_angle);

            component.register_input("/gimbal/shooter/mode", shoot_mode);
        }
    } input{*this};

    struct Output {
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;

        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNan);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNan);
        }
    } output{*this};
};

} // namespace rmcs_core
