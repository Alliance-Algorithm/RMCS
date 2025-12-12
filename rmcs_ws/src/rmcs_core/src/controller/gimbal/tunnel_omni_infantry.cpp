#include "description/tf/tunnel_omni_infantry.hpp"

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/shoot_mode.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node.hpp>

#include <eigen3/Eigen/Geometry>

namespace rmcs_core::tunnel_omni {

struct GimbalController : public rmcs_executor::Component {

    constexpr static auto kNan = std::numeric_limits<double>::quiet_NaN();

    rmcs_util::RclcppNode rclcpp{Component::get_component_name()};

    struct Config {
        double yaw_upper_limit = 0.;
        double yaw_lower_limit = 0.;

        explicit Config(rmcs_util::RclcppNode& rclcpp) {
            const auto& params = rclcpp.params();

            yaw_upper_limit = params->get_double("yaw_upper_limit");
            yaw_lower_limit = params->get_double("yaw_lower_limit");

            // ...
        }
    } config{rclcpp};

    struct ComponentInput {
        InputInterface<double> gimbal_pitch_angle;
        InputInterface<Eigen::Vector2d> joystick_left;

        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse_event;

        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;

        InputInterface<rmcs_msgs::ShootMode> shoot_mode;

        explicit ComponentInput(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse_event);

            component.register_input("/gimbal/pitch/angle", gimbal_pitch_angle);

            component.register_input("/gimbal/shooter/mode", shoot_mode);
        }
    } input{*this};

    struct ComponentOutput {
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;

        explicit ComponentOutput(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNan);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNan);
        }
    } output{*this};

    GimbalController() {
        std::ignore = config.yaw_upper_limit;
        std::ignore = config.yaw_lower_limit;
    }

    auto update() -> void override {
        {
            using Tf = TunnelOmniInfantryTf;
            std::ignore = Tf::look_up<"pitch_link", "yaw_link", Eigen::Quaterniond>();
        }
    }
};

} // namespace rmcs_core::tunnel_omni
