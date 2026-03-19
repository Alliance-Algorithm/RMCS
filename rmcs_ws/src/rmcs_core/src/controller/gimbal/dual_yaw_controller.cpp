#include <cmath>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gimbal {

class DualYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DualYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , top_yaw_angle_pid_(
              get_parameter("top_yaw_angle_kp").as_double(),
              get_parameter("top_yaw_angle_ki").as_double(),
              get_parameter("top_yaw_angle_kd").as_double()) {
        register_input("/gimbal/top_yaw/angle", top_yaw_angle_);
        register_input("/gimbal/top_yaw/velocity", top_yaw_velocity_);
        register_input("/gimbal/bottom_yaw/angle", bottom_yaw_angle_);
        register_input("/gimbal/bottom_yaw/velocity", bottom_yaw_velocity_);

        register_output("/gimbal/top_yaw/control_velocity", top_yaw_control_velocity_, 0.0);

        register_output("/gimbal/yaw/angle", yaw_angle_, 0.0);
        register_output("/gimbal/yaw/velocity", yaw_velocity_, 0.0);
    }

    void update() override {
        const auto top_yaw_angle = std::remainder(*top_yaw_angle_, 2 * std::numbers::pi);
        *top_yaw_control_velocity_ = top_yaw_angle_pid_.update(-top_yaw_angle);

        double yaw_angle = *bottom_yaw_angle_;
        if (yaw_angle < 0)
            yaw_angle += 2 * std::numbers::pi;
        else if (yaw_angle > 2 * std::numbers::pi)
            yaw_angle -= 2 * std::numbers::pi;
        *yaw_angle_ = yaw_angle;

        *yaw_velocity_ = *bottom_yaw_velocity_;
    }

private:
    pid::PidCalculator top_yaw_angle_pid_;
    InputInterface<double> top_yaw_angle_, top_yaw_velocity_;
    InputInterface<double> bottom_yaw_angle_, bottom_yaw_velocity_;

    OutputInterface<double> top_yaw_control_velocity_;

    OutputInterface<double> yaw_angle_, yaw_velocity_;
};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::DualYawController, rmcs_executor::Component)
