#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::navigation {

class Navigation
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Navigation()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // Manual setpoint for navigation command:
        // [vx (m/s), vy (m/s), yaw_rate (rad/s)].
        // Edit this vector directly when tuning navigation behavior.
        command_velocity_setpoint_ << 0.0, 0.0, 0.0;
        register_output(
            "/rmcs_navigation/command_velocity", command_velocity_, Eigen::Vector3d::Zero());
    }

    void update() override { *command_velocity_ = command_velocity_setpoint_; }

private:
    Eigen::Vector3d command_velocity_setpoint_ = Eigen::Vector3d::Zero();
    OutputInterface<Eigen::Vector3d> command_velocity_;
};

} // namespace rmcs_core::navigation

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::navigation::Navigation, rmcs_executor::Component)
