
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>


namespace rmcs_core::controller::gimbal {

class SimpleYawController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleYawController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/yaw/target_velocity", yaw_target_velocity_);

        register_output("/gimbal/yaw/velocity_error", yaw_velocity_error, 0.0);

    }

    void update() override {
        *yaw_velocity_error = *yaw_target_velocity_ - *yaw_velocity_;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<double> yaw_velocity_;
    InputInterface<double> yaw_target_velocity_;
    OutputInterface<double> yaw_velocity_error;

};
} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gimbal::SimpleYawController, rmcs_executor::Component)