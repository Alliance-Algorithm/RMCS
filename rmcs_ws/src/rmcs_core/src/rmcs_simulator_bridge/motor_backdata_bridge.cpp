#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rmcs_executor/component.hpp>
namespace rmcs_core::simulator {

class MotorBackdataBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorBackdataBridge()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_output(get_parameter("simulated_motor_angle").as_string(), simulated_motor_angle_, 0);
        register_output(get_parameter("simulated_motor_velocity").as_string(), simulated_motor_velocity_, 0);
        register_output(get_parameter("simulated_motor_torque").as_string(), simulated_motor_torque_, 0);

        subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "motor_simulator_data_output", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
               *simulated_motor_angle_ = msg->x;
                *simulated_motor_velocity_ = msg->y;
                *simulated_motor_torque_ = msg->z;
                RCLCPP_INFO(get_logger(), "电机速度：%lf", *simulated_motor_velocity_);
                RCLCPP_INFO(get_logger(), "电机角度：%lf", *simulated_motor_angle_);
            });

        
    }

    void update() override {}

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    OutputInterface<double> simulated_motor_angle_;
    OutputInterface<double> simulated_motor_velocity_;
    OutputInterface<double> simulated_motor_torque_;
    double * controlled_angle;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription;
};
} // namespace rmcs_core::simulator

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::simulator::MotorBackdataBridge, rmcs_executor::Component)