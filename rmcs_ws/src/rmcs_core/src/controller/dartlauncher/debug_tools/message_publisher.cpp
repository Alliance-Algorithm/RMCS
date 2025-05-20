#include <chrono>
#include <cmath>
#include <cstddef>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace rmcs_core::controller::dartlauncher {

class MotorDataPublisher
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorDataPublisher()
        : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/dart/first_left_friction/velocity", input_friction_velocity_[0]);
        register_input("/dart/first_right_friction/velocity", input_friction_velocity_[1]);
        register_input("/dart/second_left_friction/velocity", input_friction_velocity_[2]);
        register_input("/dart/second_right_friction/velocity", input_friction_velocity_[3]);

        register_input("/dart/first_left_friction/filtered_velocity", input_friction_filter_[0]);
        register_input("/dart/first_right_friction/filtered_velocity", input_friction_filter_[1]);
        register_input("/dart/second_left_friction/filtered_velocity", input_friction_filter_[2]);
        register_input("/dart/second_right_friction/filtered_velocity", input_friction_filter_[3]);

        register_input("/dart/pitch_left/velocity", input_pitch_data_[0]);
        register_input("/dart/pitch_right/velocity", input_pitch_data_[1]);

        pub_velocity_[0] = this->create_publisher<std_msgs::msg::String>("first_left_friction_velocity", 10);
        pub_velocity_[1] = this->create_publisher<std_msgs::msg::String>("first_right_friction_velocity", 10);
        pub_velocity_[2] = this->create_publisher<std_msgs::msg::String>("second_left_friction_velocity", 10);
        pub_velocity_[3] = this->create_publisher<std_msgs::msg::String>("second_right_friction_velocity", 10);

        pub_filter_[0] = this->create_publisher<std_msgs::msg::String>("first_left_friction_filter", 10);
        pub_filter_[1] = this->create_publisher<std_msgs::msg::String>("first_right_friction_filter", 10);
        pub_filter_[2] = this->create_publisher<std_msgs::msg::String>("second_left_friction_filter", 10);
        pub_filter_[3] = this->create_publisher<std_msgs::msg::String>("second_right_friction_filter", 10);

        pub_pitch_data_[0] = this->create_publisher<std_msgs::msg::String>("pitch_left_velocity", 10);
        pub_pitch_data_[1] = this->create_publisher<std_msgs::msg::String>("pitch_right_velocity", 10);
        pub_pitch_data_[2] = this->create_publisher<std_msgs::msg::String>("pitch_left_position", 10);
        pub_pitch_data_[3] = this->create_publisher<std_msgs::msg::String>("pitch_right_position", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() { this->publish_message(); });
    }

    void update() override {
        for (size_t i = 0; i < 4; i++) {
            msg_friction_velocity_[i].data = std::to_string(*input_friction_velocity_[i]);
            msg_friction_filter_[i].data   = std::to_string(*input_friction_filter_[i]);
        }

        msg_pitch_data_[0].data = std::to_string(*input_pitch_data_[0]);
        msg_pitch_data_[1].data = std::to_string(*input_pitch_data_[1]);
        pitch_velocity_integral_[0] += *input_pitch_data_[0];
        pitch_velocity_integral_[1] += *input_pitch_data_[1];
        msg_pitch_data_[2].data = std::to_string(pitch_velocity_integral_[0]);
        msg_pitch_data_[3].data = std::to_string(pitch_velocity_integral_[1]);
    }

private:
    void publish_message() {
        for (size_t i = 0; i < 4; i++) {
            pub_velocity_[i]->publish(msg_friction_velocity_[i]);
            pub_filter_[i]->publish(msg_friction_filter_[i]);
            pub_pitch_data_[i]->publish(msg_pitch_data_[i]);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_velocity_[4];
    InputInterface<double> input_friction_velocity_[4];
    std_msgs::msg::String msg_friction_velocity_[4];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_filter_[4];
    InputInterface<double> input_friction_filter_[4];
    std_msgs::msg::String msg_friction_filter_[4];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_pitch_data_[4];
    InputInterface<double> input_pitch_data_[2];
    double pitch_velocity_integral_[2] = {0, 0};
    std_msgs::msg::String msg_pitch_data_[4];
};
} // namespace rmcs_core::controller::dartlauncher

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dartlauncher::MotorDataPublisher, rmcs_executor::Component)
