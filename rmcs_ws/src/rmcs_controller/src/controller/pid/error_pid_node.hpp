#pragma once

#include <cmath>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "rmcs_controller/qos.hpp"

namespace controller {
namespace pid {

class ErrorPidNode
    : public rclcpp::Node
    , public PidCalculator {
public:
    ErrorPidNode(
        const std::string& error_topic_name, const std::string& control_topic_name,
        const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        , PidCalculator() {

        control_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(control_topic_name, kCoreQoS);

        error_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            error_topic_name, kCoreQoS,
            std::bind(&ErrorPidNode::error_subscription_callback, this, std::placeholders::_1));
    }

private:
    void error_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> msg) {
        msg->data = update(msg->data);
        control_publisher_->publish(std::move(msg));
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr error_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
};

} // namespace pid
} // namespace controller