#pragma once

#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <utility>

#include "controller/pid/pid_calculator.hpp"
#include "rmcs_controller/qos.hpp"

namespace controller {
namespace pid {

class PidNode
    : public rclcpp::Node
    , public PidCalculator {
public:
    PidNode(
        const std::string& measurement_topic_name, const std::string& setpoint_topic_name,
        const std::string& control_topic_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        , PidCalculator() {

        control_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(control_topic_name, kCoreQoS);

        measurement_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            measurement_topic_name, kCoreQoS,
            std::bind(&PidNode::measurement_subscription_callback, this, std::placeholders::_1));
        setpoint_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            setpoint_topic_name, kCoreQoS,
            std::bind(&PidNode::setpoint_subscription_callback, this, std::placeholders::_1));
    }

    virtual ~PidNode() = default;

    double setpoint = nan;

protected:
    virtual double calculate_err(double measurement) { return setpoint - measurement; }

private:
    void measurement_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> msg) {
        msg->data = update(calculate_err(msg->data));
        control_publisher_->publish(std::move(msg));
    }

    void setpoint_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> msg) {
        setpoint = msg->data;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measurement_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
};

} // namespace pid
} // namespace controller