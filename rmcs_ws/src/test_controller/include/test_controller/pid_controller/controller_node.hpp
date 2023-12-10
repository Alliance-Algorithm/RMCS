#pragma once

#include "test_controller/usb_cdc_forwarder/qos.hpp"
#include <atomic>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace pid_controller {

class ControllerNode : public rclcpp::Node {
public:
    explicit ControllerNode(
        const std::string& measurement_topic_name, const std::string& setpoint_topic_name,
        const std::string& control_topic_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {

        control_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            control_topic_name, usb_cdc_forwarder::kControlQoS);

        measurement_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            measurement_topic_name, usb_cdc_forwarder::kSensorQoS,
            std::bind(
                &ControllerNode::measurement_subscription_callback, this, std::placeholders::_1));
        setpoint_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            setpoint_topic_name, usb_cdc_forwarder::kControlQoS,
            std::bind(
                &ControllerNode::setpoint_subscription_callback, this, std::placeholders::_1));
    }

    virtual ~ControllerNode() = default;

    static_assert(std::atomic<double>::is_always_lock_free);
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    std::atomic<double> setpoint = 0;

    std::atomic<double> kp = 0, ki = 0, kd = 0;
    std::atomic<double> integral_min = -inf, integral_max = inf;
    std::atomic<double> output_min = -inf, output_max = inf;

protected:
    double limit(double value, double min, double max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    virtual double calculate_err(double measurement) { return setpoint - measurement; }

private:
    void measurement_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> measurement) {
        double err = calculate_err(measurement->data);

        double control = kp * err + ki * err_integral_;
        err_integral_  = limit(err_integral_ + err, integral_min, integral_max);

        if (!std::isnan(last_err_))
            control += kd * (err - last_err_);
        last_err_ = err;

        auto msg  = std::make_unique<std_msgs::msg::Float64>();
        msg->data = limit(control, output_min, output_max);
        control_publisher_->publish(std::move(msg));
    }

    void setpoint_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> msg) {
        setpoint = msg->data;
    }

    double last_err_ = nan, err_integral_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measurement_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
};

} // namespace pid_controller