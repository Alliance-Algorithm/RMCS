#pragma once

#include <atomic>
#include <cmath>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <utility>

#include "test_controller/qos.hpp"

namespace controller {
namespace pid {

class PidNode : public rclcpp::Node {
public:
    explicit PidNode(
        const std::string& measurement_topic_name, const std::string& setpoint_topic_name,
        const std::string& control_topic_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true)) {

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

    static_assert(std::atomic<double>::is_always_lock_free);
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    double setpoint = nan;

    double kp = 0, ki = 0, kd = 0;
    double integral_min = -inf, integral_max = inf;
    double output_min = -inf, output_max = inf;

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
        auto msg = std::make_unique<std_msgs::msg::Float64>();

        if (std::isnan(setpoint)) {
            msg->data = nan;
        } else {
            double err = calculate_err(measurement->data);

            double control = kp * err + ki * err_integral_;
            err_integral_  = limit(err_integral_ + err, integral_min, integral_max);

            if (!std::isnan(last_err_))
                control += kd * (err - last_err_);
            last_err_ = err;

            msg->data = limit(control, output_min, output_max);
        }

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

} // namespace pid
} // namespace controller