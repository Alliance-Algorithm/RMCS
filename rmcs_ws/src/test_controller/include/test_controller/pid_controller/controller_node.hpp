#pragma once

#include <atomic>
#include <cmath>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace pid_controller {

inline const rclcpp::QoS kSensorQoS = rclcpp::QoS(1).best_effort().durability_volatile();
inline const rclcpp::QoS kControlQoS = rclcpp::QoS(1).best_effort().transient_local();

class ControllerNode : public rclcpp::Node {
public:
    explicit ControllerNode(
        const std::string& measurement_topic_name, const std::string& setpoint_topic_name,
        const std::string& control_topic_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        , measurement_topic_name_(measurement_topic_name)
        , setpoint_topic_name_(setpoint_topic_name) {
        control_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>(control_topic_name, kControlQoS);
    }

    virtual ~ControllerNode() = default;

    void on_subscribe() {
        measurement_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            measurement_topic_name_, kSensorQoS,
            std::bind(
                &ControllerNode::measurement_subscription_callback, this, std::placeholders::_1));
        // setpoint_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        //     setpoint_topic_name_, kControlQoS,
        //     std::bind(
        //         &ControllerNode::setpoint_subscription_callback, this, std::placeholders::_1));
    }

    static_assert(std::atomic<double>::is_always_lock_free);
    static constexpr double inf = std::numeric_limits<double>::infinity();
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

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

    virtual double calculate_err(double measurement) { return setpoint_ - measurement; }

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

    void setpoint_subscription_callback(std::unique_ptr<std_msgs::msg::Float64> setpoint) {
        setpoint_ = setpoint->data;
    }

    std::atomic<double> setpoint_ = 0;
    double last_err_ = nan, err_integral_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_publisher_;

    std::string measurement_topic_name_, setpoint_topic_name_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr measurement_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr setpoint_subscription_;
};

} // namespace pid_controller