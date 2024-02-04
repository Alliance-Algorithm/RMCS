#pragma once

#include <cmath>
#include <deque>
#include <limits>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmcs_controller/qos.hpp"

namespace filter {

class MeanFilterNode : public rclcpp::Node {
public:
    MeanFilterNode(
        size_t _filter_size, int publish_freq, const std::string& input_name,
        const std::string& output_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        , filter_size(_filter_size) {

        input_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            input_name, kCoreQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { input_value_ = msg->data; });
        output_publisher_ = this->create_publisher<std_msgs::msg::Float64>(output_name, kCoreQoS);

        using namespace std::chrono_literals;
        auto period    = std::chrono::nanoseconds(1'000'000'000 / publish_freq);
        publish_timer_ = this->create_wall_timer(period, [this]() { this->update(); });
    }

    size_t filter_size;

private:
    void update() {
        queue_.push_back(input_value_);
        while (queue_.size() > filter_size)
            queue_.pop_front();

        double sum = 0;
        for (auto& item : queue_)
            sum += item;

        auto msg  = std::make_unique<std_msgs::msg::Float64>();
        msg->data = sum / queue_.size();
        output_publisher_->publish(std::move(msg));
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr output_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    double input_value_ = std::numeric_limits<double>::quiet_NaN();

    std::deque<double> queue_;
};

} // namespace filter