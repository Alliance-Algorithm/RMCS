#pragma once

#include <cstddef>
#include <iostream>
#include <memory>
#include <queue>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>

#include "test_controller/usb_cdc_forwarder/qos.hpp"

namespace filter {

class MeanFilterNode : public rclcpp::Node {
public:
    MeanFilterNode(
        int queue_size, int publish_freq, const std::string& input_name,
        const std::string& output_name, const std::string& node_name)
        : Node(node_name, rclcpp::NodeOptions().use_intra_process_comms(true))
        , queue_size_(queue_size) {

        input_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            input_name, usb_cdc_forwarder::kControlQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { input_value_ = msg->data; });
        output_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            output_name, usb_cdc_forwarder::kControlQoS);

        using namespace std::chrono_literals;
        auto period    = std::chrono::nanoseconds(1'000'000'000 / publish_freq);
        publish_timer_ = this->create_wall_timer(1ms, [this]() { this->update(); });
    }

private:
    void update() {
        queue_.push_back(input_value_);
        while (queue_.size() > queue_size_)
            queue_.erase(queue_.begin());

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

    double input_value_;

    std::vector<double> queue_;
    int queue_size_;
};

} // namespace filter