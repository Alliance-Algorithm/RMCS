#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>

#include "test_controller/usb_cdc_forwarder/qos.hpp"

namespace chassis_controller {
namespace omni {

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode()
        : Node("omni_chassis_controller", rclcpp::NodeOptions().use_intra_process_comms(true)) {

        remote_control_subscription_ = this->create_subscription<rm_msgs::msg::RemoteControl>(
            "/remote_control", usb_cdc_forwarder::kSensorQoS,
            std::bind(&ControllerNode::remote_control_callback, this, std::placeholders::_1));

        right_front_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/right_front/control_velocity", usb_cdc_forwarder::kControlQoS);
        left_front_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/left_front/control_velocity", usb_cdc_forwarder::kControlQoS);
        left_back_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/left_back/control_velocity", usb_cdc_forwarder::kControlQoS);
        right_back_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/right_back/control_velocity", usb_cdc_forwarder::kControlQoS);

        using namespace std::chrono_literals;
        remote_control_watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&ControllerNode::remote_control_watchdog_callback, this));
        remote_control_watchdog_timer_->cancel();
    }

private:
    void remote_control_callback(rm_msgs::msg::RemoteControl::SharedPtr msg) {
        constexpr double velocity_limit = 800;

        auto x = msg->channel_right_x;
        auto y = msg->channel_right_y;

        double right_oblique = velocity_limit * (x * cos_45 + y * sin_45);
        double left_oblique  = velocity_limit * (y * cos_45 - x * sin_45);

        auto turn = msg->channel_left_x;

        double velocities[4] = {-left_oblique, right_oblique, left_oblique, -right_oblique};
        double max_velocity  = 0;
        for (auto& velocity : velocities) {
            velocity += 0.4 * velocity_limit * turn;
            max_velocity = std::max(std::abs(velocity), max_velocity);
        }
        if (max_velocity > velocity_limit) {
            double scale = velocity_limit / max_velocity;
            for (auto& velocity : velocities)
                velocity *= scale;
        }

        // RCLCPP_INFO(
        //     this->get_logger(), "%f, %f, %f, %f", velocities[0], velocities[1], velocities[2],
        //     velocities[3]);

        publish_control_velocities(velocities[0], velocities[1], velocities[2], velocities[3]);

        remote_control_watchdog_timer_->reset();
    }

    void remote_control_watchdog_callback() {
        remote_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Remote control message timeout, will reset wheel velocities.");
        publish_control_velocities(0, 0, 0, 0);
    }

    void publish_control_velocities(
        double right_front, double left_front, double left_back, double right_back) {
        std::unique_ptr<std_msgs::msg::Float64> velocity;

        velocity       = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = right_front;
        right_front_control_velocity_publisher_->publish(std::move(velocity));

        velocity       = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = left_front;
        left_front_control_velocity_publisher_->publish(std::move(velocity));

        velocity       = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = left_back;
        left_back_control_velocity_publisher_->publish(std::move(velocity));

        velocity       = std::make_unique<std_msgs::msg::Float64>();
        velocity->data = right_back;
        right_back_control_velocity_publisher_->publish(std::move(velocity));
    }

    // The sine and cosine functions are not constexprs, so we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    rclcpp::Subscription<rm_msgs::msg::RemoteControl>::SharedPtr remote_control_subscription_;
    rclcpp::TimerBase::SharedPtr remote_control_watchdog_timer_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_back_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_back_control_velocity_publisher_;
};

} // namespace omni
} // namespace chassis_controller