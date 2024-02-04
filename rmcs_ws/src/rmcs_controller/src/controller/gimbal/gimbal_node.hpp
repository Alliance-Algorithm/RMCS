#pragma once

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmcs_controller/qos.hpp"

namespace controller {
namespace gimbal {

class GimbalNode : public rclcpp::Node {
public:
    GimbalNode()
        : Node("gimbal_controller", rclcpp::NodeOptions().use_intra_process_comms(true)) {

        remote_control_subscription_ = this->create_subscription<rm_msgs::msg::RemoteControl>(
            "/remote_control", kCoreQoS,
            std::bind(&GimbalNode::remote_control_callback, this, std::placeholders::_1));

        gimbal_imu_yaw_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/yaw/angle_imu", kCoreQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { gimbal_imu_yaw_ = msg->data; });
        gimbal_control_yaw_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("/gimbal/yaw/control_angle", kCoreQoS);

        gimbal_imu_pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/pitch/angle_imu", kCoreQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { gimbal_imu_pitch_ = msg->data; });
        gimbal_pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/pitch/angle", kCoreQoS, [this](std_msgs::msg::Float64::UniquePtr msg) {
                using namespace std::numbers;
                double pitch = msg->data;
                double diff  = gimbal_imu_pitch_ - pitch;
                double min   = fmod(diff + gimbal_pitch_min_, 2 * pi);
                double max   = fmod(diff + gimbal_pitch_max_, 2 * pi);
                if (min < -pi / 2 || min > pi / 2)
                    min = -pi / 2;
                if (max < -pi / 2 || max > pi / 2)
                    max = pi / 2;
                if (min <= max) {
                    gimbal_imu_pitch_min_ = min;
                    gimbal_imu_pitch_max_ = max;
                } else {
                    gimbal_imu_pitch_min_ = gimbal_imu_pitch_max_ = gimbal_imu_pitch_;
                }
            });
        gimbal_control_pitch_publisher_ =
            this->create_publisher<std_msgs::msg::Float64>("/gimbal/pitch/control_angle", kCoreQoS);

        using namespace std::chrono_literals;
        remote_control_watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&GimbalNode::remote_control_watchdog_callback, this));
        remote_control_watchdog_timer_->cancel();
    }

private:
    void remote_control_callback(rm_msgs::msg::RemoteControl::SharedPtr msg) {
        remote_control_watchdog_timer_->reset();

        if (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN
            && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            gimbal_control_yaw_ = gimbal_control_pitch_ = nan;
            publish_control_angles();
            return;
        }

        if (std::isnan(gimbal_control_yaw_) || std::isnan(gimbal_control_pitch_)) {
            gimbal_control_yaw_   = gimbal_imu_yaw_;
            gimbal_control_pitch_ = 0;
        }

        gimbal_control_yaw_ += -0.08 * msg->channel_left_x;

        gimbal_control_pitch_ += -0.04 * msg->channel_left_y;
        if (gimbal_control_pitch_ < gimbal_imu_pitch_min_)
            gimbal_control_pitch_ = gimbal_imu_pitch_min_;
        else if (gimbal_control_pitch_ > gimbal_imu_pitch_max_)
            gimbal_control_pitch_ = gimbal_imu_pitch_max_;

        publish_control_angles();
    }

    void remote_control_watchdog_callback() {
        remote_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Remote control message timeout, will disable gimbal control.");
        gimbal_control_yaw_ = gimbal_control_pitch_ = nan;
        publish_control_angles();
    }

    void publish_control_angles() {
        std::unique_ptr<std_msgs::msg::Float64> msg;

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = gimbal_control_yaw_;
        gimbal_control_yaw_publisher_->publish(std::move(msg));

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = gimbal_control_pitch_;
        gimbal_control_pitch_publisher_->publish(std::move(msg));
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    rclcpp::Subscription<rm_msgs::msg::RemoteControl>::SharedPtr remote_control_subscription_;
    rclcpp::TimerBase::SharedPtr remote_control_watchdog_timer_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_imu_yaw_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_yaw_publisher_;
    double gimbal_imu_yaw_ = 0, gimbal_control_yaw_ = nan;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_pitch_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_imu_pitch_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_pitch_publisher_;
    double gimbal_imu_pitch_ = 0, gimbal_control_pitch_ = nan;
    double gimbal_imu_pitch_min_ = 0, gimbal_imu_pitch_max_ = 0;
    const double gimbal_pitch_min_ = 2.7, gimbal_pitch_max_ = 3.6;
};

} // namespace gimbal
} // namespace controller