#pragma once

#include <cmath>
#include <functional>
#include <memory>
#include <numbers>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>

#include <eigen3/Eigen/Dense>

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

        gimbal_yaw_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/yaw/angle", usb_cdc_forwarder::kSensorQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { gimbal_yaw_ = msg->data; });
        gimbal_control_yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/yaw/control_angle", usb_cdc_forwarder::kControlQoS);

        gimbal_imu_pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/pitch/angle_imu", usb_cdc_forwarder::kSensorQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { gimbal_imu_pitch_ = msg->data; });
        gimbal_pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/pitch/angle", usb_cdc_forwarder::kSensorQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) {
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
        gimbal_control_pitch_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/pitch/control_angle", usb_cdc_forwarder::kControlQoS);

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

        auto rotation = Eigen::Rotation2Dd{gimbal_yaw_ - 1.03};
        Eigen::Vector2d channel =
            rotation * Eigen::Vector2d{msg->channel_right_x, msg->channel_right_y};
        // std::cout << channel.x() << ' ' << channel.y() << '\n';

        double right_oblique = velocity_limit * (channel.x() * cos_45 + channel.y() * sin_45);
        double left_oblique  = velocity_limit * (channel.y() * cos_45 - channel.x() * sin_45);

        auto turn = 0.5; // msg->channel_left_x;

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
        publish_control_velocities(velocities[0], velocities[1], velocities[2], velocities[3]);

        auto control_yaw_msg = std::make_unique<std_msgs::msg::Float64>();
        gimbal_control_yaw_ += -0.08 * msg->channel_left_x;
        // gimbal_control_yaw_   = fmod(gimbal_control_yaw_, std::numbers::pi);
        control_yaw_msg->data = gimbal_control_yaw_;
        gimbal_control_yaw_publisher_->publish(std::move(control_yaw_msg));

        auto control_pitch_msg = std::make_unique<std_msgs::msg::Float64>();
        gimbal_control_pitch_ += -0.04 * msg->channel_left_y;
        if (gimbal_control_pitch_ < gimbal_imu_pitch_min_)
            gimbal_control_pitch_ = gimbal_imu_pitch_min_;
        else if (gimbal_control_pitch_ > gimbal_imu_pitch_max_)
            gimbal_control_pitch_ = gimbal_imu_pitch_max_;
        control_pitch_msg->data = gimbal_control_pitch_;
        gimbal_control_pitch_publisher_->publish(std::move(control_pitch_msg));

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

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_yaw_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_yaw_publisher_;
    double gimbal_yaw_ = 0, gimbal_control_yaw_ = 0;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_pitch_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_imu_pitch_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_pitch_publisher_;
    double gimbal_imu_pitch_ = 0, gimbal_control_pitch_ = 0;
    double gimbal_imu_pitch_min_ = 0, gimbal_imu_pitch_max_ = 0;
    const double gimbal_pitch_min_ = 2.7, gimbal_pitch_max_ = 3.6;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_back_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_back_control_velocity_publisher_;
};

} // namespace omni
} // namespace chassis_controller