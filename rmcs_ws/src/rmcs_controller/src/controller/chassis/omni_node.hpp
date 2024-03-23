#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <numbers>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rmcs_controller/qos.hpp"
#include "rmcs_controller/type.hpp"

namespace controller {
namespace chassis {

class OmniNode : public rclcpp::Node {
public:
    OmniNode()
        : Node("omni_chassis_controller", rclcpp::NodeOptions().use_intra_process_comms(true)) {

        remote_control_subscription_ = this->create_subscription<rm_msgs::msg::RemoteControl>(
            "/remote_control", kCoreQoS,
            std::bind(&OmniNode::remote_control_callback, this, std::placeholders::_1));

        gimbal_yaw_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/yaw/angle", kCoreQoS,
            [this](std_msgs::msg::Float64::UniquePtr msg) { gimbal_yaw_ = msg->data; });

        right_front_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/right_front/control_velocity", kCoreQoS);
        left_front_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/left_front/control_velocity", kCoreQoS);
        left_back_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/left_back/control_velocity", kCoreQoS);
        right_back_control_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/chassis_wheel/right_back/control_velocity", kCoreQoS);

        using namespace std::chrono_literals;

        remote_control_watchdog_timer_ = this->create_wall_timer(
            500ms, std::bind(&OmniNode::remote_control_watchdog_callback, this));
        remote_control_watchdog_timer_->cancel();
    }

private:
    void remote_control_callback(rm_msgs::msg::RemoteControl::SharedPtr msg) {

        remote_control_watchdog_timer_->reset();

        auto keys = reinterpret_cast<KeyboardType*>(&msg->keyboard);

        if (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN
            && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            spinning_mode_     = SpinningMode::NO;
            last_switch_left_  = msg->switch_left;
            last_switch_right_ = msg->switch_right;
            publish_control_velocities(0, 0, 0, 0);
            return;
        }

        if (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_MIDDLE
            && last_switch_right_ == rm_msgs::msg::RemoteControl::SWITCH_STATE_MIDDLE
            && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            if (spinning_mode_ == SpinningMode::NO)
                spinning_mode_ = SpinningMode::LOW;
            else if (spinning_mode_ == SpinningMode::LOW)
                spinning_mode_ = SpinningMode::HIGH;
            else if (spinning_mode_ == SpinningMode::HIGH)
                spinning_mode_ = SpinningMode::NO;
        }

        if (keys->ctrl && keys->c) {
            spinning_mode_ = SpinningMode::NO;
        } else if (!keys->ctrl && keys->c) {
            spinning_mode_ = SpinningMode::HIGH;
        }

        last_switch_left_  = msg->switch_left;
        last_switch_right_ = msg->switch_right;

        // Control Motion
        //
        constexpr double velocity_limit = 800;

        auto rotation = Eigen::Rotation2Dd{gimbal_yaw_};

        // TODO
        //
        // value to change
        auto channel  = Eigen::Vector2d{msg->channel_right_x, msg->channel_right_y};
        auto keyboard = Eigen::Vector2d{0.5 * (keys->d - keys->a), 0.5 * (keys->w - keys->s)};
        auto move     = (rotation * (channel + keyboard)).eval();

        double right_oblique = velocity_limit * (move.x() * cos_45 + move.y() * sin_45);
        double left_oblique  = velocity_limit * (move.y() * cos_45 - move.x() * sin_45);

        double spinning_velocity = 0;
        if (spinning_mode_ == SpinningMode::LOW)
            spinning_velocity = 0.4;
        else if (spinning_mode_ == SpinningMode::HIGH)
            spinning_velocity = 0.8;

        double velocities[4] = {-left_oblique, right_oblique, left_oblique, -right_oblique};
        double max_velocity  = 0;

        for (auto& velocity : velocities) {
            velocity += 0.4 * velocity_limit * spinning_velocity;
            max_velocity = std::max(std::abs(velocity), max_velocity);
        }
        if (max_velocity > velocity_limit) {
            double scale = velocity_limit / max_velocity;
            for (auto& velocity : velocities)
                velocity *= scale;
        }
        publish_control_velocities(velocities[0], velocities[1], velocities[2], velocities[3]);

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "\n|0: %d|1: %d|2: %d|3: %d|4: %d|5: %d|6: %d|7: %d"
        //     "|8: %d|9: %d|10: %d|11: %d|12: %d|13: %d|14: %d|15: %d|",
        //     static_cast<uint8_t>(keyboard_bits[0]), static_cast<uint8_t>(keyboard_bits[1]),
        //     static_cast<uint8_t>(keyboard_bits[2]), static_cast<uint8_t>(keyboard_bits[3]),
        //     static_cast<uint8_t>(keyboard_bits[4]), static_cast<uint8_t>(keyboard_bits[5]),
        //     static_cast<uint8_t>(keyboard_bits[6]), static_cast<uint8_t>(keyboard_bits[7]),
        //     static_cast<uint8_t>(keyboard_bits[8]), static_cast<uint8_t>(keyboard_bits[9]),
        //     static_cast<uint8_t>(keyboard_bits[10]), static_cast<uint8_t>(keyboard_bits[11]),
        //     static_cast<uint8_t>(keyboard_bits[12]), static_cast<uint8_t>(keyboard_bits[13]),
        //     static_cast<uint8_t>(keyboard_bits[14]), static_cast<uint8_t>(keyboard_bits[15]));
    }

    void remote_control_watchdog_callback() {
        remote_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Remote control message timeout, will reset wheel velocities.");
        spinning_mode_ = SpinningMode::NO;
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

    rm_msgs::msg::RemoteControl::_switch_left_type last_switch_left_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;
    rm_msgs::msg::RemoteControl::_switch_right_type last_switch_right_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;

    enum class SpinningMode : uint8_t {
        NO   = 0,
        LOW  = 1,
        HIGH = 2
    } spinning_mode_ = SpinningMode::NO;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_yaw_subscription_;
    double gimbal_yaw_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_back_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_back_control_velocity_publisher_;
};

} // namespace chassis
} // namespace controller