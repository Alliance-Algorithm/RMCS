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

#include <geometry_msgs/msg/vector3.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rmcs_controller/qos.hpp"
#include "rmcs_controller/type.hpp"

namespace controller {
namespace chassis {

class OmniNode : public rclcpp::Node {
public:
    OmniNode()
        : Node("omni_chassis_controller", rclcpp::NodeOptions().use_intra_process_comms(true))
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_) {

        remote_control_subscription_ = this->create_subscription<rm_msgs::msg::RemoteControl>(
            "/remote_control", kCoreQoS,
            std::bind(&OmniNode::remote_control_callback, this, std::placeholders::_1));

        decision_control_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/watcher/decision_maker/rmul/dest_dir", kCoreQoS,
            std::bind(&OmniNode::decision_control_callback, this, std::placeholders::_1));

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
        decision_control_watchdog_timer_ = this->create_wall_timer(
            500ms, std::bind(&OmniNode::decision_control_watchdog_callback, this));

        remote_control_watchdog_timer_->cancel();
        decision_control_watchdog_timer_->cancel();
    }

private:
    void remote_control_callback(rm_msgs::msg::RemoteControl::SharedPtr msg) {

        remote_control_watchdog_timer_->reset();

        auto keys = reinterpret_cast<KeyboardType*>(&msg->keyboard);

        if (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN
            && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            spinning_mode_     = SpinningMode::NO;
            auto_mode_         = false;
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

        auto_mode_ =
            (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_MIDDLE
             && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_UP);

        last_switch_left_  = msg->switch_left;
        last_switch_right_ = msg->switch_right;

        if (!auto_mode_) {
            // TODO: change the value
            auto channel  = Eigen::Vector2d{msg->channel_right_x, msg->channel_right_y};
            auto keyboard = Eigen::Vector2d{0.5 * (keys->d - keys->a), 0.5 * (keys->w - keys->s)};

            update_control_velocities(channel + keyboard);
        }
    }

    void decision_control_callback(geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (!auto_mode_)
            return;

        if (msg->x == 0 && msg->y == 0 && msg->z == 0) {
            update_control_velocities({0, 0});
            return;
        }

        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_.lookupTransform("pitch_link", "odom", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        auto q = Eigen::Quaterniond{
            t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z};
        auto vec = (q * Eigen::Vector3d{msg->x, msg->y, msg->z}).eval();

        auto vec_norm = vec.norm();
        if (vec_norm > 1) {
            RCLCPP_WARN(get_logger(), "Norm of destination vector is too large: %f", vec_norm);
            return;
        }

        auto projection      = Eigen::Vector2d{vec.x(), vec.y()};
        // auto projection_norm = projection.norm();
        // projection *= (vec_norm / projection_norm);

        // RCLCPP_INFO(get_logger(), "%f, %f", projection.x(), projection.y());

        update_control_velocities({-projection.y(), projection.x()});
    }

    void remote_control_watchdog_callback() {
        remote_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Remote control message timeout, will reset wheel velocities.");
        spinning_mode_ = SpinningMode::NO;
        publish_control_velocities(0, 0, 0, 0);
    }

    void decision_control_watchdog_callback() {
        decision_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Decision control message timeout, will reset control velocities.");
        update_control_velocities({0, 0});
    }

    void update_control_velocities(Eigen::Vector2d move) {
        move = Eigen::Rotation2Dd{gimbal_yaw_} * move;

        constexpr double speed_limit = 800;
        double right_oblique         = speed_limit * (move.x() * cos_45 + move.y() * sin_45);
        double left_oblique          = speed_limit * (move.y() * cos_45 - move.x() * sin_45);

        double spinning_velocity = 0;
        if (spinning_mode_ == SpinningMode::LOW)
            spinning_velocity = 0.4;
        else if (spinning_mode_ == SpinningMode::HIGH)
            spinning_velocity = 0.8;

        double velocities[4] = {-left_oblique, right_oblique, left_oblique, -right_oblique};
        double max_velocity  = 0;

        for (auto& velocity : velocities) {
            velocity += 0.4 * speed_limit * spinning_velocity;
            max_velocity = std::max(std::abs(velocity), max_velocity);
        }
        if (max_velocity > speed_limit) {
            double scale = speed_limit / max_velocity;
            for (auto& velocity : velocities)
                velocity *= scale;
        }

        publish_control_velocities(velocities[0], velocities[1], velocities[2], velocities[3]);
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

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // The sine and cosine functions are not constexprs, so we calculate once and cache them.
    static inline const double sin_45 = std::sin(std::numbers::pi / 4.0);
    static inline const double cos_45 = std::cos(std::numbers::pi / 4.0);

    rclcpp::Subscription<rm_msgs::msg::RemoteControl>::SharedPtr remote_control_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr decision_control_subscription_;
    rclcpp::TimerBase::SharedPtr remote_control_watchdog_timer_;
    rclcpp::TimerBase::SharedPtr decision_control_watchdog_timer_;

    rm_msgs::msg::RemoteControl::_switch_left_type last_switch_left_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;
    rm_msgs::msg::RemoteControl::_switch_right_type last_switch_right_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;

    enum class SpinningMode : uint8_t {
        NO   = 0,
        LOW  = 1,
        HIGH = 2
    } spinning_mode_ = SpinningMode::NO;
    bool auto_mode_  = false;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_yaw_subscription_;
    double gimbal_yaw_ = 0;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_front_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_back_control_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_back_control_velocity_publisher_;
};

} // namespace chassis
} // namespace controller