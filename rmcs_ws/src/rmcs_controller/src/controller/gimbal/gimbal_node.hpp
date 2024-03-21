#pragma once

#include <cmath>
#include <limits>
#include <memory>

#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rm_msgs/msg/remote_control.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rmcs_controller/qos.hpp"

namespace controller {
namespace gimbal {

class GimbalNode : public rclcpp::Node {
public:
    GimbalNode()
        : Node("gimbal_controller", rclcpp::NodeOptions().use_intra_process_comms(true))
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_) {

        remote_control_subscription_ = this->create_subscription<rm_msgs::msg::RemoteControl>(
            "/remote_control", kCoreQoS,
            std::bind(&GimbalNode::remote_control_callback, this, std::placeholders::_1));

        gimbal_friction_left_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/left_friction/control_velocity", kCoreQoS);
        gimbal_friction_right_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/right_friction/control_velocity", kCoreQoS);
        gimbal_bullet_deliver_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/bullet_deliver/control_velocity", kCoreQoS);

        gimbal_control_yaw_error_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/yaw/control_angle_error", kCoreQoS);

        gimbal_pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/gimbal/pitch/angle", kCoreQoS, [this](std_msgs::msg::Float64::UniquePtr msg) {
                (void)msg;
                // using namespace std::numbers;
                // double pitch = msg->data;
                // double diff  = gimbal_imu_pitch_ - pitch;
                // double min   = fmod(diff + gimbal_pitch_min_, 2 * pi);
                // double max   = fmod(diff + gimbal_pitch_max_, 2 * pi);
                // if (min < -pi / 2 || min > pi / 2)
                //     min = -pi / 2;
                // if (max < -pi / 2 || max > pi / 2)
                //     max = pi / 2;
                // if (min <= max) {
                //     gimbal_imu_pitch_min_ = min;
                //     gimbal_imu_pitch_max_ = max;
                // } else {
                //     gimbal_imu_pitch_min_ = gimbal_imu_pitch_max_ = gimbal_imu_pitch_;
                // }
            });
        gimbal_control_pitch_error_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "/gimbal/pitch/control_angle_error", kCoreQoS);

        gimbal_auto_aim_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/gimbal/auto_aim", kCoreQoS, [this](geometry_msgs::msg::Vector3::UniquePtr msg) {
                if (!control_direction_.isZero()) {
                    control_direction_ = {msg->x, msg->y, msg->z};
                }
            });

        using namespace std::chrono_literals;

        this->control_error_publish_timer_ = this->create_wall_timer(1ms, [this]() {
            if (control_direction_.isZero()) {
                control_direction_filtered_ = Eigen::Vector3d::Zero();
                publish_control_angle_error(nan, nan);
                return;
            }

            if (control_direction_filtered_.isZero()) {
                control_direction_filtered_ = control_direction_;
            } else {
                control_direction_filtered_.x() +=
                    0.1 * (control_direction_.x() - control_direction_filtered_.x());
                control_direction_filtered_.y() +=
                    0.1 * (control_direction_.y() - control_direction_filtered_.y());
                control_direction_filtered_.z() +=
                    0.1 * (control_direction_.z() - control_direction_filtered_.z());
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

            // v is the control_direction in pitch_link coordinate system
            auto v = (q * control_direction_filtered_).eval();
            if (v.x() == 0 && v.y() == 0) {
                publish_control_angle_error(nan, nan);
                return;
            }

            auto yaw   = std::atan2(v.y(), v.x());
            auto pitch = -std::atan2(v.z(), std::sqrt(v.y() * v.y() + v.x() + v.x()));
            publish_control_angle_error(yaw, pitch);
        });

        remote_control_watchdog_timer_ = this->create_wall_timer(
            500ms, std::bind(&GimbalNode::remote_control_watchdog_callback, this));
        remote_control_watchdog_timer_->cancel();
    }

private:
    void publish_control_angle_error(double yaw, double pitch) {
        std::unique_ptr<std_msgs::msg::Float64> msg;

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = yaw;
        gimbal_control_yaw_error_publisher_->publish(std::move(msg));

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = pitch;
        gimbal_control_pitch_error_publisher_->publish(std::move(msg));
    }

    void remote_control_callback(rm_msgs::msg::RemoteControl::SharedPtr msg) {
        remote_control_watchdog_timer_->reset();

        if (msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN
            && msg->switch_right == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            friction_mode_ = false;
            publish_friction_mode();
            bullet_deliver_mode_ = false;
            publish_bullet_deliver_mode();
            control_direction_ = Eigen::Vector3d::Zero();
            return;
        }

        if (msg->switch_right != rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN) {
            if (last_switch_left_ == rm_msgs::msg::RemoteControl::SWITCH_STATE_MIDDLE
                && msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_UP) {
                friction_mode_ = !friction_mode_;
                publish_friction_mode();
            }
            bullet_deliver_mode_ =
                friction_mode_
                && msg->switch_left == rm_msgs::msg::RemoteControl::SWITCH_STATE_DOWN;
            publish_bullet_deliver_mode();
        }

        last_switch_left_  = msg->switch_left;
        last_switch_right_ = msg->switch_right;

        geometry_msgs::msg::TransformStamped odom_to_pitch_link;
        try {
            odom_to_pitch_link =
                tf_buffer_.lookupTransform("odom", "pitch_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        auto gimbal_pose = Eigen::Quaterniond{
            odom_to_pitch_link.transform.rotation.w, odom_to_pitch_link.transform.rotation.x,
            odom_to_pitch_link.transform.rotation.y, odom_to_pitch_link.transform.rotation.z};

        if (control_direction_.isZero()) {
            auto vec = gimbal_pose * Eigen::Vector3d::UnitX();
            if (vec.x() != 0 || vec.y() != 0)
                control_direction_ = Eigen::Vector3d{vec.x(), vec.y(), 0}.normalized();
        }

        auto delta_yaw =
            Eigen::AngleAxisd{-0.08 * msg->channel_left_x, gimbal_pose * Eigen::Vector3d::UnitZ()};
        auto delta_pitch =
            Eigen::AngleAxisd{-0.06 * msg->channel_left_y, gimbal_pose * Eigen::Vector3d::UnitY()};
        control_direction_ = (delta_pitch * (delta_yaw * control_direction_)).eval();

        // if (gimbal_control_pitch_ < gimbal_imu_pitch_min_)
        //     gimbal_control_pitch_ = gimbal_imu_pitch_min_;
        // else if (gimbal_control_pitch_ > gimbal_imu_pitch_max_)
        //     gimbal_control_pitch_ = gimbal_imu_pitch_max_;
    }

    void remote_control_watchdog_callback() {
        remote_control_watchdog_timer_->cancel();
        RCLCPP_INFO(
            this->get_logger(), "Remote control message timeout, will disable gimbal control.");

        friction_mode_ = false;
        publish_friction_mode();
        bullet_deliver_mode_ = false;
        publish_bullet_deliver_mode();
        control_direction_ = Eigen::Vector3d::Zero();
    }

    void publish_friction_mode() {
        std::unique_ptr<std_msgs::msg::Float64> msg;
        double velocity = friction_mode_ ? 900.0 : 0.0;

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = velocity;
        gimbal_friction_left_publisher_->publish(std::move(msg));

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = velocity;
        gimbal_friction_right_publisher_->publish(std::move(msg));
    }

    void publish_bullet_deliver_mode() {
        std::unique_ptr<std_msgs::msg::Float64> msg;
        double velocity = bullet_deliver_mode_ ? 200.0 : nan;

        msg       = std::make_unique<std_msgs::msg::Float64>();
        msg->data = velocity;
        gimbal_bullet_deliver_publisher_->publish(std::move(msg));
    }

    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    Eigen::Vector3d control_direction_          = Eigen::Vector3d::Zero();
    Eigen::Vector3d control_direction_filtered_ = Eigen::Vector3d::Zero();

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_yaw_error_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gimbal_pitch_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_control_pitch_error_publisher_;

    rclcpp::Subscription<rm_msgs::msg::RemoteControl>::SharedPtr remote_control_subscription_;

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_auto_aim_subscription_;

    rm_msgs::msg::RemoteControl::_switch_left_type last_switch_left_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;
    rm_msgs::msg::RemoteControl::_switch_right_type last_switch_right_ =
        rm_msgs::msg::RemoteControl::SWITCH_STATE_UNKNOWN;

    bool friction_mode_ = false, bullet_deliver_mode_ = false;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_friction_left_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_friction_right_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_bullet_deliver_publisher_;

    // double gimbal_imu_pitch_min_ = 0, gimbal_imu_pitch_max_ = 0;
    // const double gimbal_pitch_min_ = -0.42, gimbal_pitch_max_ = 0.38;

    rclcpp::TimerBase::SharedPtr control_error_publish_timer_;
    rclcpp::TimerBase::SharedPtr remote_control_watchdog_timer_;
};

} // namespace gimbal
} // namespace controller