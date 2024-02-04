#pragma once

#include <cmath>

#include <memory>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "test_controller/qos.hpp"

namespace forwarder {

class Imu {
public:
    Imu(rclcpp::Node* node)
        : node_(node)
        , tf_broadcaster_(node) {

        gimbal_yaw_velocity_imu_ =
            node->create_publisher<std_msgs::msg::Float64>("/gimbal/yaw/velocity_imu", kCoreQoS);
        gimbal_yaw_angle_imu_ =
            node->create_publisher<std_msgs::msg::Float64>("/gimbal/yaw/angle_imu", kCoreQoS);

        gimbal_pitch_velocity_imu_ =
            node->create_publisher<std_msgs::msg::Float64>("/gimbal/pitch/velocity_imu", kCoreQoS);
        gimbal_pitch_angle_imu_ =
            node->create_publisher<std_msgs::msg::Float64>("/gimbal/pitch/angle_imu", kCoreQoS);
    };

    void update(double gx, double gy, double gz, double ax, double ay, double az) {
        mahony_ahrs_update_imu(gx, gy, gz, ax, ay, az);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp            = node_->get_clock()->now();
        t.header.frame_id         = "world";
        t.child_frame_id          = "body";
        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 1;
        t.transform.rotation.w    = q0;
        t.transform.rotation.x    = q1;
        t.transform.rotation.y    = q2;
        t.transform.rotation.z    = q3;
        tf_broadcaster_.sendTransform(t);

        auto yaw_velocity  = std::make_unique<std_msgs::msg::Float64>();
        yaw_velocity->data = gz;
        gimbal_yaw_velocity_imu_->publish(std::move(yaw_velocity));

        auto pitch_velocity  = std::make_unique<std_msgs::msg::Float64>();
        pitch_velocity->data = gx;
        gimbal_pitch_velocity_imu_->publish(std::move(pitch_velocity));

        auto q      = Eigen::Quaterniond{q0, q1, q2, q3};
        auto barrel = q * (-Eigen::Vector3d::UnitY());

        auto yaw_angle  = std::make_unique<std_msgs::msg::Float64>();
        yaw_angle->data = atan2(barrel.y(), barrel.x());
        gimbal_yaw_angle_imu_->publish(std::move(yaw_angle));

        auto pitch_angle  = std::make_unique<std_msgs::msg::Float64>();
        pitch_angle->data = -asin(barrel.dot(Eigen::Vector3d::UnitZ()));
        gimbal_pitch_angle_imu_->publish(std::move(pitch_angle));
    }

    // Quaternion of sensor frame relative to auxiliary frame
    double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

private:
    void mahony_ahrs_update_imu(double gx, double gy, double gz, double ax, double ay, double az) {
        // Madgwick's implementation of Mayhony's AHRS algorithm.
        // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms

        double recipNorm;
        double halfvx, halfvy, halfvz;
        double halfex, halfey, halfez;
        double qa, qb, qc;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer
        // normalisation)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

            // Normalise accelerometer measurement
            recipNorm = 1 / std::sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity and vector perpendicular to magnetic flux
            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5 + q3 * q3;

            // Error is sum of cross product between estimated and measured direction of gravity
            halfex = ay * halfvz - az * halfvy;
            halfey = az * halfvx - ax * halfvz;
            halfez = ax * halfvy - ay * halfvx;

            // Compute and apply integral feedback if enabled
            if (kKi > 0.0) {
                // integral error scaled by Ki
                integral_fbx_ += 2.0 * kKi * halfex * (1.0 / kSampleFreq);
                integral_fby_ += 2.0 * kKi * halfey * (1.0 / kSampleFreq);
                integral_fbz_ += 2.0 * kKi * halfez * (1.0 / kSampleFreq);
                // apply integral feedback
                gx += integral_fbx_;
                gy += integral_fby_;
                gz += integral_fbz_;
            } else {
                // prevent integral windup
                integral_fbx_ = 0.0;
                integral_fby_ = 0.0;
                integral_fbz_ = 0.0;
            }

            // Apply proportional feedback
            gx += 2.0 * kKp * halfex;
            gy += 2.0 * kKp * halfey;
            gz += 2.0 * kKp * halfez;
        }

        // Integrate rate of change of quaternion
        gx *= (0.5 * (1.0 / kSampleFreq)); // pre-multiply common factors
        gy *= (0.5 * (1.0 / kSampleFreq));
        gz *= (0.5 * (1.0 / kSampleFreq));
        qa = q0;
        qb = q1;
        qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        // Normalise quaternion
        recipNorm = 1 / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    static constexpr double kSampleFreq = 2000.0; // sample frequency in Hz
    static constexpr double kKp         = 0.5;    // proportional gain
    static constexpr double kKi         = 0.0;    // integral gain

    // Integral error terms scaled by Ki
    double integral_fbx_ = 0.0, integral_fby_ = 0.0, integral_fbz_ = 0.0;

    rclcpp::Node* node_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_yaw_angle_imu_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_yaw_velocity_imu_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_pitch_angle_imu_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gimbal_pitch_velocity_imu_;
};

} // namespace forwarder