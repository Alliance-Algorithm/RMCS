#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace rmcs_core::broadcaster {

// One sample time for joint feedback and both Body-frame IMU observation paths.
// This component publishes telemetry in every control mode, including disarm.
class WheelLegStateBroadcaster
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WheelLegStateBroadcaster()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const double rate = get_parameter_or<double>("publish_rate", 50.0);
        if (!std::isfinite(rate) || rate <= 0.0 || rate > 200.0)
            throw std::invalid_argument("WheelLegStateBroadcaster: publish_rate must be in (0,200]");
        period_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / rate));
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kNames[i];
            register_input(prefix + "/angle", positions_[i]);
            register_input(prefix + "/velocity", velocities_[i]);
            register_input(prefix + "/torque", torques_[i]);
        }
        register_input("/wheel_leg/imu/quaternion", orientation_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_body_);
        register_input("/wheel_leg/rl/imu/projected_gravity", gravity_rl_);
        register_input("/wheel_leg/rl/imu/angular_velocity", gyro_rl_);

        const auto qos = rclcpp::SensorDataQoS().keep_last(2);
        joints_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
            "/wheel_leg/telemetry/joint_states", qos);
        imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
            "/wheel_leg/telemetry/imu_body", qos);
        gravity_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/wheel_leg/telemetry/rl_projected_gravity", qos);
        gyro_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/wheel_leg/telemetry/rl_angular_velocity", qos);

        // TF for Foxglove 3D: odom -> chassis_body (q_WB) is dynamic below;
        // Training IMU observations use Body xyz, so chassis_body -> rl_base
        // is identity. The URDF display frame must not rotate IMU telemetry.
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now();
            tf.header.frame_id = "chassis_body";
            tf.child_frame_id = "rl_base";
            tf.transform.rotation.w = 1.0;
            static_tf_broadcaster_->sendTransform(tf);
        }
    }

    void update() override {
        const auto current = Clock::now();
        if (current - last_publish_ < period_)
            return;
        last_publish_ = current;

        sensor_msgs::msg::JointState joints;
        sensor_msgs::msg::Imu imu;
        geometry_msgs::msg::Vector3Stamped gravity, gyro;
        joints.header.stamp = imu.header.stamp = gravity.header.stamp = gyro.header.stamp = now();
        joints.header.frame_id = gravity.header.frame_id = gyro.header.frame_id = "rl_base";
        imu.header.frame_id = "chassis_body";
        for (std::size_t i = 0; i < kNames.size(); ++i) {
            joints.name.emplace_back(kNames[i]);
            joints.position.push_back(*positions_[i]);
            joints.velocity.push_back(*velocities_[i]);
            joints.effort.push_back(*torques_[i]);
        }
        // q_WB (body -> world). Preserve invalid values for the receiver to
        // reject; never replace a bad sample with a plausible identity pose.
        imu.orientation.w = orientation_->w();
        imu.orientation.x = orientation_->x();
        imu.orientation.y = orientation_->y();
        imu.orientation.z = orientation_->z();
        imu.angular_velocity = vector_message(*gyro_body_);
        imu.linear_acceleration_covariance[0] = -1.0; // Acceleration is not provided.
        gravity.vector = vector_message(*gravity_rl_);
        gyro.vector = vector_message(*gyro_rl_);
        joints_publisher_->publish(joints);
        imu_publisher_->publish(imu);
        gravity_publisher_->publish(gravity);
        gyro_publisher_->publish(gyro);

        // Publish attitude only when the sample is finite; never broadcast NaN.
        if (orientation_->coeffs().allFinite() && orientation_->squaredNorm() > 1e-12) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = imu.header.stamp;
            tf.header.frame_id = "odom";
            tf.child_frame_id = "chassis_body";
            const Eigen::Quaterniond q = orientation_->normalized();
            tf.transform.rotation.w = q.w();
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf_broadcaster_->sendTransform(tf);
        }
    }

private:
    using Clock = std::chrono::steady_clock;
    static constexpr std::array<const char*, 6> kNames{
        "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint",
        "left_wheel", "right_wheel"};

    static geometry_msgs::msg::Vector3 vector_message(const Eigen::Vector3d& value) {
        geometry_msgs::msg::Vector3 result;
        result.x = value.x();
        result.y = value.y();
        result.z = value.z();
        return result;
    }

    std::array<InputInterface<double>, 6> positions_, velocities_, torques_;
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> gyro_body_, gravity_rl_, gyro_rl_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_publisher_, gyro_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    Clock::duration period_{};
    Clock::time_point last_publish_{};
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::WheelLegStateBroadcaster, rmcs_executor::Component)
