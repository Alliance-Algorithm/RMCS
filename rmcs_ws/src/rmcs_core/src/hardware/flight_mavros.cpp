#include <atomic>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class FlightMavros
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FlightMavros()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        odometry_topic_ = get_parameter("odometry_topic").as_string();
        const auto mavros_pose_topic = get_parameter("mavros_pose_topic").as_string();
        const auto sensor_roll_offset_rad = get_parameter("sensor_roll_offset_rad").as_double();
        const auto sensor_pitch_offset_rad = get_parameter("sensor_pitch_offset_rad").as_double();
        const auto sensor_yaw_offset_rad = get_parameter("sensor_yaw_offset_rad").as_double();

        sensor_to_body_rotation_ = quaternion_from_rpy(
            sensor_roll_offset_rad, sensor_pitch_offset_rad, sensor_yaw_offset_rad);
        sensor_to_body_rotation_.normalize();

        mavros_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            mavros_pose_topic, rclcpp::QoS{10}.reliable());
        odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic_, rclcpp::QoS{10}.reliable(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { odometry_callback(msg); });
        last_odometry_received_time_ns_.store(
            get_clock()->now().nanoseconds(), std::memory_order_relaxed);

        RCLCPP_INFO(
            logger_,
            "FlightMavros bridging odometry '%s' to '%s' with sensor RPY [%.1f, %.1f, %.1f] "
            "deg.",
            odometry_topic_.c_str(), mavros_pose_publisher_->get_topic_name(),
            radians_to_degrees(sensor_roll_offset_rad), radians_to_degrees(sensor_pitch_offset_rad),
            radians_to_degrees(sensor_yaw_offset_rad));
    }

    ~FlightMavros() override = default;

    void update() override {
        const auto now_ns = get_clock()->now().nanoseconds();
        const auto last_odometry_received_time_ns =
            last_odometry_received_time_ns_.load(std::memory_order_relaxed);
        if (now_ns - last_odometry_received_time_ns >= kOdometryTimeoutNs) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000, "No odometry received on '%s' for more than %.1f s.",
                odometry_topic_.c_str(), static_cast<double>(kOdometryTimeoutNs) / 1'000'000'000.0);
        }
    }

private:
    static constexpr std::int64_t kOdometryTimeoutNs{1'000'000'000}; // 1s

    static double radians_to_degrees(double radians) { return radians * 180.0 / std::numbers::pi; }

    static Eigen::Quaterniond quaternion_from_rpy(double roll, double pitch, double yaw) {
        const Eigen::AngleAxisd roll_rotation{roll, Eigen::Vector3d::UnitX()};
        const Eigen::AngleAxisd pitch_rotation{pitch, Eigen::Vector3d::UnitY()};
        const Eigen::AngleAxisd yaw_rotation{yaw, Eigen::Vector3d::UnitZ()};
        return Eigen::Quaterniond{roll_rotation * pitch_rotation * yaw_rotation};
    }

    Eigen::Quaterniond
        rotate_body_orientation(const geometry_msgs::msg::Quaternion& orientation) const {
        const Eigen::Quaterniond quat_sensor{
            orientation.w, orientation.x, orientation.y, orientation.z};

        Eigen::Quaterniond quat_body = quat_sensor * sensor_to_body_rotation_;
        quat_body.normalize();
        return quat_body;
    }

    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
        last_odometry_received_time_ns_.store(
            get_clock()->now().nanoseconds(), std::memory_order_relaxed);

        RCLCPP_INFO_ONCE(
            logger_, "Received odometry from '%s' with frame '%s' and child frame '%s'.",
            odometry_topic_.c_str(), msg->header.frame_id.c_str(), msg->child_frame_id.c_str());

        if (msg->header.frame_id != "odom" || msg->child_frame_id != "odin1_base_link") {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000,
                "Unexpected odometry frames: header='%s', child='%s'. Expected 'odom' and "
                "'odin1_base_link'.",
                msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
        }

        geometry_msgs::msg::PoseStamped pose_msg{};
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = msg->header.frame_id;

        pose_msg.pose.position = msg->pose.pose.position;

        const Eigen::Quaterniond rotated_orientation =
            rotate_body_orientation(msg->pose.pose.orientation);
        pose_msg.pose.orientation.x = rotated_orientation.x();
        pose_msg.pose.orientation.y = rotated_orientation.y();
        pose_msg.pose.orientation.z = rotated_orientation.z();
        pose_msg.pose.orientation.w = rotated_orientation.w();

        mavros_pose_publisher_->publish(pose_msg);
    }

    rclcpp::Logger logger_;
    std::string odometry_topic_;
    Eigen::Quaterniond sensor_to_body_rotation_{Eigen::Quaterniond::Identity()};
    std::atomic<std::int64_t> last_odometry_received_time_ns_{0};

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FlightMavros, rmcs_executor::Component)
