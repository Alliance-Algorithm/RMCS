#include <cmath>
#include <cstdint>
#include <mutex>
#include <numbers>
#include <string>

#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class FlightMavros
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FlightMavros()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger())
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_) {
        load_parameters();

        mavros_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            mavros_pose_topic_, rclcpp::QoS{10}.reliable());

        RCLCPP_INFO(
            logger_,
            "FlightMavros bridging TF '%s' -> '%s' to '%s' with sensor RPY [%.1f, %.1f, %.1f] "
            "deg.",
            source_frame_id_.c_str(), target_frame_id_.c_str(),
            mavros_pose_publisher_->get_topic_name(), radians_to_degrees(sensor_roll_offset_rad_),
            radians_to_degrees(sensor_pitch_offset_rad_),
            radians_to_degrees(sensor_yaw_offset_rad_));
    }

    ~FlightMavros() override = default;

    void update() override {
        update_pose_from_tf();
        if (++update_counter_ >= publish_interval_updates_) {
            update_counter_ = 0;
            publish_pose();
        }
    }

private:
    static double radians_to_degrees(double radians) {
        return radians * 180.0 / std::numbers::pi_v<double>;
    }

    static Eigen::Quaterniond quaternion_from_rpy(double roll, double pitch, double yaw) {
        const Eigen::AngleAxisd roll_rotation{roll, Eigen::Vector3d::UnitX()};
        const Eigen::AngleAxisd pitch_rotation{pitch, Eigen::Vector3d::UnitY()};
        const Eigen::AngleAxisd yaw_rotation{yaw, Eigen::Vector3d::UnitZ()};
        return Eigen::Quaterniond{roll_rotation * pitch_rotation * yaw_rotation};
    }

    void load_parameters() {
        mavros_pose_topic_ = get_parameter("mavros_pose_topic").as_string();
        mavros_pose_frame_id_ = get_parameter("mavros_pose_frame_id").as_string();
        target_frame_id_ = get_parameter("target_frame_id").as_string();
        source_frame_id_ = get_parameter("source_frame_id").as_string();
        output_rate_hz_ = get_parameter("output_rate").as_double();
        sensor_roll_offset_rad_ = get_parameter("sensor_roll_offset_rad").as_double();
        sensor_pitch_offset_rad_ = get_parameter("sensor_pitch_offset_rad").as_double();
        sensor_yaw_offset_rad_ = get_parameter("sensor_yaw_offset_rad").as_double();

        publish_interval_updates_ =
            static_cast<std::uint32_t>(std::max(1.0, std::round(1000.0 / output_rate_hz_)));
    }

    Eigen::Quaterniond sensor_to_body_rotation() const {
        auto sensor_to_body = quaternion_from_rpy(
            sensor_roll_offset_rad_, sensor_pitch_offset_rad_, sensor_yaw_offset_rad_);
        sensor_to_body.normalize();
        return sensor_to_body;
    }

    static bool finite_transform(const geometry_msgs::msg::TransformStamped& transform) {
        const auto& t = transform.transform.translation;
        const auto& r = transform.transform.rotation;
        return std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z) && std::isfinite(r.x)
            && std::isfinite(r.y) && std::isfinite(r.z) && std::isfinite(r.w);
    }

    Eigen::Vector3d rotate_world_position(const geometry_msgs::msg::Vector3& position) const {
        return Eigen::Vector3d{position.x, position.y, position.z};
    }

    Eigen::Quaterniond
        rotate_body_orientation(const geometry_msgs::msg::Quaternion& orientation) const {
        const Eigen::Quaterniond quat_sensor{
            orientation.w, orientation.x, orientation.y, orientation.z};

        Eigen::Quaterniond quat_body = quat_sensor * sensor_to_body_rotation();
        quat_body.normalize();
        return quat_body;
    }

    void update_pose_from_tf() {
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform =
                tf_buffer_.lookupTransform(target_frame_id_, source_frame_id_, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000, "TF lookup failed for '%s' -> '%s': %s",
                source_frame_id_.c_str(), target_frame_id_.c_str(), ex.what());
            return;
        }

        if (!finite_transform(transform))
            return;

        const rclcpp::Time tf_stamp{transform.header.stamp};

        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (last_tf_stamp_.nanoseconds() != 0 && tf_stamp <= last_tf_stamp_)
            return;

        const Eigen::Vector3d rotated_position =
            rotate_world_position(transform.transform.translation);
        const Eigen::Quaterniond rotated_orientation =
            rotate_body_orientation(transform.transform.rotation);

        latest_pose_.position.x = rotated_position.x();
        latest_pose_.position.y = rotated_position.y();
        latest_pose_.position.z = rotated_position.z();
        latest_pose_.orientation.x = rotated_orientation.x();
        latest_pose_.orientation.y = rotated_orientation.y();
        latest_pose_.orientation.z = rotated_orientation.z();
        latest_pose_.orientation.w = rotated_orientation.w();
        last_tf_stamp_ = tf_stamp;
        pose_ready_ = true;

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 1000, "Received TF '%s' -> '%s'.", source_frame_id_.c_str(),
            target_frame_id_.c_str());
    }

    void publish_pose() {
        geometry_msgs::msg::Pose pose;
        rclcpp::Time tf_stamp;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!pose_ready_)
                return;

            if (last_tf_stamp_ == last_published_tf_stamp_)
                return;

            tf_stamp = last_tf_stamp_;
            pose = latest_pose_;
        }

        const rclcpp::Duration max_pose_age = rclcpp::Duration::from_seconds(1.0 / output_rate_hz_);
        const rclcpp::Time now = get_clock()->now();
        if ((now - tf_stamp) >= max_pose_age)
            return;

        geometry_msgs::msg::PoseStamped pose_msg{};
        pose_msg.header.stamp = now;
        pose_msg.header.frame_id = mavros_pose_frame_id_;
        pose_msg.pose = pose;
        mavros_pose_publisher_->publish(pose_msg);

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (last_tf_stamp_ == tf_stamp)
                last_published_tf_stamp_ = tf_stamp;
        }

        RCLCPP_INFO_THROTTLE(
            logger_, *get_clock(), 1000, "Published MAVROS vision pose at %.1f Hz to '%s'.",
            output_rate_hz_, mavros_pose_publisher_->get_topic_name());
    }

    rclcpp::Logger logger_;
    std::string mavros_pose_topic_;
    std::string mavros_pose_frame_id_;
    std::string target_frame_id_;
    std::string source_frame_id_;
    double output_rate_hz_;
    double sensor_roll_offset_rad_;
    double sensor_pitch_offset_rad_;
    double sensor_yaw_offset_rad_;
    std::uint32_t publish_interval_updates_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher_;

    std::mutex pose_mutex_;
    geometry_msgs::msg::Pose latest_pose_{};
    rclcpp::Time last_tf_stamp_{};
    rclcpp::Time last_published_tf_stamp_{};
    std::uint32_t update_counter_{0};
    bool pose_ready_{false};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FlightMavros, rmcs_executor::Component)
