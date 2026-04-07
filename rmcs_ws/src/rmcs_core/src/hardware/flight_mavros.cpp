#include <array>
#include <cmath>
#include <cstdint>
#include <mutex>

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class FlightMavros
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FlightMavros()
        : Node{get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)}
        , logger_(get_logger()) {
        mavros_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            get_parameter("mavros_pose_topic").as_string(), rclcpp::QoS{10}.reliable());
        mavros_twist_publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            get_parameter("mavros_twist_topic").as_string(), rclcpp::QoS{10}.reliable());

        auto odin_odom_qos = rclcpp::QoS{1};
        odin_odom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        odin_odom_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        odin_odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            get_parameter("odin_odometry_topic").as_string(), odin_odom_qos,
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
                odin_odometry_subscription_callback(*msg);
            });

        RCLCPP_INFO(
            logger_, "FlightMavros bridging '%s' -> '%s' and '%s'.",
            odin_odometry_subscription_->get_topic_name(), mavros_pose_publisher_->get_topic_name(),
            mavros_twist_publisher_->get_topic_name());
    }

    ~FlightMavros() override = default;

    void update() override { update_local_position(); }

private:
    void update_local_position() {
        if (++odom_publish_counter_ < 20)
            return;
        odom_publish_counter_ = 0;

        OdomSnapshot snap;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!odom_ready_)
                return;
            snap = latest_odom_;
            odom_ready_ = false;
        }

        if (std::isnan(snap.pos_x) || std::isnan(snap.pos_y) || std::isnan(snap.pos_z)
            || std::isnan(snap.vel_x) || std::isnan(snap.vel_y) || std::isnan(snap.vel_z)
            || std::isnan(snap.q_w) || std::isnan(snap.q_x) || std::isnan(snap.q_y)
            || std::isnan(snap.q_z)) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000, "Odometry snapshot contains NaN, skipping publish");
            return;
        }

        const auto timestamp =
            snap.timestamp_sample.nanoseconds() > 0 ? snap.timestamp_sample : get_clock()->now();
        Eigen::Quaterniond orientation{
            static_cast<double>(snap.q_w), static_cast<double>(snap.q_x),
            static_cast<double>(snap.q_y), static_cast<double>(snap.q_z)};
        if (orientation.norm() <= 1e-6) {
            RCLCPP_WARN_THROTTLE(
                logger_, *get_clock(), 1000, "Odometry snapshot contains invalid quaternion");
            return;
        }
        orientation.normalize();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg{};
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = get_parameter("mavros_pose_frame_id").as_string();
        pose_msg.pose.pose.position.x = snap.pos_x;
        pose_msg.pose.pose.position.y = snap.pos_y;
        pose_msg.pose.pose.position.z = snap.pos_z;
        pose_msg.pose.pose.orientation.x = orientation.x();
        pose_msg.pose.pose.orientation.y = orientation.y();
        pose_msg.pose.pose.orientation.z = orientation.z();
        pose_msg.pose.pose.orientation.w = orientation.w();
        pose_msg.pose.covariance = make_pose_covariance(snap);

        geometry_msgs::msg::TwistWithCovarianceStamped twist_msg{};
        twist_msg.header.stamp = timestamp;
        twist_msg.header.frame_id = get_parameter("mavros_pose_frame_id").as_string();
        twist_msg.twist.twist.linear.x = snap.vel_x;
        twist_msg.twist.twist.linear.y = snap.vel_y;
        twist_msg.twist.twist.linear.z = snap.vel_z;
        twist_msg.twist.twist.angular.x = snap.ang_x;
        twist_msg.twist.twist.angular.y = snap.ang_y;
        twist_msg.twist.twist.angular.z = snap.ang_z;
        twist_msg.twist.covariance = make_twist_covariance(snap);

        mavros_pose_publisher_->publish(pose_msg);
        mavros_twist_publisher_->publish(twist_msg);
    }

    struct OdomSnapshot {
        rclcpp::Time timestamp_sample{};
        float pos_x{}, pos_y{}, pos_z{};
        float vel_x{}, vel_y{}, vel_z{};
        float ang_x{}, ang_y{}, ang_z{};
        float q_x{}, q_y{}, q_z{}, q_w{1.f};
        float cov_xx{}, cov_yy{}, cov_zz{};
        float vel_cov_xx{}, vel_cov_yy{}, vel_cov_zz{};
    };

    static std::array<double, 36> make_pose_covariance(const OdomSnapshot& snap) {
        std::array<double, 36> covariance{};
        covariance[0] = sanitize_covariance(snap.cov_xx, 0.05);
        covariance[7] = sanitize_covariance(snap.cov_yy, 0.05);
        covariance[14] = sanitize_covariance(snap.cov_zz, 0.05);
        covariance[21] = 0.05;
        covariance[28] = 0.05;
        covariance[35] = 0.05;
        return covariance;
    }

    static std::array<double, 36> make_twist_covariance(const OdomSnapshot& snap) {
        std::array<double, 36> covariance{};
        covariance[0] = sanitize_covariance(snap.vel_cov_xx, 0.1);
        covariance[7] = sanitize_covariance(snap.vel_cov_yy, 0.1);
        covariance[14] = sanitize_covariance(snap.vel_cov_zz, 0.1);
        covariance[21] = 0.1;
        covariance[28] = 0.1;
        covariance[35] = 0.1;
        return covariance;
    }

    static double sanitize_covariance(float value, double fallback) {
        return std::isfinite(value) && value > 0.f ? static_cast<double>(value) : fallback;
    }

    void odin_odometry_subscription_callback(const nav_msgs::msg::Odometry& msg) {
        OdomSnapshot snap{};
        snap.timestamp_sample = rclcpp::Time{msg.header.stamp};
        snap.pos_x = static_cast<float>(msg.pose.pose.position.x);
        snap.pos_y = static_cast<float>(msg.pose.pose.position.y);
        snap.pos_z = static_cast<float>(msg.pose.pose.position.z);
        snap.vel_x = static_cast<float>(msg.twist.twist.linear.x);
        snap.vel_y = static_cast<float>(msg.twist.twist.linear.y);
        snap.vel_z = static_cast<float>(msg.twist.twist.linear.z);
        snap.ang_x = static_cast<float>(msg.twist.twist.angular.x);
        snap.ang_y = static_cast<float>(msg.twist.twist.angular.y);
        snap.ang_z = static_cast<float>(msg.twist.twist.angular.z);
        snap.q_x = static_cast<float>(msg.pose.pose.orientation.x);
        snap.q_y = static_cast<float>(msg.pose.pose.orientation.y);
        snap.q_z = static_cast<float>(msg.pose.pose.orientation.z);
        snap.q_w = static_cast<float>(msg.pose.pose.orientation.w);
        snap.cov_xx = static_cast<float>(msg.pose.covariance[0]);
        snap.cov_yy = static_cast<float>(msg.pose.covariance[7]);
        snap.cov_zz = static_cast<float>(msg.pose.covariance[14]);
        snap.vel_cov_xx = static_cast<float>(msg.twist.covariance[0]);
        snap.vel_cov_yy = static_cast<float>(msg.twist.covariance[7]);
        snap.vel_cov_zz = static_cast<float>(msg.twist.covariance[14]);

        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = snap;
        odom_ready_ = true;
    }

    rclcpp::Logger logger_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        mavros_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
        mavros_twist_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odin_odometry_subscription_;

    std::mutex odom_mutex_;
    OdomSnapshot latest_odom_;
    bool odom_ready_{false};
    uint32_t odom_publish_counter_{0};
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::FlightMavros, rmcs_executor::Component)
