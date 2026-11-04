#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "controller/chassis/wheel_leg/joint_coordinates.hpp"

namespace rmcs_core::broadcaster {

class WheelLegTelemetry
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
    using JointVector = controller::chassis::JointVector;

public:
    WheelLegTelemetry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)} {
        enabled_ = get_parameter_or("enabled", true);
        const double rate = get_parameter_or("update_rate", 100.0);
        if (!std::isfinite(rate) || rate <= 0. || rate > 1000.)
            throw std::invalid_argument("Wheel-leg telemetry update_rate must be in (0, 1000]");
        period_ =
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>{1. / rate});
        if (!enabled_)
            return;

        register_input("/predefined/timestamp", timestamp_);
        register_input("/chassis/reset_count", reset_);
        for (std::size_t i = 0; i < controller::chassis::kMotorNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + controller::chassis::kMotorNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
        }
        register_input("/wheel_leg/imu/quaternion", orientation_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/imu/last_steady_ns", imu_ns_);
        register_input("/wheel_leg/feedback_fresh", fresh_);
        register_input("/wheel_leg/state/valid", valid_);
        register_input("/wheel_leg/state/position", model_position_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/state/passive_knee", knee_);
        register_input("/wheel_leg/state/leg_length", length_);

        const auto qos = rclcpp::QoS{5}.best_effort().durability_volatile();
        joints_pub_ = create_publisher<sensor_msgs::msg::JointState>(kPrefix + "joint_states", qos);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(kPrefix + "imu_body", qos);
        gravity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            kPrefix + "rl_projected_gravity", qos);
        gyro_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
            kPrefix + "rl_angular_velocity", qos);
        diagnostics_pub_ =
            create_publisher<diagnostic_msgs::msg::DiagnosticArray>(kPrefix + "diagnostics", qos);
    }

    void update() override {
        if (!enabled_)
            return;
        const auto now = *timestamp_;
        if (*reset_ != last_reset_) {
            last_publish_.reset();
            last_reset_ = *reset_;
        }
        if (last_publish_ && now >= *last_publish_) {
            const auto elapsed = now - *last_publish_;
            if (elapsed < period_)
                return;
            last_publish_ = now - elapsed % period_;
        } else {
            last_publish_ = now;
        }

        // One executor snapshot, one ROS timestamp for every topic. Per-sensor
        // ages below retain the asynchronous CAN/IMU acquisition information.
        std_msgs::msg::Header header;
        header.stamp = get_clock()->now();
        header.frame_id = "chassis_body";
        sensor_msgs::msg::JointState joints;
        joints.header = header;
        for (std::size_t i = 0; i < angle_.size(); ++i) {
            joints.name.emplace_back(controller::chassis::kMotorNames[i]);
            joints.position.push_back(*angle_[i]);
            joints.velocity.push_back(*velocity_[i]);
        }

        sensor_msgs::msg::Imu imu;
        imu.header = header;
        const auto& q = *orientation_;
        imu.orientation.w = q.w();
        imu.orientation.x = q.x();
        imu.orientation.y = q.y();
        imu.orientation.z = q.z();
        imu.angular_velocity.x = gyro_->x();
        imu.angular_velocity.y = gyro_->y();
        imu.angular_velocity.z = gyro_->z();
        // Covariances are unknown; acceleration is not part of this snapshot.
        imu.linear_acceleration_covariance[0] = -1.;
        if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1.) > .01)
            imu.orientation_covariance[0] = -1.;

        geometry_msgs::msg::Vector3Stamped gravity, gyro;
        gravity.header = gyro.header = header;
        gravity.vector.x = gravity_->x();
        gravity.vector.y = gravity_->y();
        gravity.vector.z = gravity_->z();
        // Unscaled physical inputs to the policy, before observation scaling.
        gyro.vector = imu.angular_velocity;

        diagnostic_msgs::msg::DiagnosticArray diagnostics;
        diagnostics.header = header;
        auto& status = diagnostics.status.emplace_back();
        status.name = "wheel_leg/telemetry";
        status.hardware_id = "wheel_leg";
        status.level = *fresh_ && *valid_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                          : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = *fresh_ && *valid_ ? "valid" : "feedback or estimator invalid";
        const auto add = [&](const std::string& key, const std::string& value) {
            auto& entry = status.values.emplace_back();
            entry.key = key;
            entry.value = value;
        };
        add("schema", "wheel_leg_telemetry_v1");
        add("coordinate", "rmcs_hardware_output_shaft_v1");
        add("calibration_sha256",
            std::string{controller::chassis::full_vehicle_calibration110::kCalibrationSha256});
        add("calibration_model_spec_sha256",
            std::string{controller::chassis::full_vehicle_calibration110::kModelSpecSha256});
        add("feedback_fresh", *fresh_ ? "true" : "false");
        add("state_valid", *valid_ ? "true" : "false");
        add("reset_count", std::to_string(*reset_));
        const auto steady_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
                .count();
        const auto age = [&](std::uint64_t sample_ns) {
            return sample_ns == 0 || sample_ns > static_cast<std::uint64_t>(steady_ns)
                     ? -1.
                     : static_cast<double>(steady_ns - sample_ns) * 1e-9;
        };
        add("imu_age_s", number(age(*imu_ns_)));
        for (std::size_t i = 0; i < angle_.size(); ++i) {
            const auto name = std::string{controller::chassis::kMotorNames[i]};
            add("motor_age_s/" + name, number(age(*feedback_ns_[i])));
            add("model_position_rad/" + name, number((*model_position_)[i]));
            if (i < 4)
                add("model_offset_rad/" + name, number(controller::chassis::kMotorOffsets[i]));
        }
        for (std::size_t side = 0; side < 2; ++side) {
            const auto prefix = std::string{side == 0 ? "left/" : "right/"};
            add(prefix + "passive_knee_rad", number((*knee_)[side]));
            add(prefix + "leg_length_m", number((*length_)[side]));
        }

        joints_pub_->publish(joints);
        imu_pub_->publish(imu);
        gravity_pub_->publish(gravity);
        gyro_pub_->publish(gyro);
        diagnostics_pub_->publish(diagnostics);
    }

private:
    static std::string number(double value) {
        std::ostringstream stream;
        stream << std::setprecision(17) << value;
        return stream.str();
    }

    inline static const std::string kPrefix = "/wheel_leg/telemetry/";
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> reset_;
    std::array<InputInterface<double>, 6> angle_, velocity_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_ns_;
    InputInterface<Eigen::Quaterniond> orientation_;
    InputInterface<Eigen::Vector3d> gyro_, gravity_;
    InputInterface<std::uint64_t> imu_ns_;
    InputInterface<bool> fresh_, valid_;
    InputInterface<JointVector> model_position_;
    InputInterface<std::array<double, 2>> knee_, length_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr gravity_pub_, gyro_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    bool enabled_ = true;
    Clock::duration period_{};
    std::optional<Clock::time_point> last_publish_;
    std::size_t last_reset_ = 0;
};

} // namespace rmcs_core::broadcaster

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::broadcaster::WheelLegTelemetry, rmcs_executor::Component)
