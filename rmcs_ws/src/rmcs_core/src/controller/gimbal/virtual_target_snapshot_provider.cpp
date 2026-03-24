#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

namespace rmcs_core::controller::gimbal {

namespace {

constexpr double kDegToRad = 1.0 / 57.3;

template <typename T>
void declare_parameter_if_missing(rclcpp::Node& node, const std::string& name, const T& value) {
    if (!node.has_parameter(name))
        node.declare_parameter<T>(name, value);
}

double limit_rad(double angle) {
    constexpr double kPi = std::numbers::pi_v<double>;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle <= -kPi)
        angle += 2.0 * kPi;
    return angle;
}

std::string normalize_token(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(value.begin(), value.end(), '-', '_');
    return value;
}

rmcs_msgs::TargetSnapshotArmorType parse_armor_type(const std::string& value) {
    const std::string normalized = normalize_token(value);
    if (normalized == "big")
        return rmcs_msgs::TargetSnapshotArmorType::BIG;
    if (normalized == "small")
        return rmcs_msgs::TargetSnapshotArmorType::SMALL;
    throw std::runtime_error("Unsupported virtual_armor_type: " + value);
}

rmcs_msgs::TargetSnapshotArmorName parse_armor_name(const std::string& value) {
    const std::string normalized = normalize_token(value);
    if (normalized == "one")
        return rmcs_msgs::TargetSnapshotArmorName::ONE;
    if (normalized == "two")
        return rmcs_msgs::TargetSnapshotArmorName::TWO;
    if (normalized == "three")
        return rmcs_msgs::TargetSnapshotArmorName::THREE;
    if (normalized == "four")
        return rmcs_msgs::TargetSnapshotArmorName::FOUR;
    if (normalized == "five")
        return rmcs_msgs::TargetSnapshotArmorName::FIVE;
    if (normalized == "sentry")
        return rmcs_msgs::TargetSnapshotArmorName::SENTRY;
    if (normalized == "outpost")
        return rmcs_msgs::TargetSnapshotArmorName::OUTPOST;
    if (normalized == "base")
        return rmcs_msgs::TargetSnapshotArmorName::BASE;
    if (normalized == "not_armor")
        return rmcs_msgs::TargetSnapshotArmorName::NOT_ARMOR;
    throw std::runtime_error("Unsupported virtual_armor_name: " + value);
}

} // namespace

class VirtualTargetSnapshotProvider
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    using Clock = rmcs_msgs::TargetSnapshot::Clock;

    VirtualTargetSnapshotProvider()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        declare_parameter_if_missing(*this, "virtual_valid", true);
        declare_parameter_if_missing(*this, "virtual_converged", false);
        declare_parameter_if_missing(*this, "virtual_armor_count", int64_t{4});
        declare_parameter_if_missing(*this, "virtual_armor_type", std::string{"small"});
        declare_parameter_if_missing(*this, "virtual_armor_name", std::string{"three"});

        declare_parameter_if_missing(*this, "virtual_center_distance", 5.0);
        declare_parameter_if_missing(*this, "virtual_center_yaw", 0.0);
        declare_parameter_if_missing(*this, "virtual_center_height", 0.0);
        declare_parameter_if_missing(*this, "virtual_center_velocity_x", 0.0);
        declare_parameter_if_missing(*this, "virtual_center_velocity_y", 0.0);
        declare_parameter_if_missing(*this, "virtual_center_velocity_z", 0.0);
        declare_parameter_if_missing(*this, "virtual_angular_velocity", 2.0);
        declare_parameter_if_missing(*this, "virtual_base_radius", 0.2);
        declare_parameter_if_missing(*this, "virtual_radius_delta", 0.05);
        declare_parameter_if_missing(*this, "virtual_height_delta", 0.0);
        declare_parameter_if_missing(*this, "virtual_initial_phase", 0.0);

        register_input("/predefined/timestamp", timestamp_);
        register_output(
            "/gimbal/auto_aim/target_snapshot", target_snapshot_, rmcs_msgs::TargetSnapshot{});
    }

    void before_updating() override {
        config_ = load_config(*this);
        start_time_initialized_ = false;
    }

    void update() override {
        const auto now = *timestamp_;
        if (!start_time_initialized_) {
            start_time_ = now;
            start_time_initialized_ = true;
        }

        double elapsed_seconds = std::chrono::duration<double>(now - start_time_).count();
        if (elapsed_seconds < 0.0)
            elapsed_seconds = 0.0;

        *target_snapshot_ = build_snapshot(now, elapsed_seconds);
    }

private:
    struct VirtualTargetConfig {
        bool valid = true;
        bool converged = false;
        uint8_t armor_count = 4;
        rmcs_msgs::TargetSnapshotArmorType armor_type = rmcs_msgs::TargetSnapshotArmorType::SMALL;
        rmcs_msgs::TargetSnapshotArmorName armor_name =
            rmcs_msgs::TargetSnapshotArmorName::THREE;

        double center_distance = 5.0;
        double center_yaw = 0.0;
        double center_height = 0.0;
        double center_velocity_x = 0.0;
        double center_velocity_y = 0.0;
        double center_velocity_z = 0.0;
        double angular_velocity = 2.0;
        double base_radius = 0.2;
        double radius_delta = 0.05;
        double height_delta = 0.0;
        double initial_phase = 0.0;
    };

    static VirtualTargetConfig load_config(rclcpp::Node& node) {
        VirtualTargetConfig config;

        config.valid = node.get_parameter("virtual_valid").as_bool();
        config.converged = node.get_parameter("virtual_converged").as_bool();

        const auto armor_count = node.get_parameter("virtual_armor_count").as_int();
        if (armor_count <= 0 || armor_count > std::numeric_limits<uint8_t>::max()) {
            throw std::runtime_error{"virtual_armor_count must be in [1, 255]"};
        }
        config.armor_count = static_cast<uint8_t>(armor_count);
        config.armor_type =
            parse_armor_type(node.get_parameter("virtual_armor_type").as_string());
        config.armor_name =
            parse_armor_name(node.get_parameter("virtual_armor_name").as_string());

        config.center_distance = node.get_parameter("virtual_center_distance").as_double();
        config.center_yaw = node.get_parameter("virtual_center_yaw").as_double() * kDegToRad;
        config.center_height = node.get_parameter("virtual_center_height").as_double();
        config.center_velocity_x = node.get_parameter("virtual_center_velocity_x").as_double();
        config.center_velocity_y = node.get_parameter("virtual_center_velocity_y").as_double();
        config.center_velocity_z = node.get_parameter("virtual_center_velocity_z").as_double();
        config.angular_velocity = node.get_parameter("virtual_angular_velocity").as_double();
        config.base_radius = node.get_parameter("virtual_base_radius").as_double();
        config.radius_delta = node.get_parameter("virtual_radius_delta").as_double();
        config.height_delta = node.get_parameter("virtual_height_delta").as_double();
        config.initial_phase =
            node.get_parameter("virtual_initial_phase").as_double() * kDegToRad;

        if (config.center_distance <= 0.0)
            throw std::runtime_error{"virtual_center_distance must be positive"};
        if (config.base_radius <= 0.0)
            throw std::runtime_error{"virtual_base_radius must be positive"};
        if (config.base_radius + config.radius_delta <= 0.0) {
            throw std::runtime_error{"virtual_base_radius + virtual_radius_delta must be positive"};
        }

        return config;
    }

    rmcs_msgs::TargetSnapshot
        build_snapshot(const Clock::time_point& timestamp, double elapsed_seconds) const {
        rmcs_msgs::TargetSnapshot snapshot;
        snapshot.valid = config_.valid;
        snapshot.converged = config_.converged;
        snapshot.armor_count = config_.armor_count;
        snapshot.armor_type = config_.armor_type;
        snapshot.armor_name = config_.armor_name;
        snapshot.timestamp = timestamp;

        if (!snapshot.valid)
            return snapshot;

        const double initial_center_x = config_.center_distance * std::cos(config_.center_yaw);
        const double initial_center_y = config_.center_distance * std::sin(config_.center_yaw);

        snapshot.state[0] = initial_center_x + config_.center_velocity_x * elapsed_seconds;
        snapshot.state[1] = config_.center_velocity_x;
        snapshot.state[2] = initial_center_y + config_.center_velocity_y * elapsed_seconds;
        snapshot.state[3] = config_.center_velocity_y;
        snapshot.state[4] = config_.center_height + config_.center_velocity_z * elapsed_seconds;
        snapshot.state[5] = config_.center_velocity_z;
        snapshot.state[6] =
            limit_rad(config_.initial_phase + config_.angular_velocity * elapsed_seconds);
        snapshot.state[7] = config_.angular_velocity;
        snapshot.state[8] = config_.base_radius;
        snapshot.state[9] = config_.radius_delta;
        snapshot.state[10] = config_.height_delta;
        return snapshot;
    }

    InputInterface<Clock::time_point> timestamp_;
    OutputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;

    VirtualTargetConfig config_;
    Clock::time_point start_time_{};
    bool start_time_initialized_ = false;
};

} // namespace rmcs_core::controller::gimbal

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::VirtualTargetSnapshotProvider, rmcs_executor::Component)
