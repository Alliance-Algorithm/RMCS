#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

namespace sp_vision_25::bridge {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double kRadToDeg = 57.3;
constexpr double kDegToRad = 1.0 / kRadToDeg;

Eigen::Vector3d angles_to_direction(double yaw, double pitch) {
    Eigen::Vector3d direction{
        std::cos(pitch) * std::cos(yaw),
        std::cos(pitch) * std::sin(yaw),
        -std::sin(pitch),
    };
    if (direction.norm() > 1e-9)
        direction.normalize();
    else
        direction.setZero();
    return direction;
}

std::pair<double, double> yaw_pitch_from_direction(const Eigen::Vector3d& direction) {
    const double xy_norm = std::hypot(direction.x(), direction.y());
    return {std::atan2(direction.y(), direction.x()), std::atan2(-direction.z(), xy_norm)};
}

std::filesystem::path
    resolve_path_parameter(const std::filesystem::path& package_share, const std::string& path) {
    std::filesystem::path result(path);
    if (result.empty())
        return package_share / "configs/standard3.yaml";
    if (result.is_relative())
        result = package_share / result;
    return result.lexically_normal();
}

std::filesystem::path
    resolve_package_asset_path(const std::filesystem::path& package_root, const std::string& path) {
    std::filesystem::path result(path);
    if (result.empty() || result.is_absolute())
        return result;
    return (package_root / result).lexically_normal();
}

std::filesystem::path prepare_runtime_config(
    const std::filesystem::path& config_path, const std::string& component_name) {
    YAML::Node yaml = YAML::LoadFile(config_path.string());

    std::filesystem::path package_root = config_path.parent_path().parent_path();
    if (!std::filesystem::exists(package_root / "assets"))
        package_root = config_path.parent_path();

    constexpr std::array<const char*, 5> path_keys{
        "classify_model", "yolo11_model_path", "yolov8_model_path", "yolov5_model_path", "model",
    };
    for (const char* key : path_keys) {
        if (!yaml[key] || !yaml[key].IsScalar())
            continue;
        yaml[key] = resolve_package_asset_path(package_root, yaml[key].as<std::string>()).string();
    }

    std::filesystem::path runtime_config =
        std::filesystem::temp_directory_path() / (component_name + "_resolved.yaml");
    YAML::Emitter emitter;
    emitter << yaml;

    std::ofstream output(runtime_config);
    if (!output.is_open())
        throw std::runtime_error("Failed to create runtime config: " + runtime_config.string());
    output << emitter.c_str();
    output.close();

    return runtime_config;
}

} // namespace

class SpVisionResponseTestBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SpVisionResponseTestBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        const std::string package_share =
            ament_index_cpp::get_package_share_directory("sp_vision_25");

        if (!has_parameter("config_file"))
            declare_parameter<std::string>(
                "config_file", package_share + "/configs/standard3.yaml");
        if (!has_parameter("bullet_speed_fallback"))
            declare_parameter<double>("bullet_speed_fallback", 23.0);
        if (!has_parameter("result_timeout"))
            declare_parameter<double>("result_timeout", 0.2);
        if (!has_parameter("debug"))
            declare_parameter<bool>("debug", false);

        if (!has_parameter("virtual_center_distance"))
            declare_parameter<double>("virtual_center_distance", 5.0);
        if (!has_parameter("virtual_center_yaw"))
            declare_parameter<double>("virtual_center_yaw", 0.0);
        if (!has_parameter("virtual_center_height"))
            declare_parameter<double>("virtual_center_height", 0.0);
        if (!has_parameter("virtual_angular_velocity"))
            declare_parameter<double>("virtual_angular_velocity", 2.0);
        if (!has_parameter("virtual_base_radius"))
            declare_parameter<double>("virtual_base_radius", 0.2);
        if (!has_parameter("virtual_radius_delta"))
            declare_parameter<double>("virtual_radius_delta", 0.05);
        if (!has_parameter("virtual_height_delta"))
            declare_parameter<double>("virtual_height_delta", 0.0);
        if (!has_parameter("virtual_initial_phase"))
            declare_parameter<double>("virtual_initial_phase", 0.0);
        if (!has_parameter("fire_control_enabled"))
            declare_parameter<bool>("fire_control_enabled", true);

        if (!has_parameter("output_csv_path"))
            declare_parameter<std::string>("output_csv_path", "/tmp/sp_vision_response_test.csv");
        if (!has_parameter("csv_decimation"))
            declare_parameter<int64_t>("csv_decimation", 1);

        register_input("/tf", tf_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/predefined/update_count", update_count_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_, false);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_, false);
        register_input("/referee/shooter/initial_speed", bullet_speed_, false);

        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_output("/gimbal/auto_aim/laser_distance", laser_distance_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw", plan_yaw_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch", plan_pitch_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, 0.0);
    }

    ~SpVisionResponseTestBridge() override { close_csv(); }

    void before_updating() override {
        bullet_speed_fallback_storage_ =
            static_cast<float>(get_parameter("bullet_speed_fallback").as_double());
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(bullet_speed_fallback_storage_);

        if (!yaw_velocity_imu_.ready())
            yaw_velocity_imu_.make_and_bind_directly(0.0);
        if (!pitch_velocity_imu_.ready())
            pitch_velocity_imu_.make_and_bind_directly(0.0);

        virtual_target_config_.center_distance =
            get_parameter("virtual_center_distance").as_double();
        virtual_target_config_.center_yaw =
            get_parameter("virtual_center_yaw").as_double() * kDegToRad;
        virtual_target_config_.center_height = get_parameter("virtual_center_height").as_double();
        virtual_target_config_.angular_velocity =
            get_parameter("virtual_angular_velocity").as_double();
        virtual_target_config_.base_radius = get_parameter("virtual_base_radius").as_double();
        virtual_target_config_.radius_delta = get_parameter("virtual_radius_delta").as_double();
        virtual_target_config_.height_delta = get_parameter("virtual_height_delta").as_double();
        virtual_target_config_.initial_phase =
            get_parameter("virtual_initial_phase").as_double() * kDegToRad;

        if (virtual_target_config_.center_distance <= 0.0)
            throw std::runtime_error{"virtual_center_distance must be positive"};
        if (virtual_target_config_.base_radius <= 0.0)
            throw std::runtime_error{"virtual_base_radius must be positive"};
        if (virtual_target_config_.base_radius + virtual_target_config_.radius_delta <= 0.0)
            throw std::runtime_error{"virtual_base_radius + virtual_radius_delta must be positive"};

        fire_control_enabled_ = get_parameter("fire_control_enabled").as_bool();
        csv_path_ = get_parameter("output_csv_path").as_string();
        csv_decimation_ = get_parameter("csv_decimation").as_int();
        if (csv_path_.empty())
            throw std::runtime_error{"output_csv_path cannot be empty"};
        if (csv_decimation_ <= 0)
            throw std::runtime_error{"csv_decimation must be positive"};

        const auto config_path = resolve_path_parameter(
            ament_index_cpp::get_package_share_directory("sp_vision_25"),
            get_parameter("config_file").as_string());
        runtime_config_path_ = prepare_runtime_config(config_path, get_component_name()).string();
        planner_ = std::make_unique<auto_aim::Planner>(runtime_config_path_);

        open_csv();
        RCLCPP_INFO(
            get_logger(),
            "Started sp_vision response test bridge. config=%s, csv=%s, center_distance=%.2f, "
            "angular_velocity=%.2f",
            runtime_config_path_.c_str(), csv_path_.c_str(), virtual_target_config_.center_distance,
            virtual_target_config_.angular_velocity);
    }

    void update() override {
        if (*update_count_ % 10 != 0)
            return;

        if (!start_time_initialized_) {
            start_time_ = *timestamp_;
            start_time_initialized_ = true;
        }

        double elapsed_seconds = std::chrono::duration<double>(*timestamp_ - start_time_).count();
        if (elapsed_seconds < 0.0)
            elapsed_seconds = 0.0;

        const auto target = build_virtual_target(elapsed_seconds);
        const double bullet_speed = static_cast<double>(*bullet_speed_);
        auto plan = planner_->plan(std::optional<auto_aim::Target>{target}, bullet_speed);

        if (!fire_control_enabled_)
            plan.fire = false;

        publish_plan(plan);
        maybe_log_csv(elapsed_seconds, target, plan, bullet_speed);
    }

private:
    struct VirtualTargetConfig {
        double center_distance = 5.0;
        double center_yaw = 0.0;
        double center_height = 0.0;
        double angular_velocity = 2.0;
        double base_radius = 0.2;
        double radius_delta = 0.05;
        double height_delta = 0.0;
        double initial_phase = 0.0;
    };

    auto_aim::Target build_virtual_target(double elapsed_seconds) const {
        const Eigen::Vector3d center_xyz{
            virtual_target_config_.center_distance * std::cos(virtual_target_config_.center_yaw),
            virtual_target_config_.center_distance * std::sin(virtual_target_config_.center_yaw),
            virtual_target_config_.center_height,
        };
        const double angle = tools::limit_rad(
            virtual_target_config_.initial_phase
            + virtual_target_config_.angular_velocity * elapsed_seconds);

        return auto_aim::Target(
            center_xyz, angle, virtual_target_config_.angular_velocity,
            virtual_target_config_.base_radius, virtual_target_config_.radius_delta,
            virtual_target_config_.height_delta);
    }

    std::pair<double, double> current_world_yaw_pitch() const {
        auto direction = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        Eigen::Vector3d vector = *direction;
        if (vector.norm() > 1e-9)
            vector.normalize();
        else
            vector = Eigen::Vector3d::UnitX();
        return yaw_pitch_from_direction(vector);
    }

    void publish_plan(const auto_aim::Plan& plan) {
        if (!plan.control) {
            *control_direction_ = Eigen::Vector3d::Zero();
            *fire_control_ = false;
            *laser_distance_ = 0.0;
            *plan_yaw_ = 0.0;
            *plan_pitch_ = 0.0;
            *plan_yaw_velocity_ = 0.0;
            *plan_yaw_acceleration_ = 0.0;
            *plan_pitch_velocity_ = 0.0;
            *plan_pitch_acceleration_ = 0.0;
            return;
        }

        *control_direction_ = angles_to_direction(plan.yaw, plan.pitch);
        *fire_control_ = fire_control_enabled_ && plan.fire;
        *laser_distance_ = planner_->debug_xyza.head<3>().norm();
        *plan_yaw_ = plan.yaw;
        *plan_pitch_ = plan.pitch;
        *plan_yaw_velocity_ = plan.yaw_vel;
        *plan_yaw_acceleration_ = plan.yaw_acc;
        *plan_pitch_velocity_ = plan.pitch_vel;
        *plan_pitch_acceleration_ = plan.pitch_acc;
    }

    void open_csv() {
        close_csv();

        const std::filesystem::path path{csv_path_};
        if (path.has_parent_path())
            std::filesystem::create_directories(path.parent_path());

        csv_stream_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_stream_.is_open())
            throw std::runtime_error{"failed to open csv path: " + csv_path_};

        csv_stream_ << std::fixed << std::setprecision(9);
        csv_stream_ << "update_count,elapsed_s,bullet_speed,"
                       "target_center_x,target_center_y,target_center_z,target_angle,target_"
                       "angular_velocity,"
                       "target_base_radius,target_radius_delta,target_height_delta,"
                       "plan_target_yaw,plan_target_pitch,"
                       "expected_yaw,expected_pitch,expected_yaw_velocity,expected_pitch_velocity,"
                       "expected_yaw_acceleration,expected_pitch_acceleration,"
                       "actual_yaw,actual_pitch,actual_yaw_velocity,actual_pitch_velocity,"
                       "yaw_error,pitch_error,fire_control,laser_distance\n";
    }

    void close_csv() {
        if (!csv_stream_.is_open())
            return;

        csv_stream_.flush();
        csv_stream_.close();
    }

    void maybe_log_csv(
        double elapsed_seconds, const auto_aim::Target& target, const auto_aim::Plan& plan,
        double bullet_speed) {
        if (!csv_stream_.is_open())
            return;
        const bool allow_csv_logging =
            *switch_right_ == rmcs_msgs::Switch::UP && *switch_left_ != rmcs_msgs::Switch::DOWN;
        if (!allow_csv_logging)
            return;
        if (*update_count_ % static_cast<std::size_t>(csv_decimation_) != 0)
            return;

        const auto target_state = target.ekf_x();
        const auto [actual_yaw, actual_pitch] = current_world_yaw_pitch();
        const double plan_target_yaw = plan.control ? static_cast<double>(plan.target_yaw) : 0.0;
        const double plan_target_pitch = plan.control ? static_cast<double>(plan.target_pitch) : 0.0;
        const double expected_yaw = plan.control ? static_cast<double>(plan.yaw) : 0.0;
        const double expected_pitch = plan.control ? static_cast<double>(plan.pitch) : 0.0;
        const double expected_yaw_velocity = plan.control ? static_cast<double>(plan.yaw_vel) : 0.0;
        const double expected_pitch_velocity =
            plan.control ? static_cast<double>(plan.pitch_vel) : 0.0;
        const double expected_yaw_acceleration =
            plan.control ? static_cast<double>(plan.yaw_acc) : 0.0;
        const double expected_pitch_acceleration =
            plan.control ? static_cast<double>(plan.pitch_acc) : 0.0;
        const double yaw_error = plan.control ? tools::limit_rad(expected_yaw - actual_yaw) : 0.0;
        const double pitch_error = plan.control ? (expected_pitch - actual_pitch) : 0.0;
        const bool fire_control = fire_control_enabled_ && plan.control && plan.fire;
        const double laser_distance = plan.control ? planner_->debug_xyza.head<3>().norm() : 0.0;

        csv_stream_ << *update_count_ << ',' << elapsed_seconds << ',' << bullet_speed << ','
                    << target_state[0] << ',' << target_state[2] << ',' << target_state[4] << ','
                    << target_state[6] << ',' << target_state[7] << ',' << target_state[8] << ','
                    << target_state[9] << ',' << target_state[10] << ',' << plan_target_yaw << ','
                    << plan_target_pitch << ',' << expected_yaw << ',' << expected_pitch << ','
                    << expected_yaw_velocity << ',' << expected_pitch_velocity << ','
                    << expected_yaw_acceleration << ',' << expected_pitch_acceleration << ','
                    << actual_yaw << ',' << actual_pitch << ',' << *yaw_velocity_imu_ << ','
                    << *pitch_velocity_imu_ << ',' << yaw_error << ',' << pitch_error << ','
                    << (fire_control ? 1 : 0) << ',' << laser_distance << '\n';

        if (*update_count_ % 100 == 0)
            csv_stream_.flush();
    }

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<std::size_t> update_count_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;
    InputInterface<float> bullet_speed_;

    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;
    OutputInterface<double> laser_distance_;
    OutputInterface<double> plan_yaw_;
    OutputInterface<double> plan_pitch_;
    OutputInterface<double> plan_yaw_velocity_;
    OutputInterface<double> plan_yaw_acceleration_;
    OutputInterface<double> plan_pitch_velocity_;
    OutputInterface<double> plan_pitch_acceleration_;

    Clock::time_point start_time_{};
    bool start_time_initialized_ = false;
    float bullet_speed_fallback_storage_ = 23.0F;
    bool fire_control_enabled_ = true;
    VirtualTargetConfig virtual_target_config_;
    std::string runtime_config_path_;

    std::unique_ptr<auto_aim::Planner> planner_;

    std::string csv_path_;
    int64_t csv_decimation_ = 1;
    std::ofstream csv_stream_;
};

} // namespace sp_vision_25::bridge

PLUGINLIB_EXPORT_CLASS(sp_vision_25::bridge::SpVisionResponseTestBridge, rmcs_executor::Component)
