#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

#include "controller/gimbal/planner.hpp"

namespace rmcs_core::controller::gimbal {

namespace {

constexpr double kDegToRad = 1.0 / 57.3;

Eigen::Vector3d planner_direction(double yaw, double pitch) {
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

PlannerConfig load_planner_config(rclcpp::Node& node) {
    PlannerConfig config;
    config.yaw_offset = node.get_parameter("yaw_offset").as_double() * kDegToRad;
    config.pitch_offset = node.get_parameter("pitch_offset").as_double() * kDegToRad;
    config.fire_thresh = node.get_parameter("fire_thresh").as_double();
    config.low_speed_delay_time = node.get_parameter("low_speed_delay_time").as_double();
    config.high_speed_delay_time = node.get_parameter("high_speed_delay_time").as_double();
    config.decision_speed = node.get_parameter("decision_speed").as_double();
    config.max_yaw_acc = node.get_parameter("max_yaw_acc").as_double();
    config.max_pitch_acc = node.get_parameter("max_pitch_acc").as_double();
    return config;
}

double parameter_or_declare(rclcpp::Node& node, const std::string& name, double default_value) {
    if (!node.has_parameter(name))
        node.declare_parameter<double>(name, default_value);
    return node.get_parameter(name).as_double();
}

} // namespace

// Converts TargetSnapshot (from sp_vision_bridge) into the unified auto-aim interface:
// control_direction / auto_aim_enabled / shoot_enable / feedforward / debug plan_*.
// FlightGimbalController consumes that interface without knowing the source.
class SpVisionPlanner
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SpVisionPlanner()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , planner_(load_planner_config(*this))
        , yaw_vel_ff_gain_(parameter_or_declare(*this, "yaw_vel_ff_gain", 0.0))
        , yaw_acc_ff_gain_(parameter_or_declare(*this, "yaw_acc_ff_gain", 0.0))
        , pitch_vel_ff_gain_(parameter_or_declare(*this, "pitch_vel_ff_gain", 1.0)) {
        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/referee/shooter/initial_speed", bullet_speed_, false);

        register_output("/gimbal/auto_aim/auto_aim_enabled", auto_aim_enabled_, false);
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/shoot_enable", shoot_enable_, false);
        register_output("/gimbal/auto_aim/yaw_feedforward", yaw_feedforward_, 0.0);
        register_output("/gimbal/auto_aim/pitch_feedforward", pitch_feedforward_, 0.0);
        register_output("/gimbal/auto_aim/laser_distance", laser_distance_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw", plan_yaw_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch", plan_pitch_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, 0.0);

        if (!has_parameter("bullet_speed_fallback"))
            declare_parameter<double>("bullet_speed_fallback", 23.0);
        if (!has_parameter("result_timeout"))
            declare_parameter<double>("result_timeout", 0.2);

        result_timeout_ =
            std::chrono::duration<double>(get_parameter("result_timeout").as_double());
        bullet_speed_fallback_storage_ =
            static_cast<float>(get_parameter("bullet_speed_fallback").as_double());
    }

    void before_updating() override {
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(bullet_speed_fallback_storage_);
        clear_outputs();
    }

    void update() override {
        const auto now = *timestamp_;
        const auto snapshot = *target_snapshot_;
        const double age_s = std::chrono::duration<double>(now - snapshot.timestamp).count();
        const bool fresh = snapshot.valid && age_s <= result_timeout_.count();
        if (!fresh) {
            clear_outputs();
            return;
        }

        const PlannerResult result = planner_.plan(
            std::optional<rmcs_msgs::TargetSnapshot>{snapshot}, now,
            static_cast<double>(*bullet_speed_));
        if (!result.control) {
            clear_outputs();
            return;
        }

        *auto_aim_enabled_ = true;
        *control_direction_ = planner_direction(result.yaw, result.pitch);
        *shoot_enable_ = result.fire;
        *yaw_feedforward_ =
            yaw_vel_ff_gain_ * result.yaw_velocity + yaw_acc_ff_gain_ * result.yaw_acceleration;
        *pitch_feedforward_ = pitch_vel_ff_gain_ * result.pitch_velocity;
        *laser_distance_ = result.control_xyza.head<3>().norm();
        *plan_yaw_ = result.yaw;
        *plan_pitch_ = result.pitch;
        *plan_yaw_velocity_ = result.yaw_velocity;
        *plan_yaw_acceleration_ = result.yaw_acceleration;
        *plan_pitch_velocity_ = result.pitch_velocity;
        *plan_pitch_acceleration_ = result.pitch_acceleration;
    }

private:
    void clear_outputs() {
        *auto_aim_enabled_ = false;
        *control_direction_ = Eigen::Vector3d::Zero();
        *shoot_enable_ = false;
        *yaw_feedforward_ = 0.0;
        *pitch_feedforward_ = 0.0;
        *laser_distance_ = 0.0;
        *plan_yaw_ = 0.0;
        *plan_pitch_ = 0.0;
        *plan_yaw_velocity_ = 0.0;
        *plan_yaw_acceleration_ = 0.0;
        *plan_pitch_velocity_ = 0.0;
        *plan_pitch_acceleration_ = 0.0;
    }

    Planner planner_;
    std::chrono::duration<double> result_timeout_{0.2};
    float bullet_speed_fallback_storage_ = 23.0F;

    const double yaw_vel_ff_gain_;
    const double yaw_acc_ff_gain_;
    const double pitch_vel_ff_gain_;

    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<float> bullet_speed_;

    OutputInterface<bool> auto_aim_enabled_;
    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> shoot_enable_;
    OutputInterface<double> yaw_feedforward_;
    OutputInterface<double> pitch_feedforward_;
    OutputInterface<double> laser_distance_;
    OutputInterface<double> plan_yaw_;
    OutputInterface<double> plan_pitch_;
    OutputInterface<double> plan_yaw_velocity_;
    OutputInterface<double> plan_yaw_acceleration_;
    OutputInterface<double> plan_pitch_velocity_;
    OutputInterface<double> plan_pitch_acceleration_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SpVisionPlanner, rmcs_executor::Component)
