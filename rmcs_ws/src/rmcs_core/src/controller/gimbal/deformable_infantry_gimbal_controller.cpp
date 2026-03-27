#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_msgs/target_snapshot.hpp>

#include "controller/gimbal/planner.hpp"
#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

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

void load_optional_parameter(rclcpp::Node& node, const std::string& name, double& value) {
    node.get_parameter(name, value);
}

void configure_pid(rclcpp::Node& node, const std::string& prefix, pid::PidCalculator& calculator) {
    load_optional_parameter(node, prefix + "_integral_min", calculator.integral_min);
    load_optional_parameter(node, prefix + "_integral_max", calculator.integral_max);
    load_optional_parameter(node, prefix + "_integral_split_min", calculator.integral_split_min);
    load_optional_parameter(node, prefix + "_integral_split_max", calculator.integral_split_max);
    load_optional_parameter(node, prefix + "_output_min", calculator.output_min);
    load_optional_parameter(node, prefix + "_output_max", calculator.output_max);
}

double parameter_or_declare(rclcpp::Node& node, const std::string& name, double default_value) {
    if (!node.has_parameter(name))
        node.declare_parameter<double>(name, default_value);
    return node.get_parameter(name).as_double();
}

PlannerConfig load_planner_config(rclcpp::Node& node) {
    PlannerConfig config;
    config.yaw_offset = parameter_or_declare(node, "yaw_offset", 0.0) * kDegToRad;
    config.pitch_offset = parameter_or_declare(node, "pitch_offset", 0.0) * kDegToRad;
    config.fire_thresh = parameter_or_declare(node, "fire_thresh", 0.0035);
    config.low_speed_delay_time = parameter_or_declare(node, "low_speed_delay_time", 0.0);
    config.high_speed_delay_time = parameter_or_declare(node, "high_speed_delay_time", 0.0);
    config.decision_speed = parameter_or_declare(node, "decision_speed", 7.0);
    config.max_yaw_acc = parameter_or_declare(node, "max_yaw_acc", 500.0);
    config.max_pitch_acc = parameter_or_declare(node, "max_pitch_acc", 100.0);
    return config;
}

} // namespace

class DeformableInfantryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    struct YawVelocityFeedback {
        double chassis = 0.0;
        double total = 0.0;
    };

    DeformableInfantryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , gimbal_solver_(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double())
        , planner_(load_planner_config(*this))
        , yaw_angle_pid_(
              get_parameter("yaw_angle_kp").as_double(), get_parameter("yaw_angle_ki").as_double(),
              get_parameter("yaw_angle_kd").as_double())
        , yaw_velocity_pid_(
              get_parameter("yaw_velocity_kp").as_double(),
              get_parameter("yaw_velocity_ki").as_double(),
              get_parameter("yaw_velocity_kd").as_double())
        , pitch_angle_pid_(
              get_parameter("pitch_angle_kp").as_double(),
              get_parameter("pitch_angle_ki").as_double(),
              get_parameter("pitch_angle_kd").as_double())
        , yaw_vel_ff_gain_(parameter_or_declare(*this, "yaw_vel_ff_gain", 0.0))
        , pitch_vel_ff_gain_(parameter_or_declare(*this, "pitch_vel_ff_gain", 1.0))
        , yaw_acc_ff_gain_(parameter_or_declare(*this, "yaw_acc_ff_gain", 0.0))
        , yaw_chassis_spin_torque_ff_gain_(
              parameter_or_declare(*this, "yaw_chassis_spin_torque_ff_gain", 0.0)) {
        if (!has_parameter("bullet_speed_fallback"))
            declare_parameter<double>("bullet_speed_fallback", 23.0);
        if (!has_parameter("result_timeout"))
            declare_parameter<double>("result_timeout", 0.2);

        planner_result_timeout_ =
            std::chrono::duration<double>(get_parameter("result_timeout").as_double());
        bullet_speed_fallback_storage_ =
            static_cast<float>(get_parameter("bullet_speed_fallback").as_double());

        configure_pid(*this, "yaw_angle", yaw_angle_pid_);
        configure_pid(*this, "yaw_velocity", yaw_velocity_pid_);
        configure_pid(*this, "pitch_angle", pitch_angle_pid_);
        load_optional_parameter(*this, "manual_joystick_sensitivity", joystick_sensitivity_);
        load_optional_parameter(*this, "manual_mouse_sensitivity", mouse_sensitivity_);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/predefined/timestamp", timestamp_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_);
        register_input("/referee/shooter/initial_speed", bullet_speed_, false);
        register_input("/gimbal/auto_aim/target_snapshot", target_snapshot_, false);

        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_output("/gimbal/auto_aim/laser_distance", laser_distance_, 0.0);
        register_output("/gimbal/auto_aim/plan_target_yaw", plan_target_yaw_, 0.0);
        register_output("/gimbal/auto_aim/plan_target_pitch", plan_target_pitch_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw", plan_yaw_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch", plan_pitch_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, 0.0);
        register_output("/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, 0.0);
        register_output("/gimbal/auto_aim/yaw_velocity_ff", yaw_velocity_ff_, 0.0);
        register_output("/gimbal/auto_aim/pitch_velocity_ff", pitch_velocity_ff_, 0.0);
        register_output("/gimbal/auto_aim/yaw_acceleration_ff", yaw_acceleration_ff_, 0.0);
        register_output("/gimbal/auto_aim/yaw_velocity_ref", yaw_velocity_ref_, 0.0);
        register_output("/gimbal/auto_aim/pitch_velocity_ref", pitch_velocity_ref_, 0.0);
        register_output("/gimbal/auto_aim/yaw_velocity_feedback", yaw_velocity_feedback_, 0.0);
        register_output("/gimbal/auto_aim/pitch_velocity_feedback", pitch_velocity_feedback_, 0.0);
        register_output(
            "/gimbal/auto_aim/chassis_yaw_velocity_imu", chassis_yaw_velocity_imu_, 0.0);
        register_output(
            "/gimbal/auto_aim/yaw_chassis_spin_torque_ff", yaw_chassis_spin_torque_ff_, 0.0);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_, nan_);
        register_output("/gimbal/pitch/control_velocity", pitch_control_velocity_, nan_);
        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);
    }

    void before_updating() override {
        if (!bullet_speed_.ready())
            bullet_speed_.bind_directly(bullet_speed_fallback_storage_);
        if (!target_snapshot_.ready())
            target_snapshot_.make_and_bind_directly(rmcs_msgs::TargetSnapshot{});
        clear_planner_outputs();
        clear_control_diagnostics();
    }

    void update() override {
        const auto switch_right = *switch_right_;
        const auto switch_left = *switch_left_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            clear_planner_outputs();
            clear_control_diagnostics();
            reset_all_controls();
            return;
        }

        const PlannerResult planner_result = update_planner_outputs();
        const bool planner_active = auto_aim_requested() && planner_result.control;
        const TwoAxisGimbalSolver::AngleError angle_error =
            planner_active ? update_auto_aim_control(*control_direction_) : update_manual_control();

        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;

        if (!std::isfinite(angle_error.yaw_angle_error)
            || !std::isfinite(angle_error.pitch_angle_error)) {
            reset_control_outputs();
            return;
        }

        const YawVelocityFeedback yaw_feedback = yaw_velocity_feedback();
        const double pitch_feedback = *pitch_velocity_imu_;
        const double yaw_velocity_ff =
            planner_active ? yaw_vel_ff_gain_ * *plan_yaw_velocity_ : 0.0;
        const double pitch_velocity_ff =
            planner_active ? pitch_vel_ff_gain_ * *plan_pitch_velocity_ : 0.0;
        const double yaw_acceleration_ff =
            planner_active ? yaw_acc_ff_gain_ * *plan_yaw_acceleration_ : 0.0;
        const double yaw_chassis_spin_torque_ff =
            planner_active ? yaw_chassis_spin_torque_ff_gain_ * yaw_feedback.chassis : 0.0;

        const double yaw_velocity_ref =
            yaw_angle_pid_.update(angle_error.yaw_angle_error) + yaw_velocity_ff;
        const double pitch_velocity_ref =
            pitch_angle_pid_.update(angle_error.pitch_angle_error) + pitch_velocity_ff;

        *yaw_velocity_ff_ = yaw_velocity_ff;
        *pitch_velocity_ff_ = pitch_velocity_ff;
        *yaw_acceleration_ff_ = yaw_acceleration_ff;
        *yaw_velocity_ref_ = yaw_velocity_ref;
        *pitch_velocity_ref_ = pitch_velocity_ref;
        *yaw_velocity_feedback_ = yaw_feedback.total;
        *pitch_velocity_feedback_ = pitch_feedback;
        *chassis_yaw_velocity_imu_ = yaw_feedback.chassis;
        *yaw_chassis_spin_torque_ff_ = yaw_chassis_spin_torque_ff;

        *yaw_control_torque_ = yaw_velocity_pid_.update(yaw_velocity_ref - yaw_feedback.total)
                             + yaw_acceleration_ff + yaw_chassis_spin_torque_ff;
        *pitch_control_velocity_ = pitch_velocity_ref;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    bool auto_aim_requested() const {
        return mouse_->right || *switch_right_ == rmcs_msgs::Switch::UP;
    }

    PlannerResult update_planner_outputs() {
        const auto now = *timestamp_;
        const auto snapshot = *target_snapshot_;

        const bool fresh =
            snapshot.valid
            && std::chrono::duration<double>(now - snapshot.timestamp) <= planner_result_timeout_;
        if (!fresh) {
            clear_planner_outputs();
            return {};
        }

        const PlannerResult result = planner_.plan(
            std::optional<rmcs_msgs::TargetSnapshot>{snapshot}, now,
            static_cast<double>(*bullet_speed_));
        if (!result.control) {
            clear_planner_outputs();
            return {};
        }

        *control_direction_ = planner_direction(result.yaw, result.pitch);
        *fire_control_ = result.fire;
        *laser_distance_ = result.control_xyza.head<3>().norm();
        *plan_target_yaw_ = result.target_yaw;
        *plan_target_pitch_ = result.target_pitch;
        *plan_yaw_ = result.yaw;
        *plan_pitch_ = result.pitch;
        *plan_yaw_velocity_ = result.yaw_velocity;
        *plan_yaw_acceleration_ = result.yaw_acceleration;
        *plan_pitch_velocity_ = result.pitch_velocity;
        *plan_pitch_acceleration_ = result.pitch_acceleration;
        return result;
    }

    void clear_planner_outputs() {
        *control_direction_ = Eigen::Vector3d::Zero();
        *fire_control_ = false;
        *laser_distance_ = 0.0;
        *plan_target_yaw_ = 0.0;
        *plan_target_pitch_ = 0.0;
        *plan_yaw_ = 0.0;
        *plan_pitch_ = 0.0;
        *plan_yaw_velocity_ = 0.0;
        *plan_yaw_acceleration_ = 0.0;
        *plan_pitch_velocity_ = 0.0;
        *plan_pitch_acceleration_ = 0.0;
    }

    void clear_control_diagnostics() {
        *yaw_velocity_ff_ = 0.0;
        *pitch_velocity_ff_ = 0.0;
        *yaw_acceleration_ff_ = 0.0;
        *yaw_velocity_ref_ = 0.0;
        *pitch_velocity_ref_ = 0.0;
        *yaw_velocity_feedback_ = 0.0;
        *pitch_velocity_feedback_ = 0.0;
        *chassis_yaw_velocity_imu_ = 0.0;
        *yaw_chassis_spin_torque_ff_ = 0.0;
    }

    TwoAxisGimbalSolver::AngleError
        update_auto_aim_control(const Eigen::Vector3d& control_direction) {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{OdomImu::DirectionVector{control_direction}});
    }

    TwoAxisGimbalSolver::AngleError update_manual_control() {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        const double yaw_shift =
            joystick_sensitivity_ * joystick_left_->y() + mouse_sensitivity_ * mouse_velocity_->y();
        const double pitch_shift = -joystick_sensitivity_ * joystick_left_->x()
                                 + mouse_sensitivity_ * mouse_velocity_->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    void reset_control_outputs() {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        clear_control_diagnostics();
        *yaw_control_torque_ = nan_;
        *pitch_control_velocity_ = nan_;
    }

    void reset_all_controls() {
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *yaw_angle_error_ = nan_;
        *pitch_angle_error_ = nan_;
        reset_control_outputs();
    }

    YawVelocityFeedback yaw_velocity_feedback() {
        const double chassis_yaw_velocity_imu = *yaw_velocity_imu_ - *yaw_velocity_;
        return {
            chassis_yaw_velocity_imu,
            chassis_yaw_velocity_imu_filter_.update(chassis_yaw_velocity_imu) + *yaw_velocity_,
        };
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;
    InputInterface<float> bullet_speed_;
    InputInterface<rmcs_msgs::TargetSnapshot> target_snapshot_;

    filter::LowPassFilter<> chassis_yaw_velocity_imu_filter_{0.5, 1000.0};

    TwoAxisGimbalSolver gimbal_solver_;
    Planner planner_;

    pid::PidCalculator yaw_angle_pid_;
    pid::PidCalculator yaw_velocity_pid_;
    pid::PidCalculator pitch_angle_pid_;
    double joystick_sensitivity_ = 0.006;
    double mouse_sensitivity_ = 0.5;
    double yaw_vel_ff_gain_;
    double pitch_vel_ff_gain_;
    double yaw_acc_ff_gain_;
    double yaw_chassis_spin_torque_ff_gain_;

    std::chrono::duration<double> planner_result_timeout_{0.2};
    float bullet_speed_fallback_storage_ = 23.0F;

    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;
    OutputInterface<double> laser_distance_;
    OutputInterface<double> plan_target_yaw_;
    OutputInterface<double> plan_target_pitch_;
    OutputInterface<double> plan_yaw_;
    OutputInterface<double> plan_pitch_;
    OutputInterface<double> plan_yaw_velocity_;
    OutputInterface<double> plan_yaw_acceleration_;
    OutputInterface<double> plan_pitch_velocity_;
    OutputInterface<double> plan_pitch_acceleration_;
    OutputInterface<double> yaw_velocity_ff_;
    OutputInterface<double> pitch_velocity_ff_;
    OutputInterface<double> yaw_acceleration_ff_;
    OutputInterface<double> yaw_velocity_ref_;
    OutputInterface<double> pitch_velocity_ref_;
    OutputInterface<double> yaw_velocity_feedback_;
    OutputInterface<double> pitch_velocity_feedback_;
    OutputInterface<double> chassis_yaw_velocity_imu_;
    OutputInterface<double> yaw_chassis_spin_torque_ff_;

    OutputInterface<double> yaw_control_torque_;
    OutputInterface<double> pitch_control_velocity_;
    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
