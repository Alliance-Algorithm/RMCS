#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

namespace {

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

Eigen::Vector3d
    planner_direction_dot(double yaw, double pitch, double yaw_velocity, double pitch_velocity) {
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_pitch = std::sin(pitch);
    const double cos_pitch = std::cos(pitch);

    const Eigen::Vector3d d_yaw{-cos_pitch * sin_yaw, cos_pitch * cos_yaw, 0.0};
    const Eigen::Vector3d d_pitch{-sin_pitch * cos_yaw, -sin_pitch * sin_yaw, -cos_pitch};
    return d_yaw * yaw_velocity + d_pitch * pitch_velocity;
}

Eigen::Vector3d planner_direction_ddot(
    double yaw, double pitch, double yaw_velocity, double pitch_velocity, double yaw_acceleration,
    double pitch_acceleration) {
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_pitch = std::sin(pitch);
    const double cos_pitch = std::cos(pitch);

    const Eigen::Vector3d d_yaw{-cos_pitch * sin_yaw, cos_pitch * cos_yaw, 0.0};
    const Eigen::Vector3d d_pitch{-sin_pitch * cos_yaw, -sin_pitch * sin_yaw, -cos_pitch};
    const Eigen::Vector3d d_yawyaw{-cos_pitch * cos_yaw, -cos_pitch * sin_yaw, 0.0};
    const Eigen::Vector3d d_pitchpitch{-cos_pitch * cos_yaw, -cos_pitch * sin_yaw, sin_pitch};
    const Eigen::Vector3d d_yawpitch{sin_pitch * sin_yaw, -sin_pitch * cos_yaw, 0.0};

    return d_yaw * yaw_acceleration + d_pitch * pitch_acceleration
         + d_yawyaw * std::pow(yaw_velocity, 2) + d_pitchpitch * std::pow(pitch_velocity, 2)
         + 2.0 * d_yawpitch * yaw_velocity * pitch_velocity;
}

Eigen::Vector2d solve_joint_velocity(
    const Eigen::Vector3d& direction, const Eigen::Vector3d& direction_derivative,
    const Eigen::Vector3d& yaw_axis, const Eigen::Vector3d& pitch_axis) {
    Eigen::Matrix<double, 3, 2> jacobian;
    jacobian.col(0) = yaw_axis.cross(direction);
    jacobian.col(1) = pitch_axis.cross(direction);

    Eigen::Vector2d result = jacobian.completeOrthogonalDecomposition().solve(direction_derivative);
    if (!result.allFinite())
        result.setZero();
    return result;
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

} // namespace

class OmniInfantryPlannerGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    OmniInfantryPlannerGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , gimbal_solver_(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double())
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
        , pitch_velocity_pid_(
              get_parameter("pitch_velocity_kp").as_double(),
              get_parameter("pitch_velocity_ki").as_double(),
              get_parameter("pitch_velocity_kd").as_double())
        , yaw_acc_ff_gain_(get_parameter("yaw_acc_ff_gain").as_double())
        , pitch_acc_ff_gain_(get_parameter("pitch_acc_ff_gain").as_double()) {
        configure_pid(*this, "yaw_angle", yaw_angle_pid_);
        configure_pid(*this, "yaw_velocity", yaw_velocity_pid_);
        configure_pid(*this, "pitch_angle", pitch_angle_pid_);
        configure_pid(*this, "pitch_velocity", pitch_velocity_pid_);

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/tf", tf_);
        register_input("/gimbal/yaw/velocity", yaw_velocity_);
        register_input("/gimbal/pitch/velocity", pitch_velocity_);
        register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu_);
        register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);
        register_input("/gimbal/auto_aim/plan_yaw", plan_yaw_, false);
        register_input("/gimbal/auto_aim/plan_pitch", plan_pitch_, false);
        register_input("/gimbal/auto_aim/plan_yaw_velocity", plan_yaw_velocity_, false);
        register_input("/gimbal/auto_aim/plan_yaw_acceleration", plan_yaw_acceleration_, false);
        register_input("/gimbal/auto_aim/plan_pitch_velocity", plan_pitch_velocity_, false);
        register_input("/gimbal/auto_aim/plan_pitch_acceleration", plan_pitch_acceleration_, false);

        register_output("/gimbal/yaw/control_torque", yaw_control_torque_, nan_);
        register_output("/gimbal/pitch/control_torque", pitch_control_torque_, nan_);
        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);
    }

    void before_updating() override {
        if (!auto_aim_control_direction_.ready())
            auto_aim_control_direction_.make_and_bind_directly(Eigen::Vector3d::Zero());
        if (!plan_yaw_.ready())
            plan_yaw_.make_and_bind_directly(0.0);
        if (!plan_pitch_.ready())
            plan_pitch_.make_and_bind_directly(0.0);
        if (!plan_yaw_velocity_.ready())
            plan_yaw_velocity_.make_and_bind_directly(0.0);
        if (!plan_yaw_acceleration_.ready())
            plan_yaw_acceleration_.make_and_bind_directly(0.0);
        if (!plan_pitch_velocity_.ready())
            plan_pitch_velocity_.make_and_bind_directly(0.0);
        if (!plan_pitch_acceleration_.ready())
            plan_pitch_acceleration_.make_and_bind_directly(0.0);
    }

    void update() override {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            return;
        }

        const bool planner_active = auto_aim_requested() && !auto_aim_control_direction_->isZero();
        TwoAxisGimbalSolver::AngleError angle_error =
            planner_active ? update_auto_aim_control() : update_manual_control();

        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;

        if (!std::isfinite(angle_error.yaw_angle_error)
            || !std::isfinite(angle_error.pitch_angle_error)) {
            reset_torque_outputs();
            return;
        }

        Eigen::Vector2d velocity_ff = Eigen::Vector2d::Zero();
        Eigen::Vector2d acceleration_ff = Eigen::Vector2d::Zero();
        if (planner_active) {
            const auto planner_feedforward = compute_planner_feedforward();
            velocity_ff = planner_feedforward.first;
            acceleration_ff = planner_feedforward.second;
        }

        const double yaw_velocity_ref =
            yaw_angle_pid_.update(angle_error.yaw_angle_error) + velocity_ff.x();
        const double pitch_velocity_ref =
            pitch_angle_pid_.update(angle_error.pitch_angle_error) + velocity_ff.y();

        *yaw_control_torque_ = yaw_velocity_pid_.update(yaw_velocity_ref - yaw_velocity_imu())
                             + yaw_acc_ff_gain_ * acceleration_ff.x();
        *pitch_control_torque_ =
            pitch_velocity_pid_.update(pitch_velocity_ref - *pitch_velocity_imu_)
            + pitch_acc_ff_gain_ * acceleration_ff.y();
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double joystick_sensitivity_ = 0.006;
    static constexpr double mouse_sensitivity_ = 0.5;

    bool auto_aim_requested() const {
        return mouse_->right || *switch_right_ == rmcs_msgs::Switch::UP;
    }

    TwoAxisGimbalSolver::AngleError update_auto_aim_control() {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*auto_aim_control_direction_}});
    }

    TwoAxisGimbalSolver::AngleError update_manual_control() {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        double yaw_shift =
            joystick_sensitivity_ * joystick_left_->y() + mouse_sensitivity_ * mouse_velocity_->y();
        double pitch_shift = -joystick_sensitivity_ * joystick_left_->x()
                           - mouse_sensitivity_ * mouse_velocity_->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    std::pair<Eigen::Vector2d, Eigen::Vector2d> compute_planner_feedforward() const {
        const double yaw = *plan_yaw_;
        const double pitch = *plan_pitch_;
        const double yaw_velocity = *plan_yaw_velocity_;
        const double pitch_velocity = *plan_pitch_velocity_;
        const double yaw_acceleration = *plan_yaw_acceleration_;
        const double pitch_acceleration = *plan_pitch_acceleration_;

        const Eigen::Vector3d direction = planner_direction(yaw, pitch);
        if (direction.squaredNorm() <= 1e-18)
            return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};

        const Eigen::Vector3d direction_dot =
            planner_direction_dot(yaw, pitch, yaw_velocity, pitch_velocity);
        const Eigen::Vector3d direction_ddot = planner_direction_ddot(
            yaw, pitch, yaw_velocity, pitch_velocity, yaw_acceleration, pitch_acceleration);

        const auto yaw_axis_in_world = fast_tf::cast<OdomImu>(
            GimbalCenterLink::DirectionVector{Eigen::Vector3d::UnitZ()}, *tf_);
        const auto pitch_axis_in_world =
            fast_tf::cast<OdomImu>(YawLink::DirectionVector{Eigen::Vector3d::UnitY()}, *tf_);
        Eigen::Vector3d yaw_axis = *yaw_axis_in_world;
        Eigen::Vector3d pitch_axis = *pitch_axis_in_world;
        yaw_axis.normalize();
        pitch_axis.normalize();

        const Eigen::Vector2d velocity_ff =
            solve_joint_velocity(direction, direction_dot, yaw_axis, pitch_axis);
        const Eigen::Vector2d acceleration_ff =
            solve_joint_velocity(direction, direction_ddot, yaw_axis, pitch_axis);
        return {velocity_ff, acceleration_ff};
    }

    void reset_torque_outputs() {
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        *yaw_control_torque_ = nan_;
        *pitch_control_torque_ = nan_;
    }

    void reset_all_controls() {
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *yaw_angle_error_ = nan_;
        *pitch_angle_error_ = nan_;
        reset_torque_outputs();
    }

    double yaw_velocity_imu() {
        const double chassis_yaw_velocity_imu = *yaw_velocity_imu_ - *yaw_velocity_;
        return /*chassis_yaw_velocity_imu_filter_.update(chassis_yaw_velocity_imu) +*/
            *yaw_velocity_;
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<Tf> tf_;
    InputInterface<double> yaw_velocity_;
    InputInterface<double> pitch_velocity_;
    InputInterface<double> yaw_velocity_imu_;
    InputInterface<double> pitch_velocity_imu_;

    filter::LowPassFilter<> chassis_yaw_velocity_imu_filter_{0.5, 1000.0};

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;
    InputInterface<double> plan_yaw_;
    InputInterface<double> plan_pitch_;
    InputInterface<double> plan_yaw_velocity_;
    InputInterface<double> plan_yaw_acceleration_;
    InputInterface<double> plan_pitch_velocity_;
    InputInterface<double> plan_pitch_acceleration_;

    TwoAxisGimbalSolver gimbal_solver_;

    pid::PidCalculator yaw_angle_pid_;
    pid::PidCalculator yaw_velocity_pid_;
    pid::PidCalculator pitch_angle_pid_;
    pid::PidCalculator pitch_velocity_pid_;
    double yaw_acc_ff_gain_;
    double pitch_acc_ff_gain_;

    OutputInterface<double> yaw_control_torque_;
    OutputInterface<double> pitch_control_torque_;
    OutputInterface<double> yaw_angle_error_;
    OutputInterface<double> pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::OmniInfantryPlannerGimbalController, rmcs_executor::Component)
