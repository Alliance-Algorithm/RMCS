#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/two_axis_gimbal_solver.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class HeroGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    HeroGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , imu_gimbal_solver(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double()) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/gimbal/auto_aim/auto_aim_enabled", auto_aim_enabled_, false);
        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);
        register_output("/gimbal/yaw/control_angle_shift", yaw_control_angle_shift_, nan_);
        register_output("/gimbal/pitch/control_angle", pitch_control_angle_, nan_);
    }

    void update() override {
        const auto& switch_left = *switch_left_;
        const auto& switch_right = *switch_right_;

        do {
            using namespace rmcs_msgs;
            if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
                || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
                reset_all_control();
                break;
            }

            *yaw_control_angle_shift_ = nan_;
            *pitch_control_angle_ = nan_;

            const bool auto_aim_takeover = should_auto_aim_takeover(switch_right);
            if (!auto_aim_takeover && auto_aim_takeover_last_cycle_) {
                imu_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled{});
            }
            auto_aim_takeover_last_cycle_ = auto_aim_takeover;

            if (auto_aim_takeover) {
                auto angle_error = update_auto_aim_control();
                *yaw_angle_error_ = angle_error.yaw_angle_error;
                *pitch_angle_error_ = angle_error.pitch_angle_error;
                break;
            }

            auto angle_error = update_imu_control();
            *yaw_angle_error_ = angle_error.yaw_angle_error;
            *pitch_angle_error_ = angle_error.pitch_angle_error;
        } while (false);
    }

    void reset_all_control() {
        imu_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled{});
        auto_aim_takeover_last_cycle_ = false;

        *yaw_angle_error_ = nan_;
        *pitch_angle_error_ = nan_;
        *yaw_control_angle_shift_ = nan_;
        *pitch_control_angle_ = nan_;
    }

    bool should_auto_aim_takeover(rmcs_msgs::Switch switch_right) const {
        if (switch_right != rmcs_msgs::Switch::UP || !auto_aim_enabled_.ready()
            || !auto_aim_control_direction_.ready() || !*auto_aim_enabled_) {
            return false;
        }

        if (!auto_aim_control_direction_->allFinite())
            return false;

        constexpr double direction_eps = 1e-6;
        return !auto_aim_control_direction_->isApprox(Eigen::Vector3d::UnitX(), direction_eps);
    }

    TwoAxisGimbalSolver::AngleError update_auto_aim_control() {
        return imu_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*auto_aim_control_direction_}});
    }

    TwoAxisGimbalSolver::AngleError update_imu_control() {
        if (!imu_gimbal_solver.enabled())
            return imu_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel{});

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() + mouse_sensitivity * mouse_velocity_->x();

        return imu_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<bool> auto_aim_enabled_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    TwoAxisGimbalSolver imu_gimbal_solver;
    bool auto_aim_takeover_last_cycle_{false};

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
    OutputInterface<double> yaw_control_angle_shift_, pitch_control_angle_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HeroGimbalController, rmcs_executor::Component)
