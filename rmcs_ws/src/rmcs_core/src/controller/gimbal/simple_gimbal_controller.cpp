#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/chassis/wheel_leg/recovery_yaw.hpp"
#include "controller/gimbal/two_axis_gimbal_solver.hpp"

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class SimpleGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    SimpleGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , two_axis_gimbal_solver{
              *this,
              get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double(),
          } {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/auto_aim/should_control", auto_aim_should_control_, false);
        register_input("/auto_aim/control_direction", auto_aim_control_direction_, false);
        register_input("/wheel_leg/recovery/yaw_align_active", recovery_yaw_active_, false);
        register_input("/gimbal/yaw/angle", recovery_yaw_, false);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);

        if (has_parameter("yaw_manual_velocity_ff_scale")) {
            yaw_manual_velocity_ff_scale_ =
                get_parameter("yaw_manual_velocity_ff_scale").as_double();
            yaw_manual_velocity_ff_limit_ =
                get_parameter_or<double>("yaw_manual_velocity_ff_limit", .5);
            if (!std::isfinite(yaw_manual_velocity_ff_scale_) || yaw_manual_velocity_ff_scale_ < 0.
                || yaw_manual_velocity_ff_scale_ > 1.
                || !std::isfinite(yaw_manual_velocity_ff_limit_)
                || yaw_manual_velocity_ff_limit_ <= 0.)
                throw std::invalid_argument("Invalid manual yaw velocity feedforward");
            register_input("/predefined/update_rate", update_rate_);
            register_output(
                "/gimbal/yaw/manual_reference_velocity_ff", yaw_manual_velocity_ff_, 0.);
        }

        if (has_parameter("yaw_upper_limit"))
            two_axis_gimbal_solver.enable_yaw_limit(
                *this, get_parameter("yaw_upper_limit").as_double(),
                get_parameter("yaw_lower_limit").as_double());
    }

    void update() override {
        if (yaw_manual_velocity_ff_.active())
            *yaw_manual_velocity_ff_ = 0.;
        auto angle_error = calculate_angle_error();
        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;
    }

    TwoAxisGimbalSolver::AngleError calculate_angle_error() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            recovery_yaw_alignment_.reset();
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());
        }

        if (recovery_yaw_active_.ready() && *recovery_yaw_active_) {
            two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());
            const double now =
                std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                    .count();
            return {
                recovery_yaw_.ready() ? recovery_yaw_alignment_.update(*recovery_yaw_, now) : nan_,
                nan_};
        }
        recovery_yaw_alignment_.reset();

        const auto auto_aim_requested = mouse_->right || switch_right == Switch::UP;
        const auto should_control = auto_aim_should_control_.ready() && *auto_aim_should_control_;
        const auto valid_control =
            auto_aim_control_direction_.ready() && auto_aim_control_direction_->allFinite();

        if (auto_aim_requested && should_control && valid_control) {
            return two_axis_gimbal_solver.update(
                TwoAxisGimbalSolver::SetControlDirection(
                    OdomImu::DirectionVector(*auto_aim_control_direction_)));
        }

        if (!two_axis_gimbal_solver.enabled())
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel());

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() + mouse_sensitivity * mouse_velocity_->x();

        auto error = two_axis_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
        if (yaw_manual_velocity_ff_.active() && std::isfinite(error.yaw_angle_error)
            && std::isfinite(yaw_shift) && std::isfinite(*update_rate_) && *update_rate_ > 0.) {
            *yaw_manual_velocity_ff_ = std::clamp(
                yaw_shift * *update_rate_ * yaw_manual_velocity_ff_scale_,
                -yaw_manual_velocity_ff_limit_, yaw_manual_velocity_ff_limit_);
        }
        return error;
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<bool> auto_aim_should_control_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;
    InputInterface<bool> recovery_yaw_active_;
    InputInterface<double> recovery_yaw_;
    chassis::wheel_leg::RecoveryYawAlignment recovery_yaw_alignment_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    InputInterface<double> update_rate_;
    OutputInterface<double> yaw_angle_error_, pitch_angle_error_, yaw_manual_velocity_ff_;
    double yaw_manual_velocity_ff_scale_ = 0., yaw_manual_velocity_ff_limit_ = .5;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)
