#include <cmath>
#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

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
        , upper_limit_(get_parameter("upper_limit").as_double())
        , lower_limit_(get_parameter("lower_limit").as_double())
        , two_axis_gimbal_solver(*this, upper_limit_, lower_limit_) {

        navigation_scan_yaw_speed_ = get_parameter_or<double>("navigation_scan_yaw_speed", 0.0);
        navigation_scan_pitch_speed_ =
            std::abs(get_parameter_or<double>("navigation_scan_pitch_speed", 0.0));
        navigation_scan_pitch_upper_ = upper_limit_ * 0.5;
        navigation_scan_pitch_lower_ = lower_limit_ * 0.5;

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/rmcs_navigation/gimbal_velocity", navigation_command_velocity_, false);
        register_input("/rmcs_navigation/gimbal_scanning", navigation_gimbal_scanning_, false);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);
        register_input("/gimbal/pitch/angle", gimbal_pitch_angle_);

        register_output("/gimbal/yaw/control_angle_error", yaw_angle_error_, nan_);
        register_output("/gimbal/pitch/control_angle_error", pitch_angle_error_, nan_);
    }

    void update() override {
        auto angle_error = calculate_angle_error();
        *yaw_angle_error_ = angle_error.yaw_angle_error;
        *pitch_angle_error_ = angle_error.pitch_angle_error;
    }

    TwoAxisGimbalSolver::AngleError calculate_angle_error() {
        auto switch_right = *switch_right_;
        auto switch_left = *switch_left_;
        auto mouse = *mouse_;

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN))
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled());

        if (auto_aim_control_direction_.ready() && (mouse.right || switch_right == Switch::UP)
            && !auto_aim_control_direction_->isZero())
            return two_axis_gimbal_solver.update(
                TwoAxisGimbalSolver::SetControlDirection(
                    OdomImu::DirectionVector(*auto_aim_control_direction_)));

        if (!two_axis_gimbal_solver.enabled())
            return two_axis_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel());

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() - mouse_sensitivity * mouse_velocity_->x();

        // Navigation Control
        if (navigation_command_velocity_.ready()) {
            auto yaw_speed = navigation_command_velocity_->x();
            if (std::isfinite(yaw_speed))
                yaw_shift += yaw_speed * control_dt_;

            auto pitch_speed = navigation_command_velocity_->y();
            if (std::isfinite(pitch_speed))
                pitch_shift += pitch_speed * control_dt_;
        }

        apply_navigation_gimbal_scanning(yaw_shift, pitch_shift);

        return two_axis_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

private:
    void apply_navigation_gimbal_scanning(double& yaw_shift, double& pitch_shift) {
        const auto scanning_enabled =
            navigation_gimbal_scanning_.ready() && *navigation_gimbal_scanning_;
        if (!scanning_enabled) {
            last_navigation_gimbal_scanning_ = false;
            scanning_pitch_direction_ = +1.0;
            return;
        }

        auto current_pitch = *gimbal_pitch_angle_;
        auto middle_pitch = (navigation_scan_pitch_upper_ + navigation_scan_pitch_lower_) * 0.5;

        if (!last_navigation_gimbal_scanning_) {
            scanning_pitch_direction_ = current_pitch >= middle_pitch ? -1.0 : +1.0;
        }

        constexpr double kBoundaryTolerance = 1e-3;
        if (current_pitch > std::numbers::pi) {
            current_pitch -= 2 * std::numbers::pi;
        }

        if (current_pitch <= navigation_scan_pitch_upper_ + kBoundaryTolerance) {
            scanning_pitch_direction_ = +1.0;
        } else if (current_pitch >= navigation_scan_pitch_lower_ - kBoundaryTolerance) {
            scanning_pitch_direction_ = -1.0;
        }

        yaw_shift += navigation_scan_yaw_speed_ * control_dt_;
        pitch_shift += scanning_pitch_direction_ * navigation_scan_pitch_speed_ * control_dt_;

        last_navigation_gimbal_scanning_ = true;
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double control_dt_ = 1e-3;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;
    InputInterface<double> gimbal_pitch_angle_;

    const double upper_limit_;
    const double lower_limit_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;

    // For Navigation
    InputInterface<Eigen::Vector2d> navigation_command_velocity_;
    InputInterface<bool> navigation_gimbal_scanning_;

    double navigation_scan_yaw_speed_ = 0.0;
    double navigation_scan_pitch_speed_ = 0.0;
    double navigation_scan_pitch_upper_ = 0.0;
    double navigation_scan_pitch_lower_ = 0.0;
    double scanning_pitch_direction_ = +1.0;
    bool last_navigation_gimbal_scanning_ = false;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)
