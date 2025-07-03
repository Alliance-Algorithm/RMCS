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
        , two_axis_gimbal_solver(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double()) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

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

        return two_axis_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)