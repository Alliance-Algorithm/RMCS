#include <limits>
#include <algorithm>
#include <cmath>
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
              get_parameter("lower_limit").as_double()),
              yaw_lower_limit_(get_parameter("yaw_lower_limit").as_double()),
              yaw_upper_limit_(get_parameter("yaw_upper_limit").as_double()){
                
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);

        register_input("/auto_aim/should_control", auto_aim_should_control_, false);
        register_input("/auto_aim/control_direction", auto_aim_control_direction_, false);

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
        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN))
            return update_with_yaw_limit(TwoAxisGimbalSolver::SetDisabled());

        const auto auto_aim_active =
            switch_right == Switch::UP && auto_aim_should_control_.ready()
            && *auto_aim_should_control_ && auto_aim_control_direction_.ready()
            && auto_aim_control_direction_->allFinite() && !auto_aim_control_direction_->isZero();
        if (auto_aim_active) {
            return update_with_yaw_limit(
                TwoAxisGimbalSolver::SetControlDirection(
                    OdomImu::DirectionVector(*auto_aim_control_direction_)));
        }

        if (!two_axis_gimbal_solver.enabled())
            return update_with_yaw_limit(TwoAxisGimbalSolver::SetToLevel());

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() - mouse_sensitivity * mouse_velocity_->x();

        return update_with_yaw_limit(
            TwoAxisGimbalSolver::SetControlShift(yaw_shift, pitch_shift));
    }

private:
    auto update_with_yaw_limit(const TwoAxisGimbalSolver::Operation& operation)
        -> TwoAxisGimbalSolver::AngleError{
            
        auto& solver =two_axis_gimbal_solver;
        solver.update_yaw_axis();

        PitchLink::DirectionVector control_direction =operation.update(solver);
        if(!solver.control_enabled_)
            return {nan_,nan_};

        clamp_control_direction_yaw_in_odom(control_direction);

        auto [control_direction_yaw_link,pitch]=
            solver.use_encoder_pitch_
                ?solver.pitch_link_to_yaw_link_from_encoder(control_direction)
                :solver.pitch_link_to_yaw_link(control_direction);
        
        solver.clamp_control_direction(control_direction_yaw_link);
        if(!solver.control_enabled_)
            return {nan_,nan_};

        solver.control_direction_ =fast_tf::cast<OdomImu>(
            TwoAxisGimbalSolver::yaw_link_to_pitch_link(control_direction_yaw_link, pitch),*solver.tf_);

        return TwoAxisGimbalSolver::calculate_control_errors(control_direction_yaw_link, pitch);
        }
    void clamp_control_direction_yaw_in_odom(PitchLink::DirectionVector& control_direction){
        auto odom_direction =fast_tf::cast<OdomImu>(control_direction,*two_axis_gimbal_solver.tf_);

        const Eigen::Vector2d horizon{odom_direction->x(),odom_direction->y()};
        const double horizon_norm = horizon.norm();
        if(horizon_norm <=1e-9)
            return;

        const double yaw =std::atan2(horizon.y(),horizon.x());

        const double clamped_yaw =std::clamp(yaw,yaw_lower_limit_,yaw_upper_limit_);
        if(clamped_yaw ==yaw)
            return;

        odom_direction->x() =horizon_norm* std::cos(clamped_yaw);
        odom_direction->y() =horizon_norm* std::sin(clamped_yaw);
        odom_direction->normalize();

        control_direction =fast_tf::cast<PitchLink>(odom_direction,*two_axis_gimbal_solver.tf_);
    }

    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;

    InputInterface<bool> auto_aim_should_control_;
    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    TwoAxisGimbalSolver two_axis_gimbal_solver;
    double yaw_lower_limit_;
    double yaw_upper_limit_;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::SimpleGimbalController, rmcs_executor::Component)
