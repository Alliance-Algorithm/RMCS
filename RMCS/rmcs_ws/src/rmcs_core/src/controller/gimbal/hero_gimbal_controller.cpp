#include <limits>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/gimbal_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/precise_two_axis_gimbal_solver.hpp"
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
              get_parameter("lower_limit").as_double())
        , encoder_gimbal_solver(
              *this, get_parameter("upper_limit").as_double(),
              get_parameter("lower_limit").as_double()) {

        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);

        register_input("/gimbal/auto_aim/control_direction", auto_aim_control_direction_, false);

        register_output("/gimbal/mode", gimbal_mode_, rmcs_msgs::GimbalMode::IMU);

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

            if (!last_keyboard_.q && keyboard_->q) {
                if (gimbal_mode_keyboard_ == GimbalMode::IMU)
                    gimbal_mode_keyboard_ = GimbalMode::ENCODER;
                else
                    gimbal_mode_keyboard_ = GimbalMode::IMU;
            }
            *gimbal_mode_ =
                *switch_right_ == Switch::UP ? GimbalMode::ENCODER : gimbal_mode_keyboard_;

            if (*gimbal_mode_ == GimbalMode::IMU) {
                auto angle_error = update_imu_control();
                *yaw_angle_error_ = angle_error.yaw_angle_error;
                *pitch_angle_error_ = angle_error.pitch_angle_error;

                encoder_gimbal_solver.update(PreciseTwoAxisGimbalSolver::SetDisabled{});
                *yaw_control_angle_shift_ = nan_;
                *pitch_control_angle_ = nan_;
            } else {
                imu_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled{});
                *yaw_angle_error_ = nan_;
                *pitch_angle_error_ = nan_;

                auto control_angle = update_encoder_control();
                *yaw_control_angle_shift_ = control_angle.yaw_shift;
                *pitch_control_angle_ = control_angle.pitch_angle;
            }
        } while (false);

        last_keyboard_ = *keyboard_;
    }

    void reset_all_control() {
        imu_gimbal_solver.update(TwoAxisGimbalSolver::SetDisabled{});
        encoder_gimbal_solver.update(PreciseTwoAxisGimbalSolver::SetDisabled{});

        gimbal_mode_keyboard_ = rmcs_msgs::GimbalMode::IMU;
        *gimbal_mode_ = rmcs_msgs::GimbalMode::IMU;

        *yaw_angle_error_ = nan_;
        *pitch_angle_error_ = nan_;
        *yaw_control_angle_shift_ = nan_;
        *pitch_control_angle_ = nan_;
    }

    TwoAxisGimbalSolver::AngleError update_imu_control() {
        if (auto_aim_control_direction_.ready()
            && (mouse_->right || *switch_right_ == rmcs_msgs::Switch::UP)
            && !auto_aim_control_direction_->isZero()) {
            return imu_gimbal_solver.update(
                TwoAxisGimbalSolver::SetControlDirection{
                    OdomImu::DirectionVector{*auto_aim_control_direction_}});
        }

        if (!imu_gimbal_solver.enabled())
            return imu_gimbal_solver.update(TwoAxisGimbalSolver::SetToLevel{});

        constexpr double joystick_sensitivity = 0.006;
        constexpr double mouse_sensitivity = 0.5;

        double yaw_shift =
            joystick_sensitivity * joystick_left_->y() + mouse_sensitivity * mouse_velocity_->y();
        double pitch_shift =
            -joystick_sensitivity * joystick_left_->x() - mouse_sensitivity * mouse_velocity_->x();

        return imu_gimbal_solver.update(
            TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    PreciseTwoAxisGimbalSolver::ControlAngle update_encoder_control() {
        if (!encoder_gimbal_solver.enabled())
            return encoder_gimbal_solver.update(PreciseTwoAxisGimbalSolver::SetControlPitch{-0.31});

        constexpr double joystick_sensitivity = 0.006 * 0.1;
        constexpr double mouse_yaw_sensitivity = 0.5 * 0.114;
        constexpr double mouse_pitch_sensitivity = 0.5 * 0.095;

        double yaw_shift = joystick_sensitivity * joystick_left_->y()
                         + mouse_yaw_sensitivity * mouse_velocity_->y();
        double pitch_shift = -joystick_sensitivity * joystick_left_->x()
                           - mouse_pitch_sensitivity * mouse_velocity_->x();

        return encoder_gimbal_solver.update(
            PreciseTwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

private:
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    rmcs_msgs::Keyboard last_keyboard_ = rmcs_msgs::Keyboard::zero();

    InputInterface<Eigen::Vector3d> auto_aim_control_direction_;

    rmcs_msgs::GimbalMode gimbal_mode_keyboard_ = rmcs_msgs::GimbalMode::IMU;
    OutputInterface<rmcs_msgs::GimbalMode> gimbal_mode_;

    TwoAxisGimbalSolver imu_gimbal_solver;
    PreciseTwoAxisGimbalSolver encoder_gimbal_solver;

    OutputInterface<double> yaw_angle_error_, pitch_angle_error_;
    OutputInterface<double> yaw_control_angle_shift_, pitch_control_angle_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::HeroGimbalController, rmcs_executor::Component)