#include "controller/climber/climber_controller.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/switch.hpp"
#include <cmath>
#include <eigen3/Eigen/Core>
#include <limits>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>

namespace rmcs_core::controller::chassis {

class EngineerChassisClimberController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    EngineerChassisClimberController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , front_velocity_pid_calculator_(
              get_parameter("front_kp").as_double(), get_parameter("front_ki").as_double(),
              get_parameter("front_kd").as_double())
        , back_velocity_pid_calculator_(
              get_parameter("back_kp").as_double(), get_parameter("back_ki").as_double(),
              get_parameter("back_kd").as_double()) {

        sync_coefficient_ = get_parameter("sync_coefficient").as_double();

        climber_controller_ = climber::ClimberController({
            .track_velocity_max = get_parameter("front_climber_velocity").as_double(),
            .climber_back_control_velocity_abs = get_parameter("back_climber_velocity").as_double(),
            .support_retract_velocity_abs =
                get_parameter("auto_climb_support_retract_velocity").as_double(),
            .approach_chassis_velocity =
                get_parameter("auto_climb_approach_chassis_velocity").as_double(),
            .support_deploy_chassis_velocity =
                get_parameter("auto_climb_support_deploy_chassis_velocity").as_double(),
            .dash_chassis_velocity = get_parameter("auto_climb_dash_chassis_velocity").as_double(),
            .leveled_pitch_threshold =
                get_parameter("auto_climb_leveled_pitch_threshold").as_double(),
            .first_stair_approach_pitch  = get_parameter("first_stair_approach_pitch").as_double(),
            .second_stair_approach_pitch = get_parameter("second_stair_approach_pitch").as_double(),
        });

        register_output("/climber/track/l/control_torque", climber_front_left_control_torque_, NAN);
        register_output(
            "/climber/track/r/control_torque", climber_front_right_control_torque_, NAN);
        register_output("/climber/lift/l/control_torque", climber_back_left_control_torque_, NAN);
        register_output("/climber/lift/r/control_torque", climber_back_right_control_torque_, NAN);
        register_output("/chassis/climbing_forward_velocity", climbing_forward_velocity_, NAN);

        register_input("/climber/track/l/velocity", climber_front_left_velocity_);
        register_input("/climber/track/r/velocity", climber_front_right_velocity_);
        register_input("/climber/lift/l/velocity", climber_back_left_velocity_);
        register_input("/climber/lift/r/velocity", climber_back_right_velocity_);

        register_input("/climber/lift/l/torque", climber_back_left_torque_);
        register_input("/climber/lift/r/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/arm/mode", arm_mode_);
        register_input("pitch_imu_angle", chassis_pitch_imu_);
    }

    void update() override {
        using namespace rmcs_msgs;

        const auto switch_right = *switch_right_;
        const auto switch_left  = *switch_left_;

        if (!initial_check_done_) {
            reset_all_controls();
            last_arm_mode_ = *arm_mode_;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN)
                initial_check_done_ = true;
            return;
        }

        if (switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
            last_arm_mode_ = *arm_mode_;
            return;
        }

        if (!climber_controller_.active() && last_arm_mode_ != *arm_mode_) {
            switch (*arm_mode_) {
            case ArmMode::Auto_Up_One_Stairs:
                reset_pid_controllers();
                climber_controller_.start(climber::ClimberController::Mode::OneStair);
                RCLCPP_INFO(logger_, "Auto climb started in one-stair mode.");
                break;
            case ArmMode::Auto_Up_Two_Stairs:
                reset_pid_controllers();
                climber_controller_.start(climber::ClimberController::Mode::TwoStairs);
                RCLCPP_INFO(logger_, "Auto climb started in two-stairs mode.");
                break;
            default: break;
            }
        }

        if (climber_controller_.active()) {
            apply_climb_control(update_auto_climb_control());
            if (!climber_controller_.active())
                reset_pid_controllers();
        } else {
            set_control_outputs_nan();
        }

        last_arm_mode_ = *arm_mode_;
    }

private:
    using AutoClimbControl = climber::ClimberController::Output;

    AutoClimbControl update_auto_climb_control() {
        const bool was_active      = climber_controller_.active();
        const int stair_index      = climber_controller_.stair_index();
        const AutoClimbControl out = climber_controller_.update({
            .chassis_pitch_imu           = *chassis_pitch_imu_,
            .climber_back_left_torque    = *climber_back_left_torque_,
            .climber_back_right_torque   = *climber_back_right_torque_,
            .climber_back_left_velocity  = *climber_back_left_velocity_,
            .climber_back_right_velocity = *climber_back_right_velocity_,
        });

        if (was_active && !climber_controller_.active()) {
            RCLCPP_INFO(logger_, "Auto climb completed (finished %d steps).", stair_index + 1);
        }

        return out;
    }

    void apply_climb_control(const AutoClimbControl& control) {
        *climbing_forward_velocity_ = control.override_chassis_vx;
        if (control.back_climber_velocity != 0.0 && !std::isnan(control.back_climber_velocity)) {
            RCLCPP_INFO(
                this->get_logger(), "back_climber_velocity: %lf", control.back_climber_velocity);
        }

        dual_motor_sync_control(
            control.front_track_velocity, *climber_front_left_velocity_,
            *climber_front_right_velocity_, front_velocity_pid_calculator_,
            *climber_front_left_control_torque_, *climber_front_right_control_torque_);

        dual_motor_sync_control(
            control.back_climber_velocity, *climber_back_left_velocity_,
            *climber_back_right_velocity_, back_velocity_pid_calculator_,
            *climber_back_left_control_torque_, *climber_back_right_control_torque_);
    }

    void reset_all_controls() {
        set_control_outputs_nan();
        reset_pid_controllers();
        climber_controller_.reset();
    }

    void set_control_outputs_nan() {
        *climber_front_left_control_torque_  = NAN;
        *climber_front_right_control_torque_ = NAN;
        *climber_back_left_control_torque_   = NAN;
        *climber_back_right_control_torque_  = NAN;
        *climbing_forward_velocity_          = NAN;
    }

    void reset_pid_controllers() {
        front_velocity_pid_calculator_.reset();
        back_velocity_pid_calculator_.reset();
    }

    void dual_motor_sync_control(
        double setpoint, double left_velocity, double right_velocity,
        pid::MatrixPidCalculator<2>& pid_calculator, double& left_torque_out,
        double& right_torque_out) {

        if (std::isnan(setpoint)) {
            left_torque_out  = NAN;
            right_torque_out = NAN;
            return;
        }

        Eigen::Vector2d setpoint_error{setpoint - left_velocity, setpoint - right_velocity};
        Eigen::Vector2d relative_velocity{
            left_velocity - right_velocity, right_velocity - left_velocity};

        Eigen::Vector2d control_error = setpoint_error - sync_coefficient_ * relative_velocity;
        auto control_torques          = pid_calculator.update(control_error);

        left_torque_out  = control_torques[0];
        right_torque_out = control_torques[1];
    }

    rclcpp::Logger logger_;

    double sync_coefficient_;

    climber::ClimberController climber_controller_{climber::ClimberController::Config{}};
    bool initial_check_done_ = false;

    OutputInterface<double> climber_front_left_control_torque_;
    OutputInterface<double> climber_front_right_control_torque_;
    OutputInterface<double> climber_back_left_control_torque_;
    OutputInterface<double> climber_back_right_control_torque_;
    OutputInterface<double> climbing_forward_velocity_;

    InputInterface<double> climber_front_left_velocity_;
    InputInterface<double> climber_front_right_velocity_;
    InputInterface<double> climber_back_left_velocity_;
    InputInterface<double> climber_back_right_velocity_;

    InputInterface<double> climber_back_left_torque_;
    InputInterface<double> climber_back_right_torque_;

    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::ArmMode> arm_mode_;

    InputInterface<double> chassis_pitch_imu_;

    rmcs_msgs::ArmMode last_arm_mode_ = rmcs_msgs::ArmMode::None;

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::EngineerChassisClimberController, rmcs_executor::Component)
