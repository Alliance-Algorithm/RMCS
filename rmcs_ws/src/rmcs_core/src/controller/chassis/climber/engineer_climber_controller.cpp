#include "controller/chassis/climber/climber_controller.hpp"
#include "controller/pid/matrix_pid_calculator.hpp"
#include "rmcs_msgs/keyboard.hpp"
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

        register_output(
            "/climber/track/l/control_torque", climber_front_left_control_torque_, nan_);
        register_output(
            "/climber/track/r/control_torque", climber_front_right_control_torque_, nan_);
        register_output("/climber/lift/l/control_torque", climber_back_left_control_torque_, nan_);
        register_output("/climber/lift/r/control_torque", climber_back_right_control_torque_, nan_);
        register_output("/chassis/climbing_forward_velocity", climbing_forward_velocity_, nan_);

        register_input("/climber/track/l/velocity", climber_front_left_velocity_);
        register_input("/climber/track/r/velocity", climber_front_right_velocity_);
        register_input("/climber/lift/l/velocity", climber_back_left_velocity_);
        register_input("/climber/lift/r/velocity", climber_back_right_velocity_);

        register_input("/climber/lift/l/torque", climber_back_left_torque_);
        register_input("/climber/lift/r/torque", climber_back_right_torque_);

        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_);
        register_input("pitch_imu_angle", chassis_pitch_imu_);

        register_input("/remote/joystick/right", joystick_right_);
    }

    void update() override {
        using namespace rmcs_msgs;

        auto switch_right       = *switch_right_;
        auto switch_left        = *switch_left_;
        auto keyboard           = *keyboard_;
        auto knob               = *rotary_knob_switch_;
        bool keyboard_b_pressed = !last_keyboard_.b && keyboard.b;

        bool rotary_knob_to_down =
            (last_rotary_knob_switch_ != Switch::DOWN && knob == Switch::DOWN);
        bool rotary_knob_from_down =
            (last_rotary_knob_switch_ == Switch::DOWN && knob != Switch::DOWN);
        bool rotary_knob_to_up   = (last_rotary_knob_switch_ != Switch::UP && knob == Switch::UP);
        bool rotary_knob_from_up = (last_rotary_knob_switch_ == Switch::UP && knob != Switch::UP);

        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            reset_all_controls();
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            double joystick_right_x     = joystick_right_->x();
            bool joystick_to_one_stair  = last_joystick_right_x_ <= 0.8 && joystick_right_x > 0.8;
            bool joystick_to_two_stairs = last_joystick_right_x_ >= -0.8 && joystick_right_x < -0.8;
            bool joystick_trigger_clear =
                (last_joystick_right_x_ > 0.8 && joystick_right_x <= 0.8)
                || (last_joystick_right_x_ < -0.8 && joystick_right_x >= -0.8);

            if (joystick_to_one_stair) {
                RCLCPP_INFO(this->get_logger(), "ONE");
                // arm_mode_ = rmcs_msgs::ArmMode::Auto_Up_One_Stairs;
            } else if (joystick_to_two_stairs) {
                RCLCPP_INFO(this->get_logger(), "TWO");
                // arm_mode_ = rmcs_msgs::ArmMode::Auto_Up_Two_Stairs;
            }
            if (keyboard_b_pressed) {
                // arm_mode_ = keyboard.shift ? rmcs_msgs::ArmMode::Auto_Up_Two_Stairs
                //                            : rmcs_msgs::ArmMode::Auto_Up_One_Stairs;
            }
            handle_auto_climb_requests(
                keyboard_b_pressed || joystick_to_one_stair || joystick_to_two_stairs,
                joystick_trigger_clear, joystick_to_one_stair || joystick_to_two_stairs);
            if (climber_controller_.active()) {
                stop_manual_support();
                apply_climb_control(update_auto_climb_control());
            } else {
                apply_climb_control(update_manual_support_control(keyboard));
                apply_manual_support_zero_output();
            }
            last_joystick_right_x_ = joystick_right_x;
        }

        last_keyboard_           = keyboard;
        last_rotary_knob_switch_ = knob;
    }

private:
    using AutoClimbControl = climber::ClimberController::Output;

    void handle_auto_climb_requests(
        bool start_requested, bool abort_by_rotary, bool start_by_rotary) {

        if (start_requested) {
            if (!climber_controller_.active()) {
                start_auto_climb(start_by_rotary ? "Rotary Knob" : "Keyboard B");
            } else {
                abort_auto_climb("toggled again");
            }
        } else if (abort_by_rotary && climber_controller_.active()) {
            abort_auto_climb("rotary knob left trigger position");
        }
    }

    void start_auto_climb(const char* source) {
        stop_manual_support();
        switch (*arm_mode_) {
        case rmcs_msgs::ArmMode::Auto_Up_One_Stairs:
            climber_controller_.start(climber::ClimberController::Mode::OneStair);
            break;
        case rmcs_msgs::ArmMode::Auto_Up_Two_Stairs:
            climber_controller_.start(climber::ClimberController::Mode::TwoStairs);
            break;
        default:
            climber_controller_.start(climber::ClimberController::Mode::OneStair);
            // climber_controller_.start(climber::ClimberController::Mode::TwoStairs);
            RCLCPP_WARN(
                logger_, "Auto climb start requested, but arm mode is not set to a valid climb "
                         "mode. Aborting.");
            return;
        }

        RCLCPP_INFO(logger_, "Auto climb started by %s. Entering ALIGN.", source);
    }

    void abort_auto_climb(const char* reason) {
        reset_all_controls();
        RCLCPP_INFO(logger_, "Auto climb aborted (%s).", reason);
    }

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

    AutoClimbControl update_manual_support_control(const rmcs_msgs::Keyboard& /*keyboard*/) {
        AutoClimbControl control;

        // if (keyboard.b || *rotary_knob_switch_ == rmcs_msgs::Switch::UP) {
        //     stop_manual_support();
        //     control.back_climber_velocity = climber_back_control_velocity_abs_;
        //     return control;
        // }

        // if (last_keyboard_.b) {
        //     manual_support_retracting_          = true;
        //     manual_support_retract_block_count_ = 0;
        //     manual_support_zero_output_         = false;
        //     RCLCPP_INFO(logger_, "Manual support retract started.");
        // }

        // if (!manual_support_retracting_)
        //     return control;

        // control.back_climber_velocity = -auto_climb_support_retract_velocity_abs_;

        // if (is_back_climber_blocked())
        //     manual_support_retract_block_count_++;
        // else
        //     manual_support_retract_block_count_ = 0;

        // RCLCPP_INFO_THROTTLE(
        //     logger_, *get_clock(), 500, "MANUAL_SUPPORT_RETRACT: blocked_ticks=%d",
        //     manual_support_retract_block_count_);

        // if (manual_support_retract_block_count_ >= kManualSupportRetractConfirmTicks) {
        //     stop_manual_support();
        //     manual_support_zero_output_ = true;
        //     RCLCPP_INFO(logger_, "Manual support retract completed.");
        //     return {};
        // }

        return control;
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
        *climber_front_left_control_torque_  = nan_;
        *climber_front_right_control_torque_ = nan_;
        *climber_back_left_control_torque_   = nan_;
        *climber_back_right_control_torque_  = nan_;
        *climbing_forward_velocity_          = nan_;
        stop_manual_support();
        stop_auto_climb();
    }

    void apply_manual_support_zero_output() {
        if (!manual_support_zero_output_)
            return;

        *climber_back_left_control_torque_  = 0.0;
        *climber_back_right_control_torque_ = 0.0;
    }

    void stop_manual_support() {
        manual_support_retracting_          = false;
        manual_support_retract_block_count_ = 0;
        manual_support_zero_output_         = false;
    }

    void stop_auto_climb() { climber_controller_.reset(); }

    void dual_motor_sync_control(
        double setpoint, double left_velocity, double right_velocity,
        pid::MatrixPidCalculator<2>& pid_calculator, double& left_torque_out,
        double& right_torque_out) {

        if (std::isnan(setpoint)) {
            left_torque_out  = nan_;
            right_torque_out = nan_;
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
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();

    double sync_coefficient_;

    climber::ClimberController climber_controller_{climber::ClimberController::Config{}};
    bool manual_support_retracting_         = false;
    int manual_support_retract_block_count_ = 0;
    bool manual_support_zero_output_        = false;

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
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;

    InputInterface<double> chassis_pitch_imu_;

    rmcs_msgs::Switch last_rotary_knob_switch_ = rmcs_msgs::Switch::UNKNOWN;
    rmcs_msgs::Keyboard last_keyboard_         = rmcs_msgs::Keyboard::zero();
    double last_joystick_right_x_              = 0.0;

    pid::MatrixPidCalculator<2> front_velocity_pid_calculator_, back_velocity_pid_calculator_;
    InputInterface<Eigen::Vector2d> joystick_right_;
};
} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::EngineerChassisClimberController, rmcs_executor::Component)
