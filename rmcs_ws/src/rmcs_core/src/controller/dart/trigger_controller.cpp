#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

class TriggerController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using TriggerCmd = rmcs_dart_guidance::msg::TriggerCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    TriggerController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/trigger/command", command_, false);
        register_input("/dart/trigger/setpoint", setpoint_, false);
        register_output("/dart/trigger/status", status_, MechStatus::IDLE);

        register_input("/dart/trigger/position_motor/velocity", motor_velocity_, false);
        register_input("/dart/trigger/position_motor/torque", motor_torque_, false);
        register_input("/dart/trigger/position_motor/encoder_angle", motor_encoder_angle_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch_, false);

        register_output(
            "/dart/trigger/position_motor/control_velocity", motor_control_velocity_, NAN);
        register_output("/dart/trigger/position_motor/control_torque", motor_control_torque_, NAN);
        register_output(
            "/dart/trigger/position_motor/control_torque_limit", motor_control_torque_limit_, NAN);
        register_output("/dart/trigger_servo/value", servo_value_output_, double{0.0});

        get_parameter("trigger_free_angle", trigger_free_angle_);
        get_parameter("trigger_lock_angle", trigger_lock_angle_);
        int64_t trigger_action_ticks = 0;
        if (!get_parameter("trigger_action_ticks", trigger_action_ticks)) {
            throw std::runtime_error("Missing required parameter 'trigger_action_ticks'");
        }
        if (trigger_action_ticks < kMinimumActiveTicks) {
            RCLCPP_WARN(
                get_logger(),
                "trigger_action_ticks=%lld is less than minimum %d. Clamped to %d.",
                static_cast<long long>(trigger_action_ticks), kMinimumActiveTicks,
                kMinimumActiveTicks);
            trigger_action_ticks = kMinimumActiveTicks;
        }
        trigger_action_ticks_ = static_cast<int>(trigger_action_ticks);

        get_parameter("carriage_velocity", carriage_velocity_);
        get_parameter_or("calibrate_rollback", calibrate_rollback_, int64_t{80000});
        get_parameter_or("calibrate_rollback_velocity", calibrate_rollback_velocity_, 50.0);
        get_parameter_or("carriage_torque_limit", carriage_torque_limit_, 5.0);
        get_parameter_or("carriage_calibrate_torque_limit", carriage_calibrate_torque_limit_, 2.0);
        int64_t calibrate_launch_ticks = 100;
        get_parameter_or(
            "carriage_calibrate_launch_ticks", calibrate_launch_ticks, int64_t{100});
        carriage_calibrate_launch_ticks_ = static_cast<int>(calibrate_launch_ticks);
        get_parameter_or(
            "carriage_calibrate_control_torque", carriage_calibrate_control_torque_,
            carriage_calibrate_torque_limit_);
        configure_carriage_velocity_pid();

        get_parameter("carriage_position_kp", carriage_position_kp_);
        get_parameter("carriage_position_ki", carriage_position_ki_);
        get_parameter("carriage_position_kd", carriage_position_kd_);
        get_parameter("carriage_position_integral_max", carriage_position_integral_max_);

        int64_t tolerance = 10;
        get_parameter("carriage_position_tolerance", tolerance);
        carriage_position_tolerance_ = static_cast<int64_t>(tolerance);

        int64_t settle_ticks = 250;
        get_parameter("carriage_position_settle_ticks", settle_ticks);
        carriage_position_settle_ticks_ = static_cast<int>(settle_ticks);

        int64_t stall_ticks = 50;
        get_parameter("carriage_stall_ticks", stall_ticks);
        carriage_stall_ticks_ = static_cast<int>(stall_ticks);

        get_parameter("carriage_stall_velocity_threshold", carriage_stall_velocity_threshold_);
        get_parameter_or(
            "manual_carriage_velocity_sensitivity", manual_carriage_velocity_sensitivity_, 0.0);

        servo_value_ = trigger_free_angle_;
        *servo_value_output_ = servo_value_;
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(TriggerCmd::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/trigger/command\". Set to IDLE.");
        }
        if (!setpoint_.ready()) {
            setpoint_.make_and_bind_directly(std::numeric_limits<double>::quiet_NaN());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/trigger/setpoint\". Set to NaN.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!joystick_right_.ready()) {
            joystick_right_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }
        if (!rotary_knob_switch_.ready()) {
            rotary_knob_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/remote/rotary_knob_switch\". Set to UNKNOWN.");
        }
    }

    void update() override {
        const double motor_vel = motor_velocity_.ready() ? *motor_velocity_ : 0.0;
        const bool motor_encoder_ready = motor_encoder_angle_.ready();
        const int64_t motor_encoder = motor_encoder_ready ? *motor_encoder_angle_ : int64_t{0};
        log_relative_encoder_angle(motor_encoder_ready, motor_encoder);

        if (manual_mode()) {
            update_manual(motor_vel);
            return;
        }

        const auto cmd = command_.ready() ? *command_ : TriggerCmd::IDLE;
        const double setpoint = setpoint_.ready() ? *setpoint_ : NAN;

        MechStatus status = MechStatus::IDLE;
        double target_servo = servo_value_;
        double target_motor_vel = NAN;
        double target_motor_torque = NAN;
        double target_motor_torque_limit = NAN;

        const bool is_active_command = rmcs_dart_guidance::msg::is_active(cmd);
        const bool is_new_command = is_active_command && cmd != active_cmd_;
        update_active_ticks(cmd, is_new_command);

        switch (cmd) {
        case TriggerCmd::IDLE:
            active_cmd_ = TriggerCmd::IDLE;
            active_ticks_ = 0;
            stage_ = kCalibStageLaunch;
            reset_calibration_state();
            status = MechStatus::IDLE;
            break;

        case TriggerCmd::ABORT:
            active_cmd_ = TriggerCmd::IDLE;
            active_ticks_ = 0;
            stage_ = kCalibStageLaunch;
            reset_calibration_state();
            status = MechStatus::ABORTED;
            break;

        case TriggerCmd::TRIGGER_FREE:
            status = handle_trigger_action(
                is_new_command, TriggerCmd::TRIGGER_FREE, trigger_free_angle_, target_servo);
            break;

        case TriggerCmd::TRIGGER_LOCK:
            status = handle_trigger_action(
                is_new_command, TriggerCmd::TRIGGER_LOCK, trigger_lock_angle_, target_servo);
            break;

        case TriggerCmd::CARRIAGE_UP:
            target_motor_torque_limit = carriage_torque_limit_;
            status = handle_carriage_up_down(is_new_command, true, target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_DOWN:
            target_motor_torque_limit = carriage_torque_limit_;
            status = handle_carriage_up_down(is_new_command, false, target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_GOTO:
            target_motor_torque_limit = carriage_torque_limit_;
            status = handle_carriage_goto(
                is_new_command, motor_encoder_ready, motor_encoder, setpoint, target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_CALIBRATE:
            target_motor_torque_limit = carriage_calibrate_torque_limit_;
            status = handle_carriage_calibrate(
                is_new_command, motor_vel, motor_encoder, target_motor_vel, target_motor_torque);
            break;
        }

        status = enforce_minimum_active_ticks(status);

        if (status == MechStatus::SUCCEEDED) {
            if (active_cmd_ == TriggerCmd::CARRIAGE_GOTO
                || active_cmd_ == TriggerCmd::CARRIAGE_CALIBRATE) {
                target_motor_vel = NAN;
                target_motor_torque = NAN;
                target_motor_torque_limit = NAN;
            }
        }

        if (std::isfinite(target_motor_torque)) {
            carriage_velocity_pid_.reset();
        } else if (std::isfinite(target_motor_vel)) {
            target_motor_torque = calculate_velocity_control_torque(
                target_motor_vel, motor_vel, target_motor_torque_limit);
        } else {
            carriage_velocity_pid_.reset();
        }

        *servo_value_output_ = target_servo;
        servo_value_ = target_servo;
        *motor_control_velocity_ = target_motor_vel;
        *motor_control_torque_ = target_motor_torque;
        *motor_control_torque_limit_ = target_motor_torque_limit;
        *status_ = status;
        pending_status_ = status;
    }

private:
    static constexpr int kMinimumActiveTicks = 10;
    static constexpr int kRelativeEncoderLogThrottleMs = 2000;
    static constexpr int kCalibStageLaunch = 0;
    static constexpr int kCalibStageDown = 1;
    static constexpr int kCalibStageRollback = 2;
    static constexpr int kCalibStageFinalRollback = 3;
    static constexpr int kCalibSampleTarget = 3;

    bool manual_mode() const {
        return switch_left_.ready() && *switch_left_ == rmcs_msgs::Switch::UP;
    }

    bool manual_trigger_mode() const {
        return manual_mode() && switch_right_.ready()
            && *switch_right_ == rmcs_msgs::Switch::MIDDLE;
    }

    void log_relative_encoder_angle(bool encoder_ready, int64_t motor_encoder) {
        const double relative_encoder_angle =
            encoder_ready && calibrated_zero_valid_
                ? static_cast<double>(motor_encoder - calibrated_zero_)
                : std::numeric_limits<double>::quiet_NaN();

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), kRelativeEncoderLogThrottleMs,
            "[TriggerController] carriage relative encoder-angle=%.0f", relative_encoder_angle);
    }

    void update_manual(double motor_vel) {
        active_cmd_ = TriggerCmd::IDLE;
        active_ticks_ = 0;
        stage_ = kCalibStageLaunch;
        settle_count_ = 0;
        reset_calibration_state();

        double target_motor_vel = 0.0;
        double target_servo = servo_value_;
        if (manual_trigger_mode()) {
            if (joystick_right_.ready()) {
                target_motor_vel = manual_carriage_velocity_sensitivity_ * joystick_right_->x();
            }
            if (rotary_knob_switch_.ready()) {
                if (*rotary_knob_switch_ == rmcs_msgs::Switch::UP) {
                    target_servo = trigger_lock_angle_;
                } else if (*rotary_knob_switch_ == rmcs_msgs::Switch::DOWN) {
                    target_servo = trigger_free_angle_;
                }
            }
        }

        const double target_motor_torque =
            calculate_velocity_control_torque(target_motor_vel, motor_vel, carriage_torque_limit_);

        *servo_value_output_ = target_servo;
        servo_value_ = target_servo;
        *motor_control_velocity_ = target_motor_vel;
        *motor_control_torque_ = target_motor_torque;
        *motor_control_torque_limit_ = carriage_torque_limit_;
        *status_ = MechStatus::BUSY;
        pending_status_ = MechStatus::BUSY;
    }

    void configure_carriage_velocity_pid() {
        get_parameter_or("carriage_velocity_pid_kp", carriage_velocity_pid_.kp, 0.3);
        get_parameter_or("carriage_velocity_pid_ki", carriage_velocity_pid_.ki, 0.0);
        get_parameter_or("carriage_velocity_pid_kd", carriage_velocity_pid_.kd, 0.0);

        const double inf = std::numeric_limits<double>::infinity();
        get_parameter_or(
            "carriage_velocity_pid_integral_min", carriage_velocity_pid_.integral_min, -inf);
        get_parameter_or(
            "carriage_velocity_pid_integral_max", carriage_velocity_pid_.integral_max, inf);
        get_parameter_or(
            "carriage_velocity_pid_integral_split_min", carriage_velocity_pid_.integral_split_min,
            -inf);
        get_parameter_or(
            "carriage_velocity_pid_integral_split_max", carriage_velocity_pid_.integral_split_max,
            inf);
        get_parameter_or(
            "carriage_velocity_pid_output_min", carriage_velocity_pid_.output_min, -inf);
        get_parameter_or(
            "carriage_velocity_pid_output_max", carriage_velocity_pid_.output_max, inf);
    }

    double calculate_velocity_control_torque(
        double target_velocity, double measured_velocity, double torque_limit) {
        const double torque = carriage_velocity_pid_.update(target_velocity - measured_velocity);
        return clamp_torque(torque, torque_limit);
    }

    static double clamp_torque(double torque, double torque_limit) {
        if (!std::isfinite(torque))
            return torque;

        const double abs_limit = std::abs(torque_limit);
        if (std::isfinite(abs_limit))
            return std::clamp(torque, -abs_limit, abs_limit);
        return torque;
    }

    void update_active_ticks(TriggerCmd cmd, bool is_new_command) {
        if (!rmcs_dart_guidance::msg::is_active(cmd)) {
            active_ticks_ = 0;
            return;
        }
        if (is_new_command) {
            active_ticks_ = 1;
            return;
        }
        ++active_ticks_;
    }

    MechStatus enforce_minimum_active_ticks(MechStatus status) const {
        if (status == MechStatus::SUCCEEDED && active_ticks_ < kMinimumActiveTicks)
            return MechStatus::BUSY;
        return status;
    }

    MechStatus handle_trigger_action(
        bool is_new_command, TriggerCmd cmd, double target_angle, double& target_servo) {
        if (is_new_command)
            active_cmd_ = cmd;

        target_servo = target_angle;
        return active_ticks_ >= trigger_action_ticks_ ? MechStatus::SUCCEEDED : MechStatus::BUSY;
    }

    bool carriage_stall_detected(double velocity, int& counter) {
        if (std::abs(velocity) <= carriage_stall_velocity_threshold_) {
            ++counter;
        } else {
            counter = 0;
        }
        return counter >= carriage_stall_ticks_;
    }

    void reset_calibration_state() {
        calib_launch_counter_ = 0;
        calib_stall_counter_ = 0;
        calib_sample_count_ = 0;
        calib_encoder_sum_ = 0;
        calib_rollback_start_encoder_ = 0;
    }

    void start_calibrate_launch_stage() {
        stage_ = kCalibStageLaunch;
        calib_launch_counter_ = 0;
        calib_stall_counter_ = 0;
    }

    void update_calibrate_launch_stage(double& target_motor_vel, double& target_motor_torque) {
        target_motor_vel = NAN;
        if (calib_launch_counter_ >= carriage_calibrate_launch_ticks_) {
            stage_ = kCalibStageDown;
            calib_stall_counter_ = 0;
            target_motor_torque = -std::abs(carriage_calibrate_control_torque_);
            return;
        }

        target_motor_torque = -std::abs(carriage_calibrate_torque_limit_);
        ++calib_launch_counter_;
    }

    bool calibrate_rollback_reached(int64_t motor_encoder) const {
        const int64_t delta = motor_encoder >= calib_rollback_start_encoder_
                                ? motor_encoder - calib_rollback_start_encoder_
                                : calib_rollback_start_encoder_ - motor_encoder;
        return delta >= calibrate_rollback_;
    }

    MechStatus handle_carriage_up_down(
        bool is_new_command, bool is_up, double& target_motor_vel) {

        if (is_new_command) {
            active_cmd_ = is_up ? TriggerCmd::CARRIAGE_UP : TriggerCmd::CARRIAGE_DOWN;
            stage_ = kCalibStageLaunch;
        }

        target_motor_vel = is_up ? carriage_velocity_ : -carriage_velocity_;
        return MechStatus::BUSY;
    }

    MechStatus handle_carriage_goto(
        bool is_new_command, bool encoder_ready, int64_t motor_encoder, double setpoint,
        double& target_motor_vel) {

        if (is_new_command) {
            active_cmd_ = TriggerCmd::CARRIAGE_GOTO;
            pid_integral_ = 0.0;
            pid_last_error_ = 0;
            settle_count_ = 0;
            stage_ = kCalibStageLaunch;
        }

        if (!encoder_ready || !calibrated_zero_valid_ || !std::isfinite(setpoint))
            return MechStatus::FAILED;

        const int64_t target_encoder = calibrated_zero_ + static_cast<int64_t>(setpoint);
        const int64_t error = motor_encoder - target_encoder;

        pid_integral_ += error;
        if (pid_integral_ > carriage_position_integral_max_)
            pid_integral_ = carriage_position_integral_max_;
        else if (pid_integral_ < -carriage_position_integral_max_)
            pid_integral_ = -carriage_position_integral_max_;

        const double derivative = static_cast<double>(error - pid_last_error_);
        pid_last_error_ = error;

        target_motor_vel = carriage_position_kp_ * static_cast<double>(error)
                         + carriage_position_ki_ * pid_integral_
                         + carriage_position_kd_ * derivative;

        if (std::abs(error) <= carriage_position_tolerance_) {
            ++settle_count_;
            if (settle_count_ >= carriage_position_settle_ticks_)
                return MechStatus::SUCCEEDED;
        } else {
            settle_count_ = 0;
        }

        return MechStatus::BUSY;
    }

    MechStatus handle_carriage_calibrate(
        bool is_new_command, double motor_vel, int64_t motor_encoder, double& target_motor_vel,
        double& target_motor_torque) {

        if (is_new_command) {
            active_cmd_ = TriggerCmd::CARRIAGE_CALIBRATE;
            reset_calibration_state();
            start_calibrate_launch_stage();
        }

        if (stage_ == kCalibStageRollback || stage_ == kCalibStageFinalRollback) {
            target_motor_vel = calibrate_rollback_velocity_;
            if (calibrate_rollback_reached(motor_encoder)) {
                if (stage_ == kCalibStageFinalRollback)
                    return MechStatus::SUCCEEDED;

                start_calibrate_launch_stage();
                update_calibrate_launch_stage(target_motor_vel, target_motor_torque);
            }
            return MechStatus::BUSY;
        }

        if (stage_ == kCalibStageLaunch) {
            update_calibrate_launch_stage(target_motor_vel, target_motor_torque);
            return MechStatus::BUSY;
        }

        target_motor_vel = NAN;
        target_motor_torque = -std::abs(carriage_calibrate_control_torque_);
        const bool stalled = carriage_stall_detected(motor_vel, calib_stall_counter_);
        if (stalled) {
            ++calib_sample_count_;
            calib_encoder_sum_ += motor_encoder;
            if (calib_sample_count_ >= kCalibSampleTarget) {
                calibrated_zero_ = calib_encoder_sum_ / calib_sample_count_;
                calibrated_zero_valid_ = true;
                stage_ = kCalibStageFinalRollback;
                calib_rollback_start_encoder_ = motor_encoder;
                calib_stall_counter_ = 0;
                target_motor_vel = calibrate_rollback_velocity_;
                target_motor_torque = NAN;
                return MechStatus::BUSY;
            }

            stage_ = kCalibStageRollback;
            calib_rollback_start_encoder_ = motor_encoder;
            calib_stall_counter_ = 0;
            target_motor_vel = calibrate_rollback_velocity_;
            target_motor_torque = NAN;
        }

        return MechStatus::BUSY;
    }

    InputInterface<rmcs_dart_guidance::msg::TriggerCommand> command_;
    InputInterface<double> setpoint_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;

    InputInterface<double> motor_velocity_;
    InputInterface<double> motor_torque_;
    InputInterface<int64_t> motor_encoder_angle_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch_;

    OutputInterface<double> motor_control_velocity_;
    OutputInterface<double> motor_control_torque_;
    OutputInterface<double> motor_control_torque_limit_;
    OutputInterface<double> servo_value_output_;

    double trigger_free_angle_ = 0.0;
    double trigger_lock_angle_ = 0.0;
    int trigger_action_ticks_ = kMinimumActiveTicks;

    double carriage_velocity_ = 1.0;
    int64_t calibrate_rollback_ = 80000;
    double calibrate_rollback_velocity_ = 50.0;
    double carriage_torque_limit_ = 5.0;
    double carriage_calibrate_torque_limit_ = 2.0;
    int carriage_calibrate_launch_ticks_ = 100;
    double carriage_calibrate_control_torque_ = 2.0;
    rmcs_core::controller::pid::PidCalculator carriage_velocity_pid_{0.3, 0.0, 0.0};

    double carriage_position_kp_ = 0.1;
    double carriage_position_ki_ = 0.01;
    double carriage_position_kd_ = 0.0;
    double carriage_position_integral_max_ = 10.0;
    int64_t carriage_position_tolerance_ = 10;
    int carriage_position_settle_ticks_ = 250;

    double carriage_stall_velocity_threshold_ = 0.1;
    double manual_carriage_velocity_sensitivity_ = 0.0;
    int carriage_stall_ticks_ = 50;

    TriggerCmd active_cmd_{TriggerCmd::IDLE};
    int active_ticks_{0};
    int stage_{0};
    MechStatus pending_status_{MechStatus::IDLE};

    double servo_value_ = 0.0;

    double pid_integral_ = 0.0;
    int64_t pid_last_error_ = 0;
    int settle_count_ = 0;

    int calib_launch_counter_ = 0;
    int calib_stall_counter_ = 0;
    int calib_sample_count_ = 0;
    int64_t calib_encoder_sum_ = 0;
    int64_t calib_rollback_start_encoder_ = 0;
    int64_t calibrated_zero_ = 0;
    bool calibrated_zero_valid_ = false;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::TriggerController, rmcs_executor::Component)
