#include <cmath>
#include <limits>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_dart_guidance/msg/trigger_command.hpp>
#include <rmcs_executor/component.hpp>

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

        register_output(
            "/dart/trigger/position_motor/control_velocity", motor_control_velocity_, NAN);
        register_output("/dart/trigger_servo/value", servo_value_output_, double{0.0});

        get_parameter("trigger_free_angle", trigger_free_angle_);
        get_parameter("trigger_lock_angle", trigger_lock_angle_);

        get_parameter("carriage_velocity", carriage_velocity_);
        get_parameter("carriage_calibrate_velocity", carriage_calibrate_velocity_);

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

        get_parameter(
            "carriage_stall_velocity_threshold", carriage_stall_velocity_threshold_);
        get_parameter(
            "carriage_stall_torque_threshold", carriage_stall_torque_threshold_);

        servo_value_ = trigger_lock_angle_;
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(TriggerCmd::IDLE);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/trigger/command\". Set to IDLE.");
        }
        if (!setpoint_.ready()) {
            setpoint_.make_and_bind_directly(std::numeric_limits<double>::quiet_NaN());
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/trigger/setpoint\". Set to NaN.");
        }
    }

    void update() override {
        const auto cmd = command_.ready() ? *command_ : TriggerCmd::IDLE;
        const double setpoint = setpoint_.ready() ? *setpoint_ : NAN;

        const double motor_vel = motor_velocity_.ready() ? *motor_velocity_ : 0.0;
        const double motor_torque = motor_torque_.ready() ? *motor_torque_ : 0.0;
        const int64_t motor_encoder =
            motor_encoder_angle_.ready() ? *motor_encoder_angle_ : int64_t{0};

        MechStatus status = MechStatus::IDLE;
        double target_servo = servo_value_;
        double target_motor_vel = NAN;

        const bool is_active_command = rmcs_dart_guidance::msg::is_active(cmd);
        const bool is_new_command = is_active_command && cmd != active_cmd_;
        update_active_ticks(cmd, is_new_command);

        switch (cmd) {
        case TriggerCmd::IDLE:
            active_cmd_ = TriggerCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            status = MechStatus::IDLE;
            break;

        case TriggerCmd::ABORT:
            active_cmd_ = TriggerCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            status = MechStatus::ABORTED;
            break;

        case TriggerCmd::TRIGGER_FREE:
            if (is_new_command)
                active_cmd_ = TriggerCmd::TRIGGER_FREE;
            target_servo = trigger_free_angle_;
            status = MechStatus::SUCCEEDED;
            break;

        case TriggerCmd::TRIGGER_LOCK:
            if (is_new_command)
                active_cmd_ = TriggerCmd::TRIGGER_LOCK;
            target_servo = trigger_lock_angle_;
            status = MechStatus::SUCCEEDED;
            break;

        case TriggerCmd::CARRIAGE_UP:
            status = handle_carriage_up_down(is_new_command, true, motor_vel, motor_torque,
                                             target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_DOWN:
            status = handle_carriage_up_down(is_new_command, false, motor_vel, motor_torque,
                                             target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_GOTO:
            status = handle_carriage_goto(is_new_command, motor_encoder, setpoint,
                                          target_motor_vel);
            break;

        case TriggerCmd::CARRIAGE_CALIBRATE:
            status = handle_carriage_calibrate(
                is_new_command, motor_vel, motor_torque, motor_encoder, target_motor_vel);
            break;
        }

        status = enforce_minimum_active_ticks(status);

        if (status == MechStatus::SUCCEEDED) {
            if (active_cmd_ == TriggerCmd::CARRIAGE_GOTO
                || active_cmd_ == TriggerCmd::CARRIAGE_CALIBRATE)
                target_motor_vel = NAN;
        }

        *servo_value_output_ = target_servo;
        servo_value_ = target_servo;
        *motor_control_velocity_ = target_motor_vel;
        *status_ = status;
        pending_status_ = status;
    }

private:
    static constexpr int kMinimumActiveTicks = 10;

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

    bool carriage_stall_detected(double velocity, double torque, int& counter) {
        if (std::abs(velocity) < carriage_stall_velocity_threshold_
            && std::abs(torque) > carriage_stall_torque_threshold_) {
            ++counter;
        } else {
            counter = 0;
        }
        return counter >= carriage_stall_ticks_;
    }

    MechStatus handle_carriage_up_down(
        bool is_new_command, bool is_up, double motor_vel, double motor_torque,
        double& target_motor_vel) {

        if (is_new_command) {
            active_cmd_ = is_up ? TriggerCmd::CARRIAGE_UP : TriggerCmd::CARRIAGE_DOWN;
            stage_ = 0;
        }

        target_motor_vel = is_up ? carriage_velocity_ : -carriage_velocity_;
        (void)motor_vel;
        (void)motor_torque;
        return MechStatus::BUSY;
    }

    MechStatus handle_carriage_goto(
        bool is_new_command, int64_t motor_encoder, double setpoint, double& target_motor_vel) {

        if (is_new_command) {
            active_cmd_ = TriggerCmd::CARRIAGE_GOTO;
            pid_integral_ = 0.0;
            pid_last_error_ = 0;
            settle_count_ = 0;
            stage_ = 0;
        }

        if (std::isnan(setpoint))
            return MechStatus::BUSY;

        const int64_t error =
            (calibrated_zero_ + static_cast<int64_t>(setpoint)) - motor_encoder;

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
        bool is_new_command, double motor_vel, double motor_torque, int64_t motor_encoder,
        double& target_motor_vel) {

        if (is_new_command) {
            active_cmd_ = TriggerCmd::CARRIAGE_CALIBRATE;
            calib_stall_counter_ = 0;
            calib_sample_count_ = 0;
            calib_encoder_sum_ = 0;
            stage_ = 0;
        }

        target_motor_vel = -carriage_calibrate_velocity_;

        const bool stalled =
            carriage_stall_detected(motor_vel, motor_torque, calib_stall_counter_);
        if (stalled) {
            ++calib_sample_count_;
            calib_encoder_sum_ += motor_encoder;
            if (calib_sample_count_ >= 3) {
                calibrated_zero_ = calib_encoder_sum_ / calib_sample_count_;
                return MechStatus::SUCCEEDED;
            }
        } else {
            calib_sample_count_ = 0;
            calib_encoder_sum_ = 0;
        }

        return MechStatus::BUSY;
    }

    InputInterface<rmcs_dart_guidance::msg::TriggerCommand> command_;
    InputInterface<double> setpoint_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;

    InputInterface<double> motor_velocity_;
    InputInterface<double> motor_torque_;
    InputInterface<int64_t> motor_encoder_angle_;

    OutputInterface<double> motor_control_velocity_;
    OutputInterface<double> servo_value_output_;

    double trigger_free_angle_ = 0.0;
    double trigger_lock_angle_ = 0.0;

    double carriage_velocity_ = 1.0;
    double carriage_calibrate_velocity_ = 0.5;

    double carriage_position_kp_ = 0.1;
    double carriage_position_ki_ = 0.01;
    double carriage_position_kd_ = 0.0;
    double carriage_position_integral_max_ = 10.0;
    int64_t carriage_position_tolerance_ = 10;
    int carriage_position_settle_ticks_ = 250;

    double carriage_stall_velocity_threshold_ = 0.1;
    double carriage_stall_torque_threshold_ = 1.0;
    int carriage_stall_ticks_ = 50;

    TriggerCmd active_cmd_{TriggerCmd::IDLE};
    int active_ticks_{0};
    int stage_{0};
    MechStatus pending_status_{MechStatus::IDLE};

    double servo_value_ = 0.0;

    double pid_integral_ = 0.0;
    int64_t pid_last_error_ = 0;
    int settle_count_ = 0;

    int calib_stall_counter_ = 0;
    int calib_sample_count_ = 0;
    int64_t calib_encoder_sum_ = 0;
    int64_t calibrated_zero_ = 0;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::TriggerController, rmcs_executor::Component)
