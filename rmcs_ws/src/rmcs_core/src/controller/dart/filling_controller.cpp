#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/filling_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class FillingController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using FillingCmd = rmcs_dart_guidance::msg::FillingCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    FillingController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/filling/command", command_, false);
        register_output("/dart/filling/status", status_, MechStatus::IDLE);

        register_input("/dart/filling_lift/left_motor/velocity", left_lift_velocity_, false);
        register_input("/dart/filling_lift/left_motor/torque", left_lift_torque_, false);
        register_input("/dart/filling_lift/right_motor/velocity", right_lift_velocity_, false);
        register_input("/dart/filling_lift/right_motor/torque", right_lift_torque_, false);

        register_output(
            "/dart/filling_lift/left_motor/control_velocity", left_lift_control_velocity_, NAN);
        register_output(
            "/dart/filling_lift/right_motor/control_velocity", right_lift_control_velocity_, NAN);

        register_output("/dart/limiting_servo/control_angle", servo_control_angle_, uint16_t{0});

        get_parameter("lift_control_velocity", lift_control_velocity_);

        int64_t stall_ticks = 50;
        get_parameter("lift_stall_ticks", stall_ticks);
        lift_stall_ticks_ = static_cast<int>(stall_ticks);
        get_parameter("lift_stall_velocity_threshold", lift_stall_velocity_threshold_);
        get_parameter("lift_stall_torque_threshold", lift_stall_torque_threshold_);

        int64_t limit_angle = 0;
        get_parameter("limit_free_angle", limit_angle);
        limit_free_angle_ = static_cast<uint16_t>(limit_angle);
        limit_angle = 0;
        get_parameter("limit_lock_angle", limit_angle);
        limit_lock_angle_ = static_cast<uint16_t>(limit_angle);
        servo_angle_ = limit_lock_angle_;

        int64_t complete_ticks = 100;
        get_parameter("limit_complete_ticks", complete_ticks);
        limit_complete_ticks_ = static_cast<int>(complete_ticks);

        int64_t pulse_ticks = 100;
        get_parameter("limit_pulse_ticks", pulse_ticks);
        limit_pulse_ticks_ = static_cast<int>(pulse_ticks);
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(FillingCmd::IDLE);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/filling/command\". Set to IDLE.");
        }
    }

    void update() override {
        const auto cmd = command_.ready() ? *command_ : FillingCmd::IDLE;

        const double l_vel = left_lift_velocity_.ready() ? *left_lift_velocity_ : 0.0;
        const double l_torque = left_lift_torque_.ready() ? *left_lift_torque_ : 0.0;
        const double r_vel = right_lift_velocity_.ready() ? *right_lift_velocity_ : 0.0;
        const double r_torque = right_lift_torque_.ready() ? *right_lift_torque_ : 0.0;

        MechStatus status = MechStatus::IDLE;
        double target_l_vel = NAN;
        double target_r_vel = NAN;
        uint16_t target_angle = servo_angle_;

        const bool is_new_command =
            rmcs_dart_guidance::msg::is_active(cmd) && cmd != active_cmd_;
        update_active_ticks(cmd, is_new_command);

        switch (cmd) {
        case FillingCmd::IDLE:
            active_cmd_ = FillingCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            status = MechStatus::IDLE;
            break;

        case FillingCmd::ABORT:
            active_cmd_ = FillingCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            target_l_vel = target_r_vel = 0.0;
            status = MechStatus::ABORTED;
            break;

        case FillingCmd::LIFT_UP:
            status = handle_lift(
                is_new_command, true, l_vel, l_torque, r_vel, r_torque, target_l_vel, target_r_vel);
            break;

        case FillingCmd::LIFT_DOWN:
            status = handle_lift(
                is_new_command, false, l_vel, l_torque, r_vel, r_torque, target_l_vel, target_r_vel);
            break;

        case FillingCmd::LIMIT_FREE:
            status = handle_limit_free(is_new_command, target_angle);
            break;

        case FillingCmd::LIMIT_LOCK:
            status = handle_limit_lock(is_new_command, target_angle);
            break;

        case FillingCmd::LIMIT_PULSE_FILL:
            status = handle_limit_pulse_fill(is_new_command, target_angle);
            break;
        }

        status = enforce_minimum_active_ticks(status);

        if (status == MechStatus::SUCCEEDED) {
            if (active_cmd_ == FillingCmd::LIFT_UP || active_cmd_ == FillingCmd::LIFT_DOWN)
                target_l_vel = target_r_vel = 0.0;
            else
                target_l_vel = target_r_vel = NAN;
        }

        *left_lift_control_velocity_ = target_l_vel;
        *right_lift_control_velocity_ = target_r_vel;
        *servo_control_angle_ = target_angle;
        servo_angle_ = target_angle;
        *status_ = status;
        pending_status_ = status;
    }

private:
    static constexpr int kMinimumActiveTicks = 10;

    void update_active_ticks(FillingCmd cmd, bool is_new_command) {
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

    void reset_lift(bool is_up) {
        active_cmd_ = is_up ? FillingCmd::LIFT_UP : FillingCmd::LIFT_DOWN;
        stall_count_left_ = 0;
        stall_count_right_ = 0;
        stage_ = 0;
    }

    bool lift_stall_detected(double velocity, double torque, int& counter) {
        if (std::abs(velocity) < lift_stall_velocity_threshold_
            && std::abs(torque) > lift_stall_torque_threshold_) {
            ++counter;
        } else {
            counter = 0;
        }
        return counter >= lift_stall_ticks_;
    }

    MechStatus handle_lift(
        bool is_new_command, bool is_up, double l_vel, double l_torque, double r_vel,
        double r_torque, double& target_l_vel, double& target_r_vel) {

        if (is_new_command)
            reset_lift(is_up);

        const double vel = is_up ? lift_control_velocity_ : -lift_control_velocity_;
        target_l_vel = target_r_vel = vel;

        const bool l_stall = lift_stall_detected(l_vel, l_torque, stall_count_left_);
        const bool r_stall = lift_stall_detected(r_vel, r_torque, stall_count_right_);

        return (l_stall || r_stall) ? MechStatus::SUCCEEDED : MechStatus::BUSY;
    }

    MechStatus handle_limit_free(bool is_new_command, uint16_t& target_angle) {
        if (is_new_command) {
            active_cmd_ = FillingCmd::LIMIT_FREE;
            tick_counter_ = 0;
            stage_ = 0;
        }

        target_angle = limit_free_angle_;

        if (++tick_counter_ >= limit_complete_ticks_)
            return MechStatus::SUCCEEDED;
        return MechStatus::BUSY;
    }

    MechStatus handle_limit_lock(bool is_new_command, uint16_t& target_angle) {
        if (is_new_command) {
            active_cmd_ = FillingCmd::LIMIT_LOCK;
            tick_counter_ = 0;
            stage_ = 0;
        }

        target_angle = limit_lock_angle_;

        if (++tick_counter_ >= limit_complete_ticks_)
            return MechStatus::SUCCEEDED;
        return MechStatus::BUSY;
    }

    MechStatus handle_limit_pulse_fill(bool is_new_command, uint16_t& target_angle) {
        if (is_new_command) {
            active_cmd_ = FillingCmd::LIMIT_PULSE_FILL;
            tick_counter_ = 0;
            stage_ = 0;
        }

        if (stage_ == 0) {
            target_angle = limit_free_angle_;
            if (++tick_counter_ >= limit_pulse_ticks_) {
                stage_ = 1;
                tick_counter_ = 0;
            }
            return MechStatus::BUSY;
        }

        target_angle = limit_lock_angle_;
        if (++tick_counter_ >= limit_complete_ticks_)
            return MechStatus::SUCCEEDED;
        return MechStatus::BUSY;
    }

    InputInterface<rmcs_dart_guidance::msg::FillingCommand> command_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;

    InputInterface<double> left_lift_velocity_;
    InputInterface<double> left_lift_torque_;
    InputInterface<double> right_lift_velocity_;
    InputInterface<double> right_lift_torque_;

    OutputInterface<double> left_lift_control_velocity_;
    OutputInterface<double> right_lift_control_velocity_;
    OutputInterface<uint16_t> servo_control_angle_;

    double lift_control_velocity_ = 1.0;
    double lift_stall_velocity_threshold_ = 0.1;
    double lift_stall_torque_threshold_ = 1.0;
    int lift_stall_ticks_ = 50;

    uint16_t limit_free_angle_ = 0;
    uint16_t limit_lock_angle_ = 0;
    int limit_complete_ticks_ = 100;
    int limit_pulse_ticks_ = 100;

    FillingCmd active_cmd_{FillingCmd::IDLE};
    int active_ticks_{0};
    int stage_{0};
    int tick_counter_{0};
    uint16_t servo_angle_{0};
    MechStatus pending_status_{MechStatus::IDLE};
    int stall_count_left_{0};
    int stall_count_right_{0};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::FillingController, rmcs_executor::Component)
