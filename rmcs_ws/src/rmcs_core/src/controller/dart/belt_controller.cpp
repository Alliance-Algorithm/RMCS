#include <cmath>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/belt_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::dart {

class BeltController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using BeltCmd = rmcs_dart_guidance::msg::BeltCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    BeltController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/belt/command", command_, false);
        register_output("/dart/belt/status", status_, MechStatus::IDLE);

        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/left_motor/velocity", left_velocity_, false);
        register_input("/dart/belt/left_motor/torque", left_torque_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/right_motor/velocity", right_velocity_, false);
        register_input("/dart/belt/right_motor/torque", right_torque_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);

        register_output("/dart/belt/left_motor/control_velocity", left_control_velocity_, NAN);
        register_output("/dart/belt/right_motor/control_velocity", right_control_velocity_, NAN);
        register_output(
            "/dart/belt/left_motor/control_torque_limit", left_control_torque_limit_, NAN);
        register_output(
            "/dart/belt/right_motor/control_torque_limit", right_control_torque_limit_, NAN);

        get_parameter("belt_slow_down_velocity", belt_slow_down_velocity_);
        get_parameter("belt_fast_down_velocity", belt_fast_down_velocity_);
        get_parameter("belt_up_soft_stage1_velocity", belt_up_soft_stage1_velocity_);
        get_parameter("belt_up_soft_stage2_velocity", belt_up_soft_stage2_velocity_);
        get_parameter("belt_up_soft_stage3_velocity", belt_up_soft_stage3_velocity_);
        get_parameter("belt_up_stage1_velocity", belt_up_stage1_velocity_);
        get_parameter("belt_up_stage2_velocity", belt_up_stage2_velocity_);
        get_parameter("slider_rail_length", slider_rail_length_);
        get_parameter("soft_stage1_persent", soft_stage1_persent_);
        get_parameter("soft_stage2_persent", soft_stage2_persent_);
        get_parameter("part_persent", part_persent_);
        get_parameter_or("belt_load_torque_limit", belt_load_torque_limit_, 5.0);
        get_parameter_or("belt_unload_torque_limit", belt_unload_torque_limit_, 2.0);

        int64_t stall_ticks = 50;
        get_parameter("belt_stall_ticks", stall_ticks);
        stall_ticks_ = static_cast<int>(stall_ticks);

        get_parameter("belt_stall_velocity_threshold", stall_velocity_threshold_);
        get_parameter("belt_stall_torque_threshold", stall_torque_threshold_);
        get_parameter_or(
            "manual_belt_velocity_sensitivity", manual_belt_velocity_sensitivity_, 0.0);
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(BeltCmd::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/belt/command\". Set to IDLE.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!joystick_left_.ready()) {
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }
    }

    void update() override {
        if (manual_mode()) {
            update_manual();
            return;
        }

        const auto cmd = command_.ready() ? *command_ : BeltCmd::IDLE;

        const double l_angle = left_angle_.ready() ? *left_angle_ : 0.0;
        const double l_velocity = left_velocity_.ready() ? *left_velocity_ : 0.0;
        const double l_torque = left_torque_.ready() ? *left_torque_ : 0.0;
        const double r_angle = right_angle_.ready() ? *right_angle_ : 0.0;
        const double r_velocity = right_velocity_.ready() ? *right_velocity_ : 0.0;
        const double r_torque = right_torque_.ready() ? *right_torque_ : 0.0;

        MechStatus status = MechStatus::IDLE;
        double target_l_vel = NAN;
        double target_r_vel = NAN;
        double target_l_torque_limit = NAN;
        double target_r_torque_limit = NAN;

        const auto set_torque_limit = [&](double limit) {
            target_l_torque_limit = target_r_torque_limit = limit;
        };

        const bool is_active_command = rmcs_dart_guidance::msg::is_active(cmd);
        const bool is_new_command = is_active_command && cmd != active_cmd_;
        update_active_ticks(cmd, is_new_command);

        switch (cmd) {
        case BeltCmd::IDLE:
            active_cmd_ = BeltCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            status = MechStatus::IDLE;
            break;

        case BeltCmd::ABORT:
            active_cmd_ = BeltCmd::IDLE;
            active_ticks_ = 0;
            stage_ = 0;
            target_l_vel = target_r_vel = 0.0;
            status = MechStatus::ABORTED;
            break;

        case BeltCmd::BRAKE:
            set_torque_limit(belt_load_torque_limit_);
            if (is_new_command) {
                active_cmd_ = BeltCmd::BRAKE;
                stage_ = 0;
            }
            target_l_vel = target_r_vel = 0.0;
            status = MechStatus::BUSY;
            break;

        case BeltCmd::INIT:
            set_torque_limit(belt_unload_torque_limit_);
            status = handle_init(
                is_new_command, l_velocity, l_torque, r_velocity, r_torque, target_l_vel,
                target_r_vel);
            break;

        case BeltCmd::DOWN_SLOW:
        case BeltCmd::DOWN_FAST:
            set_torque_limit(belt_load_torque_limit_);
            status = handle_down_full(
                cmd, is_new_command, l_angle, l_velocity, l_torque, r_angle, r_velocity, r_torque,
                target_l_vel, target_r_vel);
            break;

        case BeltCmd::DOWN_SLOW_PART:
            set_torque_limit(belt_load_torque_limit_);
            status = handle_down_part(
                is_new_command, l_angle, l_velocity, l_torque, r_angle, r_velocity, r_torque,
                target_l_vel, target_r_vel);
            break;

        case BeltCmd::UP_SOFT:
            status = handle_up_soft(
                is_new_command, l_angle, l_velocity, l_torque, r_angle, r_velocity, r_torque,
                target_l_vel, target_r_vel);
            set_torque_limit(stage_ == 1 ? belt_load_torque_limit_ : belt_unload_torque_limit_);
            break;

        case BeltCmd::UP_SOFT_PART:
            set_torque_limit(belt_load_torque_limit_);
            status =
                handle_up_soft_part(is_new_command, l_angle, r_angle, target_l_vel, target_r_vel);
            break;

        case BeltCmd::UP_HARD:
            set_torque_limit(belt_load_torque_limit_);
            status = handle_up_hard(
                is_new_command, l_angle, l_velocity, l_torque, r_angle, r_velocity, r_torque,
                target_l_vel, target_r_vel);
            break;
        }

        status = enforce_minimum_active_ticks(status);

        if (status == MechStatus::SUCCEEDED) {
            target_l_vel = target_r_vel = post_completion_velocity(active_cmd_);
            target_l_torque_limit = target_r_torque_limit =
                post_completion_torque_limit(active_cmd_);
        }

        *left_control_velocity_ = target_l_vel;
        *right_control_velocity_ = target_r_vel;
        *left_control_torque_limit_ = target_l_torque_limit;
        *right_control_torque_limit_ = target_r_torque_limit;
        *status_ = status;
        pending_status_ = status;
    }

private:
    static constexpr int kMinimumActiveTicks = 10;

    bool manual_mode() const {
        return switch_left_.ready() && *switch_left_ == rmcs_msgs::Switch::UP;
    }

    bool manual_belt_mode() const {
        return manual_mode() && switch_right_.ready()
            && *switch_right_ == rmcs_msgs::Switch::MIDDLE;
    }

    void update_manual() {
        active_cmd_ = BeltCmd::IDLE;
        active_ticks_ = 0;
        stage_ = 0;
        stall_count_left_ = 0;
        stall_count_right_ = 0;

        double target_velocity = 0.0;
        if (manual_belt_mode() && joystick_left_.ready()) {
            target_velocity = manual_belt_velocity_sensitivity_ * joystick_left_->x();
        }

        *left_control_velocity_ = target_velocity;
        *right_control_velocity_ = target_velocity;
        *left_control_torque_limit_ = NAN;
        *right_control_torque_limit_ = NAN;
        *status_ = MechStatus::BUSY;
        pending_status_ = MechStatus::BUSY;
    }

    void update_active_ticks(BeltCmd cmd, bool is_new_command) {
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

    void reset_down(BeltCmd cmd, double l_angle, double r_angle) {
        active_cmd_ = cmd;
        start_angle_left_ = l_angle;
        start_angle_right_ = r_angle;
        stall_count_left_ = 0;
        stall_count_right_ = 0;
        stage_ = 0;
    }

    bool stall_detected(double velocity, double torque, int& counter) {
        if (std::abs(velocity) < stall_velocity_threshold_
            && std::abs(torque) > stall_torque_threshold_) {
            ++counter;
        } else {
            counter = 0;
        }
        return counter >= stall_ticks_;
    }

    MechStatus handle_init(
        bool is_new_command, double l_velocity, double l_torque, double r_velocity, double r_torque,
        double& l_vel, double& r_vel) {

        if (is_new_command) {
            active_cmd_ = BeltCmd::INIT;
            stall_count_left_ = 0;
            stall_count_right_ = 0;
            stage_ = 0;
        }

        l_vel = r_vel = -belt_up_soft_stage1_velocity_;

        const bool l_stall = stall_detected(l_velocity, l_torque, stall_count_left_);
        const bool r_stall = stall_detected(r_velocity, r_torque, stall_count_right_);

        return (l_stall || r_stall) ? MechStatus::SUCCEEDED : MechStatus::BUSY;
    }

    MechStatus handle_down_full(
        BeltCmd cmd, bool is_new_command, double l_angle, double l_velocity, double l_torque,
        double r_angle, double r_velocity, double r_torque, double& l_vel, double& r_vel) {

        if (is_new_command)
            reset_down(cmd, l_angle, r_angle);

        l_vel = (cmd == BeltCmd::DOWN_SLOW) ? belt_slow_down_velocity_ : belt_fast_down_velocity_;
        r_vel = l_vel;

        const bool l_stall = stall_detected(l_velocity, l_torque, stall_count_left_);
        const bool r_stall = stall_detected(r_velocity, r_torque, stall_count_right_);
        const bool l_reach = (l_angle - start_angle_left_) >= slider_rail_length_;
        const bool r_reach = (r_angle - start_angle_right_) >= slider_rail_length_;

        return (l_reach || r_reach || l_stall || r_stall) ? MechStatus::SUCCEEDED
                                                          : MechStatus::BUSY;
    }

    MechStatus handle_down_part(
        bool is_new_command, double l_angle, double l_velocity, double l_torque, double r_angle,
        double r_velocity, double r_torque, double& l_vel, double& r_vel) {

        if (is_new_command)
            reset_down(BeltCmd::DOWN_SLOW_PART, l_angle, r_angle);

        l_vel = belt_slow_down_velocity_;
        r_vel = belt_slow_down_velocity_;

        const double target_dist = slider_rail_length_ * (part_persent_ + 0.1);
        const bool l_stall = stall_detected(l_velocity, l_torque, stall_count_left_);
        const bool r_stall = stall_detected(r_velocity, r_torque, stall_count_right_);
        const bool l_reach = (l_angle - start_angle_left_) >= target_dist;
        const bool r_reach = (r_angle - start_angle_right_) >= target_dist;

        return (l_reach || r_reach || l_stall || r_stall) ? MechStatus::SUCCEEDED
                                                          : MechStatus::BUSY;
    }

    MechStatus handle_up_soft(
        bool is_new_command, double l_angle, double l_velocity, double l_torque, double r_angle,
        double r_velocity, double r_torque, double& l_vel, double& r_vel) {

        if (is_new_command) {
            active_cmd_ = BeltCmd::UP_SOFT;
            start_angle_left_ = l_angle;
            start_angle_right_ = r_angle;
            stall_count_left_ = 0;
            stall_count_right_ = 0;
            stage_ = 1;
        }

        const double stage1_dist = slider_rail_length_ * soft_stage1_persent_;
        const double stage2_dist = slider_rail_length_ * soft_stage2_persent_;
        const double l_delta = start_angle_left_ - l_angle;
        const double r_delta = start_angle_right_ - r_angle;
        const double max_delta = std::max(l_delta, r_delta);

        if (stage_ == 1) {
            if (max_delta >= stage2_dist)
                stage_ = 3;
            else if (max_delta >= stage1_dist)
                stage_ = 2;
        } else if (stage_ == 2) {
            if (max_delta >= stage2_dist)
                stage_ = 3;
        }

        switch (stage_) {
        case 1: l_vel = r_vel = -belt_up_soft_stage1_velocity_; break;
        case 2: l_vel = r_vel = -belt_up_soft_stage2_velocity_; break;
        default: l_vel = r_vel = -belt_up_soft_stage3_velocity_; break;
        }

        if (stage_ == 3) {
            const bool l_stall = stall_detected(l_velocity, l_torque, stall_count_left_);
            const bool r_stall = stall_detected(r_velocity, r_torque, stall_count_right_);
            if (l_stall || r_stall)
                return MechStatus::SUCCEEDED;
        }

        return MechStatus::BUSY;
    }

    MechStatus handle_up_soft_part(
        bool is_new_command, double l_angle, double r_angle, double& l_vel, double& r_vel) {

        if (is_new_command) {
            active_cmd_ = BeltCmd::UP_SOFT_PART;
            start_angle_left_ = l_angle;
            start_angle_right_ = r_angle;
            stage_ = 0;
        }

        l_vel = r_vel = -belt_up_stage2_velocity_;

        const double target_dist = slider_rail_length_ * part_persent_;
        const double l_delta = start_angle_left_ - l_angle;
        const double r_delta = start_angle_right_ - r_angle;

        return (l_delta >= target_dist || r_delta >= target_dist) ? MechStatus::SUCCEEDED
                                                                  : MechStatus::BUSY;
    }

    MechStatus handle_up_hard(
        bool is_new_command, double l_angle, double l_velocity, double l_torque, double r_angle,
        double r_velocity, double r_torque, double& l_vel, double& r_vel) {

        if (is_new_command) {
            active_cmd_ = BeltCmd::UP_HARD;
            start_angle_left_ = l_angle;
            start_angle_right_ = r_angle;
            stall_count_left_ = 0;
            stall_count_right_ = 0;
            stage_ = 0;
        }

        l_vel = r_vel = -belt_up_stage2_velocity_;

        const bool l_stall = stall_detected(l_velocity, l_torque, stall_count_left_);
        const bool r_stall = stall_detected(r_velocity, r_torque, stall_count_right_);
        const double l_delta = start_angle_left_ - l_angle;
        const double r_delta = start_angle_right_ - r_angle;

        return (l_delta >= slider_rail_length_ || r_delta >= slider_rail_length_ || l_stall
                || r_stall)
                 ? MechStatus::SUCCEEDED
                 : MechStatus::BUSY;
    }

    static double post_completion_velocity(BeltCmd cmd) {
        switch (cmd) {
        case BeltCmd::DOWN_SLOW:
        case BeltCmd::DOWN_FAST:
        case BeltCmd::DOWN_SLOW_PART:
        case BeltCmd::UP_SOFT_PART:
        case BeltCmd::UP_HARD:
        case BeltCmd::INIT: return 0.0;
        case BeltCmd::UP_SOFT: return NAN;
        default: return NAN;
        }
    }

    double post_completion_torque_limit(BeltCmd cmd) const {
        switch (cmd) {
        case BeltCmd::DOWN_SLOW:
        case BeltCmd::DOWN_FAST:
        case BeltCmd::DOWN_SLOW_PART:
        case BeltCmd::UP_SOFT_PART:
        case BeltCmd::UP_HARD: return belt_load_torque_limit_;
        case BeltCmd::INIT: return belt_unload_torque_limit_;
        default: return NAN;
        }
    }

    InputInterface<rmcs_dart_guidance::msg::BeltCommand> command_;
    OutputInterface<rmcs_dart_guidance::msg::MechanismStatus> status_;

    InputInterface<double> left_angle_;
    InputInterface<double> left_velocity_;
    InputInterface<double> left_torque_;
    InputInterface<double> right_angle_;
    InputInterface<double> right_velocity_;
    InputInterface<double> right_torque_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;

    OutputInterface<double> left_control_velocity_;
    OutputInterface<double> right_control_velocity_;
    OutputInterface<double> left_control_torque_limit_;
    OutputInterface<double> right_control_torque_limit_;

    double belt_slow_down_velocity_ = 1.0;
    double belt_fast_down_velocity_ = 2.0;
    double belt_up_soft_stage1_velocity_ = 1.0;
    double belt_up_soft_stage2_velocity_ = 0.5;
    double belt_up_soft_stage3_velocity_ = 0.3;
    double belt_up_stage1_velocity_ = 1.0;
    double belt_up_stage2_velocity_ = 2.0;
    double slider_rail_length_ = 1.0;
    double soft_stage1_persent_ = 0.3;
    double soft_stage2_persent_ = 0.8;
    double part_persent_ = 0.5;
    double belt_load_torque_limit_ = 5.0;
    double belt_unload_torque_limit_ = 2.0;

    double stall_velocity_threshold_ = 0.1;
    double stall_torque_threshold_ = 1.0;
    double manual_belt_velocity_sensitivity_ = 0.0;
    int stall_ticks_ = 50;

    BeltCmd active_cmd_{BeltCmd::IDLE};
    int active_ticks_{0};
    int stage_{0};
    MechStatus pending_status_{MechStatus::IDLE};

    double start_angle_left_{0.0};
    double start_angle_right_{0.0};
    int stall_count_left_{0};
    int stall_count_right_{0};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::dart::BeltController, rmcs_executor::Component)
