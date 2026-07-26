#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/four_z_chassis_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::dart {

class FourZAxisChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using FourZCommand = rmcs_dart_guidance::msg::FourZChassisCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    FourZAxisChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/dart/chassis/4z/command", command_, false);

        register_input("/dart/chassis/pitch", pitch_, false);
        register_input("/dart/chassis/roll", roll_, false);
        register_input("/dart/chassis/height", height_, false);
        register_input("/dart/chassis/height_velocity", height_velocity_, false);
        register_input("/dart/chassis/pitch_rate", pitch_rate_, false);
        register_input("/dart/chassis/roll_rate", roll_rate_, false);
        register_input("/dart/chassis/height_calibrated", height_calibrated_, false);

        for (size_t i = 0; i < kAxisCount; ++i) {
            register_input(kMotorPrefixes[i] + "/height", axis_height_[i], false);
            register_input(kMotorPrefixes[i] + "/height_velocity", axis_height_velocity_[i], false);
            register_input(kMotorPrefixes[i] + "/stroke_angle", axis_stroke_angle_[i], false);
            register_input(
                kMotorPrefixes[i] + "/bottom_limit_switch", bottom_limit_switch_[i], false);

            register_output(kMotorPrefixes[i] + "/control_torque", control_torque_[i], kNaN);
            register_output(kMotorPrefixes[i] + "/axis_effort", axis_effort_output_[i], kNaN);
        }

        register_output("/dart/chassis/4z/status", status_output_, MechStatus::IDLE);
        register_output("/dart/chassis/height_degraded", height_degraded_output_, false);
        register_output(
            "/dart/chassis/height_target_reachable", height_target_reachable_output_, true);

        read_parameters();
    }

    void update() override {
        const auto command = command_.ready() ? *command_ : FourZCommand::IDLE;
        const auto feedback = read_feedback();

        if (command == FourZCommand::IDLE) {
            enter_idle(MechStatus::IDLE);
            publish_status(MechStatus::IDLE);
            return;
        }

        if (command == FourZCommand::ABORT) {
            enter_idle(MechStatus::ABORTED);
            publish_status(MechStatus::ABORTED);
            return;
        }

        const bool is_new_command = command != active_command_;
        if (is_new_command)
            enter_command(command, feedback);

        if (is_terminal(current_status_)) {
            release_outputs();
            publish_status(current_status_);
            return;
        }

        ++active_ticks_;

        std::array<double, kAxisCount> axis_effort{};
        axis_effort.fill(0.0);

        MechStatus next_status = MechStatus::BUSY;
        switch (active_command_) {
        case FourZCommand::CALIBRATE_BOTTOM:
            next_status = update_calibrate_bottom(feedback, axis_effort);
            break;
        case FourZCommand::LEVEL_ZERO:
            next_status = update_level_zero(feedback, axis_effort);
            break;
        case FourZCommand::DOWN: next_status = update_down(feedback, axis_effort); break;
        case FourZCommand::IDLE:
        case FourZCommand::ABORT: next_status = MechStatus::IDLE; break;
        }

        current_status_ = next_status;
        if (is_terminal(next_status)) {
            reset_all_pid();
            release_outputs();
        } else {
            apply_gated_outputs(feedback, axis_effort);
        }

        publish_status(next_status);
    }

private:
    static constexpr size_t kAxisCount = 4;
    static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    static const inline std::array<std::string, kAxisCount> kMotorPrefixes{
        "/dart/chassis/front_left_motor",
        "/dart/chassis/front_back_motor",
        "/dart/chassis/back_left_motor",
        "/dart/chassis/back_right_motor",
    };
    static constexpr std::array<const char*, kAxisCount> kAxisEffortMaxParameters{
        "front_left_axis_effort_max",
        "front_back_axis_effort_max",
        "back_left_axis_effort_max",
        "back_right_axis_effort_max",
    };
    static constexpr std::array<const char*, kAxisCount> kAxisTorqueDirectionParameters{
        "front_left_axis_torque_direction",
        "front_back_axis_torque_direction",
        "back_left_axis_torque_direction",
        "back_right_axis_torque_direction",
    };

    struct ModalPid {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        double integral = 0.0;
        double integral_min = -std::numeric_limits<double>::infinity();
        double integral_max = std::numeric_limits<double>::infinity();
        double output_min = -std::numeric_limits<double>::infinity();
        double output_max = std::numeric_limits<double>::infinity();

        void reset() { integral = 0.0; }

        double update(double error) {
            if (!std::isfinite(error))
                return kNaN;
            integral = std::clamp(integral + error, integral_min, integral_max);
            return std::clamp(kp * error + ki * integral, output_min, output_max);
        }
    };

    struct Feedback {
        double height = 0.0;
        double height_velocity = 0.0;
        double pitch = 0.0;
        double roll = 0.0;
        double pitch_rate = 0.0;
        double roll_rate = 0.0;
        bool height_calibrated = false;
        std::array<double, kAxisCount> z{};
        std::array<double, kAxisCount> velocity{};
        std::array<double, kAxisCount> stroke_angle{};
        std::array<bool, kAxisCount> bottom{};
    };

    static bool is_terminal(MechStatus status) {
        return status == MechStatus::SUCCEEDED || status == MechStatus::FAILED
            || status == MechStatus::ABORTED;
    }

    void read_parameters() {
        get_parameter_or("axis_x", axis_x_, 0.20);
        get_parameter_or("axis_y", axis_y_, 0.15);

        get_parameter_or("calibrate_down_effort", calibrate_down_effort_, 0.2);
        get_parameter_or("down_effort", down_effort_, 0.2);
        get_parameter_or("max_stroke", max_stroke_, 20.0);
        get_parameter_or("sync_output_max", sync_output_max_, 0.1);
        get_parameter_or("height_degrade_margin", height_degrade_margin_, 0.001);
        get_parameter_or("limit_hold_margin", limit_hold_margin_, 0.02);

        get_parameter_or("level_height_tolerance", level_height_tolerance_, 0.005);
        get_parameter_or("level_height_velocity_tolerance", level_height_velocity_tolerance_, 0.01);
        get_parameter_or("level_pitch_tolerance", level_pitch_tolerance_, 0.01);
        get_parameter_or("level_roll_tolerance", level_roll_tolerance_, 0.01);
        get_parameter_or("level_pitch_rate_tolerance", level_pitch_rate_tolerance_, 0.02);
        get_parameter_or("level_roll_rate_tolerance", level_roll_rate_tolerance_, 0.02);

        get_parameter_or("level_settle_ticks", level_settle_ticks_, int64_t{250});
        get_parameter_or("level_timeout_ticks", level_timeout_ticks_, int64_t{5000});
        get_parameter_or("calibrate_timeout_ticks", calibrate_timeout_ticks_, int64_t{5000});
        get_parameter_or("down_timeout_ticks", down_timeout_ticks_, int64_t{5000});

        double default_axis_effort_max = 1.0;
        get_parameter_or("axis_effort_max", default_axis_effort_max, default_axis_effort_max);
        double default_axis_torque_direction = 1.0;
        get_parameter_or(
            "axis_torque_direction", default_axis_torque_direction, default_axis_torque_direction);

        axis_effort_max_.fill(default_axis_effort_max);
        axis_torque_direction_.fill(default_axis_torque_direction);
        for (size_t i = 0; i < kAxisCount; ++i) {
            get_parameter_or(kAxisEffortMaxParameters[i], axis_effort_max_[i], axis_effort_max_[i]);
            get_parameter_or(
                kAxisTorqueDirectionParameters[i], axis_torque_direction_[i],
                axis_torque_direction_[i]);
        }

        read_pid("height", height_pid_);
        read_pid("height_bias", height_bias_pid_);
        read_pid("pitch", pitch_pid_);
        read_pid("roll", roll_pid_);
        read_pid("sync", sync_pid_template_);
        sync_pid_.fill(sync_pid_template_);
    }

    void read_pid(const std::string& name, ModalPid& pid) {
        get_parameter_or(name + "_kp", pid.kp, 0.0);
        get_parameter_or(name + "_ki", pid.ki, 0.0);
        get_parameter_or(name + "_kd", pid.kd, 0.0);
        get_parameter_or(name + "_integral_min", pid.integral_min, pid.integral_min);
        get_parameter_or(name + "_integral_max", pid.integral_max, pid.integral_max);
        get_parameter_or(name + "_output_min", pid.output_min, pid.output_min);
        get_parameter_or(name + "_output_max", pid.output_max, pid.output_max);
    }

    Feedback read_feedback() const {
        Feedback feedback;
        feedback.height = height_.ready() ? *height_ : 0.0;
        feedback.height_velocity = height_velocity_.ready() ? *height_velocity_ : 0.0;
        feedback.pitch = pitch_.ready() ? *pitch_ : 0.0;
        feedback.roll = roll_.ready() ? *roll_ : 0.0;
        feedback.pitch_rate = pitch_rate_.ready() ? *pitch_rate_ : 0.0;
        feedback.roll_rate = roll_rate_.ready() ? *roll_rate_ : 0.0;
        feedback.height_calibrated = height_calibrated_.ready() && *height_calibrated_;

        feedback.stroke_angle.fill(kNaN);
        for (size_t i = 0; i < kAxisCount; ++i) {
            feedback.z[i] = axis_height_[i].ready() ? *axis_height_[i] : 0.0;
            feedback.velocity[i] =
                axis_height_velocity_[i].ready() ? *axis_height_velocity_[i] : 0.0;
            feedback.stroke_angle[i] =
                axis_stroke_angle_[i].ready() ? *axis_stroke_angle_[i] : kNaN;
            feedback.bottom[i] = bottom_limit_switch_[i].ready() && *bottom_limit_switch_[i];
        }
        return feedback;
    }

    void enter_idle(MechStatus idle_status) {
        active_command_ = FourZCommand::IDLE;
        active_ticks_ = 0;
        settle_count_ = 0;
        height_degraded_ = false;
        height_target_reachable_ = true;
        calibrate_touched_.fill(false);
        reset_all_pid();
        release_outputs();
        current_status_ = idle_status;
    }

    void enter_command(FourZCommand command, const Feedback& feedback) {
        active_command_ = command;
        active_ticks_ = 0;
        settle_count_ = 0;
        current_status_ = MechStatus::BUSY;
        height_degraded_ = false;
        height_target_reachable_ = true;
        reset_all_pid();

        switch (command) {
        case FourZCommand::CALIBRATE_BOTTOM: calibrate_touched_ = feedback.bottom; break;
        case FourZCommand::LEVEL_ZERO:
            target_height_ = feedback.height;
            target_pitch_ = 0.0;
            target_roll_ = 0.0;
            break;
        case FourZCommand::DOWN:
            target_pitch_ = feedback.pitch;
            target_roll_ = feedback.roll;
            target_height_ = feedback.height;
            break;
        case FourZCommand::IDLE:
        case FourZCommand::ABORT: break;
        }
    }

    MechStatus update_calibrate_bottom(
        const Feedback& feedback, std::array<double, kAxisCount>& axis_effort) {
        for (size_t i = 0; i < kAxisCount; ++i)
            calibrate_touched_[i] = calibrate_touched_[i] || feedback.bottom[i];

        if (std::ranges::all_of(calibrate_touched_, [](bool touched) { return touched; }))
            return MechStatus::SUCCEEDED;

        if (active_ticks_ >= calibrate_timeout_ticks_)
            return MechStatus::FAILED;

        axis_effort.fill(-calibrate_down_effort_);
        height_degraded_ = false;
        height_target_reachable_ = true;
        return MechStatus::BUSY;
    }

    MechStatus
        update_level_zero(const Feedback& feedback, std::array<double, kAxisCount>& axis_effort) {
        height_target_reachable_ = compute_height_target_reachable(feedback);

        if (!height_degraded_) {
            compute_normal_pose_effort(
                feedback, target_height_, target_pitch_, target_roll_, axis_effort);
            if (!height_target_reachable_ || normal_level_should_degrade(feedback, axis_effort)) {
                height_degraded_ = true;
                height_pid_.reset();
                height_bias_pid_.reset();
            }
        }

        if (height_degraded_)
            compute_height_degraded_effort(feedback, target_pitch_, target_roll_, axis_effort);

        if (level_settled(feedback)) {
            ++settle_count_;
            if (settle_count_ >= level_settle_ticks_)
                return MechStatus::SUCCEEDED;
        } else {
            settle_count_ = 0;
        }

        if (active_ticks_ >= level_timeout_ticks_)
            return MechStatus::FAILED;

        return MechStatus::BUSY;
    }

    MechStatus update_down(const Feedback& feedback, std::array<double, kAxisCount>& axis_effort) {
        height_degraded_ = false;
        height_target_reachable_ = true;

        if (std::ranges::any_of(feedback.bottom, [](bool bottom) { return bottom; }))
            return MechStatus::SUCCEEDED;

        if (active_ticks_ >= down_timeout_ticks_)
            return MechStatus::FAILED;

        compute_down_effort(feedback, axis_effort);
        return MechStatus::BUSY;
    }

    void compute_normal_pose_effort(
        const Feedback& feedback, double target_height, double target_pitch, double target_roll,
        std::array<double, kAxisCount>& axis_effort) {
        const auto sync_effort = compute_sync_effort(feedback);
        const double u_height = height_pid_.update(target_height - feedback.height)
                              - height_pid_.kd * feedback.height_velocity;
        const double u_pitch =
            pitch_pid_.update(target_pitch - feedback.pitch) - pitch_pid_.kd * feedback.pitch_rate;
        const double u_roll =
            roll_pid_.update(target_roll - feedback.roll) - roll_pid_.kd * feedback.roll_rate;

        for (size_t i = 0; i < kAxisCount; ++i)
            axis_effort[i] = u_height - u_pitch * axis_x(i) + u_roll * axis_y(i) + sync_effort[i];
    }

    void
        compute_down_effort(const Feedback& feedback, std::array<double, kAxisCount>& axis_effort) {
        const auto sync_effort = compute_sync_effort(feedback);
        const double u_pitch =
            pitch_pid_.update(target_pitch_ - feedback.pitch) - pitch_pid_.kd * feedback.pitch_rate;
        const double u_roll =
            roll_pid_.update(target_roll_ - feedback.roll) - roll_pid_.kd * feedback.roll_rate;

        for (size_t i = 0; i < kAxisCount; ++i)
            axis_effort[i] =
                -down_effort_ - u_pitch * axis_x(i) + u_roll * axis_y(i) + sync_effort[i];
    }

    void compute_height_degraded_effort(
        const Feedback& feedback, double target_pitch, double target_roll,
        std::array<double, kAxisCount>& axis_effort) {
        const auto sync_effort = compute_sync_effort(feedback);
        const double u_pitch =
            pitch_pid_.update(target_pitch - feedback.pitch) - pitch_pid_.kd * feedback.pitch_rate;
        const double u_roll =
            roll_pid_.update(target_roll - feedback.roll) - roll_pid_.kd * feedback.roll_rate;

        std::array<double, kAxisCount> raw{};
        double height_bias = 0.0;
        bool has_bottom_limited_axis = false;
        for (size_t i = 0; i < kAxisCount; ++i) {
            raw[i] = -u_pitch * axis_x(i) + u_roll * axis_y(i) + sync_effort[i];
            if (feedback.bottom[i]) {
                has_bottom_limited_axis = true;
                height_bias = std::max(height_bias, -raw[i] + limit_hold_margin_);
            }
        }

        if (!has_bottom_limited_axis && !height_target_reachable_) {
            double min_height_for_target_attitude = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < kAxisCount; ++i) {
                const double base = -target_pitch * axis_x(i) + target_roll * axis_y(i);
                min_height_for_target_attitude =
                    std::max(min_height_for_target_attitude, height_degrade_margin_ - base);
            }

            const double effective_height_target =
                std::max(feedback.height, min_height_for_target_attitude);
            const double height_bias_error = effective_height_target - feedback.height;
            height_bias = std::max(0.0, height_bias_pid_.update(height_bias_error));
        }

        height_bias = std::max(0.0, height_bias);
        for (size_t i = 0; i < kAxisCount; ++i)
            axis_effort[i] = height_bias + raw[i];
    }

    std::array<double, kAxisCount> compute_sync_effort(const Feedback& feedback) {
        std::array<double, kAxisCount> sync{};

        for (size_t i = 0; i < kAxisCount; ++i) {
            const double plane_z =
                feedback.height - feedback.pitch * axis_x(i) + feedback.roll * axis_y(i);
            const double sync_error = plane_z - feedback.z[i];
            double u_sync = sync_pid_[i].update(sync_error)
                          - sync_pid_[i].kd * (feedback.velocity[i] - feedback.height_velocity);

            if (feedback.bottom[i] && u_sync < 0.0)
                u_sync = 0.0;

            if (upper_stroke_limited(feedback, i) && u_sync > 0.0)
                u_sync = 0.0;

            sync[i] = std::clamp(u_sync, -sync_output_max_, sync_output_max_);
        }

        return sync;
    }

    bool compute_height_target_reachable(const Feedback& feedback) const {
        for (size_t i = 0; i < kAxisCount; ++i) {
            const double target_z =
                target_height_ - target_pitch_ * axis_x(i) + target_roll_ * axis_y(i);
            if (feedback.bottom[i] && target_z < feedback.z[i] + height_degrade_margin_)
                return false;
            if (feedback.height_calibrated && target_z < height_degrade_margin_)
                return false;
        }
        return true;
    }

    bool normal_level_should_degrade(
        const Feedback& feedback, const std::array<double, kAxisCount>& axis_effort) const {
        for (size_t i = 0; i < kAxisCount; ++i) {
            const double limited_axis_effort =
                std::clamp(axis_effort[i], -axis_effort_max_[i], axis_effort_max_[i]);
            if (feedback.bottom[i] && limited_axis_effort < 0.0)
                return true;
        }
        return false;
    }

    bool level_settled(const Feedback& feedback) const {
        const double height_error = target_height_ - feedback.height;
        const double pitch_error = target_pitch_ - feedback.pitch;
        const double roll_error = target_roll_ - feedback.roll;

        const bool pose_settled = std::abs(pitch_error) < level_pitch_tolerance_
                               && std::abs(roll_error) < level_roll_tolerance_
                               && std::abs(feedback.pitch_rate) < level_pitch_rate_tolerance_
                               && std::abs(feedback.roll_rate) < level_roll_rate_tolerance_;
        if (height_degraded_)
            return pose_settled;

        return pose_settled && std::abs(height_error) < level_height_tolerance_
            && std::abs(feedback.height_velocity) < level_height_velocity_tolerance_;
    }

    void apply_gated_outputs(
        const Feedback& feedback, const std::array<double, kAxisCount>& axis_effort) {
        for (size_t i = 0; i < kAxisCount; ++i) {
            const double limited_axis_effort =
                std::clamp(axis_effort[i], -axis_effort_max_[i], axis_effort_max_[i]);

            *axis_effort_output_[i] = limited_axis_effort;
            if (feedback.bottom[i] && limited_axis_effort < 0.0) {
                *control_torque_[i] = kNaN;
            } else if (upper_stroke_limited(feedback, i) && limited_axis_effort > 0.0) {
                *control_torque_[i] = kNaN;
            } else {
                *control_torque_[i] = axis_torque_direction_[i] * limited_axis_effort;
            }
        }
    }

    bool upper_stroke_limited(const Feedback& feedback, size_t index) const {
        return feedback.height_calibrated && std::isfinite(feedback.stroke_angle[index])
            && feedback.stroke_angle[index] >= max_stroke_;
    }

    void release_outputs() {
        for (size_t i = 0; i < kAxisCount; ++i) {
            *control_torque_[i] = kNaN;
            *axis_effort_output_[i] = kNaN;
        }
    }

    void publish_status(MechStatus status) {
        *status_output_ = status;
        *height_degraded_output_ = height_degraded_;
        *height_target_reachable_output_ = height_target_reachable_;
    }

    void reset_all_pid() {
        height_pid_.reset();
        height_bias_pid_.reset();
        pitch_pid_.reset();
        roll_pid_.reset();
        for (auto& pid : sync_pid_)
            pid.reset();
    }

    double axis_x(size_t index) const { return index < 2 ? axis_x_ : -axis_x_; }

    double axis_y(size_t index) const { return index == 0 || index == 2 ? axis_y_ : -axis_y_; }

    InputInterface<FourZCommand> command_;
    InputInterface<double> pitch_;
    InputInterface<double> roll_;
    InputInterface<double> height_;
    InputInterface<double> height_velocity_;
    InputInterface<double> pitch_rate_;
    InputInterface<double> roll_rate_;
    InputInterface<bool> height_calibrated_;
    std::array<InputInterface<double>, kAxisCount> axis_height_;
    std::array<InputInterface<double>, kAxisCount> axis_height_velocity_;
    std::array<InputInterface<double>, kAxisCount> axis_stroke_angle_;
    std::array<InputInterface<bool>, kAxisCount> bottom_limit_switch_;

    std::array<OutputInterface<double>, kAxisCount> control_torque_;
    std::array<OutputInterface<double>, kAxisCount> axis_effort_output_;
    OutputInterface<MechStatus> status_output_;
    OutputInterface<bool> height_degraded_output_;
    OutputInterface<bool> height_target_reachable_output_;

    double axis_x_ = 0.20;
    double axis_y_ = 0.15;
    double calibrate_down_effort_ = 0.2;
    double down_effort_ = 0.2;
    double max_stroke_ = 20.0;
    double sync_output_max_ = 0.1;
    double height_degrade_margin_ = 0.001;
    double limit_hold_margin_ = 0.02;
    double level_height_tolerance_ = 0.005;
    double level_height_velocity_tolerance_ = 0.01;
    double level_pitch_tolerance_ = 0.01;
    double level_roll_tolerance_ = 0.01;
    double level_pitch_rate_tolerance_ = 0.02;
    double level_roll_rate_tolerance_ = 0.02;
    int64_t level_settle_ticks_ = 250;
    int64_t level_timeout_ticks_ = 5000;
    int64_t calibrate_timeout_ticks_ = 5000;
    int64_t down_timeout_ticks_ = 5000;
    std::array<double, kAxisCount> axis_effort_max_{1.0, 1.0, 1.0, 1.0};
    std::array<double, kAxisCount> axis_torque_direction_{1.0, 1.0, 1.0, 1.0};

    ModalPid height_pid_;
    ModalPid height_bias_pid_;
    ModalPid pitch_pid_;
    ModalPid roll_pid_;
    ModalPid sync_pid_template_;
    std::array<ModalPid, kAxisCount> sync_pid_;

    FourZCommand active_command_ = FourZCommand::IDLE;
    MechStatus current_status_ = MechStatus::IDLE;
    int64_t active_ticks_ = 0;
    int64_t settle_count_ = 0;
    double target_height_ = 0.0;
    double target_pitch_ = 0.0;
    double target_roll_ = 0.0;
    bool height_degraded_ = false;
    bool height_target_reachable_ = true;
    std::array<bool, kAxisCount> calibrate_touched_{false, false, false, false};
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::FourZAxisChassisController, rmcs_executor::Component)
