#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/chassis_command.hpp>
#include <rmcs_dart_guidance/msg/mechanism_status.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::dart {

class FourZChassisController
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using ChassisCmd = rmcs_dart_guidance::msg::ChassisCommand;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    FourZChassisController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        load_parameters();

        register_input("/dart/chassis/command", command_, false);
        register_output("/dart/chassis/status", status_, MechStatus::IDLE);

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            register_motor_inputs(kMotorPrefixes[i], angles_[i], velocities_[i], torques_[i]);
            register_bottom_limit_input(kMotorPrefixes[i], bottom_limits_[i]);
            register_control_outputs(
                kMotorPrefixes[i], control_velocities_[i], control_torque_limits_[i]);
        }

        register_input("/dart/chassis/imu/pitch", pitch_, false);
        register_input("/dart/chassis/imu/roll", roll_, false);
        register_input("/dart/chassis/imu/pitch_rate", pitch_rate_, false);
        register_input("/dart/chassis/imu/roll_rate", roll_rate_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
    }

    void before_updating() override {
        if (!command_.ready()) {
            command_.make_and_bind_directly(ChassisCmd::IDLE);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/chassis/command\". Set to IDLE.");
        }
        if (!pitch_rate_.ready()) {
            pitch_rate_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/chassis/imu/pitch_rate\". Set to 0.");
        }
        if (!roll_rate_.ready()) {
            roll_rate_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/chassis/imu/roll_rate\". Set to 0.");
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
        // RCLCPP_INFO(
        //     get_logger(), "fl:%d,fr:%d,bl:%d,br:%d", *bottom_limits_[0], *bottom_limits_[1],
        //     *bottom_limits_[2], *bottom_limits_[3]);
        if (manual_mode()) {
            update_manual();
            return;
        }

        const auto cmd = command_.ready() ? *command_ : ChassisCmd::IDLE;

        if (cmd == ChassisCmd::IDLE) {
            active_cmd_ = ChassisCmd::IDLE;
            reset_command_state();
            publish_outputs(nan_velocities());
            status_held_ = MechStatus::IDLE;
            *status_ = status_held_;
            return;
        }

        if (cmd == ChassisCmd::ABORT) {
            active_cmd_ = ChassisCmd::IDLE;
            reset_command_state();
            publish_outputs(nan_velocities());
            status_held_ = MechStatus::ABORTED;
            *status_ = status_held_;
            return;
        }

        if (cmd != active_cmd_ || status_held_ == MechStatus::IDLE) {
            start_command(cmd);
        } else if (is_terminal(status_held_)) {
            publish_outputs(nan_velocities());
            *status_ = status_held_;
            return;
        }

        AxisSnapshot snapshot;
        std::array<double, kAxisCount> target_velocities = nan_velocities();
        MechStatus next_status = MechStatus::BUSY;

        switch (active_cmd_) {
        case ChassisCmd::ZERO_CALIBRATE:
            next_status = update_zero_calibrate(snapshot, target_velocities);
            break;
        case ChassisCmd::LEVEL: next_status = update_level(snapshot, target_velocities); break;
        case ChassisCmd::IDLE:
        case ChassisCmd::ABORT: next_status = status_held_; break;
        }

        if (next_status == MechStatus::FAILED || next_status == MechStatus::SUCCEEDED) {
            target_velocities = nan_velocities();
        }

        publish_outputs(target_velocities, snapshot);
        status_held_ = next_status;
        *status_ = status_held_;
    }

private:
    static constexpr std::size_t kAxisCount = 4;
    static constexpr std::size_t kFrontLeft = 0;
    static constexpr std::size_t kNoAxis = kAxisCount;
    static constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    static constexpr int kLogThrottleMs = 1000;

    enum class Stage {
        NONE,
        ZERO_FIND_BOTTOM,
        ZERO_ROLLBACK,
        ZERO_FINAL_HOLD,
        LEVEL_ADJUST,
        LEVEL_DESCEND,
        LEVEL_REFERENCE_DESCEND,
        LEVEL_THREE_AXIS,
    };

    struct AxisSnapshot {
        std::array<double, kAxisCount> angle{};
        std::array<bool, kAxisCount> bottom_limit{};
        double pitch{kNan};
        double roll{kNan};
        double pitch_rate{0.0};
        double roll_rate{0.0};
        bool has_axis_feedback{false};
        bool has_attitude{false};
    };

    static constexpr std::array<const char*, kAxisCount> kAxisNames{
        "front_left",
        "front_right",
        "back_left",
        "back_right",
    };

    static constexpr std::array<const char*, kAxisCount> kMotorPrefixes{
        "/dart/chassis/front_left_motor",
        "/dart/chassis/front_right_motor",
        "/dart/chassis/back_left_motor",
        "/dart/chassis/back_right_motor",
    };

    static constexpr std::array<double, kAxisCount> kPitchSign{1.0, 1.0, -1.0, -1.0};
    static constexpr std::array<double, kAxisCount> kRollSign{1.0, -1.0, 1.0, -1.0};

    static bool is_terminal(MechStatus status) {
        return status == MechStatus::SUCCEEDED || status == MechStatus::FAILED
            || status == MechStatus::ABORTED;
    }

    static std::array<double, kAxisCount> nan_velocities() {
        std::array<double, kAxisCount> velocities{};
        velocities.fill(kNan);
        return velocities;
    }

    void register_motor_inputs(
        const std::string& prefix, InputInterface<double>& angle, InputInterface<double>& velocity,
        InputInterface<double>& torque) {
        register_input(prefix + "/angle", angle, false);
        register_input(prefix + "/velocity", velocity, false);
        register_input(prefix + "/torque", torque, false);
    }

    void register_bottom_limit_input(const std::string& prefix, InputInterface<bool>& limit) {
        register_input(prefix + "/bottom_limit_switch", limit, false);
    }

    void register_control_outputs(
        const std::string& prefix, OutputInterface<double>& velocity,
        OutputInterface<double>& torque_limit) {
        register_output(prefix + "/control_velocity", velocity, kNan);
        register_output(prefix + "/control_torque_limit", torque_limit, kNan);
    }

    void load_parameters() {
        zero_down_speed_ = std::abs(get_parameter_or("zero_down_speed", 2.0));
        zero_rollback_speed_ = std::abs(get_parameter_or("zero_rollback_speed", 2.0));
        zero_rollback_angle_ = std::abs(get_parameter_or("zero_rollback_angle", 12.566));
        level_descend_speed_ = std::abs(get_parameter_or("level_descend_speed", 0.5));
        upper_soft_limit_angle_ = std::abs(get_parameter_or("upper_soft_limit_angle", 250.0));
        chassis_velocity_limit_ = std::abs(get_parameter_or("chassis_velocity_limit", 3.0));
        chassis_torque_limit_ = std::abs(get_parameter_or("chassis_torque_limit", 1.0));
        manual_chassis_velocity_sensitivity_ =
            get_parameter_or("manual_chassis_velocity_sensitivity", 1.0);

        int64_t zero_hold_ticks = 500;
        get_parameter_or("zero_hold_ticks", zero_hold_ticks, int64_t{500});
        zero_hold_ticks_ = std::max<int64_t>(zero_hold_ticks, 0);

        int64_t level_settle_ticks = 200;
        get_parameter_or("level_settle_ticks", level_settle_ticks, int64_t{200});
        level_settle_ticks_ = std::max<int64_t>(level_settle_ticks, 1);

        level_pitch_tolerance_ = std::abs(get_parameter_or("level_pitch_tolerance", 0.01));
        level_roll_tolerance_ = std::abs(get_parameter_or("level_roll_tolerance", 0.01));
        level_pitch_offset_ = get_parameter_or("pitch_offset", 0.0);
        level_roll_offset_ = get_parameter_or("roll_offset", 0.0);
        level_correction_velocity_deadband_ =
            std::abs(get_parameter_or("level_correction_velocity_deadband", 0.02));
        level_pitch_rate_kd_ = get_parameter_or("level_pitch_rate_kd", 0.0);
        level_roll_rate_kd_ = get_parameter_or("level_roll_rate_kd", 0.0);

        load_pid_parameters("level_pitch_pid_", level_pitch_pid_, 0.5, 0.0, 0.0);
        load_pid_parameters("level_roll_pid_", level_roll_pid_, 0.5, 0.0, 0.0);
    }

    void load_pid_parameters(
        const std::string& prefix, pid::PidCalculator& calculator, double kp_default,
        double ki_default, double kd_default) {
        calculator.kp = get_parameter_or(prefix + "kp", kp_default);
        calculator.ki = get_parameter_or(prefix + "ki", ki_default);
        calculator.kd = get_parameter_or(prefix + "kd", kd_default);
        calculator.integral_min =
            get_parameter_or(prefix + "integral_min", calculator.integral_min);
        calculator.integral_max =
            get_parameter_or(prefix + "integral_max", calculator.integral_max);
        calculator.output_min = get_parameter_or(prefix + "output_min", calculator.output_min);
        calculator.output_max = get_parameter_or(prefix + "output_max", calculator.output_max);
        calculator.reset();
    }

    void start_command(ChassisCmd cmd) {
        reset_command_state();
        active_cmd_ = cmd;
        status_held_ = MechStatus::BUSY;
        switch (cmd) {
        case ChassisCmd::ZERO_CALIBRATE: stage_ = Stage::ZERO_FIND_BOTTOM; break;
        case ChassisCmd::LEVEL:
            stage_ = Stage::LEVEL_ADJUST;
            RCLCPP_INFO(get_logger(), "[FourZChassisController] LEVEL phase 1 start: adjust");
            break;
        case ChassisCmd::IDLE:
        case ChassisCmd::ABORT: stage_ = Stage::NONE; break;
        }
    }

    void reset_command_state() {
        stage_ = Stage::NONE;
        pending_zero_angle_.fill(kNan);
        zero_latched_.fill(false);
        axis_hold_ticks_.fill(0);
        rollback_done_.fill(false);
        final_hold_ticks_ = 0;
        reference_axis_ = kNoAxis;
        settle_ticks_ = 0;
        level_pitch_pid_.reset();
        level_roll_pid_.reset();
    }

    bool manual_mode() const {
        return switch_left_.ready() && *switch_left_ == rmcs_msgs::Switch::UP;
    }

    bool manual_chassis_mode() const {
        return manual_mode() && switch_right_.ready() && *switch_right_ == rmcs_msgs::Switch::DOWN;
    }

    void update_manual() {
        active_cmd_ = ChassisCmd::IDLE;
        reset_command_state();

        double target_velocity = 0.0;
        if (manual_chassis_mode() && joystick_left_.ready()) {
            target_velocity = manual_chassis_velocity_sensitivity_ * joystick_left_->x();
        }

        std::array<double, kAxisCount> target_velocities{};
        target_velocities.fill(target_velocity);
        publish_outputs(target_velocities);

        status_held_ = MechStatus::BUSY;
        *status_ = status_held_;
    }

    bool read_axis_feedback(AxisSnapshot& snapshot) const {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!angles_[i].ready() || !std::isfinite(*angles_[i]) || !bottom_limits_[i].ready()) {
                snapshot.has_axis_feedback = false;
                return false;
            }
            snapshot.angle[i] = *angles_[i];
            snapshot.bottom_limit[i] = *bottom_limits_[i];
        }
        snapshot.has_axis_feedback = true;
        return true;
    }

    bool read_attitude(AxisSnapshot& snapshot) const {
        if (!pitch_.ready() || !std::isfinite(*pitch_) || !roll_.ready()
            || !std::isfinite(*roll_)) {
            snapshot.has_attitude = false;
            return false;
        }

        snapshot.pitch = *pitch_;
        snapshot.roll = *roll_;
        snapshot.pitch_rate =
            pitch_rate_.ready() && std::isfinite(*pitch_rate_) ? *pitch_rate_ : 0.0;
        snapshot.roll_rate = roll_rate_.ready() && std::isfinite(*roll_rate_) ? *roll_rate_ : 0.0;
        snapshot.has_attitude = true;
        return true;
    }

    MechStatus fail(const char* message) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "%s", message);
        return MechStatus::FAILED;
    }

    MechStatus update_zero_calibrate(
        AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        if (!read_axis_feedback(snapshot)) {
            return fail("[FourZChassisController] ZERO_CALIBRATE missing axis feedback");
        }

        switch (stage_) {
        case Stage::ZERO_FIND_BOTTOM:
            update_zero_find_bottom(snapshot, target_velocities);
            if (all_zero_axes_ready_to_commit()) {
                zero_angle_ = pending_zero_angle_;
                zero_valid_ = true;
                rollback_done_.fill(false);
                stage_ = Stage::ZERO_ROLLBACK;
            }
            return MechStatus::BUSY;

        case Stage::ZERO_ROLLBACK:
            update_zero_rollback(snapshot, target_velocities);
            if (all_true(rollback_done_)) {
                final_hold_ticks_ = 0;
                stage_ = Stage::ZERO_FINAL_HOLD;
            }
            return MechStatus::BUSY;

        case Stage::ZERO_FINAL_HOLD:
            if (final_hold_ticks_ < zero_hold_ticks_) {
                target_velocities.fill(0.0);
                ++final_hold_ticks_;
                return MechStatus::BUSY;
            }
            return MechStatus::SUCCEEDED;

        default: return fail("[FourZChassisController] ZERO_CALIBRATE invalid stage");
        }
    }

    void update_zero_find_bottom(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!zero_latched_[i] && snapshot.bottom_limit[i]) {
                zero_latched_[i] = true;
                pending_zero_angle_[i] = snapshot.angle[i];
                axis_hold_ticks_[i] = 0;
            }

            if (!zero_latched_[i]) {
                target_velocities[i] = -zero_down_speed_;
            } else if (axis_hold_ticks_[i] < zero_hold_ticks_) {
                target_velocities[i] = 0.0;
                ++axis_hold_ticks_[i];
            } else {
                target_velocities[i] = kNan;
            }
        }
    }

    bool all_zero_axes_ready_to_commit() const {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!zero_latched_[i] || axis_hold_ticks_[i] < zero_hold_ticks_
                || !std::isfinite(pending_zero_angle_[i])) {
                return false;
            }
        }
        return true;
    }

    void update_zero_rollback(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (rollback_done_[i]) {
                target_velocities[i] = 0.0;
                continue;
            }

            const double height = snapshot.angle[i] - zero_angle_[i];
            if (height >= zero_rollback_angle_) {
                rollback_done_[i] = true;
                target_velocities[i] = 0.0;
            } else {
                target_velocities[i] = zero_rollback_speed_;
            }
        }
    }

    MechStatus
        update_level(AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        if (!zero_valid_) {
            return fail("[FourZChassisController] LEVEL ignored because zero is invalid");
        }
        if (!read_axis_feedback(snapshot)) {
            return fail("[FourZChassisController] LEVEL missing axis feedback");
        }
        if (!read_attitude(snapshot)) {
            return fail("[FourZChassisController] LEVEL missing attitude feedback");
        }

        switch (stage_) {
        case Stage::LEVEL_ADJUST: return update_level_adjust(snapshot, target_velocities);
        case Stage::LEVEL_DESCEND: return update_level_descend(snapshot, target_velocities);
        case Stage::LEVEL_REFERENCE_DESCEND:
            return update_level_reference_descend(snapshot, target_velocities);
        case Stage::LEVEL_THREE_AXIS: return update_level_three_axis(snapshot, target_velocities);
        default: return fail("[FourZChassisController] LEVEL invalid stage");
        }
    }

    MechStatus update_level_adjust(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        std::array<bool, kAxisCount> allowed{};
        allowed.fill(true);

        const auto correction = level_correction(snapshot, allowed);
        if (!correction.valid) {
            return fail("[FourZChassisController] LEVEL four-axis correction failed");
        }

        target_velocities = correction.velocity;
        if (level_stable(snapshot, correction.velocity, allowed)) {
            ++settle_ticks_;
        } else {
            settle_ticks_ = 0;
        }

        if (settle_ticks_ >= level_settle_ticks_) {
            reference_axis_ = select_reference_axis(snapshot);
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 1 end: pitch=%.6f roll=%.6f "
                "reference_axis=%s",
                snapshot.pitch, snapshot.roll, kAxisNames[reference_axis_]);
            RCLCPP_INFO(
                get_logger(), "[FourZChassisController] LEVEL phase 2 start: descend");
            settle_ticks_ = 0;
            stage_ = Stage::LEVEL_DESCEND;
        }

        return MechStatus::BUSY;
    }

    MechStatus update_level_descend(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        if (reference_axis_ == kNoAxis) {
            return fail("[FourZChassisController] LEVEL reference axis missing");
        }

        if (snapshot.bottom_limit[reference_axis_]) {
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 2 end: reference_axis=%s reached bottom",
                kAxisNames[reference_axis_]);
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 3 start: three-axis adjust");
            stage_ = Stage::LEVEL_THREE_AXIS;
            target_velocities.fill(kNan);
            target_velocities[reference_axis_] = 0.0;
            return MechStatus::BUSY;
        }

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (i != reference_axis_ && snapshot.bottom_limit[i]) {
                RCLCPP_INFO(
                    get_logger(),
                    "[FourZChassisController] LEVEL phase 2 reference descend: %s reached "
                    "bottom before reference_axis=%s",
                    kAxisNames[i], kAxisNames[reference_axis_]);
                stage_ = Stage::LEVEL_REFERENCE_DESCEND;
                target_velocities.fill(kNan);
                target_velocities[reference_axis_] = -level_descend_speed_;
                return MechStatus::BUSY;
            }
        }

        std::array<bool, kAxisCount> allowed{};
        allowed.fill(true);
        const auto correction = level_correction(snapshot, allowed);
        if (!correction.valid) {
            return fail("[FourZChassisController] LEVEL descend correction failed");
        }

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            target_velocities[i] = correction.velocity[i] - level_descend_speed_;
        }
        return MechStatus::BUSY;
    }

    MechStatus update_level_reference_descend(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        if (reference_axis_ == kNoAxis) {
            return fail("[FourZChassisController] LEVEL reference axis missing");
        }
        if (snapshot.bottom_limit[reference_axis_]) {
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 2 end: reference_axis=%s reached bottom",
                kAxisNames[reference_axis_]);
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 3 start: three-axis adjust");
            stage_ = Stage::LEVEL_THREE_AXIS;
            target_velocities.fill(kNan);
            target_velocities[reference_axis_] = 0.0;
            return MechStatus::BUSY;
        }

        target_velocities.fill(kNan);
        target_velocities[reference_axis_] = -level_descend_speed_;
        return MechStatus::BUSY;
    }

    MechStatus update_level_three_axis(
        const AxisSnapshot& snapshot, std::array<double, kAxisCount>& target_velocities) {
        if (reference_axis_ == kNoAxis) {
            return fail("[FourZChassisController] LEVEL reference axis missing");
        }

        std::array<bool, kAxisCount> allowed{};
        allowed.fill(true);
        allowed[reference_axis_] = false;

        const auto correction = level_correction(snapshot, allowed);
        if (!correction.valid) {
            return fail("[FourZChassisController] LEVEL three-axis correction failed");
        }

        target_velocities = correction.velocity;
        target_velocities[reference_axis_] = 0.0;

        if (level_stable(snapshot, correction.velocity, allowed)) {
            ++settle_ticks_;
        } else {
            settle_ticks_ = 0;
        }

        if (settle_ticks_ >= level_settle_ticks_) {
            RCLCPP_INFO(
                get_logger(),
                "[FourZChassisController] LEVEL phase 3 end: pitch=%.6f roll=%.6f "
                "settle_ticks=%ld",
                snapshot.pitch, snapshot.roll, settle_ticks_);
            return MechStatus::SUCCEEDED;
        }

        return MechStatus::BUSY;
    }

    struct CorrectionResult {
        CorrectionResult() { velocity.fill(kNan); }

        std::array<double, kAxisCount> velocity{};
        bool valid{false};
    };

    CorrectionResult level_correction(
        const AxisSnapshot& snapshot, const std::array<bool, kAxisCount>& allowed) {
        const double pitch_error = level_pitch_offset_ - snapshot.pitch;
        const double roll_error = level_roll_offset_ - snapshot.roll;
        const double pitch_cmd =
            level_pitch_pid_.update(pitch_error) - level_pitch_rate_kd_ * snapshot.pitch_rate;
        const double roll_cmd =
            level_roll_pid_.update(roll_error) - level_roll_rate_kd_ * snapshot.roll_rate;
        if (!std::isfinite(pitch_cmd) || !std::isfinite(roll_cmd)) {
            return {};
        }

        double m00 = 0.0;
        double m01 = 0.0;
        double m11 = 0.0;
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (!allowed[i]) {
                continue;
            }
            m00 += kPitchSign[i] * kPitchSign[i];
            m01 += kPitchSign[i] * kRollSign[i];
            m11 += kRollSign[i] * kRollSign[i];
        }

        const double det = m00 * m11 - m01 * m01;
        if (std::abs(det) < 1e-9) {
            return {};
        }

        const double y0 = (m11 * pitch_cmd - m01 * roll_cmd) / det;
        const double y1 = (-m01 * pitch_cmd + m00 * roll_cmd) / det;

        CorrectionResult result;
        result.velocity.fill(kNan);
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (allowed[i]) {
                result.velocity[i] = kPitchSign[i] * y0 + kRollSign[i] * y1;
            }
        }
        result.valid = true;
        return result;
    }

    bool level_stable(
        const AxisSnapshot& snapshot, const std::array<double, kAxisCount>& correction,
        const std::array<bool, kAxisCount>& allowed) const {
        if (std::abs(snapshot.pitch - level_pitch_offset_) > level_pitch_tolerance_
            || std::abs(snapshot.roll - level_roll_offset_) > level_roll_tolerance_) {
            return false;
        }

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            if (allowed[i] && std::isfinite(correction[i])
                && std::abs(correction[i]) > level_correction_velocity_deadband_) {
                return false;
            }
        }
        return true;
    }

    std::size_t select_reference_axis(const AxisSnapshot& snapshot) const {
        std::size_t reference = kFrontLeft;
        double min_height = snapshot.angle[kFrontLeft] - zero_angle_[kFrontLeft];
        for (std::size_t i = 1; i < kAxisCount; ++i) {
            const double height = snapshot.angle[i] - zero_angle_[i];
            if (height < min_height) {
                min_height = height;
                reference = i;
            }
        }
        RCLCPP_INFO(
            get_logger(), "[FourZChassisController] LEVEL reference_axis=%s",
            kAxisNames[reference]);
        return reference;
    }

    static bool all_true(const std::array<bool, kAxisCount>& values) {
        return std::all_of(values.begin(), values.end(), [](bool value) { return value; });
    }

    void publish_outputs(const std::array<double, kAxisCount>& target_velocities) {
        AxisSnapshot snapshot;
        if (read_axis_feedback(snapshot)) {
            publish_outputs(target_velocities, snapshot);
            return;
        }

        for (std::size_t i = 0; i < kAxisCount; ++i) {
            *control_velocities_[i] = target_velocities[i];
            *control_torque_limits_[i] =
                std::isfinite(target_velocities[i]) ? chassis_torque_limit_ : kNan;
        }
    }

    void publish_outputs(
        const std::array<double, kAxisCount>& target_velocities, const AxisSnapshot& snapshot) {
        for (std::size_t i = 0; i < kAxisCount; ++i) {
            double velocity = target_velocities[i];
            if (std::isfinite(velocity)) {
                velocity = std::clamp(velocity, -chassis_velocity_limit_, chassis_velocity_limit_);

                if (snapshot.has_axis_feedback && snapshot.bottom_limit[i] && velocity < 0.0) {
                    velocity = kNan;
                }

                if (zero_valid_ && snapshot.has_axis_feedback) {
                    const double height = snapshot.angle[i] - zero_angle_[i];
                    if (height >= upper_soft_limit_angle_ && velocity > 0.0) {
                        velocity = kNan;
                    }
                }
            }

            *control_velocities_[i] = velocity;
            *control_torque_limits_[i] = std::isfinite(velocity) ? chassis_torque_limit_ : kNan;
        }
    }

    ChassisCmd active_cmd_{ChassisCmd::IDLE};
    MechStatus status_held_{MechStatus::IDLE};
    Stage stage_{Stage::NONE};

    std::array<double, kAxisCount> zero_angle_{};
    std::array<double, kAxisCount> pending_zero_angle_{nan_velocities()};
    std::array<bool, kAxisCount> zero_latched_{};
    std::array<int64_t, kAxisCount> axis_hold_ticks_{};
    std::array<bool, kAxisCount> rollback_done_{};
    bool zero_valid_{false};
    int64_t final_hold_ticks_{0};
    std::size_t reference_axis_{kNoAxis};
    int64_t settle_ticks_{0};

    double zero_down_speed_{2.0};
    double zero_rollback_speed_{2.0};
    double zero_rollback_angle_{12.566};
    int64_t zero_hold_ticks_{500};
    double level_descend_speed_{0.5};
    double upper_soft_limit_angle_{250.0};
    int64_t level_settle_ticks_{200};
    double level_pitch_tolerance_{0.01};
    double level_roll_tolerance_{0.01};
    double level_pitch_offset_{0.0};
    double level_roll_offset_{0.0};
    double level_correction_velocity_deadband_{0.02};
    double level_pitch_rate_kd_{0.0};
    double level_roll_rate_kd_{0.0};
    double chassis_velocity_limit_{3.0};
    double chassis_torque_limit_{1.0};
    double manual_chassis_velocity_sensitivity_{1.0};
    pid::PidCalculator level_pitch_pid_;
    pid::PidCalculator level_roll_pid_;

    InputInterface<ChassisCmd> command_;
    OutputInterface<MechStatus> status_;

    std::array<InputInterface<double>, kAxisCount> angles_;
    std::array<InputInterface<double>, kAxisCount> velocities_;
    std::array<InputInterface<double>, kAxisCount> torques_;
    std::array<InputInterface<bool>, kAxisCount> bottom_limits_;

    InputInterface<double> pitch_;
    InputInterface<double> roll_;
    InputInterface<double> pitch_rate_;
    InputInterface<double> roll_rate_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;

    std::array<OutputInterface<double>, kAxisCount> control_velocities_;
    std::array<OutputInterface<double>, kAxisCount> control_torque_limits_;
};

} // namespace rmcs_core::controller::dart

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::dart::FourZChassisController, rmcs_executor::Component)
