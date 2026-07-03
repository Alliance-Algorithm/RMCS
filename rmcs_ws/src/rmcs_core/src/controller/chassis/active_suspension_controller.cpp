#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>

#include <fmt/format.h>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class ActiveSuspensionController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ActiveSuspensionController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        load_pid_("pitch_outer_", pitch_outer_pid_, 8.0, 0.35, 0.28);
        load_pid_("pitch_inner_", pitch_inner_pid_, 2.0, 0.0, 0.0);
        load_pid_("roll_outer_", roll_outer_pid_, 8.0, 0.35, 0.28);
        load_pid_("roll_inner_", roll_inner_pid_, 2.0, 0.0, 0.0);

        min_angle_deg_ = get_parameter_or("min_angle", 8.0);
        max_angle_deg_ = get_parameter_or("max_angle", 58.0);
        active_suspension_enable_ = get_parameter_or("active_suspension_enable", true);
        pitch_angle_diff_limit_deg_ = get_parameter_or("pitch_angle_diff_limit_deg", 45.0);
        roll_angle_diff_limit_deg_ = get_parameter_or("roll_angle_diff_limit_deg", 45.0);

        register_input("/chassis/ctrl_hold_active", ctrl_hold_active_);
        register_input("/chassis/imu/pitch", chassis_imu_pitch_);
        register_input("/chassis/imu/roll", chassis_imu_roll_);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_);

        for (size_t i = 0; i < kJointCount; ++i) {
            register_input(
                fmt::format("/chassis/{}_joint/base_target_physical_angle", kJointNames[i]),
                base_angles_[i]);
            register_input(
                fmt::format("/chassis/{}_joint/base_target_physical_velocity", kJointNames[i]),
                base_velocities_[i], false);
            register_input(
                fmt::format("/chassis/{}_joint/base_target_physical_acceleration", kJointNames[i]),
                base_accelerations_[i], false);
            register_input(
                fmt::format("/chassis/{}_joint/physical_angle", kJointNames[i]), physical_angles_[i]);

            register_output(
                fmt::format("/chassis/{}_joint/target_physical_angle", kJointNames[i]),
                target_angles_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/target_physical_velocity", kJointNames[i]),
                target_velocities_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/target_physical_acceleration", kJointNames[i]),
                target_accelerations_[i], nan_);
            register_output(
                fmt::format("/chassis/{}_joint/control_angle_error", kJointNames[i]),
                angle_errors_[i], nan_);
        }
    }

    void update() override {
        if (!base_targets_ready_()) {
            reset_outputs_nan_();
            return;
        }

        if (!active_suspension_enable_ || !ctrl_hold_active_ready_() || !feedback_ready_()) {
            passthrough_outputs_();
            return;
        }

        const double pitch = std::clamp(*chassis_imu_pitch_, -kMaxAttitude, kMaxAttitude);
        const double roll = std::clamp(*chassis_imu_roll_, -kMaxAttitude, kMaxAttitude);
        const double pitch_rate = *chassis_imu_pitch_rate_;
        const double roll_rate = *chassis_imu_roll_rate_;

        const double pitch_outer = pitch_outer_pid_.update(-pitch);
        const double roll_outer = roll_outer_pid_.update(-roll);

        const double pitch_inner = pitch_inner_pid_.update(pitch_outer - pitch_rate);
        const double roll_inner = roll_inner_pid_.update(roll_outer - roll_rate);

        if (!std::isfinite(pitch_inner) || !std::isfinite(roll_inner)) {
            reset_outputs_nan_();
            return;
        }

        const double base_angle = deg_to_rad_(min_angle_deg_ - 5.0);
        const double max_angle = deg_to_rad_(max_angle_deg_);
        const double pitch_limit = deg_to_rad_(pitch_angle_diff_limit_deg_);
        const double roll_limit = deg_to_rad_(roll_angle_diff_limit_deg_);

        const double pitch_delta = std::clamp(pitch_inner, -pitch_limit, pitch_limit);
        const double roll_delta = std::clamp(roll_inner, -roll_limit, roll_limit);

        static constexpr std::array<double, kJointCount> kPitchSign = {-1, 1, 1, -1};
        static constexpr std::array<double, kJointCount> kRollSign = {1, 1, -1, -1};

        for (size_t i = 0; i < kJointCount; ++i)
            *target_angles_[i] =
                std::clamp(
                    base_angle + std::max(kPitchSign[i] * pitch_delta, 0.0)
                        + std::max(kRollSign[i] * roll_delta, 0.0),
                    base_angle, max_angle);

        for (size_t i = 0; i < kJointCount; ++i) {
            *target_velocities_[i] = optional_input_or_(base_velocities_[i], 0.0);
            *target_accelerations_[i] = optional_input_or_(base_accelerations_[i], 0.0);
        }
        publish_angle_errors_();
    }

private:
    static constexpr size_t kJointCount = 4;
    static constexpr std::array<const char*, kJointCount> kJointNames = {
        "left_front", "left_back", "right_back", "right_front"};
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double kMaxAttitude = 30.0 * std::numbers::pi / 180.0;

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    static double optional_input_or_(const InputInterface<double>& input, double fallback) {
        return input.ready() && std::isfinite(*input) ? *input : fallback;
    }

    static double load_double_or_(rclcpp::Node& node, const std::string& name, double fallback) {
        double value = fallback;
        node.get_parameter_or(name, value, fallback);
        return value;
    }

    void load_pid_(
        const std::string& prefix, pid::PidCalculator& pid, double kp_default, double ki_default,
        double kd_default) {
        pid.kp = load_double_or_(*this, prefix + "kp", kp_default);
        pid.ki = load_double_or_(*this, prefix + "ki", ki_default);
        pid.kd = load_double_or_(*this, prefix + "kd", kd_default);
        pid.integral_min = load_double_or_(*this, prefix + "integral_min", -1e9);
        pid.integral_max = load_double_or_(*this, prefix + "integral_max", 1e9);
        pid.integral_split_min = load_double_or_(*this, prefix + "integral_split_min", -1e9);
        pid.integral_split_max = load_double_or_(*this, prefix + "integral_split_max", 1e9);
        pid.output_min = load_double_or_(*this, prefix + "output_min", -1e9);
        pid.output_max = load_double_or_(*this, prefix + "output_max", 1e9);
    }

    bool ctrl_hold_active_ready_() const {
        return ctrl_hold_active_.ready() && *ctrl_hold_active_;
    }

    bool base_targets_ready_() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (!base_angles_[i].ready())
                return false;
        return true;
    }

    bool feedback_ready_() const {
        if (!chassis_imu_pitch_.ready() || !chassis_imu_roll_.ready()
            || !chassis_imu_pitch_rate_.ready() || !chassis_imu_roll_rate_.ready())
            return false;
        for (size_t i = 0; i < kJointCount; ++i)
            if (!physical_angles_[i].ready())
                return false;
        return true;
    }

    void passthrough_outputs_() {
        for (size_t i = 0; i < kJointCount; ++i) {
            *target_angles_[i] = *base_angles_[i];
            *target_velocities_[i] = optional_input_or_(base_velocities_[i], 0.0);
            *target_accelerations_[i] = optional_input_or_(base_accelerations_[i], 0.0);
        }
        publish_angle_errors_();
    }

    void reset_outputs_nan_() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();

        for (size_t i = 0; i < kJointCount; ++i) {
            *target_angles_[i] = nan_;
            *target_velocities_[i] = nan_;
            *target_accelerations_[i] = nan_;
            *angle_errors_[i] = nan_;
        }
    }

    void publish_angle_errors_() {
        for (size_t i = 0; i < kJointCount; ++i)
            *angle_errors_[i] = physical_angles_[i].ready() && std::isfinite(*physical_angles_[i])
                                    ? *physical_angles_[i] - *target_angles_[i]
                                    : nan_;
    }

    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};

    InputInterface<bool> ctrl_hold_active_;
    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;

    std::array<InputInterface<double>, kJointCount> base_angles_;
    std::array<InputInterface<double>, kJointCount> base_velocities_;
    std::array<InputInterface<double>, kJointCount> base_accelerations_;
    std::array<InputInterface<double>, kJointCount> physical_angles_;

    std::array<OutputInterface<double>, kJointCount> target_angles_;
    std::array<OutputInterface<double>, kJointCount> target_velocities_;
    std::array<OutputInterface<double>, kJointCount> target_accelerations_;
    std::array<OutputInterface<double>, kJointCount> angle_errors_;

    double min_angle_deg_ = 8.0;
    double max_angle_deg_ = 58.0;
    bool active_suspension_enable_ = true;
    double pitch_angle_diff_limit_deg_ = 45.0;
    double roll_angle_diff_limit_deg_ = 45.0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::ActiveSuspensionController, rmcs_executor::Component)
