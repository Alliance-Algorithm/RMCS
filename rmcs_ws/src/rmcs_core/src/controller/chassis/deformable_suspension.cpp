#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>
#include <stdexcept>

#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::chassis {

class DeformableSuspension
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableSuspension()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        load_config_();

        register_input("/predefined/update_rate", update_rate_, false);

        register_input("/chassis/active_suspension/active", active_suspension_active_);
        register_input("/chassis/deformable/reset_count", reset_count_, false);
        register_input(
            "/chassis/deformable/passive_suspension_active", passive_suspension_active_);
        register_input("/chassis/deformable/low_prone_active", low_prone_active_);
        register_input(
            "/chassis/deformable/symmetric_posture_target", symmetric_posture_target_);
        register_input("/chassis/deformable/correction_inverted", correction_inverted_);
        register_input("/chassis/deformable/min_angle_deg", min_angle_deg_);
        register_input("/chassis/deformable/max_angle_deg", max_angle_deg_);
        register_input(
            "/chassis/deformable/suspension_reference_angle_deg",
            suspension_reference_angle_deg_);

        register_input("/chassis/imu/pitch", chassis_imu_pitch_, false);
        register_input("/chassis/imu/roll", chassis_imu_roll_, false);
        register_input("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, false);
        register_input("/chassis/imu/roll_rate", chassis_imu_roll_rate_, false);

        for (size_t i = 0; i < kJointCount; ++i) {
            register_input(
                std::string{"/chassis/deformable/"} + kJointName[i] + "_joint/posture_target_angle",
                joint_posture_target_angle_rad_[i]);
            register_input(
                std::string{"/chassis/"} + kJointName[i] + "_joint/physical_angle",
                joint_physical_angle_[i], false);
            register_input(
                std::string{"/chassis/"} + kJointName[i] + "_joint/torque", joint_torque_[i],
                false);
            register_output(
                std::string{"/chassis/"} + kJointName[i] + "_joint/target_physical_angle",
                joint_target_angle_[i], nan_);
            register_output(
                std::string{"/chassis/"} + kJointName[i]
                    + "_joint/target_physical_velocity",
                joint_target_velocity_[i], nan_);
            register_output(
                std::string{"/chassis/"} + kJointName[i]
                    + "_joint/target_physical_acceleration",
                joint_target_acceleration_[i], nan_);
            register_output(
                std::string{"/chassis/"} + kJointName[i] + "_joint/control_angle_error",
                joint_angle_error_[i], nan_);
        }
    }

    void before_updating() override {
        if (!update_rate_.ready())
            update_rate_.make_and_bind_directly(1000.0);
        if (!reset_count_.ready())
            reset_count_.make_and_bind_directly(static_cast<size_t>(0));
        if (!chassis_imu_pitch_.ready())
            chassis_imu_pitch_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_.ready())
            chassis_imu_roll_.make_and_bind_directly(0.0);
        if (!chassis_imu_pitch_rate_.ready())
            chassis_imu_pitch_rate_.make_and_bind_directly(0.0);
        if (!chassis_imu_roll_rate_.ready())
            chassis_imu_roll_rate_.make_and_bind_directly(0.0);

        validate_joint_feedback_inputs_();
        reset_all_controls_();
        last_reset_count_ = *reset_count_;
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            reset_all_controls_();
            last_reset_count_ = *reset_count_;
            return;
        }

        const auto current_physical_angles = read_feedback_();
        const auto current_joint_torques = read_joint_torques_();

        if (!init_joint_targets_from_feedback_(current_physical_angles)) {
            publish_nan_joint_targets_();
            return;
        }

        const auto posture_target_angles_rad = read_posture_target_angles_rad_();
        const auto dt = update_dt_();

        if (*active_suspension_active_)
            calibrate_(*chassis_imu_pitch_, *chassis_imu_roll_, *symmetric_posture_target_, dt);

        std::array<double, kJointCount> joint_angle_states{};
        copy_joint_angle_states_(joint_angle_states);
        update_suspension_state_(
            *chassis_imu_pitch_ - pitch_offset_value_, *chassis_imu_roll_ - roll_offset_value_,
            *chassis_imu_pitch_rate_, *chassis_imu_roll_rate_, *active_suspension_active_,
            *passive_suspension_active_, *low_prone_active_, *min_angle_deg_, *max_angle_deg_,
            *suspension_reference_angle_deg_, *correction_inverted_, joint_angle_states,
            current_joint_torques, dt);

        const auto target_angles_rad = compute_joint_trajectory_targets_(
            posture_target_angles_rad, *active_suspension_active_, *low_prone_active_,
            *min_angle_deg_, *suspension_reference_angle_deg_);

        run_joint_trajectory_(target_angles_rad, *active_suspension_active_, dt);
        publish_joint_targets_(current_physical_angles);
    }

private:
    static constexpr size_t kJointCount = 4;
    static constexpr double nan_ = std::numeric_limits<double>::quiet_NaN();
    static constexpr double offset_limit_rad_ = 1.0 * std::numbers::pi / 180.0;
    static constexpr size_t kLeftFront = 0;
    static constexpr size_t kLeftBack = 1;
    static constexpr size_t kRightBack = 2;
    static constexpr size_t kRightFront = 3;
    static constexpr const char* kJointName[] = {
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    static double deg_to_rad_(double deg) { return deg * std::numbers::pi / 180.0; }

    void validate_joint_feedback_inputs_() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (!joint_physical_angle_[i].ready())
                throw std::runtime_error(
                    "missing deformable chassis feedback interfaces: expected "
                    "/chassis/*_joint/physical_angle");
    }

    double update_dt_() const {
        if (update_rate_.ready() && std::isfinite(*update_rate_) && *update_rate_ > 1e-6)
            return 1.0 / *update_rate_;
        return 1e-3;
    }

    void load_pid_(
        const std::string& prefix, pid::PidCalculator& pid, double kp_default,
        double ki_default, double kd_default, double integral_min_default,
        double integral_max_default, double output_min_default, double output_max_default) {
        pid.kp = get_parameter_or(prefix + "kp", kp_default);
        pid.ki = get_parameter_or(prefix + "ki", ki_default);
        pid.kd = get_parameter_or(prefix + "kd", kd_default);
        pid.integral_min = get_parameter_or(prefix + "integral_min", integral_min_default);
        pid.integral_max = get_parameter_or(prefix + "integral_max", integral_max_default);
        pid.output_min = get_parameter_or(prefix + "output_min", output_min_default);
        pid.output_max = get_parameter_or(prefix + "output_max", output_max_default);
    }

    void load_config_() {
        joint_target_vel_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or("target_physical_velocity_limit", 180.0))),
            1e-6);
        joint_target_acc_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or("target_physical_acceleration_limit", 720.0))),
            1e-6);
        suspension_target_vel_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or(
                "active_suspension_target_velocity_limit_deg",
                get_parameter_or("target_physical_velocity_limit", 180.0)))),
            1e-6);
        suspension_target_acc_limit_ = std::max(
            deg_to_rad_(std::abs(get_parameter_or(
                "active_suspension_target_acceleration_limit_deg",
                get_parameter_or("target_physical_acceleration_limit", 720.0)))),
            1e-6);

        load_pid_(
            "active_suspension_pitch_outer_", pitch_outer_pid_, 8.0, 0.35, 0.28, -2.0, 2.0,
            -3.0, 3.0);
        load_pid_(
            "active_suspension_pitch_inner_", pitch_inner_pid_, 2.0, 0.0, 0.0, -1.0, 1.0,
            -0.785, 0.785);
        load_pid_(
            "active_suspension_roll_outer_", roll_outer_pid_, 8.0, 0.35, 0.28, -2.0, 2.0,
            -3.0, 3.0);
        load_pid_(
            "active_suspension_roll_inner_", roll_inner_pid_, 2.0, 0.0, 0.0, -1.0, 1.0,
            -0.785, 0.785);

        active_correction_vel_limit_ = std::max(
            deg_to_rad_(std::abs(
                get_parameter_or("active_suspension_correction_velocity_limit_deg", 720.0))),
            1e-6);
        active_correction_acc_limit_ = std::max(
            deg_to_rad_(std::abs(
                get_parameter_or("active_suspension_correction_acceleration_limit_deg", 3600.0))),
            1e-6);

        passive_enabled_ = get_parameter_or("passive_suspension_enable", true);
        passive_kp_ = get_parameter_or("passive_suspension_kp", 0.035);
        passive_ki_ = get_parameter_or("passive_suspension_ki", 0.005);
        passive_deadband_ = std::max(get_parameter_or("passive_suspension_deadband", 0.08), 0.0);
        passive_integral_limit_ =
            std::max(get_parameter_or("passive_suspension_integral_limit", 3.0), 0.0);
        passive_load_lpf_alpha_ =
            std::clamp(get_parameter_or("passive_suspension_load_lpf_alpha", 0.08), 0.0, 1.0);
        passive_max_correction_rad_ = std::max(
            deg_to_rad_(
                std::abs(get_parameter_or("passive_suspension_max_angle_correction_deg", 8.0))),
            1e-6);
        passive_correction_vel_limit_ = std::max(
            deg_to_rad_(std::abs(
                get_parameter_or("passive_suspension_correction_velocity_limit_deg", 180.0))),
            1e-6);
        passive_correction_acc_limit_ = std::max(
            deg_to_rad_(std::abs(
                get_parameter_or("passive_suspension_correction_acceleration_limit_deg", 720.0))),
            1e-6);

        passive_load_sign_[kLeftFront] =
            get_parameter_or("passive_suspension_load_sign_left_front", -1.0);
        passive_load_sign_[kLeftBack] =
            get_parameter_or("passive_suspension_load_sign_left_back", -1.0);
        passive_load_sign_[kRightBack] =
            get_parameter_or("passive_suspension_load_sign_right_back", -1.0);
        passive_load_sign_[kRightFront] =
            get_parameter_or("passive_suspension_load_sign_right_front", -1.0);

        passive_load_bias_[kLeftFront] =
            get_parameter_or("passive_suspension_load_bias_left_front", 0.0);
        passive_load_bias_[kLeftBack] =
            get_parameter_or("passive_suspension_load_bias_left_back", 0.0);
        passive_load_bias_[kRightBack] =
            get_parameter_or("passive_suspension_load_bias_right_back", 0.0);
        passive_load_bias_[kRightFront] =
            get_parameter_or("passive_suspension_load_bias_right_front", 0.0);

        calibration_wait_time_ = std::max(get_parameter_or("chassis_imu_calibration_wait_s", 2.0), 0.0);
        calibration_sample_time_ =
            std::max(get_parameter_or("chassis_imu_calibration_sample_s", 3.0), 1e-6);
    }

    std::array<double, kJointCount> read_feedback_() const {
        std::array<double, kJointCount> angles;
        angles.fill(nan_);

        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_physical_angle_[i].ready() && std::isfinite(*joint_physical_angle_[i]))
                angles[i] = *joint_physical_angle_[i];

        return angles;
    }

    std::array<double, kJointCount> read_joint_torques_() const {
        std::array<double, kJointCount> torques;
        torques.fill(nan_);

        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_torque_[i].ready() && std::isfinite(*joint_torque_[i]))
                torques[i] = *joint_torque_[i];

        return torques;
    }

    std::array<double, kJointCount> read_posture_target_angles_rad_() const {
        std::array<double, kJointCount> targets{};
        for (size_t i = 0; i < kJointCount; ++i)
            targets[i] = *joint_posture_target_angle_rad_[i];
        return targets;
    }

    std::array<double, kJointCount> compute_joint_trajectory_targets_(
        const std::array<double, kJointCount>& posture_target_angles_rad, bool suspension_active,
        bool low_prone_active, double min_angle_deg, double suspension_reference_angle_deg) const {
        if (!suspension_active)
            return posture_target_angles_rad;

        std::array<double, kJointCount> target_angles_rad{};
        double target_angle_rad = low_prone_active ? deg_to_rad_(min_angle_deg - 5.0)
                                                   : deg_to_rad_(suspension_reference_angle_deg);
        target_angles_rad.fill(target_angle_rad);
        return target_angles_rad;
    }

    void reset_attitude_() {
        pitch_outer_pid_.reset();
        pitch_inner_pid_.reset();
        roll_outer_pid_.reset();
        roll_inner_pid_.reset();
        correction_target_rad_.fill(0.0);
    }

    void reset_passive_state_() {
        passive_load_initialized_ = false;
        passive_filtered_load_proxy_.fill(0.0);
        passive_load_integral_.fill(0.0);
    }

    void reset_calibration_window_() {
        calibration_hold_elapsed_ = 0.0;
        sample_count_ = 0;
        pitch_sum_ = 0.0;
        roll_sum_ = 0.0;
        calibration_completed_for_window_ = false;
    }

    void reset_all_controls_() {
        reset_attitude_();
        correction_state_rad_.fill(0.0);
        correction_velocity_state_rad_.fill(0.0);
        correction_acceleration_state_rad_.fill(0.0);
        joint_target_active_.fill(false);
        joint_target_angle_state_rad_.fill(nan_);
        joint_target_velocity_state_rad_.fill(0.0);
        joint_target_acceleration_state_rad_.fill(0.0);
        reset_passive_state_();
        reset_calibration_window_();
        calibrated_once_ = false;
        pitch_offset_value_ = 0.0;
        roll_offset_value_ = 0.0;

        for (size_t i = 0; i < kJointCount; ++i) {
            *joint_target_angle_[i] = nan_;
            *joint_target_velocity_[i] = nan_;
            *joint_target_acceleration_[i] = nan_;
            *joint_angle_error_[i] = nan_;
        }
    }

    void calibrate_(double pitch, double roll, bool symmetric_target, double dt) {
        if (calibrated_once_)
            return;

        if (!symmetric_target) {
            reset_calibration_window_();
            return;
        }

        if (!std::isfinite(pitch) || !std::isfinite(roll))
            return;

        calibration_hold_elapsed_ += dt;
        if (calibration_hold_elapsed_ < calibration_wait_time_)
            return;

        const double calibration_end = calibration_wait_time_ + calibration_sample_time_;
        if (calibration_hold_elapsed_ < calibration_end) {
            pitch_sum_ += pitch;
            roll_sum_ += roll;
            ++sample_count_;
            return;
        }

        if (calibration_completed_for_window_)
            return;

        calibration_completed_for_window_ = true;
        if (sample_count_ == 0)
            return;

        pitch_offset_value_ = std::clamp(
            pitch_sum_ / static_cast<double>(sample_count_), -offset_limit_rad_, offset_limit_rad_);
        roll_offset_value_ = std::clamp(
            roll_sum_ / static_cast<double>(sample_count_), -offset_limit_rad_, offset_limit_rad_);
        calibrated_once_ = true;
    }

    bool init_joint_targets_from_feedback_(const std::array<double, kJointCount>& physical_angles) {
        bool any_active_value = false;
        for (size_t i = 0; i < kJointCount; ++i) {
            if (std::isfinite(physical_angles[i]) && !joint_target_active_[i]) {
                joint_target_angle_state_rad_[i] = physical_angles[i];
                joint_target_velocity_state_rad_[i] = 0.0;
                joint_target_acceleration_state_rad_[i] = 0.0;
                joint_target_active_[i] = true;
            }
            any_active_value = any_active_value || joint_target_active_[i];
        }
        return any_active_value;
    }

    void compute_correction_targets_(double pitch_diff, double roll_diff, bool inverted) {
        if (inverted) {
            const double front_pitch_contribution = std::max(pitch_diff, 0.0);
            const double back_pitch_contribution = std::max(-pitch_diff, 0.0);
            const double left_roll_contribution = std::max(-roll_diff, 0.0);
            const double right_roll_contribution = std::max(roll_diff, 0.0);
            correction_target_rad_[kLeftFront] =
                -(front_pitch_contribution + left_roll_contribution);
            correction_target_rad_[kLeftBack] =
                -(back_pitch_contribution + left_roll_contribution);
            correction_target_rad_[kRightBack] =
                -(back_pitch_contribution + right_roll_contribution);
            correction_target_rad_[kRightFront] =
                -(front_pitch_contribution + right_roll_contribution);
        } else {
            const double front_pitch_contribution = std::max(-pitch_diff, 0.0);
            const double back_pitch_contribution = std::max(pitch_diff, 0.0);
            const double left_roll_contribution = std::max(roll_diff, 0.0);
            const double right_roll_contribution = std::max(-roll_diff, 0.0);
            correction_target_rad_[kLeftFront] = front_pitch_contribution + left_roll_contribution;
            correction_target_rad_[kLeftBack] = back_pitch_contribution + left_roll_contribution;
            correction_target_rad_[kRightBack] = back_pitch_contribution + right_roll_contribution;
            correction_target_rad_[kRightFront] = front_pitch_contribution + right_roll_contribution;
        }
    }

    void update_passive_targets_(const std::array<double, kJointCount>& joint_torques, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            correction_target_rad_.fill(0.0);
            return;
        }

        std::array<double, kJointCount> load_proxy{};
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!std::isfinite(joint_torques[i])) {
                correction_target_rad_.fill(0.0);
                reset_passive_state_();
                return;
            }

            load_proxy[i] = passive_load_sign_[i] * (joint_torques[i] - passive_load_bias_[i]);
            if (!passive_load_initialized_) {
                passive_filtered_load_proxy_[i] = load_proxy[i];
            } else {
                passive_filtered_load_proxy_[i] +=
                    passive_load_lpf_alpha_ * (load_proxy[i] - passive_filtered_load_proxy_[i]);
            }
        }
        passive_load_initialized_ = true;

        double load_average = 0.0;
        for (double load : passive_filtered_load_proxy_)
            load_average += load;
        load_average /= static_cast<double>(kJointCount);

        std::array<double, kJointCount> raw_targets{};
        for (size_t i = 0; i < kJointCount; ++i) {
            double load_error = load_average - passive_filtered_load_proxy_[i];
            if (std::abs(load_error) <= passive_deadband_)
                load_error = 0.0;

            passive_load_integral_[i] = std::clamp(
                passive_load_integral_[i] + load_error * dt, -passive_integral_limit_,
                passive_integral_limit_);

            raw_targets[i] = passive_kp_ * load_error + passive_ki_ * passive_load_integral_[i];
        }

        double raw_target_mean = 0.0;
        for (double raw_target : raw_targets)
            raw_target_mean += raw_target;
        raw_target_mean /= static_cast<double>(kJointCount);

        for (size_t i = 0; i < kJointCount; ++i) {
            correction_target_rad_[i] = std::clamp(
                raw_targets[i] - raw_target_mean, -passive_max_correction_rad_,
                passive_max_correction_rad_);
        }
    }

    void run_correction_trajectory_(
        bool low_prone_override_active, double min_angle_deg, double max_angle_deg,
        double base_angle_deg, const std::array<double, kJointCount>& base_joint_angles,
        double correction_vel_limit, double correction_acc_limit, double dt) {
        const double max_target_rad = deg_to_rad_(max_angle_deg);
        const double min_susp_rad = deg_to_rad_(min_angle_deg - 5.0);

        for (size_t i = 0; i < kJointCount; ++i) {
            const double base_angle = std::isfinite(base_joint_angles[i])
                                          ? base_joint_angles[i]
                                          : (low_prone_override_active ? min_susp_rad
                                                                       : deg_to_rad_(base_angle_deg));

            const double correction_min = min_susp_rad - base_angle;
            const double correction_max = max_target_rad - base_angle;
            const double target =
                std::clamp(correction_target_rad_[i], correction_min, correction_max);

            double& angle_state = correction_state_rad_[i];
            double& velocity_state = correction_velocity_state_rad_[i];
            double& acceleration_state = correction_acceleration_state_rad_[i];

            const double position_error = target - angle_state;
            const double stopping_distance =
                velocity_state * velocity_state / (2.0 * correction_acc_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(correction_vel_limit, position_error);

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state =
                std::clamp(velocity_error / dt, -correction_acc_limit, correction_acc_limit);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(velocity_state, -correction_vel_limit, correction_vel_limit);
            angle_state += velocity_state * dt;

            const double next_error = target - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    void update_suspension_state_(
        double pitch, double roll, double pitch_rate, double roll_rate, bool suspension_active,
        bool passive_suspension_active, bool low_prone_override_active, double min_angle_deg,
        double max_angle_deg, double base_angle_deg, bool correction_inverted,
        const std::array<double, kJointCount>& base_joint_angles,
        const std::array<double, kJointCount>& joint_torques, double dt) {
        if (passive_suspension_active && passive_enabled_) {
            reset_attitude_();
            update_passive_targets_(joint_torques, dt);
            run_correction_trajectory_(
                low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
                base_joint_angles, passive_correction_vel_limit_, passive_correction_acc_limit_,
                dt);
            return;
        }

        reset_passive_state_();

        if (!suspension_active) {
            reset_attitude_();
            run_correction_trajectory_(
                low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
                base_joint_angles, active_correction_vel_limit_, active_correction_acc_limit_, dt);
            return;
        }

        constexpr double max_attitude = 30.0 * std::numbers::pi / 180.0;
        const double clamped_pitch = std::clamp(pitch, -max_attitude, max_attitude);
        const double clamped_roll = std::clamp(roll, -max_attitude, max_attitude);

        const double pitch_outer = pitch_outer_pid_.update(-clamped_pitch);
        const double roll_outer = roll_outer_pid_.update(clamped_roll);
        const double pitch_diff = pitch_inner_pid_.update(pitch_outer - pitch_rate);
        const double roll_diff = roll_inner_pid_.update(roll_outer + roll_rate);

        if (!std::isfinite(pitch_diff) || !std::isfinite(roll_diff)) {
            reset_attitude_();
            return;
        }

        compute_correction_targets_(pitch_diff, roll_diff, correction_inverted);
        run_correction_trajectory_(
            low_prone_override_active, min_angle_deg, max_angle_deg, base_angle_deg,
            base_joint_angles, active_correction_vel_limit_, active_correction_acc_limit_, dt);
    }

    void run_joint_trajectory_(
        const std::array<double, kJointCount>& target_angles_rad, bool suspension_active,
        double dt) {
        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i])
                continue;

            double& angle_state = joint_target_angle_state_rad_[i];
            double& velocity_state = joint_target_velocity_state_rad_[i];
            double& acceleration_state = joint_target_acceleration_state_rad_[i];
            const double target = target_angles_rad[i];

            const double vel_limit =
                suspension_active ? suspension_target_vel_limit_ : joint_target_vel_limit_;
            const double acc_limit =
                suspension_active ? suspension_target_acc_limit_ : joint_target_acc_limit_;

            if (!std::isfinite(target) || !std::isfinite(angle_state))
                continue;

            const double position_error = target - angle_state;
            const double stopping_distance = velocity_state * velocity_state / (2.0 * acc_limit);

            double desired_velocity = 0.0;
            if (std::abs(position_error) > 1e-6 && std::abs(position_error) > stopping_distance)
                desired_velocity = std::copysign(vel_limit, position_error);

            const double velocity_error = desired_velocity - velocity_state;
            acceleration_state = std::clamp(velocity_error / dt, -acc_limit, acc_limit);

            velocity_state += acceleration_state * dt;
            velocity_state = std::clamp(velocity_state, -vel_limit, vel_limit);
            angle_state += velocity_state * dt;

            const double next_error = target - angle_state;
            if ((position_error > 0.0 && next_error < 0.0)
                || (position_error < 0.0 && next_error > 0.0)
                || (std::abs(next_error) < 1e-5 && std::abs(velocity_state) < 1e-3)) {
                angle_state = target;
                velocity_state = 0.0;
                acceleration_state = 0.0;
            }
        }
    }

    bool any_joint_target_active_() const {
        for (size_t i = 0; i < kJointCount; ++i)
            if (joint_target_active_[i])
                return true;
        return false;
    }

    void copy_joint_angle_states_(std::array<double, kJointCount>& out) const {
        out = joint_target_angle_state_rad_;
    }

    void publish_joint_targets_(const std::array<double, kJointCount>& feedback_angles) {
        const double min_angle_rad = deg_to_rad_(*min_angle_deg_ - 5.0);
        const double max_angle_rad = deg_to_rad_(*max_angle_deg_);

        if (!any_joint_target_active_()) {
            publish_nan_joint_targets_();
            return;
        }

        for (size_t i = 0; i < kJointCount; ++i) {
            if (!joint_target_active_[i]) {
                *joint_target_angle_[i] = nan_;
                *joint_target_velocity_[i] = nan_;
                *joint_target_acceleration_[i] = nan_;
                *joint_angle_error_[i] = nan_;
                continue;
            }

            const double target = joint_target_angle_state_rad_[i] + correction_state_rad_[i];
            *joint_target_angle_[i] = std::clamp(target, min_angle_rad, max_angle_rad);
            *joint_target_velocity_[i] =
                joint_target_velocity_state_rad_[i] + correction_velocity_state_rad_[i];
            *joint_target_acceleration_[i] =
                joint_target_acceleration_state_rad_[i] + correction_acceleration_state_rad_[i];
            *joint_angle_error_[i] = std::isfinite(feedback_angles[i])
                                       ? feedback_angles[i] - *joint_target_angle_[i]
                                       : nan_;
        }
    }

    void publish_nan_joint_targets_() {
        reset_all_controls_();
    }

    InputInterface<double> update_rate_;

    InputInterface<bool> active_suspension_active_;
    InputInterface<size_t> reset_count_;
    InputInterface<bool> passive_suspension_active_;
    InputInterface<bool> low_prone_active_;
    InputInterface<bool> symmetric_posture_target_;
    InputInterface<bool> correction_inverted_;
    InputInterface<double> min_angle_deg_;
    InputInterface<double> max_angle_deg_;
    InputInterface<double> suspension_reference_angle_deg_;

    InputInterface<double> chassis_imu_pitch_;
    InputInterface<double> chassis_imu_roll_;
    InputInterface<double> chassis_imu_pitch_rate_;
    InputInterface<double> chassis_imu_roll_rate_;

    std::array<InputInterface<double>, kJointCount> joint_posture_target_angle_rad_;
    std::array<InputInterface<double>, kJointCount> joint_physical_angle_;
    std::array<InputInterface<double>, kJointCount> joint_torque_;

    std::array<OutputInterface<double>, kJointCount> joint_target_angle_;
    std::array<OutputInterface<double>, kJointCount> joint_target_velocity_;
    std::array<OutputInterface<double>, kJointCount> joint_target_acceleration_;
    std::array<OutputInterface<double>, kJointCount> joint_angle_error_;

    pid::PidCalculator pitch_outer_pid_{};
    pid::PidCalculator pitch_inner_pid_{};
    pid::PidCalculator roll_outer_pid_{};
    pid::PidCalculator roll_inner_pid_{};

    double active_correction_vel_limit_ = 40.0;
    double active_correction_acc_limit_ = 200.0;

    bool passive_enabled_ = true;
    double passive_kp_ = 0.035;
    double passive_ki_ = 0.005;
    double passive_deadband_ = 0.08;
    double passive_integral_limit_ = 3.0;
    double passive_load_lpf_alpha_ = 0.08;
    double passive_max_correction_rad_ = deg_to_rad_(8.0);
    double passive_correction_vel_limit_ = deg_to_rad_(180.0);
    double passive_correction_acc_limit_ = deg_to_rad_(720.0);
    std::array<double, kJointCount> passive_load_sign_ = {-1.0, -1.0, -1.0, -1.0};
    std::array<double, kJointCount> passive_load_bias_ = {0.0, 0.0, 0.0, 0.0};

    double calibration_wait_time_ = 2.0;
    double calibration_sample_time_ = 3.0;
    double calibration_hold_elapsed_ = 0.0;
    size_t sample_count_ = 0;
    double pitch_sum_ = 0.0;
    double roll_sum_ = 0.0;
    bool calibration_completed_for_window_ = false;
    bool calibrated_once_ = false;
    double pitch_offset_value_ = 0.0;
    double roll_offset_value_ = 0.0;

    bool passive_load_initialized_ = false;
    std::array<double, kJointCount> passive_filtered_load_proxy_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> passive_load_integral_ = {0.0, 0.0, 0.0, 0.0};

    std::array<double, kJointCount> correction_target_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> correction_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};

    std::array<bool, kJointCount> joint_target_active_ = {false, false, false, false};
    std::array<double, kJointCount> joint_target_angle_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_velocity_state_rad_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, kJointCount> joint_target_acceleration_state_rad_ = {0.0, 0.0, 0.0, 0.0};

    double joint_target_vel_limit_ = 0.0;
    double joint_target_acc_limit_ = 0.0;
    double suspension_target_vel_limit_ = 0.0;
    double suspension_target_acc_limit_ = 0.0;
    size_t last_reset_count_ = 0;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::DeformableSuspension, rmcs_executor::Component)
