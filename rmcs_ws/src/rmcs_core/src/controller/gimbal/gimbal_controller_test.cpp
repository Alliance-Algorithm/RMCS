#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <vector>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/gimbal/velocity_reference.hpp"
#include "controller/gimbal/coordinated_yaw.hpp"
#include "controller/gimbal/big_yaw_identification_waveform.hpp"

namespace rmcs_core::controller::gimbal {

class GimbalControllerTest final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GimbalControllerTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        get_parameter_or("enabled", configured_enabled_, false);
        get_parameter_or("robust_control_enabled", robust_control_enabled_, false);
        get_parameter_or("coordinated_yaw_enabled", coordinated_yaw_enabled_, false);
        get_parameter_or("max_joystick_angular_velocity_rad_s", max_joystick_angular_velocity_rad_s_, 0.5);
        get_parameter_or("max_joystick_angular_acceleration_rad_s2", max_joystick_angular_acceleration_rad_s2_, 1.0);
        get_parameter_or("joystick_velocity_direction", joystick_velocity_direction_, 1.0);
        get_parameter_or("joystick_input", joystick_input_, std::string{"/remote/joystick/right"});
        get_parameter_or("big_yaw_torque_limit_nm", big_yaw_torque_limit_nm_, 3.6);
        get_parameter_or("identification_enabled", identification_enabled_, true);
        get_parameter_or("remote_timeout_ms", remote_timeout_ms_, 200.0);
        if (robust_control_enabled_ && identification_enabled_)
            throw std::invalid_argument("robust velocity control and automatic identification are mutually exclusive");
        double joystick_velocity_limit = 2.0;
        if (coordinated_yaw_enabled_) {
            if (!robust_control_enabled_ || identification_enabled_)
                throw std::invalid_argument("coordinated yaw requires robust control and excludes identification");
            CoordinatedYawParameters p;
            p.small_yaw_direction = get_parameter_or("coordinated_small_yaw_direction", p.small_yaw_direction);
            p.heading_gain = get_parameter_or("coordinated_heading_gain", p.heading_gain);
            p.heading_error_deadband_rad = get_parameter_or(
                "coordinated_heading_error_deadband_rad", p.heading_error_deadband_rad);
            p.follow_gain = get_parameter_or("coordinated_follow_gain", p.follow_gain);
            p.follow_deadband_rad = get_parameter_or("coordinated_follow_deadband_rad", p.follow_deadband_rad);
            p.follow_start_offset_rad = get_parameter_or(
                "coordinated_follow_start_offset_rad", p.follow_start_offset_rad);
            p.centered_follow_gain = get_parameter_or(
                "coordinated_centered_follow_gain", p.centered_follow_gain);
            p.counterrotation_speed_fraction = get_parameter_or(
                "coordinated_counterrotation_speed_fraction", p.counterrotation_speed_fraction);
            p.max_target_error_rad = get_parameter_or("coordinated_max_target_error_rad", p.max_target_error_rad);
            p.max_small_offset_rad = get_parameter_or("coordinated_max_small_offset_rad", p.max_small_offset_rad);
            p.small_speed_zero_offset_rad = get_parameter_or(
                "coordinated_small_speed_zero_offset_rad", p.small_speed_zero_offset_rad);
            p.small_braking_response_time_s = get_parameter_or(
                "coordinated_small_braking_response_time_s", p.small_braking_response_time_s);
            p.big_velocity_noise_threshold_rad_s = get_parameter_or(
                "coordinated_big_velocity_noise_threshold_rad_s", p.big_velocity_noise_threshold_rad_s);
            p.max_small_velocity_rad_s = get_parameter_or("coordinated_small_max_velocity_rad_s", p.max_small_velocity_rad_s);
            p.max_small_acceleration_rad_s2 = get_parameter_or("coordinated_small_max_acceleration_rad_s2", p.max_small_acceleration_rad_s2);
            p.max_big_velocity_rad_s = get_parameter_or("coordinated_big_max_velocity_rad_s", p.max_big_velocity_rad_s);
            p.max_big_acceleration_rad_s2 = get_parameter_or("coordinated_big_max_acceleration_rad_s2", p.max_big_acceleration_rad_s2);
            coordinated_yaw_.emplace(p);
            joystick_velocity_limit = std::max(joystick_velocity_limit, p.max_big_velocity_rad_s);
        }
        if (joystick_input_ != "/remote/joystick/left" && joystick_input_ != "/remote/joystick/right")
            throw std::invalid_argument("joystick_input must be /remote/joystick/left or /remote/joystick/right");
        if (!std::isfinite(big_yaw_torque_limit_nm_) || big_yaw_torque_limit_nm_ < 0.0
            || big_yaw_torque_limit_nm_ > 10.0) {
            throw std::invalid_argument("big_yaw_torque_limit_nm must be in [0, 10]");
        }
        if (!std::isfinite(remote_timeout_ms_) || remote_timeout_ms_ <= 0.0
            || remote_timeout_ms_ > 60'000.0) {
            throw std::invalid_argument("remote_timeout_ms must be in (0, 60000]");
        }
        if (!std::isfinite(max_joystick_angular_velocity_rad_s_)
            || max_joystick_angular_velocity_rad_s_ <= 0.0
            || max_joystick_angular_velocity_rad_s_ > joystick_velocity_limit
            || !std::isfinite(max_joystick_angular_acceleration_rad_s2_)
            || max_joystick_angular_acceleration_rad_s2_ <= 0.0
            || max_joystick_angular_acceleration_rad_s2_ > 40.0
            || (joystick_velocity_direction_ != 1.0 && joystick_velocity_direction_ != -1.0)) {
            throw std::invalid_argument(
                "joystick velocity exceeds the mode limit or acceleration exceeds 40 rad/s^2, direction must be +/-1");
        }
        if (identification_enabled_) {
            identification_require_joint2_support_ = get_parameter_or(
                "identification_require_joint2_support", true);
            BigYawIdentificationParameters parameters;
            parameters.torque_limit_nm = big_yaw_torque_limit_nm_;
            parameters.command_sign = get_parameter_or("identification_command_sign", 1.0);
            const auto amplitudes = get_parameter_or<std::vector<double>>(
                "identification_pulse_torques_nm", {2.0, 3.5, 5.0, 6.0});
            if (amplitudes.size() != parameters.pulse_torque_amplitudes_nm.size())
                throw std::invalid_argument("identification_pulse_torques_nm must contain four levels");
            std::copy(amplitudes.begin(), amplitudes.end(), parameters.pulse_torque_amplitudes_nm.begin());
            parameters.pulse_duration_s = get_parameter_or("identification_pulse_duration_s", 0.20);
            parameters.rest_duration_s = get_parameter_or("identification_rest_duration_s", 0.70);
            parameters.settle_duration_s = get_parameter_or("identification_settle_duration_s", 1.0);
            parameters.sine_torque_amplitude_nm = get_parameter_or("identification_sine_torque_amplitude_nm", 6.0);
            parameters.sine_frequency_hz = get_parameter_or("identification_sine_frequency_hz", 0.15);
            parameters.sine_frequency_end_hz = get_parameter_or("identification_sine_frequency_end_hz", 0.60);
            parameters.sine_duration_s = get_parameter_or("identification_sine_duration_s", 16.0);
            parameters.constant_torque_nm = get_parameter_or("identification_constant_torque_nm", 4.5);
            parameters.constant_duration_s = get_parameter_or("identification_constant_duration_s", 2.0);
            parameters.constant_rest_duration_s = get_parameter_or("identification_constant_rest_duration_s", 0.5);
            parameters.linear_ramp_peak_nm = get_parameter_or("identification_linear_ramp_peak_nm", 5.0);
            parameters.linear_ramp_duration_s = get_parameter_or("identification_linear_ramp_duration_s", 4.0);
            parameters.linear_ramp_group_count = get_parameter_or("identification_linear_ramp_group_count", 2);
            if (identification_require_joint2_support_) {
                identification_joint2_tolerance_rad_ = get_parameter_or(
                    "identification_joint2_position_tolerance_rad", 10.0 * std::numbers::pi / 180.0);
                identification_joint2_stable_duration_s_ = get_parameter_or(
                    "identification_joint2_stable_duration_s", 1.0);
                if (!std::isfinite(identification_joint2_tolerance_rad_)
                    || identification_joint2_tolerance_rad_ <= 0.0
                    || !std::isfinite(identification_joint2_stable_duration_s_)
                    || identification_joint2_stable_duration_s_ <= 0.0)
                    throw std::invalid_argument("identification joint2 stability parameters must be finite and positive");
            }
            identification_max_velocity_rad_s_ = get_parameter_or("identification_max_velocity_rad_s", 5.0);
            if (!std::isfinite(identification_max_velocity_rad_s_) || identification_max_velocity_rad_s_ <= 0.0)
                throw std::invalid_argument("identification max velocity must be finite and positive");
            identification_waveform_.emplace(parameters);
        }
        remote_timeout_ns_ = static_cast<int64_t>(remote_timeout_ms_ * 1'000'000.0);

        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input(joystick_input_, joystick_);
        register_input("/remote/last_received_monotonic_ns", remote_last_received_ns_);

        register_input("/arm/image_pitch/motor/raw_angle", motor_raw_angle_);
        register_input("/arm/image_pitch/motor/angle", motor_angle_);
        register_input("/arm/image_pitch/motor/velocity", motor_velocity_);
        register_input("/gimbal/feedback/motor_online", motor_online_);
        register_input("/gimbal/feedback/imu_online", imu_online_);
        register_input("/arm/joint_1/motor/raw_angle", small_yaw_raw_angle_);
        register_input("/arm/joint_1/motor/angle", small_yaw_angle_);
        register_input("/arm/joint_1/motor/velocity", small_yaw_velocity_);
        register_input("/gimbal/feedback/small_yaw_online", small_yaw_online_);
        register_input("/chassis/big_yaw/angle", big_yaw_angle_);
        register_input("/chassis/big_yaw/velocity", big_yaw_velocity_);
        register_input("/chassis/big_yaw/torque", big_yaw_torque_);
        register_input("/gimbal/feedback/big_yaw_online", big_yaw_online_);
        register_input("/gimbal/test/big_yaw_unwrapped_angle_valid", big_yaw_angle_valid_);
        register_input("/gimbal/test/big_yaw_angle_origin_valid", big_yaw_origin_valid_);
        register_input("/gimbal/test/big_yaw_position_tracking_fault", big_yaw_tracking_fault_);
        if (coordinated_yaw_enabled_)
            register_input("/gimbal/test/big_yaw_unwrapped_angle", big_yaw_unwrapped_angle_);
        if (identification_enabled_ && identification_require_joint2_support_) {
            register_input("/gimbal/test/joint2_pid_permitted", joint2_permitted_);
            register_input("/gimbal/test/joint2_pid_position_error", joint2_position_error_);
        }

        register_input("roll_imu_angle", imu_angle_[0]);
        register_input("pitch_imu_angle", imu_angle_[1]);
        register_input("yaw_imu_angle", imu_angle_[2]);
        register_input("roll_imu_velocity", imu_velocity_[0]);
        register_input("pitch_imu_velocity", imu_velocity_[1]);
        register_input("yaw_imu_velocity", imu_velocity_[2]);

        // Keep the actuator permission fail-closed until every arming interlock passes.
        register_output("/gimbal/test/enabled", test_enabled_, false);
        register_output("/gimbal/test/feedback_ready", feedback_ready_, false);
        register_output("/remote/online", remote_online_, false);
        register_output("/gimbal/test/double_down_protection", double_down_protection_, false);
        register_output("/gimbal/test/double_down_confirmed", double_down_confirmed_status_, false);
        register_output("/gimbal/test/big_yaw_identification_torque", big_yaw_control_torque_, 0.0);
        register_output("/gimbal/test/big_yaw_target_velocity", big_yaw_target_velocity_, 0.0);
        register_output("/gimbal/test/big_yaw_target_acceleration", big_yaw_target_acceleration_, 0.0);
        register_output("/gimbal/test/robust_control_active", robust_control_active_, false);
        register_output("/gimbal/test/coordinated_yaw_active", coordinated_active_, false);
        register_output("/gimbal/test/yaw_target_angle", coordinated_target_angle_, 0.0);
        register_output("/gimbal/test/yaw_actual_angle", coordinated_heading_angle_, 0.0);
        register_output("/gimbal/test/yaw_angle_error", coordinated_heading_error_, 0.0);
        register_output("/gimbal/test/small_yaw_center_angle", coordinated_small_center_, 0.0);
        register_output("/gimbal/test/small_yaw_center_offset", coordinated_small_offset_, 0.0);
        register_output("/gimbal/test/yaw_reference_limited", coordinated_reference_limited_, false);
        register_output("/gimbal/test/small_yaw_coordinated_target_velocity", coordinated_small_velocity_, 0.0);
        register_output("/gimbal/test/small_yaw_coordinated_target_acceleration", coordinated_small_acceleration_, 0.0);
        register_output("/gimbal/test/identification_elapsed_s", identification_elapsed_s_, 0.0);
        register_output("/gimbal/test/state", state_, std::string{"disabled"});
        register_output("/gimbal/test/big_yaw_identification_phase", identification_phase_, std::string{"disabled"});
        register_output("/gimbal/test/big_yaw_identification_pass", identification_pass_, 0);
        register_output("/gimbal/test/big_yaw_identification_complete", identification_complete_, false);
        register_output("/gimbal/test/big_yaw_identification_fault_reason", identification_fault_reason_, std::string{"none"});
        register_output("/gimbal/test/big_yaw_identification_joint2_stable_elapsed_s", identification_joint2_stable_elapsed_, 0.0);
    }

    void update() override {
        // Fail closed every cycle. Startup and remote reconnection require DOWN/DOWN first.
        *test_enabled_                = false;
        *big_yaw_control_torque_      = 0.0;
        *big_yaw_target_acceleration_ = 0.0;
        *robust_control_active_       = false;
        const auto left               = *switch_left_;
        const auto right              = *switch_right_;
        const auto now_ns             = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                            std::chrono::steady_clock::now().time_since_epoch())
                                            .count();
        const auto remote_received_ns = *remote_last_received_ns_;
        const bool remote_fresh       = remote_received_ns > 0 && remote_received_ns <= now_ns
                                     && now_ns - remote_received_ns <= remote_timeout_ns_;
        const bool remote_valid       = remote_fresh && valid_switch(left) && valid_switch(right);
        const bool both_down = left == rmcs_msgs::Switch::DOWN && right == rmcs_msgs::Switch::DOWN;
        const double joystick_x = (*joystick_)[0];
        if (!remote_valid) {
            double_down_confirmed_       = false;
            stick_centered_after_safety_ = false;
        } else if (both_down) {
            double_down_confirmed_       = true;
            stick_centered_after_safety_ = false;
        } else if (
            double_down_confirmed_ && std::isfinite(joystick_x)
            && std::abs(joystick_x) <= kJoystickDeadband) {
            stick_centered_after_safety_ = true;
        }
        if (remote_valid && both_down) {
            identification_complete_latched_ = false;
            identification_fault_latched_ = false;
            identification_started_ = false;
            identification_elapsed_ = 0.0;
            identification_stable_since_ns_ = 0;
            identification_fault_text_ = "none";
        }
        *remote_online_                = remote_valid;
        *double_down_protection_       = remote_valid && both_down;
        *double_down_confirmed_status_ = double_down_confirmed_;

        // Keep feedback available even while the double-down protection is active.
        feedback_.raw_angle           = *motor_raw_angle_;
        feedback_.angle               = *motor_angle_;
        feedback_.velocity            = *motor_velocity_;
        feedback_.small_yaw_raw_angle = *small_yaw_raw_angle_;
        feedback_.small_yaw_angle     = *small_yaw_angle_;
        feedback_.small_yaw_velocity  = *small_yaw_velocity_;
        for (std::size_t i = 0; i < 3; ++i) {
            feedback_.imu_angle[i]    = *imu_angle_[i];
            feedback_.imu_velocity[i] = *imu_velocity_[i];
        }
        const auto finite = [](double value) { return std::isfinite(value); };
        *feedback_ready_ = *big_yaw_online_ && *big_yaw_angle_valid_
                        && *big_yaw_origin_valid_ && !*big_yaw_tracking_fault_ && finite(*big_yaw_angle_)
                        && finite(*big_yaw_velocity_) && finite(*big_yaw_torque_);
        const double coordinated_dt = last_coordinated_update_ns_ == 0 ? 0.001
            : (now_ns - last_coordinated_update_ns_) * 1e-9;
        last_coordinated_update_ns_ = now_ns;
        if (coordinated_yaw_enabled_) {
            *feedback_ready_ = *feedback_ready_ && *small_yaw_online_
                && finite(*small_yaw_angle_) && finite(*big_yaw_unwrapped_angle_);
            const bool timing_valid = finite(coordinated_dt) && coordinated_dt > 0.0
                && coordinated_dt <= 0.02;
            if (!*feedback_ready_ || !timing_valid || !finite(joystick_x)) {
                double_down_confirmed_ = false;
                stick_centered_after_safety_ = false;
            }
            *double_down_confirmed_status_ = double_down_confirmed_;
        }

        const bool control_mode_configured = identification_enabled_ || robust_control_enabled_;
        *test_enabled_ = configured_enabled_ && control_mode_configured && !identification_complete_latched_
                      && double_down_confirmed_
                      && stick_centered_after_safety_ && remote_valid && !both_down
                      && *feedback_ready_ && finite(joystick_x);

        if (identification_enabled_)
            update_identification(now_ns, *test_enabled_);
        *identification_elapsed_s_ = identification_elapsed_;
        *identification_complete_ = identification_complete_latched_;
        *identification_fault_reason_ = identification_fault_text_;
        *robust_control_active_ = *test_enabled_ && robust_control_enabled_;
        if (coordinated_yaw_enabled_)
            update_coordinated_reference(joystick_x, coordinated_dt);
        else
            update_velocity_reference(now_ns, *robust_control_active_);

        const char* state = !configured_enabled_          ? "disabled"
                          : !control_mode_configured      ? "control_mode_disabled"
                          : !remote_valid                 ? "remote_invalid"
                          : both_down                     ? "double_down_protection"
                          : !double_down_confirmed_       ? "waiting_for_double_down"
                          : !stick_centered_after_safety_ ? "waiting_for_stick_center"
                          : !*feedback_ready_             ? "big_yaw_feedback_unavailable"
                          : identification_complete_latched_ ? "identification_complete"
                          : *robust_control_active_       ? (coordinated_yaw_enabled_
                              ? "coordinated_yaw_control" : "robust_velocity_control")
                          : !identification_enabled_      ? "identification_disabled"
                          : identification_fault_latched_ ? "identification_fault"
                          : identification_status_;
        *state_           = state;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Big-yaw test: state=%s phase=%s pass=%d permitted=%d ready=%d "
            "torque_request=%.3f Nm elapsed=%.3f s angle=%.5f rad velocity=%.5f rad/s "
            "feedback_torque=%.3f Nm fault=%s",
            state, identification_phase_->c_str(), *identification_pass_,
            static_cast<int>(*test_enabled_), static_cast<int>(*feedback_ready_),
            *big_yaw_control_torque_, identification_elapsed_, *big_yaw_angle_,
            *big_yaw_velocity_, *big_yaw_torque_, identification_fault_text_);

        if (!*test_enabled_)
            return;

        // The hardware component independently repeats switch, feedback and torque-limit checks.
    }

private:
    void update_coordinated_reference(double stick, double dt) {
        const double bounded = std::isfinite(stick) ? std::clamp(stick, -1.0, 1.0) : 0.0;
        const double rate = std::abs(bounded) > kJoystickDeadband
            ? std::copysign((std::abs(bounded) - kJoystickDeadband) / (1.0 - kJoystickDeadband), bounded)
                * max_joystick_angular_velocity_rad_s_ * joystick_velocity_direction_
            : 0.0;
        const auto out = coordinated_yaw_->update(
            rate, *big_yaw_unwrapped_angle_, *small_yaw_angle_, *big_yaw_velocity_, dt,
            *robust_control_active_);
        if (*robust_control_active_ && !out.active) {
            // Feedback discontinuity or invalid timing loses the angle target.
            // Reacquiring feedback must not restart motion toward an old target.
            double_down_confirmed_ = false;
            stick_centered_after_safety_ = false;
            *double_down_confirmed_status_ = false;
            *test_enabled_ = *robust_control_active_ = false;
        }
        *coordinated_active_ = out.active && *robust_control_active_;
        *coordinated_target_angle_ = out.target_angle;
        *coordinated_heading_angle_ = out.heading_angle;
        *coordinated_heading_error_ = out.heading_error;
        *coordinated_small_center_ = out.small_center_angle;
        *coordinated_small_offset_ = out.small_offset;
        *coordinated_reference_limited_ = out.reference_limited;
        *coordinated_small_velocity_ = out.small_velocity;
        *coordinated_small_acceleration_ = out.small_acceleration;
        *big_yaw_target_velocity_ = out.big_velocity;
        *big_yaw_target_acceleration_ = out.big_acceleration;
    }

    void update_velocity_reference(int64_t now_ns, bool active) {
        double desired_velocity = 0.0;
        if (active) {
            double joystick_x = (*joystick_)[0];
            if (std::isfinite(joystick_x)) {
                joystick_x = std::clamp(joystick_x, -1.0, 1.0);
                const double magnitude = std::abs(joystick_x);
                if (magnitude > kJoystickDeadband) {
                    const double normalized = (magnitude - kJoystickDeadband) / (1.0 - kJoystickDeadband);
                    desired_velocity = std::copysign(
                        normalized * max_joystick_angular_velocity_rad_s_, joystick_x)
                                     * joystick_velocity_direction_;
                }
            }
        }

        if (last_reference_update_ns_ == 0 || now_ns <= last_reference_update_ns_) {
            velocity_reference_.update(0.0, max_joystick_angular_acceleration_rad_s2_, 0.0, false);
            last_reference_update_ns_ = now_ns;
            *big_yaw_target_velocity_ = 0.0;
            *big_yaw_target_acceleration_ = 0.0;
            return;
        }
        const double dt = std::clamp((now_ns - last_reference_update_ns_) * 1e-9, 0.0, 0.02);
        last_reference_update_ns_ = now_ns;
        velocity_reference_.update(desired_velocity, max_joystick_angular_acceleration_rad_s2_, dt, active);
        *big_yaw_target_velocity_ = velocity_reference_.velocity();
        *big_yaw_target_acceleration_ = velocity_reference_.acceleration();
    }

    void update_identification(int64_t now_ns, bool base_ready) {
        const auto fault = [this](const char* reason) {
            if (identification_fault_latched_)
                return;
            identification_fault_latched_ = true;
            identification_fault_text_ = reason;
            identification_status_ = "identification_fault";
            *identification_phase_ = "fault";
            *test_enabled_ = false;
        };
        if (!base_ready) {
            if (identification_started_ && !identification_complete_latched_
                && !*double_down_protection_)
                fault(*remote_online_ ? "feedback_invalid" : "remote_invalid");
            identification_stable_since_ns_ = 0;
            *identification_joint2_stable_elapsed_ = 0.0;
            if (!identification_complete_latched_ && !identification_fault_latched_) {
                identification_elapsed_ = 0.0;
                *identification_pass_ = 0;
                *identification_phase_ = "waiting";
            }
            return;
        }
        if (identification_fault_latched_) {
            *test_enabled_ = false;
            return;
        }
        double stable_s = 0.0;
        if (identification_require_joint2_support_) {
            const bool joint2_stable = *joint2_permitted_ && std::isfinite(*joint2_position_error_)
                && std::abs(*joint2_position_error_) <= identification_joint2_tolerance_rad_;
            if (!joint2_stable) {
                identification_stable_since_ns_ = 0;
                *identification_joint2_stable_elapsed_ = 0.0;
                if (identification_started_)
                    fault(*joint2_permitted_ ? "joint2_unstable" : "joint2_not_permitted");
                else {
                    identification_status_ = "waiting_for_joint2_stability";
                    *identification_phase_ = identification_status_;
                    *test_enabled_ = false;
                }
                return;
            }
            if (identification_stable_since_ns_ == 0)
                identification_stable_since_ns_ = now_ns;
            stable_s = (now_ns - identification_stable_since_ns_) * 1e-9;
            *identification_joint2_stable_elapsed_ = std::min(stable_s, identification_joint2_stable_duration_s_);
        } else {
            identification_stable_since_ns_ = 0;
            *identification_joint2_stable_elapsed_ = 0.0;
        }
        if (std::abs(*big_yaw_velocity_) > identification_max_velocity_rad_s_) {
            if (identification_started_)
                fault("velocity_limit");
            else {
                identification_status_ = "waiting_for_big_yaw_stability";
                *identification_phase_ = identification_status_;
                *test_enabled_ = false;
            }
            return;
        }
        if (!identification_started_) {
            if (identification_require_joint2_support_
                && stable_s < identification_joint2_stable_duration_s_) {
                identification_status_ = "waiting_for_joint2_stability";
                *identification_phase_ = identification_status_;
                *test_enabled_ = false;
                return;
            }
            identification_started_ = true;
            identification_start_ns_ = now_ns;
        }
        identification_elapsed_ = (now_ns - identification_start_ns_) * 1e-9;
        const auto sample = identification_waveform_->sample(identification_elapsed_);
        *identification_phase_ = sample.stage;
        *identification_pass_ = sample.pass;
        identification_status_ = sample.complete ? "identification_complete" : "automatic_identification";
        identification_complete_latched_ = sample.complete;
        *test_enabled_ = !sample.complete;
        *big_yaw_control_torque_ = sample.torque_nm;
    }

    static bool valid_switch(rmcs_msgs::Switch value) {
        using rmcs_msgs::Switch;
        return value == Switch::UP || value == Switch::MIDDLE || value == Switch::DOWN;
    }

    static constexpr double kJoystickDeadband = 0.05;

    bool configured_enabled_          = false;
    bool robust_control_enabled_      = false;
    bool coordinated_yaw_enabled_     = false;
    bool identification_enabled_      = true;
    bool identification_require_joint2_support_ = true;
    bool identification_started_      = false;
    bool identification_complete_latched_ = false;
    bool double_down_confirmed_       = false;
    bool stick_centered_after_safety_ = false;
    double big_yaw_torque_limit_nm_{3.6};
    double max_joystick_angular_velocity_rad_s_{0.5};
    double max_joystick_angular_acceleration_rad_s2_{1.0};
    double joystick_velocity_direction_{1.0};
    std::string joystick_input_{"/remote/joystick/right"};
    VelocityReference velocity_reference_;
    std::optional<CoordinatedYaw> coordinated_yaw_;
    int64_t last_coordinated_update_ns_{0};
    int64_t last_reference_update_ns_{0};
    std::optional<BigYawIdentificationWaveform> identification_waveform_;
    bool identification_fault_latched_ = false;
    const char* identification_fault_text_ = "none";
    const char* identification_status_ = "waiting_for_joint2_stability";
    double identification_joint2_tolerance_rad_ = 10.0 * std::numbers::pi / 180.0;
    double identification_joint2_stable_duration_s_ = 1.0;
    double identification_max_velocity_rad_s_ = 5.0;
    int64_t identification_stable_since_ns_ = 0;
    double identification_elapsed_{0.0};
    int64_t identification_start_ns_{0};
    double remote_timeout_ms_{200.0};
    int64_t remote_timeout_ns_{200'000'000};

    struct Feedback {
        int raw_angle             = 0;
        double angle              = NAN;
        double velocity           = NAN;
        int small_yaw_raw_angle   = -1;
        double small_yaw_angle    = NAN;
        double small_yaw_velocity = NAN;
        std::array<double, 3> imu_angle{NAN, NAN, NAN};
        std::array<double, 3> imu_velocity{NAN, NAN, NAN};
    } feedback_;

    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<int64_t> remote_last_received_ns_;
    InputInterface<int> motor_raw_angle_;
    InputInterface<double> motor_angle_, motor_velocity_;
    InputInterface<bool> motor_online_, imu_online_;
    InputInterface<int> small_yaw_raw_angle_;
    InputInterface<double> small_yaw_angle_, small_yaw_velocity_;
    InputInterface<bool> small_yaw_online_;
    InputInterface<double> big_yaw_angle_, big_yaw_velocity_, big_yaw_torque_;
    InputInterface<double> big_yaw_unwrapped_angle_;
    InputInterface<bool> big_yaw_online_, big_yaw_angle_valid_, joint2_permitted_;
    InputInterface<bool> big_yaw_origin_valid_, big_yaw_tracking_fault_;
    InputInterface<double> joint2_position_error_;
    std::array<InputInterface<double>, 3> imu_angle_, imu_velocity_;
    OutputInterface<bool> test_enabled_, feedback_ready_;
    OutputInterface<bool> remote_online_, double_down_protection_, double_down_confirmed_status_;
    OutputInterface<double> big_yaw_control_torque_;
    OutputInterface<double> big_yaw_target_velocity_, big_yaw_target_acceleration_;
    OutputInterface<bool> robust_control_active_;
    OutputInterface<bool> coordinated_active_, coordinated_reference_limited_;
    OutputInterface<double> coordinated_target_angle_, coordinated_heading_angle_, coordinated_heading_error_;
    OutputInterface<double> coordinated_small_center_, coordinated_small_offset_;
    OutputInterface<double> coordinated_small_velocity_, coordinated_small_acceleration_;
    OutputInterface<double> identification_elapsed_s_;
    OutputInterface<std::string> state_, identification_phase_, identification_fault_reason_;
    OutputInterface<int> identification_pass_;
    OutputInterface<bool> identification_complete_;
    OutputInterface<double> identification_joint2_stable_elapsed_;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::GimbalControllerTest, rmcs_executor::Component)
