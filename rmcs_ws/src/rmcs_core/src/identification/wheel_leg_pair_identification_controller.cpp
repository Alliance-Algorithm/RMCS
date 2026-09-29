#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_identification_phase.hpp"
#include "identification/wheel_leg_pair_identification_planner.hpp"
#include "identification/wheel_leg_pair_multiband_plan.hpp"

namespace rmcs_core::controller::identification {

class WheelLegPairIdentificationController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
    using Clock = std::chrono::steady_clock;
    using Phase = IdentificationPhase;

public:
    WheelLegPairIdentificationController()
        : Node{get_component_name(), node::options()} {
        load_parameters();
        register_interfaces();
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(),
            "Paired multiband PD: %s, 50 Hz target / 1000 Hz feedback; "
            "Kp=[%.3f,%.3f] Kd=[%.3f,%.3f]; installed gravity and springs retained",
            side_ == 0 ? "left" : "right", pd_kp_[2 * side_], pd_kp_[2 * side_ + 1],
            pd_kd_[2 * side_], pd_kd_[2 * side_ + 1]);
    }

    void update() override {
        zero_commands();
        *hip_angle_model_ = std::numeric_limits<double>::quiet_NaN();
        *knee_angle_model_ = std::numeric_limits<double>::quiet_NaN();
        *hip_velocity_model_ = std::numeric_limits<double>::quiet_NaN();
        *knee_velocity_model_ = std::numeric_limits<double>::quiet_NaN();
        *knee_inner_measured_ = std::numeric_limits<double>::quiet_NaN();
        if (phase() != Phase::kRunning)
            *knee_inner_target_ = std::numeric_limits<double>::quiet_NaN();
        if (phase() != Phase::kRunning)
            zero_diagnostics();
        const auto now = *timestamp_;
        const auto tick = *tick_;
        using rmcs_msgs::Switch;
        const auto left = *switch_left_, right = *switch_right_;
        const bool down = left == Switch::DOWN && right == Switch::DOWN;
        const bool middle = left == Switch::MIDDLE && right == Switch::MIDDLE;
        // Operator stop/reset must work even with stale motor feedback or a
        // broken executor clock. Hardware independently gates on the DR16 too.
        if (*dr16_fresh_ && down) {
            reset_on_both_down();
            return;
        }
        if (!*dr16_fresh_ || !middle) {
            if (phase() == Phase::kPreparing || phase() == Phase::kRunning)
                fail(3, "Remote stale or switches not both middle");
            have_clock_ = false;
            return;
        }
        // A held enable switch must not silently restart a failed/completed run.
        if (phase() == Phase::kFailed || phase() == Phase::kComplete)
            return;
        if (!update_clock(now, tick))
            return;

        if (phase() == Phase::kIdle) {
            set_phase(Phase::kPreparing);
            wait_start_ = now;
            RCLCPP_INFO(get_logger(), "Both-middle: preparing selected-side MIT enable");
        }
        if (phase() == Phase::kPreparing
            && std::chrono::duration<double>(now - wait_start_).count() > ready_timeout_s_) {
            fail(18, "Selected DM did not become ready / preflight timed out");
            return;
        }

        if (const int feedback_fault = feedback_failure_code(); feedback_fault != 0) {
            model_phase_initialized_.fill(false);
            // Startup may precede the first disabled-drive feedback poll. Wait
            // with zero torque, but latch feedback loss once enable was requested.
            if (phase() == Phase::kPreparing && !enable_started_)
                return;
            fail(feedback_fault, "Missing, stale, future, or nonfinite motor/IMU feedback");
            return;
        }
        Eigen::Vector4d measured = Eigen::Vector4d::Zero();
        Eigen::Vector4d speed = Eigen::Vector4d::Zero();
        std::array<int, 4> statuses{}, faults{};
        for (std::size_t i = 0; i < 4; ++i) {
            const double phase =
                model_angle_from_api(*angle_[i], limits_.sign[i], limits_.offset[i], true);
            measured[static_cast<Eigen::Index>(i)] =
                model_phase_initialized_[i] ? unwrap_model_phase(phase, previous_model_angle_[i])
                                            : phase;
            speed[static_cast<Eigen::Index>(i)] = limits_.sign[i] * *velocity_[i];
            if (!std::isfinite(*measured_max_torque_[i])
                || *measured_max_torque_[i] < limits_.max_torque[i]) {
                fail(5, "Configured torque exceeds live motor limit");
                return;
            }
            statuses[i] = *status_[i];
            faults[i] = *fault_[i];
        }

        for (std::size_t side = 0; side < 2; ++side) {
            const auto hip = static_cast<Eigen::Index>(2 * side);
            const auto knee = hip + 1;
            if (!model_phase_initialized_[2 * side + 1])
                measured[knee] = align_auxiliary_to_pair_branch(
                    measured[knee], measured[hip], limits_.spring_min[side],
                    limits_.spring_max[side]);
        }
        for (std::size_t i = 0; i < 4; ++i) {
            previous_model_angle_[i] = measured[static_cast<Eigen::Index>(i)];
            model_phase_initialized_[i] = true;
        }

        *hip_angle_model_ = measured[static_cast<Eigen::Index>(2 * side_)];
        *knee_angle_model_ = measured[static_cast<Eigen::Index>(2 * side_ + 1)];
        *hip_velocity_model_ = speed[static_cast<Eigen::Index>(2 * side_)];
        *knee_velocity_model_ = speed[static_cast<Eigen::Index>(2 * side_ + 1)];
        if (!motor_states_safe(
                side_, statuses, faults, phase() == Phase::kRunning,
                phase() == Phase::kIdle || phase() == Phase::kPreparing)) {
            fail(6, "Motor fault, torque drive disabled, or parked side enabled");
            return;
        }

        const auto base = static_cast<Eigen::Index>(2 * side_);
        const double delta = measured[base + 1] - measured[base];
        const bool observe_delta = probe_delta_diagnostic_ && phase() == Phase::kRunning;
        const double delta_min = limits_.spring_min[side_] + limits_.spring_margin;
        const double delta_max = limits_.spring_max[side_] - limits_.spring_margin;
        if (const auto fault = check_probe_pair(
                limits_, side_, {measured[base], measured[base + 1]},
                {speed[base], speed[base + 1]}, false, !observe_delta);
            fault != PairFault::kNone) {
            RCLCPP_ERROR(
                get_logger(),
                "Probe coordinate check %d: q=[%.6f, %.6f] rad, "
                "drive_delta=%.6f rad, configured delta interval=(%.6f, %.6f); "
                "drive delta is not measured physical inner-knee angle",
                static_cast<int>(fault), measured[base], measured[base + 1], delta, delta_min,
                delta_max);
            fail(100 + static_cast<int>(fault), "Measured drive-coordinate admission failed");
            return;
        }
        if (observe_delta && (delta <= delta_min || delta >= delta_max)
            && !probe_delta_warning_emitted_) {
            RCLCPP_WARN(
                get_logger(),
                "Observed drive_delta=%.6f rad outside configured "
                "proxy interval=(%.6f, %.6f); physical inner-knee angle is not measured. "
                "Continuing the unchanged reference and recording feedback (diagnostic policy)",
                delta, delta_min, delta_max);
            probe_delta_warning_emitted_ = true;
        }
        const auto estimate = check_probe_pair(
            limits_, side_, {measured[base], measured[base + 1]}, {speed[base], speed[base + 1]});
        if (estimate != PairFault::kNone && estimate != PairFault::kSpringLimit
            && !probe_envelope_warning_emitted_[static_cast<std::size_t>(estimate)]) {
            RCLCPP_WARN(
                get_logger(),
                "Probe motion estimate %d exceeded (5=speed, 6=stopping distance); "
                "continuing planned reference with torque cap; estimate is not identified",
                static_cast<int>(estimate));
            probe_envelope_warning_emitted_[static_cast<std::size_t>(estimate)] = true;
        }

        const auto other = static_cast<Eigen::Index>(2 * (1 - side_));
        if (std::abs(speed[other]) > other_side_speed_limit_
            || std::abs(speed[other + 1]) > other_side_speed_limit_) {
            if (!probe_parked_warning_emitted_) {
                RCLCPP_WARN(get_logger(), "Disabled opposite leg moved; continuing probe");
                probe_parked_warning_emitted_ = true;
            }
        }
        if (phase() != Phase::kRunning)
            *reference_model_ = measured;

        *enable_request_ = true;
        enable_started_ = true;
        if (phase() == Phase::kPreparing) {
            // The hardware may clear a fault on this pair, but must not send
            // enable until both drives report fresh fault-free disabled feedback.
            *clear_error_request_ = true;
            if (probe_plan_
                && (!*dm_control_ready_ || *status_[2 * side_] != 1
                    || *status_[2 * side_ + 1] != 1)) {
                fail(29, "Selected torque drive lost readiness during preflight");
                return;
            }
            if (!*dm_control_ready_ || *status_[2 * side_] != 1 || *status_[2 * side_ + 1] != 1)
                return;

            if (!probe_plan_) {
                try {
                    const auto base = static_cast<Eigen::Index>(2 * side_);
                    const std::array initial{measured[base], measured[base + 1]};
                    probe_plan_.emplace(
                        multiband_, initial, limits_.spring_min[side_] + limits_.spring_margin,
                        limits_.spring_max[side_] - limits_.spring_margin);
                    preflight_steps_ =
                        static_cast<std::size_t>(std::ceil(probe_duration() / 0.001));
                    if (ready_timeout_s_ < 1.0 + std::ceil((preflight_steps_ + 1) / 128.0) * .001)
                        throw std::invalid_argument(
                            "ready_timeout_s cannot cover multiband preflight plus enable");
                    parked_reference_ = measured;
                } catch (const std::exception& error) {
                    fail(19, error.what());
                    return;
                }
            }
            for (std::size_t count = 0; count < 128 && preflight_index_ <= preflight_steps_;
                 ++count, ++preflight_index_) {
                const auto sample = probe_sample(
                    probe_duration() * static_cast<double>(preflight_index_)
                    / static_cast<double>(preflight_steps_));
                if (const auto fault = check_probe_reference(limits_, side_, sample);
                    fault != PairFault::kNone) {
                    fail(200 + static_cast<int>(fault), "Probe preflight exceeds motor limits");
                    return;
                }
            }
            if (preflight_index_ <= preflight_steps_)
                return;
            set_phase(Phase::kRunning);
            run_start_ = now;
            last_control_time_ = now;
            last_measured_speed_ = speed;
            have_measured_speed_ = true;
            RCLCPP_INFO(
                get_logger(), "Measured motor-pair %s preflight passed; starting %s (%.1f s)",
                "multiband/chirp", "RL PD", probe_duration());
        }

        if (!*dm_control_ready_ || *status_[2 * side_] != 1 || *status_[2 * side_ + 1] != 1) {
            fail(29, "Selected-side MIT torque drive no longer ready");
            return;
        }
        const double elapsed = std::chrono::duration<double>(now - run_start_).count();
        if (!std::isfinite(elapsed) || elapsed < 0) {
            fail(2, "Invalid experiment elapsed time");
            return;
        }
        const bool finished = elapsed >= probe_duration();
        if (finished) {
            set_phase(Phase::kComplete);
            *segment_id_ = -1;
            *segment_role_ = 2;
            *segment_waveform_ = 0;
            *enable_request_ = false;
            *clear_error_request_ = false;
            held_torque_model_.fill(0.0);
            last_segment_id_ = -1;
            probe_plan_.reset();
            zero_diagnostics();
            RCLCPP_INFO(get_logger(), "Paired PD identification complete; disabling drive");
            return;
        }
        // Reference is held for 20 ticks; PD consumes feedback every tick.
        try {
            update_closed_loop(now, elapsed, measured, speed);
        } catch (const std::exception& error) {
            fail(19, error.what());
            return;
        }
        if (phase() == Phase::kFailed)
            return;
        const auto torques = selected_torque_api(side_, limits_, held_torque_model_);
        for (std::size_t i = 0; i < command_torque_.size(); ++i)
            *command_torque_[i] = torques[i];
    }

private:
    bool update_clock(Clock::time_point now, std::size_t tick) {
        if (!std::isfinite(*update_rate_) || *update_rate_ < 900 || *update_rate_ > 1100) {
            fail(1, "Executor must run at 1000 Hz");
            return false;
        }
        if (have_clock_) {
            const double gap = std::chrono::duration<double>(now - previous_time_).count();
            if (tick != previous_tick_ + 1 || gap <= 0 || gap > max_tick_gap_s_) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Timing discontinuity: tick %zu -> %zu, interval %.3f ms, limit %.3f ms",
                    previous_tick_, tick, gap * 1000, max_tick_gap_s_ * 1000);
                fail(
                    2, tick != previous_tick_ + 1 ? "Executor update counter discontinuity"
                       : gap <= 0                 ? "Executor clock did not advance"
                                  : "Executor interval exceeds configured timing budget");
                return false;
            }
        }
        have_clock_ = true;
        previous_tick_ = tick;
        previous_time_ = now;
        return true;
    }

    void load_parameters() {
        const auto side = get_parameter("side").as_string();
        if (side != "left" && side != "right")
            throw std::invalid_argument("Identification requires exactly one side: left or right");
        side_ = side == "left" ? 0 : 1;
        if (get_parameter("experiment_stage").as_string() != "probe"
            || get_parameter("probe_pattern").as_string() != "multiband_chirp"
            || get_parameter("control_law").as_string() != "rl_pd"
            || !get_parameter("phase_api_angle").as_bool()
            || get_parameter("control_frequency_hz").as_double() != 1000.0
            || get_parameter("reference_frequency_hz").as_double() != 50.0)
            throw std::invalid_argument(
                "Pair identification requires multiband PD: 50 Hz target, 1000 Hz feedback");

        load_array("model_sign", limits_.sign);
        load_array("model_offset", limits_.offset);
        load_array("root_min", limits_.root_min);
        load_array("root_max", limits_.root_max);
        load_array("max_speed", limits_.max_speed);
        load_array("max_acceleration", limits_.max_acceleration);
        load_array("braking_acceleration", limits_.braking_acceleration);
        load_array("max_torque", limits_.max_torque);
        if ((get_parameter("dm_motor_family").as_string() != "DM-J8009"
             || get_parameter("dm_rated_torque_nm").as_double() != 20.0
             || get_parameter("dm_peak_torque_nm").as_double() != 40.0
             || std::any_of(limits_.max_torque.begin(), limits_.max_torque.end(), [](double cap) {
                    return !std::isfinite(cap) || cap <= 0 || cap > 40;
                })))
            throw std::invalid_argument("RL PD requires DM-J8009 rated20/peak40 torque contract");
        load_array("spring_delta_min", limits_.spring_min);
        load_array("spring_delta_max", limits_.spring_max);
        limits_.joint_margin = get_parameter("joint_margin").as_double();
        limits_.spring_margin = get_parameter("spring_margin").as_double();
        validate_limits(limits_);

        if (!has_parameter("probe_measured_delta_policy"))
            declare_parameter<std::string>("probe_measured_delta_policy", "stop");
        const auto delta_policy = get_parameter("probe_measured_delta_policy").as_string();
        if (delta_policy != "stop" && delta_policy != "diagnostic")
            throw std::invalid_argument("probe_measured_delta_policy must be stop or diagnostic");
        probe_delta_diagnostic_ = delta_policy == "diagnostic";

        load_array("multiband_common_hz", multiband_.common_hz);
        load_array("multiband_relative_hz", multiband_.relative_hz);
        load_array("multiband_common_amplitude", multiband_.common_amplitude);
        load_array("multiband_relative_amplitude", multiband_.relative_amplitude);
        load_array("multiband_band_s", multiband_.band_s);
        multiband_.shape_offset = get_parameter("multiband_shape_offset").as_double();
        multiband_.eighth_turn_s = get_parameter("multiband_eighth_turn_s").as_double();
        multiband_.dwell_s = get_parameter("multiband_dwell_s").as_double();
        multiband_.ramp_s = get_parameter("multiband_ramp_s").as_double();
        multiband_.common_step = get_parameter("multiband_common_step").as_double();
        multiband_.relative_step = get_parameter("multiband_relative_step").as_double();
        multiband_.step_rise_s = get_parameter("multiband_step_rise_s").as_double();
        multiband_.validation_s = get_parameter("multiband_validation_s").as_double();
        if (!has_parameter("multiband_load_offset"))
            declare_parameter<double>("multiband_load_offset", 0.0);
        if (!has_parameter("multiband_validation_relative_scale"))
            declare_parameter<double>("multiband_validation_relative_scale", 1.0);
        multiband_.load_offset = get_parameter("multiband_load_offset").as_double();
        multiband_.validation_relative_scale =
            get_parameter("multiband_validation_relative_scale").as_double();
        PairMultibandPlan::validate_config(multiband_);

        load_array("pd_kp", pd_kp_);
        load_array("pd_kd", pd_kd_);
        for (const auto* name : {"gravity_feedforward_model", "ki_velocity", "integral_limit"}) {
            const auto values = get_parameter(name).as_double_array();
            if (values.size() != 4
                || std::any_of(values.begin(), values.end(), [](double v) { return v != 0; }))
                throw std::invalid_argument("Pure PD requires zero integral and feedforward");
        }
        load_array("max_position_error", max_position_error_);
        load_array("max_velocity_error", max_velocity_error_);
        load_array("max_measured_acceleration", max_measured_acceleration_);
        for (std::size_t i = 0; i < 4; ++i) {
            if (!std::isfinite(pd_kp_[i]) || pd_kp_[i] <= 0 || !std::isfinite(pd_kd_[i])
                || pd_kd_[i] < 0 || !std::isfinite(max_position_error_[i])
                || max_position_error_[i] <= 0 || !std::isfinite(max_velocity_error_[i])
                || max_velocity_error_[i] <= 0 || !std::isfinite(max_measured_acceleration_[i])
                || max_measured_acceleration_[i] <= 0)
                throw std::invalid_argument("Invalid PD gains or diagnostic thresholds");
        }
        ready_timeout_s_ = get_parameter("ready_timeout_s").as_double();
        feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
        max_tick_gap_s_ = get_parameter("max_tick_gap_s").as_double();
        other_side_speed_limit_ = get_parameter("other_side_speed_limit").as_double();
        const bool valid_ready = std::isfinite(ready_timeout_s_) && ready_timeout_s_ > 0;
        bool valid_feedback = std::isfinite(feedback_timeout_s_);
        valid_feedback &= feedback_timeout_s_ > 0;
        valid_feedback &= feedback_timeout_s_ <= 0.05;
        bool valid_tick = std::isfinite(max_tick_gap_s_);
        valid_tick &= max_tick_gap_s_ > 0.001;
        valid_tick &= max_tick_gap_s_ <= 0.02;
        if (!valid_ready || !valid_feedback || !valid_tick
            || !std::isfinite(other_side_speed_limit_) || other_side_speed_limit_ <= 0)
            throw std::invalid_argument(
                "Invalid identification timeouts or parked-side speed limit");
    }

    void register_interfaces() {
        register_input("/predefined/update_count", tick_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/imu/last_steady_ns", imu_feedback_ns_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
        register_input("/wheel_leg/dm_control_ready", dm_control_ready_);
        for (std::size_t i = 0; i < kAxisNames.size(); ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_input(prefix + "/angle", angle_[i]);
            register_input(prefix + "/velocity", velocity_[i]);
            register_input(prefix + "/max_torque", measured_max_torque_[i]);
            register_input(prefix + "/feedback_steady_ns", feedback_ns_[i]);
            register_output(
                prefix + "/control_torque", command_torque_[i],
                std::numeric_limits<double>::quiet_NaN());
            if (i < 4) {
                register_input(prefix + "/fault_code", fault_[i]);
                register_input(prefix + "/status_code", status_[i]);
            }
        }
        register_output("/wheel_leg/enable_request", enable_request_, false);
        register_output("/wheel_leg/clear_error_request", clear_error_request_, false);
        register_output(
            "/wheel_leg/identification/reference_model", reference_model_, Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/reference_velocity_model", reference_velocity_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/velocity_target_model", velocity_target_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/position_error_model", position_error_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/speed_error_model", speed_error_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/torque_preclip_model", torque_preclip_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/torque_preclip_api", torque_preclip_api_,
            Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN()));
        register_output(
            "/wheel_leg/identification/torque_integral_model", torque_integral_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/torque_gravity_ff_model", torque_gravity_ff_model_,
            Eigen::Vector4d::Zero());
        register_output(
            "/wheel_leg/identification/phase", phase_, std::to_underlying(Phase::kIdle));
        register_output("/wheel_leg/identification/segment_id", segment_id_, -1);
        register_output("/wheel_leg/identification/segment_role", segment_role_, std::uint8_t{2});
        register_output(
            "/wheel_leg/identification/segment_waveform", segment_waveform_, std::uint8_t{0});
        register_output(
            "/wheel_leg/identification/repetition_id", repetition_id_output_, std::uint32_t{0});
        register_output(
            "/wheel_leg/identification/selected_side", selected_side_, static_cast<int>(side_));
        register_output("/wheel_leg/identification/failure_reason", failure_reason_, 0);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        register_output(
            "/wheel_leg/identification/selected/hip/angle_model", hip_angle_model_, nan);
        register_output(
            "/wheel_leg/identification/selected/knee/angle_model", knee_angle_model_, nan);
        register_output(
            "/wheel_leg/identification/selected/hip/velocity_model", hip_velocity_model_, nan);
        register_output(
            "/wheel_leg/identification/selected/knee/velocity_model", knee_velocity_model_, nan);
        register_output(
            "/wheel_leg/identification/selected/inner_knee_measured_deg", knee_inner_measured_,
            nan);
        register_output(
            "/wheel_leg/identification/selected/inner_knee_target_deg", knee_inner_target_, nan);
        for (std::size_t i = 0; i < wheel_speed_target_.size(); ++i)
            register_output(
                std::string{"/wheel_leg/"} + (i == 0 ? "left_wheel" : "right_wheel")
                    + "/control_velocity",
                wheel_speed_target_[i], 0.0);
    }

    [[nodiscard]] Phase phase() const { return static_cast<Phase>(*phase_); }
    void set_phase(Phase value) { *phase_ = std::to_underlying(value); }

    template <std::size_t N>
    void load_array(const std::string& name, std::array<double, N>& array) {
        const auto values = get_parameter(name).as_double_array();
        if (values.size() != N)
            throw std::invalid_argument(
                name + " requires " + std::to_string(N) + " measured values");
        std::copy(values.begin(), values.end(), array.begin());
    }

    void zero_commands() {
        for (auto& torque : command_torque_)
            *torque = std::numeric_limits<double>::quiet_NaN();
        *enable_request_ = false;
        *clear_error_request_ = false;
    }

    void reset_on_both_down() {
        if (phase() != Phase::kIdle) {
            RCLCPP_INFO(get_logger(), "Both-down: release torque, disable and reset experiment");
            *repetition_id_output_ = ++repetition_id_;
        }
        set_phase(Phase::kIdle);
        *failure_reason_ = 0;
        *segment_id_ = -1;
        *knee_inner_target_ = std::numeric_limits<double>::quiet_NaN();
        held_torque_model_.fill(0.0);
        last_reference_tick_ = 0;
        probe_plan_.reset();
        preflight_index_ = 0;
        last_segment_id_ = -1;
        have_measured_speed_ = false;
        acceleration_warning_emitted_.fill(false);
        probe_envelope_warning_emitted_.fill(false);
        probe_tracking_warning_emitted_.fill(false);
        probe_parked_warning_emitted_ = false;
        probe_delta_warning_emitted_ = false;
        have_clock_ = false;
        enable_started_ = false;
        model_phase_initialized_.fill(false);
        zero_diagnostics();
    }

    void zero_diagnostics() {
        *segment_role_ = 2;
        *segment_waveform_ = 0;
        *reference_velocity_model_ = Eigen::Vector4d::Zero();
        *velocity_target_model_ = Eigen::Vector4d::Zero();
        *position_error_model_ = Eigen::Vector4d::Zero();
        *speed_error_model_ = Eigen::Vector4d::Zero();
        *torque_preclip_model_ = Eigen::Vector4d::Zero();
        *torque_preclip_api_ =
            Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
        *torque_integral_model_ = Eigen::Vector4d::Zero();
        *torque_gravity_ff_model_ = Eigen::Vector4d::Zero();
    }

    int feedback_failure_code() const {
        const auto now = Clock::now().time_since_epoch();
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        if (now_ns < 0)
            return 80;
        for (std::size_t i = 0; i < feedback_ns_.size(); ++i) {
            const auto stamp = *feedback_ns_[i];
            if (stamp == 0)
                return 40 + static_cast<int>(i);
            if (stamp > static_cast<std::uint64_t>(now_ns))
                return 50 + static_cast<int>(i);
            if (static_cast<double>(static_cast<std::uint64_t>(now_ns) - stamp) * 1e-9
                > feedback_timeout_s_)
                return 60 + static_cast<int>(i);
            if (!std::isfinite(*angle_[i]) || !std::isfinite(*velocity_[i]))
                return 70 + static_cast<int>(i);
        }
        if (!*feedback_fresh_) {
            const auto stamp = *imu_feedback_ns_;
            if (!stamp || stamp > static_cast<std::uint64_t>(now_ns)
                || now_ns - static_cast<std::int64_t>(stamp) >= 50'000'000)
                return 81;
            return 80;
        }
        return 0;
    }

    bool check_tracking(std::size_t axis, double position_error, double speed_error) {
        if (!std::isfinite(position_error) || !std::isfinite(speed_error)) {
            fail(30, "Nonfinite tracking error");
            return false;
        }
        if (std::abs(position_error) <= max_position_error_[axis]
            && std::abs(speed_error) <= max_velocity_error_[axis])
            return true;
        if (!probe_tracking_warning_emitted_[axis]) {
            RCLCPP_WARN(
                get_logger(),
                "%s probe tracking error position=%.4f rad, speed=%.4f rad/s; "
                "continuing with torque cap (diagnostic only)",
                kAxisNames[axis], position_error, speed_error);
            probe_tracking_warning_emitted_[axis] = true;
        }
        return true;
    }

    void update_closed_loop(
        Clock::time_point now, double elapsed, const Eigen::Vector4d& measured,
        const Eigen::Vector4d& speed) {
        const double dt = std::chrono::duration<double>(now - last_control_time_).count();
        if (!std::isfinite(dt) || dt < 0 || dt > max_tick_gap_s_) {
            fail(31, "PD interval exceeds configured timing budget");
            return;
        }
        if (last_reference_tick_ == 0 || *tick_ - last_reference_tick_ >= reference_ticks_) {
            held_reference_ = probe_sample(elapsed);
            last_reference_tick_ = *tick_;
        }
        const auto& sample = held_reference_;
        *knee_inner_target_ = sample.beta * 180.0 / std::numbers::pi;
        const auto fault = check_probe_reference(limits_, side_, sample);
        if (fault != PairFault::kNone) {
            fail(300 + static_cast<int>(fault), "Running reference outside preflight bounds");
            return;
        }
        const auto base = static_cast<Eigen::Index>(2 * side_);
        *reference_model_ = parked_reference_;
        *reference_velocity_model_ = Eigen::Vector4d::Zero();
        *velocity_target_model_ = Eigen::Vector4d::Zero();
        *position_error_model_ = Eigen::Vector4d::Zero();
        *speed_error_model_ = Eigen::Vector4d::Zero();
        *torque_preclip_model_ = Eigen::Vector4d::Zero();
        *torque_integral_model_ = Eigen::Vector4d::Zero();
        *torque_gravity_ff_model_ = Eigen::Vector4d::Zero();
        for (std::size_t j = 0; j < 2; ++j) {
            const auto i = static_cast<std::size_t>(base) + j;
            (*reference_model_)[static_cast<Eigen::Index>(i)] = sample.position[j];
            (*reference_velocity_model_)[static_cast<Eigen::Index>(i)] = sample.velocity[j];
            if (have_measured_speed_ && dt > 0 && !acceleration_warning_emitted_[j]) {
                // This threshold has not been identified on the suspended leg.
                // Differencing quantized asynchronous velocity samples is only
                // diagnostic; it must not latch an otherwise valid experiment.
                const double acceleration = (speed[static_cast<Eigen::Index>(i)]
                                             - last_measured_speed_[static_cast<Eigen::Index>(i)])
                                          / dt;
                if (std::abs(acceleration) > max_measured_acceleration_[i]) {
                    RCLCPP_WARN(
                        get_logger(),
                        "%s measured acceleration %.3f rad/s^2 exceeds diagnostic threshold "
                        "%.3f; continuing (logged once per axis per run)",
                        kAxisNames[i], acceleration, max_measured_acceleration_[i]);
                    acceleration_warning_emitted_[j] = true;
                }
            }
            const auto result = pd_step(
                sample.position[j], measured[static_cast<Eigen::Index>(i)],
                speed[static_cast<Eigen::Index>(i)], pd_kp_[i], pd_kd_[i],
                std::min(limits_.max_torque[i], *measured_max_torque_[i]));
            (*velocity_target_model_)[static_cast<Eigen::Index>(i)] = result.velocity_target;
            (*position_error_model_)[static_cast<Eigen::Index>(i)] = result.position_error;
            (*speed_error_model_)[static_cast<Eigen::Index>(i)] = result.speed_error;
            (*torque_preclip_model_)[static_cast<Eigen::Index>(i)] = result.preclip;
            (*torque_preclip_api_)[static_cast<Eigen::Index>(i)] = limits_.sign[i] * result.preclip;
            if (!std::isfinite(result.torque) || !std::isfinite(result.preclip)) {
                fail(30, "Nonfinite PD output");
                return;
            }
            if (!check_tracking(i, result.position_error, result.speed_error))
                return;
            held_torque_model_[j] = result.torque;
        }
        last_control_time_ = now;
        last_measured_speed_ = speed;
        *segment_id_ = sample.segment_id;
        *segment_role_ = sample.validation ? 1 : 0;
        *segment_waveform_ = sample.waveform;
        if (sample.segment_id != last_segment_id_) {
            const auto& plan = *probe_plan_;
            const auto& segment = plan.segments().at(static_cast<std::size_t>(sample.segment_id));
            RCLCPP_INFO(
                get_logger(),
                "Multiband segment %d: %s, %.1f s, validation=%d, "
                "delta_ref=%.4f measured=%.4f, relative low-band amplitude=%.4f rad",
                sample.segment_id, segment.name.c_str(), segment.duration_s, sample.validation,
                segment.from[1], measured[base + 1] - measured[base],
                plan.relative_amplitude(multiband_.relative_amplitude[0], segment.from[1]));
        }
        last_segment_id_ = sample.segment_id;
    }

    void fail(int code, const char* message) {
        if (phase() != Phase::kFailed)
            RCLCPP_ERROR(get_logger(), "Identification stopped (%d): %s", code, message);
        set_phase(Phase::kFailed);
        *knee_inner_target_ = std::numeric_limits<double>::quiet_NaN();
        *failure_reason_ = code;
        *segment_id_ = -1;
        *segment_role_ = 2;
        *segment_waveform_ = 0;
        *enable_request_ = false;
        *clear_error_request_ = false;
        held_torque_model_.fill(0.0);
        last_segment_id_ = -1;
        probe_plan_.reset();
        preflight_index_ = 0;
        zero_commands();
        zero_diagnostics();
    }

    std::size_t side_ = 0;
    std::array<double, 4> pd_kp_{}, pd_kd_{};
    static constexpr std::size_t reference_ticks_ = 20;
    std::size_t last_reference_tick_ = 0;
    PairSample held_reference_{};
    PairLimits limits_;
    double probe_duration() const { return probe_plan_->duration(); }
    PairSample probe_sample(double elapsed) const { return probe_plan_->at(elapsed); }
    PairMultibandConfig multiband_;
    bool probe_delta_diagnostic_ = false, probe_delta_warning_emitted_ = false;
    std::array<double, 4> max_position_error_{}, max_velocity_error_{},
        max_measured_acceleration_{};
    double ready_timeout_s_ = 0, feedback_timeout_s_ = 0;
    double max_tick_gap_s_ = 0, other_side_speed_limit_ = 0;

    InputInterface<std::size_t> tick_;
    InputInterface<double> update_rate_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> feedback_fresh_, dr16_fresh_, dm_control_ready_;
    InputInterface<std::uint64_t> imu_feedback_ns_;
    std::array<InputInterface<double>, 6> angle_, velocity_, measured_max_torque_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_ns_;
    std::array<InputInterface<int>, 4> fault_, status_;
    std::array<OutputInterface<double>, 6> command_torque_;
    OutputInterface<bool> enable_request_;
    OutputInterface<bool> clear_error_request_;
    OutputInterface<Eigen::Vector4d> reference_model_, reference_velocity_model_;
    OutputInterface<Eigen::Vector4d> velocity_target_model_, position_error_model_,
        speed_error_model_;
    OutputInterface<Eigen::Vector4d> torque_preclip_model_, torque_integral_model_;
    OutputInterface<Eigen::Vector4d> torque_gravity_ff_model_;
    OutputInterface<Eigen::Matrix<double, 6, 1>> torque_preclip_api_;
    OutputInterface<int> phase_, segment_id_, selected_side_, failure_reason_;
    OutputInterface<std::uint8_t> segment_role_;
    OutputInterface<std::uint8_t> segment_waveform_;
    OutputInterface<std::uint32_t> repetition_id_output_;
    std::array<OutputInterface<double>, 2> wheel_speed_target_;
    OutputInterface<double> hip_angle_model_, knee_angle_model_;
    OutputInterface<double> hip_velocity_model_, knee_velocity_model_;
    OutputInterface<double> knee_inner_measured_, knee_inner_target_;

    bool have_clock_ = false, have_measured_speed_ = false;
    bool enable_started_ = false;
    std::size_t previous_tick_ = 0, preflight_index_ = 0, preflight_steps_ = 0;
    int last_segment_id_ = -1;
    std::uint32_t repetition_id_ = 0;
    Clock::time_point previous_time_{}, wait_start_{}, run_start_{}, last_control_time_{};
    Eigen::Vector4d parked_reference_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d last_measured_speed_ = Eigen::Vector4d::Zero();
    std::array<bool, 2> acceleration_warning_emitted_{};
    std::array<bool, 9> probe_envelope_warning_emitted_{};
    std::array<bool, 4> probe_tracking_warning_emitted_{};
    bool probe_parked_warning_emitted_ = false;
    std::array<double, 4> previous_model_angle_{};
    std::array<bool, 4> model_phase_initialized_{};
    std::array<double, 2> held_torque_model_{};
    std::optional<PairMultibandPlan> probe_plan_;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegPairIdentificationController,
    rmcs_executor::Component)
