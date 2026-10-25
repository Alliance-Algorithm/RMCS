#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_limit_probe.hpp"
#include "identification/wheel_leg_sweep_session.hpp"
#include "identification/wheel_leg_wheel_identification_plan.hpp"

namespace rmcs_core::controller::identification {

class WheelLegFrequencySweepController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegFrequencySweepController()
        : Node{get_component_name(), node::options()}
        , geometry_{load_geometry("left"), load_geometry("right")}
        , session_(get_parameter("experiment").as_string() == "joints") {
        load_parameters();
        register_input("/predefined/update_rate", update_rate_);
        register_input("/predefined/update_count", tick_);
        register_input("/predefined/timestamp", scheduled_time_);
        register_input("/remote/switch/left", left_);
        register_input("/remote/switch/right", right_);
        register_input("/wheel_leg/dr16_fresh", remote_fresh_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/imu/quaternion", quaternion_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/imu/acceleration", acceleration_);
        register_input("/wheel_leg/imu/last_steady_ns", imu_ns_);
        register_input("/wheel_leg/imu/acceleration_steady_ns", acceleration_ns_);
        register_input("/wheel_leg/joint_position_pd", motor_pd_);
        register_output("/wheel_leg/enable_request", enable_, false);
        register_output("/wheel_leg/clear_error_request", clear_, false);
        register_output("/wheel_leg/sweep/heartbeat_steady_ns", heartbeat_, std::uint64_t{0});
        register_output("/wheel_leg/identification/wheel_only", wheel_only_, kind_ == 1);
        if (kind_ != 1)
            register_output("/wheel_leg/identification/selected_side", selected_side_, side_);
        register_output("/wheel_leg/sweep/experiment", experiment_output_, kind_);
        register_output("/wheel_leg/sweep/side", side_output_, side_);
        register_output("/wheel_leg/sweep/state", state_, 0);
        register_output("/wheel_leg/sweep/mode", mode_, 0);
        register_output("/wheel_leg/sweep/run_id", run_, std::uint64_t{0});
        register_output("/wheel_leg/sweep/stage", stage_, -1);
        register_output("/wheel_leg/sweep/waveform", waveform_, 0);
        register_output("/wheel_leg/sweep/validation", validation_, false);
        register_output("/wheel_leg/sweep/completed", completed_, false);
        register_output("/wheel_leg/sweep/stop_reason", stop_, 0);
        register_output("/wheel_leg/sweep/drives_ready", drives_ready_output_, false);
        register_output("/wheel_leg/sweep/reference_active", reference_active_, false);
        register_output(
            "/wheel_leg/sweep/reference_sequence", reference_sequence_, std::uint64_t{0});
        register_output("/wheel_leg/sweep/reference_elapsed_s", reference_elapsed_, 0.0);
        register_output("/wheel_leg/sweep/run_elapsed_s", run_elapsed_, 0.0);
        register_output("/wheel_leg/sweep/control_dt_s", control_dt_, 0.0);
        register_output("/wheel_leg/sweep/control_steady_ns", control_ns_, std::uint64_t{0});
        register_output("/wheel_leg/sweep/scheduled_steady_ns", scheduled_ns_, std::uint64_t{0});
        register_output(
            "/wheel_leg/sweep/control_end_steady_ns", control_end_ns_, std::uint64_t{0});
        register_output("/wheel_leg/sweep/remote_left", remote_left_, 0);
        register_output("/wheel_leg/sweep/remote_right", remote_right_, 0);
        register_output("/wheel_leg/sweep/theta_reference", theta_ref_, nan);
        register_output("/wheel_leg/sweep/beta_reference", beta_ref_, nan);
        register_output("/wheel_leg/sweep/beta_reference_valid", beta_ref_valid_, false);
        // These flags describe diagnostics; none is a motion interlock.
        register_output("/wheel_leg/sweep/motion_guards_enabled", guards_enabled_, false);
        for (std::size_t i = 0; i < 2; ++i) {
            const auto prefix = std::string{"/wheel_leg/sweep/host/"} + (i == 0 ? "hip" : "knee");
            register_output(prefix + "_kp", host_kp_[i], kp_[i]);
            register_output(prefix + "_kd", host_kd_[i], kd_[i]);
            register_output(prefix + "_torque_limit_nm", host_limit_[i], torque_cap_[i]);
        }
        register_output("/wheel_leg/sweep/host/wheel_velocity_kp", wheel_gain_, wheel_kp_);
        register_output("/wheel_leg/sweep/host/wheel_torque_limit_nm", wheel_limit_, wheel_cap_);
        for (std::size_t i = 0; i < 6; ++i) {
            const auto motor = std::string{"/wheel_leg/"} + kAxisNames[i];
            const auto out = std::string{"/wheel_leg/sweep/"} + kAxisNames[i];
            register_input(motor + "/angle", angle_[i]);
            register_input(motor + "/velocity", velocity_[i]);
            register_input(motor + "/torque", feedback_torque_[i]);
            register_input(motor + "/max_torque", max_torque_[i]);
            register_input(motor + "/feedback_steady_ns", feedback_ns_[i]);
            register_input(motor + "/feedback_sequence", feedback_sequence_[i]);
            register_output(motor + "/control_torque", torque_[i], 0.0);
            register_output(out + "/model_angle", model_q_[i], nan);
            register_output(out + "/model_velocity", model_dq_[i], nan);
            register_output(out + "/model_acceleration", model_ddq_[i], 0.0);
            register_output(out + "/estimated_stopping_distance", stopping_distance_[i], 0.0);
            register_output(out + "/reference", reference_q_[i], nan);
            register_output(out + "/reference_velocity", reference_dq_[i], 0.0);
            register_output(out + "/reference_acceleration", reference_ddq_[i], 0.0);
            register_output(out + "/tracking_error", error_[i], nan);
            register_output(out + "/torque_preclip", preclip_[i], 0.0);
            register_output(out + "/saturated", saturated_[i], false);
            register_output(out + "/diagnostic_flags", diagnostic_[i], std::uint32_t{0});
            if (i < 4) {
                register_input(motor + "/status_code", status_[i]);
                register_input(motor + "/fault_code", fault_[i]);
            }
        }
        for (std::size_t i = 0; i < 2; ++i) {
            const auto side = std::string{"/wheel_leg/sweep/"} + (i == 0 ? "left" : "right");
            register_output(side + "/beta_estimate", beta_estimate_[i], nan);
            register_output(side + "/beta_valid", beta_valid_[i], false);
            register_output(side + "/theta_continuous", theta_estimate_[i], nan);
            const auto end = std::string{"/wheel_leg/sweep/probe/"} + (i == 0 ? "low" : "high");
            register_output(end + "/result", probe_result_[i], 0);
            register_output(end + "/model_angle", probe_q_[i], nan);
            register_output(end + "/beta_estimate", probe_beta_[i], nan);
        }
        RCLCPP_INFO(
            get_logger(),
            "Sweep experiment=%s side=%d: DOWN/DOWN reset, MIDDLE/MIDDLE zero-torque arm; "
            "300 ms mode selection, 1000 Hz host torque / 200 Hz reference. Motion guards "
            "disabled.",
            get_parameter("experiment").as_string().c_str(), side_);
    }

    void before_updating() override {
        if (*motor_pd_)
            throw std::invalid_argument("Frequency sweep requires pure MIT torque mode");
    }

    void update() override {
        // The executor sets this after before_updating(), before the first tick.
        if (*update_rate_ != 1000.0)
            throw std::invalid_argument("Frequency sweep requires a 1000 Hz executor");
        const auto now_ns = steady_ns();
        const double now = static_cast<double>(now_ns) * 1e-9;
        *control_dt_ = last_control_ns_ ? static_cast<double>(now_ns - last_control_ns_) * 1e-9 : 0;
        last_control_ns_ = now_ns;
        *control_ns_ = *heartbeat_ = now_ns;
        *scheduled_ns_ =
            static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                           scheduled_time_->time_since_epoch())
                                           .count());
        *remote_left_ = static_cast<int>(*left_);
        *remote_right_ = static_cast<int>(*right_);
        for (std::size_t i = 0; i < 6; ++i) {
            *torque_[i] = *preclip_[i] = 0;
            *saturated_[i] = false;
        }
        *reference_active_ = false;
        update_feedback();
        const auto health = sensor_health(now_ns);
        const bool ready = drives_ready();
        const bool started = session_.update(now, *remote_fresh_, *left_, *right_, health, ready);
        if (session_.state == SweepState::kResetReady) {
            reference_ = {};
            have_phase_.fill(false);
            pair_aligned_.fill(false);
            last_sequence_.fill(0);
            last_feedback_ns_.fill(0);
            previous_phase_.fill(0);
            previous_velocity_.fill(0);
            previous_effort_ = 0;
            *stage_ = -1;
            *theta_ref_ = *beta_ref_ = nan;
            *beta_ref_valid_ = false;
            *reference_sequence_ = 0;
            *waveform_ = 0;
            *validation_ = false;
            *reference_elapsed_ = *run_elapsed_ = 0;
            for (std::size_t i = 0; i < 6; ++i) {
                *reference_q_[i] = *error_[i] = nan;
                *reference_dq_[i] = *reference_ddq_[i] = 0;
                *model_ddq_[i] = 0;
            }
            for (std::size_t i = 0; i < 2; ++i) {
                *probe_result_[i] = 0;
                *probe_q_[i] = *probe_beta_[i] = nan;
            }
        }
        if (started) {
            previous_effort_ = 0;
            reference_tick_ = *tick_;
            *reference_sequence_ = 0;
            if (kind_ == 2)
                joint_plan_->start(
                    session_.mode, selected_angles(), hip_zero_[side_], beta_min_, beta_max_);
            if (kind_ == 3)
                probe_->start(now, selected_angles());
        }
        if (session_.state == SweepState::kRunning) {
            const double elapsed = now - session_.start_time;
            *run_elapsed_ = elapsed;
            if ((kind_ == 1 && elapsed >= wheel_plan_->duration())
                || (kind_ == 2 && elapsed >= joint_plan_->duration()))
                session_.complete();
            else {
                if (started || *tick_ >= reference_tick_ + 5) {
                    reference_tick_ = *tick_;
                    *reference_elapsed_ = elapsed;
                    ++*reference_sequence_;
                    if (kind_ == 1)
                        wheel_sample_ = wheel_plan_->at(elapsed);
                    else if (kind_ == 2)
                        reference_ = joint_plan_->at(elapsed);
                    else {
                        reference_ = probe_->update(now, selected_angles(), previous_effort_);
                        for (std::size_t i = 0; i < 2; ++i) {
                            *probe_result_[i] = probe_->result[i];
                            *probe_q_[i] = probe_->boundary_q[i];
                            *probe_beta_[i] = probe_->boundary_beta[i];
                        }
                        if (probe_->stage == 5)
                            session_.complete();
                    }
                }
                if (session_.state == SweepState::kRunning)
                    apply_reference();
            }
        }
        for (std::size_t i = 0; i < 6; ++i)
            if (!std::isfinite(*preclip_[i]) || !std::isfinite(*torque_[i]))
                session_.fault(SweepStop::kNonfinite);
        if (session_.state != SweepState::kRunning) {
            *reference_active_ = false;
            for (std::size_t i = 0; i < 6; ++i)
                *torque_[i] = 0;
        }
        *enable_ = session_.enabled();
        *clear_ = *enable_ && !session_.ready_seen() && *left_ == rmcs_msgs::Switch::MIDDLE
               && *right_ == rmcs_msgs::Switch::MIDDLE;
        *state_ = static_cast<int>(session_.state);
        *stop_ = static_cast<int>(session_.reason);
        *mode_ = session_.mode;
        *run_ = session_.run_id;
        *completed_ = session_.completed;
        *drives_ready_output_ = ready;
        *control_end_ns_ = steady_ns();
    }

private:
    PairGeometry load_geometry(const std::string& side) {
        auto beta = get_parameter("geometry_beta_deg").as_double_array();
        for (auto& value : beta)
            value *= PairGeometry::rad;
        return {std::move(beta), get_parameter("geometry_" + side + "_delta").as_double_array()};
    }

    void load_parameters() {
        const auto experiment = get_parameter("experiment").as_string();
        kind_ = experiment == "wheels" ? 1
              : experiment == "joints" ? 2
              : experiment == "limits" ? 3
                                       : 0;
        if (!kind_)
            throw std::invalid_argument("experiment must be wheels, joints or limits");
        if (kind_ != 1) {
            const auto side = get_parameter("side").as_string();
            if (side != "left" && side != "right")
                throw std::invalid_argument("side must be left or right");
            side_ = side == "left" ? 0 : 1;
        }
        const auto copy = [this]<std::size_t N>(const char* name, std::array<double, N>& target) {
            const auto values = get_parameter(name).as_double_array();
            if (values.size() != N || !std::all_of(values.begin(), values.end(), [](double value) {
                    return std::isfinite(value);
                }))
                throw std::invalid_argument(std::string{"Invalid array: "} + name);
            std::copy(values.begin(), values.end(), target.begin());
        };
        copy("model_sign", model_sign_);
        copy("model_offset", model_offset_);
        copy("hip_sign", hip_sign_);
        copy("hip_zero", hip_zero_);
        for (double sign : model_sign_)
            if (std::abs(sign) != 1)
                throw std::invalid_argument("model_sign must be +/-1");
        for (double sign : hip_sign_)
            if (std::abs(sign) != 1)
                throw std::invalid_argument("hip_sign must be +/-1");
        feedback_timeout_ = get_parameter("feedback_timeout_s").as_double();
        if (!std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0 || feedback_timeout_ > .05
            || get_parameter("reference_frequency_hz").as_double() != 200.0
            || get_parameter("motion_guards_enabled").as_bool())
            throw std::invalid_argument(
                "Expected 200 Hz reference, <=50 ms timeout, diagnostic-only motion guards");
        diagnostic_speed_ = get_parameter("diagnostic_speed_rad_s").as_double();
        diagnostic_acceleration_ = get_parameter("diagnostic_acceleration_rad_s2").as_double();
        diagnostic_error_ = get_parameter("diagnostic_error_rad").as_double();
        diagnostic_angle_ = get_parameter("diagnostic_angle_rad").as_double();
        diagnostic_braking_ = get_parameter("diagnostic_braking_rad_s2").as_double();
        for (double value :
             {diagnostic_speed_, diagnostic_acceleration_, diagnostic_error_, diagnostic_angle_,
              diagnostic_braking_})
            if (!std::isfinite(value) || value <= 0)
                throw std::invalid_argument("Invalid diagnostic threshold");
        beta_min_ = get_parameter("beta_min_deg").as_double() * PairGeometry::rad;
        beta_max_ = get_parameter("beta_max_deg").as_double() * PairGeometry::rad;
        if (!(beta_min_ < beta_max_))
            throw std::invalid_argument("Invalid beta range");
        for (const auto& g : geometry_) {
            g.at(beta_min_);
            g.at(beta_max_);
        }
        if (kind_ == 1) {
            WheelProbeConfig c;
            copy("wheel_low_speeds", c.low_speeds);
            copy("wheel_speeds", c.speeds);
            c.ramp_s = get_parameter("wheel_ramp_s").as_double();
            c.hold_s = get_parameter("wheel_hold_s").as_double();
            c.coast_s = get_parameter("wheel_coast_s").as_double();
            c.step_hold_s = get_parameter("wheel_step_hold_s").as_double();
            c.chirp_s = get_parameter("wheel_chirp_s").as_double();
            c.validation_s = get_parameter("wheel_validation_s").as_double();
            c.step_repetitions = get_parameter("wheel_step_repetitions").as_int();
            wheel_plan_ = std::make_unique<WheelProbePlan>(c);
            wheel_kp_ = get_parameter("wheel_velocity_kp").as_double();
            const double amps = get_parameter("wheel_current_limit_a").as_double();
            if (!std::isfinite(amps) || amps <= 0 || amps > 20 || !std::isfinite(wheel_kp_)
                || wheel_kp_ <= 0)
                throw std::invalid_argument("Invalid wheel gain/current cap");
            wheel_cap_ = amps * wheel_nm_per_amp;
        } else {
            copy("joint_kp", kp_);
            copy("joint_kd", kd_);
            copy("joint_torque_limit_nm", torque_cap_);
            for (std::size_t i = 0; i < 2; ++i)
                if (kp_[i] <= 0 || kd_[i] < 0 || torque_cap_[i] <= 0 || torque_cap_[i] > 40)
                    throw std::invalid_argument("Invalid joint PD/torque cap");
            joint_plan_ =
                std::make_unique<WheelLegJointSweepPlan>(geometry_[side_], hip_sign_[side_]);
            if (kind_ == 2) {
                const double h = hip_zero_[side_];
                const std::array initial{
                    h, h + geometry_[side_].at(70 * PairGeometry::rad).position};
                for (int mode : {1, 2, 3})
                    joint_plan_->start(mode, initial, h, beta_min_, beta_max_);
            }
            if (kind_ == 3) {
                WheelLegLimitProbe::Config c;
                c.low = beta_min_;
                c.high = beta_max_;
                c.speed = get_parameter("probe_speed_deg_s").as_double() * PairGeometry::rad;
                c.retract = get_parameter("probe_retract_deg").as_double() * PairGeometry::rad;
                c.window = get_parameter("probe_stall_window_s").as_double();
                c.motion = get_parameter("probe_stall_motion_deg").as_double() * PairGeometry::rad;
                c.effort = get_parameter("probe_stall_effort_nm").as_double();
                if (c.effort >= torque_cap_[1])
                    throw std::invalid_argument("Probe stall threshold >= torque cap");
                probe_ = std::make_unique<WheelLegLimitProbe>(geometry_[side_], c);
            }
        }
    }

    static std::uint64_t steady_ns() {
        return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                              std::chrono::steady_clock::now().time_since_epoch())
                                              .count());
    }
    std::array<double, 2> selected_angles() const {
        return {*model_q_[2 * side_], *model_q_[2 * side_ + 1]};
    }
    bool fresh(std::uint64_t stamp, std::uint64_t now) const {
        return stamp && stamp <= now && static_cast<double>(now - stamp) * 1e-9 < feedback_timeout_;
    }

    SweepStop sensor_health(std::uint64_t now) const {
        if (!*feedback_fresh_ || !fresh(*imu_ns_, now) || !fresh(*acceleration_ns_, now))
            return SweepStop::kFeedbackTimeout;
        if (!quaternion_->coeffs().allFinite() || quaternion_->squaredNorm() < 1e-12
            || !gyro_->allFinite() || !acceleration_->allFinite())
            return SweepStop::kNonfinite;
        if (session_.state == SweepState::kResetReady && *left_ == rmcs_msgs::Switch::MIDDLE
            && *right_ == rmcs_msgs::Switch::MIDDLE)
            for (const auto& status : status_)
                if (*status == 1)
                    return SweepStop::kEnableLost; // Require the DOWN disable acknowledgement.
        for (std::size_t i = 0; i < 6; ++i) {
            if (!fresh(*feedback_ns_[i], now) || !*feedback_sequence_[i])
                return SweepStop::kFeedbackTimeout;
            if (!std::isfinite(*angle_[i]) || !std::isfinite(*velocity_[i])
                || !std::isfinite(*feedback_torque_[i]) || !std::isfinite(*max_torque_[i])
                || *max_torque_[i] <= 0 || !std::isfinite(*model_q_[i]))
                return SweepStop::kNonfinite;
            if (i < 4 && (*fault_[i] != 0 || (*status_[i] != 0 && *status_[i] != 1))) {
                // Only the initial selected-pair arm can clear a recoverable drive
                // fault. Any fault after acknowledgement immediately latches stop.
                const bool initial_clear =
                    kind_ != 1 && static_cast<int>(i / 2) == side_ && !session_.ready_seen()
                    && *fault_[i] >= 8 && *fault_[i] <= 14 && *status_[i] == *fault_[i]
                    && *left_ == rmcs_msgs::Switch::MIDDLE && *right_ == rmcs_msgs::Switch::MIDDLE;
                if (!initial_clear)
                    return SweepStop::kDriveFault;
            }
        }
        return SweepStop::kNone;
    }

    bool drives_ready() const {
        for (std::size_t i = 0; i < 4; ++i) {
            const bool active = kind_ != 1 && static_cast<int>(i / 2) == side_;
            if (*fault_[i] != 0 || *status_[i] != (active ? 1 : 0))
                return false;
        }
        return true;
    }

    void update_feedback() {
        for (std::size_t i = 0; i < 6; ++i) {
            const double phase =
                i < 4 ? model_sign_[i] * (*angle_[i]) + model_offset_[i] : *angle_[i];
            const double speed = i < 4 ? model_sign_[i] * (*velocity_[i]) : *velocity_[i];
            *model_dq_[i] = speed;
            double acceleration = *model_ddq_[i];
            if (*feedback_sequence_[i] != last_sequence_[i]) {
                if (i < 4 && have_phase_[i] && std::isfinite(phase))
                    *model_q_[i] +=
                        std::remainder(phase - previous_phase_[i], 2 * std::numbers::pi);
                else
                    *model_q_[i] = phase;
                if (last_feedback_ns_[i] && *feedback_ns_[i] > last_feedback_ns_[i])
                    acceleration =
                        (speed - previous_velocity_[i])
                        / (static_cast<double>(*feedback_ns_[i] - last_feedback_ns_[i]) * 1e-9);
                previous_phase_[i] = phase;
                previous_velocity_[i] = speed;
                last_feedback_ns_[i] = *feedback_ns_[i];
                last_sequence_[i] = *feedback_sequence_[i];
                have_phase_[i] = std::isfinite(phase);
            }
            *model_ddq_[i] = acceleration;
            *stopping_distance_[i] = speed * speed / (2 * diagnostic_braking_);
            const double e = i < 4 ? *reference_q_[i] - *model_q_[i] : *reference_dq_[i] - speed;
            *error_[i] = e;
            *diagnostic_[i] = (std::abs(*model_q_[i]) > diagnostic_angle_ ? 1u : 0u)
                            | (std::abs(speed) > diagnostic_speed_ ? 2u : 0u)
                            | (std::abs(acceleration) > diagnostic_acceleration_ ? 4u : 0u)
                            | (std::abs(e) > diagnostic_error_ ? 8u : 0u);
        }
        for (std::size_t side = 0; side < 2; ++side) {
            const auto h = 2 * side, k = h + 1;
            if (!pair_aligned_[side] && have_phase_[h] && have_phase_[k]) {
                const double expected =
                    *model_q_[h] + geometry_[side].at(70 * PairGeometry::rad).position;
                *model_q_[k] =
                    expected + std::remainder(*model_q_[k] - expected, 2 * std::numbers::pi);
                pair_aligned_[side] = true;
            }
            const auto beta = geometry_[side].beta(*model_q_[k] - *model_q_[h]);
            *beta_estimate_[side] = beta.value_or(nan);
            *beta_valid_[side] = beta.has_value();
            *theta_estimate_[side] = (*model_q_[h] - hip_zero_[side]) / hip_sign_[side];
            if (!beta || *beta < beta_min_ || *beta > beta_max_) {
                *diagnostic_[h] |= 16u;
                *diagnostic_[k] |= 16u;
            }
        }
    }

    void apply_reference() {
        *reference_active_ = true;
        if (kind_ == 1) {
            *stage_ = wheel_sample_.segment_id;
            *waveform_ = wheel_sample_.waveform;
            *validation_ = wheel_sample_.validation;
            for (std::size_t i = 0; i < 2; ++i) {
                const auto axis = 4 + i;
                *reference_q_[axis] = nan;
                *reference_dq_[axis] = wheel_sample_.velocity_target[i];
                *error_[axis] = wheel_sample_.velocity_target[i] - *velocity_[axis];
                const double requested = wheel_sample_.servo[i] ? wheel_kp_ * (*error_[axis]) : 0;
                const double cap = std::min(wheel_cap_, *max_torque_[axis]);
                *preclip_[axis] = requested;
                *torque_[axis] = std::clamp(requested, -cap, cap);
                *saturated_[axis] = std::abs(requested) > cap;
            }
        } else {
            *stage_ = reference_.segment;
            *waveform_ = reference_.kind;
            *validation_ = reference_.validation;
            *theta_ref_ = kind_ == 3 ? (reference_.q[0] - hip_zero_[side_]) / hip_sign_[side_]
                                     : reference_.theta;
            *beta_ref_ = reference_.beta;
            *beta_ref_valid_ = reference_.beta_valid;
            for (std::size_t i = 0; i < 2; ++i) {
                const auto axis = static_cast<std::size_t>(2 * side_) + i;
                *reference_q_[axis] = reference_.q[i];
                *reference_dq_[axis] = reference_.dq[i];
                *reference_ddq_[axis] = reference_.ddq[i];
                *error_[axis] = reference_.q[i] - *model_q_[axis];
                const double model_effort =
                    kp_[i] * (*error_[axis]) + kd_[i] * (reference_.dq[i] - *model_dq_[axis]);
                const double requested = model_sign_[axis] * model_effort;
                const double cap = std::min(torque_cap_[i], *max_torque_[axis]);
                *preclip_[axis] = requested;
                *torque_[axis] = std::clamp(requested, -cap, cap);
                *saturated_[axis] = std::abs(requested) > cap;
                if (i == 1)
                    previous_effort_ = *torque_[axis];
            }
        }
    }

    InputInterface<double> update_rate_;
    InputInterface<std::size_t> tick_;
    InputInterface<std::chrono::steady_clock::time_point> scheduled_time_;
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<bool> remote_fresh_, feedback_fresh_, motor_pd_;
    InputInterface<Eigen::Quaterniond> quaternion_;
    InputInterface<Eigen::Vector3d> gyro_, acceleration_;
    InputInterface<std::uint64_t> imu_ns_, acceleration_ns_;
    std::array<InputInterface<double>, 6> angle_, velocity_, feedback_torque_, max_torque_;
    std::array<InputInterface<std::uint64_t>, 6> feedback_ns_, feedback_sequence_;
    std::array<InputInterface<int>, 4> status_, fault_;
    OutputInterface<bool> enable_, clear_, wheel_only_, completed_, drives_ready_output_,
        reference_active_, validation_, beta_ref_valid_, guards_enabled_;
    OutputInterface<int> selected_side_, experiment_output_, side_output_, state_, mode_, stage_,
        waveform_, stop_, remote_left_, remote_right_;
    OutputInterface<std::uint64_t> heartbeat_, run_, reference_sequence_, control_ns_,
        scheduled_ns_, control_end_ns_;
    OutputInterface<double> reference_elapsed_, run_elapsed_, control_dt_, theta_ref_, beta_ref_;
    std::array<OutputInterface<double>, 6> torque_, model_q_, model_dq_, model_ddq_,
        stopping_distance_, reference_q_, reference_dq_, reference_ddq_, error_, preclip_;
    std::array<OutputInterface<bool>, 6> saturated_;
    std::array<OutputInterface<std::uint32_t>, 6> diagnostic_;
    std::array<OutputInterface<double>, 2> beta_estimate_, theta_estimate_, probe_q_, probe_beta_;
    std::array<OutputInterface<bool>, 2> beta_valid_;
    std::array<OutputInterface<int>, 2> probe_result_;
    std::array<OutputInterface<double>, 2> host_kp_, host_kd_, host_limit_;
    OutputInterface<double> wheel_gain_, wheel_limit_;

    std::array<PairGeometry, 2> geometry_;
    WheelLegSweepSession session_;
    std::unique_ptr<WheelLegJointSweepPlan> joint_plan_;
    std::unique_ptr<WheelProbePlan> wheel_plan_;
    std::unique_ptr<WheelLegLimitProbe> probe_;
    WheelLegJointSweepPlan::Sample reference_;
    WheelProbeSample wheel_sample_;
    int kind_ = 0, side_ = -1;
    std::array<double, 4> model_sign_{}, model_offset_{};
    std::array<double, 2> hip_sign_{}, hip_zero_{}, kp_{}, kd_{}, torque_cap_{};
    double beta_min_ = 0, beta_max_ = 0, feedback_timeout_ = .05, wheel_kp_ = 0, wheel_cap_ = 0;
    double diagnostic_angle_ = 0, diagnostic_speed_ = 0, diagnostic_acceleration_ = 0,
           diagnostic_error_ = 0;
    double diagnostic_braking_ = 0;
    std::array<double, 6> previous_phase_{}, previous_velocity_{};
    std::array<bool, 6> have_phase_{};
    std::array<bool, 2> pair_aligned_{};
    std::array<std::uint64_t, 6> last_sequence_{}, last_feedback_ns_{};
    std::uint64_t last_control_ns_ = 0;
    std::size_t reference_tick_ = 0;
    double previous_effort_ = 0;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    static constexpr double wheel_nm_per_amp = 15.62 * .3 * 187 / 3591;
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegFrequencySweepController,
    rmcs_executor::Component)
