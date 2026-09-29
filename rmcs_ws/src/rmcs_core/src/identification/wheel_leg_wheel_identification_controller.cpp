#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Dense>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "identification/wheel_leg_arm_dwell.hpp"
#include "identification/wheel_leg_identification_axes.hpp"
#include "identification/wheel_leg_identification_phase.hpp"
#include "identification/wheel_leg_wheel_identification_plan.hpp"

namespace rmcs_core::controller::identification {

class WheelLegWheelIdentificationController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
    using Clock = std::chrono::steady_clock;
    using Phase = IdentificationPhase;

public:
    WheelLegWheelIdentificationController()
        : Node{get_component_name(), node::options()}
        , plan_(load_plan()) {
        load_parameters();
        register_interfaces();
        RCLCPP_INFO(
            get_logger(),
            "WHEEL_MULTIBAND_V2: left, right, both; %.1f s / %zu segments; "
            "50 Hz target, 1000 Hz P=%.3f; C620 current <= %.1f A (API %.5f Nm)",
            plan_.duration(), plan_.segments().size(), kp_, current_limit_a_, torque_cap_);
        RCLCPP_INFO(
            get_logger(), "All DM drives remain disabled; airborne carrier motion is recorded for "
                          "model qualification");
    }

    void update() override {
        neutral();
        const auto now = *timestamp_;
        using rmcs_msgs::Switch;
        const auto left = *switch_left_, right = *switch_right_;
        const bool down = *dr16_fresh_ && left == Switch::DOWN && right == Switch::DOWN;
        const bool middle = *dr16_fresh_ && left == Switch::MIDDLE && right == Switch::MIDDLE;
        const bool armed = arm_dwell_.update(now, *dr16_fresh_, left, right, arm_dwell_s_);
        if (down) {
            reset_on_both_down();
            return;
        }
        if (!update_clock(now))
            return;
        if (!middle) {
            if (phase() == Phase::kPreparing || phase() == Phase::kRunning)
                fail(3, "Remote lost double-MIDDLE");
            return;
        }
        if (phase() == Phase::kFailed || phase() == Phase::kComplete)
            return;
        if (phase() == Phase::kIdle) {
            if (!armed) {
                fail(3, "Both DOWN dwell required before wheel identification");
                return;
            }
            set_phase(Phase::kPreparing);
            start_ = now;
            last_logged_segment_ = -1;
            RCLCPP_INFO(get_logger(), "Both MIDDLE: preparing wheel zero-current enable");
        }
        if (!*feedback_fresh_) {
            fail(4, "DM, wheel or IMU feedback stale");
            return;
        }
        for (std::size_t i = 0; i < 4; ++i)
            if (*dm_status_[i] != 0 || *dm_fault_[i] != 0) {
                fail(5, "All four leg DM drives must remain disabled");
                return;
            }
        const auto steady_now = Clock::now().time_since_epoch();
        const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(steady_now);
        const auto now_ns = steady_ns.count();
        for (std::size_t i = 0; i < 2; ++i) {
            const auto stamp = *wheel_feedback_ns_[i];
            if (!stamp || now_ns < 0 || stamp > static_cast<std::uint64_t>(now_ns)
                || (static_cast<std::uint64_t>(now_ns) - stamp) * 1e-9 > feedback_timeout_s_
                || !std::isfinite(*wheel_speed_[i]) || std::abs(*wheel_speed_[i]) > speed_cap_
                || !std::isfinite(*wheel_max_torque_[i])
                || std::abs(*wheel_max_torque_[i] - 20 * kOutputNmPerAmp) > 1e-9) {
                fail(6, "Wheel feedback/torque limit invalid");
                return;
            }
        }
        *enable_request_ = true;
        if (phase() == Phase::kPreparing) {
            // Submit zero current first. The board command partner gates wheels
            // independently while holding all four DM drives disabled.
            for (std::size_t i = 0; i < 2; ++i)
                *torque_[4 + i] = 0.0;
            if (std::chrono::duration<double>(now - start_).count() < .15)
                return;
            set_phase(Phase::kRunning);
            run_start_ = now;
            RCLCPP_INFO(get_logger(), "Wheel experiment: first zero-current frame submitted");
        }
        const double elapsed = std::chrono::duration<double>(now - run_start_).count();
        if (elapsed >= plan_.duration()) {
            set_phase(Phase::kComplete);
            *enable_request_ = false;
            neutral();
            RCLCPP_INFO(get_logger(), "Wheel identification complete: all outputs zero current");
            return;
        }
        const auto sample = plan_.held_at(elapsed);
        *segment_id_ = sample.segment_id;
        *segment_role_ = sample.validation ? 1 : 0;
        *segment_waveform_ = sample.waveform;
        *wheel_mode_ = sample.mode;
        if (sample.segment_id != last_logged_segment_) {
            last_logged_segment_ = sample.segment_id;
            const auto& segment = plan_.segments()[sample.segment_id];
            RCLCPP_INFO(
                get_logger(), "Wheel segment %d/%zu: %s (%.2f s)", sample.segment_id,
                plan_.segments().size(), segment.label.c_str(), segment.duration_s);
        }
        for (std::size_t i = 0; i < 2; ++i) {
            *wheel_target_[i] = sample.velocity_target[i];
            if (!sample.servo[i]) {
                *torque_[4 + i] = 0.0; // ESC still powered: zero-current coast
                (*preclip_api_)[static_cast<Eigen::Index>(4 + i)] = 0.0;
            } else {
                const double requested = kp_ * (sample.velocity_target[i] - *wheel_speed_[i]);
                (*preclip_api_)[static_cast<Eigen::Index>(4 + i)] = requested;
                *torque_[4 + i] = std::clamp(requested, -torque_cap_, torque_cap_);
            }
        }
    }

private:
    void reset_on_both_down() {
        // A stop must release the outputs and forget the old clock even after
        // a skipped tick or clock rollback. Keep the terminal stop sample and
        // require a new DOWN dwell before a later MIDDLE can restart the run.
        have_clock_ = false;
        if (phase() == Phase::kPreparing || phase() == Phase::kRunning)
            fail(3, "Both DOWN: zero wheel current and stop experiment");
        else if (phase() == Phase::kComplete || phase() == Phase::kFailed) {
            *repetition_id_output_ = ++repetition_id_;
            set_phase(Phase::kIdle);
            *failure_reason_ = 0;
        }
    }

    bool update_clock(Clock::time_point now) {
        if (!std::isfinite(*update_rate_) || *update_rate_ != 1000.) {
            fail(1, "Wheel experiment requires 1000 Hz executor");
            return false;
        }
        if (have_clock_) {
            const double gap = std::chrono::duration<double>(now - last_time_).count();
            if (*tick_ != last_tick_ + 1 || gap <= 0 || gap > max_tick_gap_s_) {
                fail(2, "Wheel experiment clock discontinuity");
                return false;
            }
        }
        have_clock_ = true;
        last_tick_ = *tick_;
        last_time_ = now;
        return true;
    }

    void load_parameters() {
        kp_ = get_parameter("wheel_velocity_kp").as_double();
        const double ki = get_parameter("wheel_velocity_ki").as_double();
        const double kd = get_parameter("wheel_velocity_kd").as_double();
        const double feedforward = get_parameter("wheel_feedforward").as_double();
        torque_cap_ = get_parameter("wheel_torque_cap").as_double();
        current_limit_a_ = get_parameter("wheel_current_limit_a").as_double();
        const double ratio = get_parameter("wheel_reduction_ratio").as_double();
        const double control_hz = get_parameter("control_frequency_hz").as_double();
        const double reference_hz = get_parameter("reference_frequency_hz").as_double();
        speed_cap_ = get_parameter("wheel_feedback_speed_cap").as_double();
        arm_dwell_s_ = get_parameter("arm_dwell_s").as_double();
        feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
        max_tick_gap_s_ = get_parameter("max_tick_gap_s").as_double();
        const auto finite = [](double value) { return std::isfinite(value); };
        if (!finite(kp_) || !finite(ki) || !finite(kd) || !finite(feedforward)
            || !finite(torque_cap_) || !finite(speed_cap_) || !finite(arm_dwell_s_)
            || !finite(feedback_timeout_s_) || !finite(max_tick_gap_s_) || !finite(current_limit_a_)
            || current_limit_a_ <= 0 || current_limit_a_ > 20 || ratio != 15.8 || control_hz != 1000
            || reference_hz != WheelProbePlan::kReferenceHz || kp_ <= 0 || ki != 0 || kd != 0
            || feedforward != 0 || torque_cap_ <= 0
            || std::abs(torque_cap_ - current_limit_a_ * kOutputNmPerAmp) > 1e-9
            || speed_cap_ <= plan_.max_target_speed() || arm_dwell_s_ < 0.1
            || feedback_timeout_s_ <= 0 || feedback_timeout_s_ > .05 || max_tick_gap_s_ <= .001
            || max_tick_gap_s_ > .02)
            throw std::invalid_argument("Invalid M3508 wheel P/current/frequency contract");
        if (has_parameter("wheel_trajectory_manifest_json"))
            throw std::invalid_argument("Generated wheel manifest cannot be overridden");
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        declare_parameter("wheel_trajectory_manifest_json", plan_.manifest_json(), descriptor);
    }

    void register_interfaces() {
        register_input("/predefined/update_count", tick_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/wheel_leg/feedback_fresh", feedback_fresh_);
        register_input("/wheel_leg/dr16_fresh", dr16_fresh_);
        for (std::size_t i = 0; i < 4; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_input(prefix + "/status_code", dm_status_[i]);
            register_input(prefix + "/fault_code", dm_fault_[i]);
        }
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kAxisNames[i];
            register_output(prefix + "/control_torque", torque_[i], nan);
            if (i >= 4) {
                register_input(prefix + "/velocity", wheel_speed_[i - 4]);
                register_input(prefix + "/max_torque", wheel_max_torque_[i - 4]);
                register_input(prefix + "/feedback_steady_ns", wheel_feedback_ns_[i - 4]);
                register_output(prefix + "/control_velocity", wheel_target_[i - 4], nan);
            }
        }
        register_output("/wheel_leg/identification/wheel_only", wheel_only_, true);
        register_output("/wheel_leg/enable_request", enable_request_, false);
        register_output(
            "/wheel_leg/identification/phase", phase_, std::to_underlying(Phase::kIdle));
        register_output("/wheel_leg/identification/segment_id", segment_id_, -1);
        register_output("/wheel_leg/identification/failure_reason", failure_reason_, 0);
        register_output(
            "/wheel_leg/identification/repetition_id", repetition_id_output_, std::uint32_t{0});
        register_output("/wheel_leg/identification/segment_role", segment_role_, std::uint8_t{2});
        register_output(
            "/wheel_leg/identification/segment_waveform", segment_waveform_, std::uint8_t{0});
        register_output("/wheel_leg/identification/wheel_mode", wheel_mode_, std::uint8_t{0});
        register_output(
            "/wheel_leg/identification/torque_preclip_api", preclip_api_,
            Eigen::Matrix<double, 6, 1>::Constant(nan));
    }

    [[nodiscard]] Phase phase() const { return static_cast<Phase>(*phase_); }
    void set_phase(Phase value) { *phase_ = std::to_underlying(value); }

    // Must match DjiMotor's M3508 conversion and the actual installed ratio.
    static constexpr double kOutputNmPerAmp = 15.8 * (0.3 * 187. / 3591.);
    WheelProbePlan load_plan() {
        WheelProbeConfig config;
        if (get_parameter("trajectory_revision").as_string() != WheelProbePlan::kRevision)
            throw std::invalid_argument("Unknown wheel trajectory revision");
        const auto speeds = get_parameter("wheel_speeds").as_double_array();
        const auto low_speeds = get_parameter("wheel_low_speeds").as_double_array();
        if (speeds.size() != config.speeds.size() || low_speeds.size() != config.low_speeds.size())
            throw std::invalid_argument("Wheel plan requires five platform and three low speeds");
        std::copy(speeds.begin(), speeds.end(), config.speeds.begin());
        std::copy(low_speeds.begin(), low_speeds.end(), config.low_speeds.begin());
        config.ramp_s = get_parameter("wheel_ramp_s").as_double();
        config.hold_s = get_parameter("wheel_hold_s").as_double();
        config.coast_s = get_parameter("wheel_coast_s").as_double();
        config.step_hold_s = get_parameter("wheel_step_hold_s").as_double();
        config.step_repetitions = get_parameter("wheel_step_repetitions").as_int();
        config.chirp_s = get_parameter("wheel_chirp_s").as_double();
        config.validation_s = get_parameter("wheel_validation_s").as_double();
        return WheelProbePlan{config};
    }

    void neutral() {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        for (auto& command : torque_)
            *command = nan;
        for (auto& target : wheel_target_)
            *target = nan;
        *preclip_api_ = Eigen::Matrix<double, 6, 1>::Constant(nan);
        *enable_request_ = false;
        *wheel_mode_ = 0;
        *segment_role_ = 2;
        *segment_waveform_ = 0;
        *segment_id_ = -1;
    }

    void fail(int code, const char* message) {
        if (phase() != Phase::kFailed)
            RCLCPP_ERROR(get_logger(), "Wheel experiment stopped (%d): %s", code, message);
        set_phase(Phase::kFailed);
        *failure_reason_ = code;
        arm_dwell_.reset();
        neutral();
    }

    WheelProbePlan plan_;
    double current_limit_a_ = 0;
    int last_logged_segment_ = -1;
    double kp_ = 0, torque_cap_ = 0, speed_cap_ = 0;
    double arm_dwell_s_ = 0, feedback_timeout_s_ = 0, max_tick_gap_s_ = 0;
    InputInterface<std::size_t> tick_;
    InputInterface<double> update_rate_;
    InputInterface<Clock::time_point> timestamp_;
    InputInterface<rmcs_msgs::Switch> switch_left_, switch_right_;
    InputInterface<bool> dr16_fresh_, feedback_fresh_;
    std::array<InputInterface<int>, 4> dm_status_, dm_fault_;
    std::array<InputInterface<double>, 2> wheel_speed_, wheel_max_torque_;
    std::array<InputInterface<std::uint64_t>, 2> wheel_feedback_ns_;
    std::array<OutputInterface<double>, 6> torque_;
    std::array<OutputInterface<double>, 2> wheel_target_;
    OutputInterface<bool> enable_request_, wheel_only_;
    OutputInterface<int> phase_, segment_id_, failure_reason_;
    OutputInterface<std::uint8_t> segment_role_, segment_waveform_, wheel_mode_;
    OutputInterface<std::uint32_t> repetition_id_output_;
    OutputInterface<Eigen::Matrix<double, 6, 1>> preclip_api_;
    std::uint32_t repetition_id_ = 0;
    WheelLegArmDwell arm_dwell_;
    std::size_t last_tick_ = 0;
    bool have_clock_ = false;
    Clock::time_point last_time_{}, start_{}, run_start_{};
};

} // namespace rmcs_core::controller::identification

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::identification::WheelLegWheelIdentificationController,
    rmcs_executor::Component)
