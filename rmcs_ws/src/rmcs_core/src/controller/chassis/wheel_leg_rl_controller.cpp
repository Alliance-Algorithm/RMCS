#include "wheel_leg/joint_solver.hpp"
#include "wheel_leg/policy/selection.hpp"
#include "wheel_leg/self_righting.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <optional>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <stdexcept>
#include <string>
namespace rmcs_core::controller::chassis {
class WheelLegRlController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegRlController()
        : Node{get_component_name(), node::options()}
        , policy_(policy_parameters(get_parameter("policy").as_string()))
        , recovery_(wheel_leg::make_self_righting()) {
        register_input("/wheel_leg/state/position", q_);
        register_input("/wheel_leg/state/velocity", dq_);
        register_input("/wheel_leg/state/valid", valid_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/state/attitude", attitude_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/dm_control_ready", drives_ready_);
        register_input("/chassis/control_enable", requested_);
        register_input("/chassis/control_velocity", command_);
        register_input("/chassis/control_height", height_);
        register_input("/chassis/control_mode", mode_);
        register_input("/chassis/reset_count", reset_);
        register_input("/predefined/update_count", tick_);
        register_input("/predefined/update_rate", rate_);
        register_input("/wheel_leg/rl/valid", action_valid_);
        register_input("/wheel_leg/rl/healthy", healthy_);
        register_input("/wheel_leg/rl/action_seq", action_sequence_);
        register_input("/wheel_leg/rl/action_steady_ns", action_stamp_);
        register_input("/wheel_leg/rl/policy_rate", policy_rate_);
        register_input("/wheel_leg/rl/model_spec_sha256", model_spec_);
        register_input("/wheel_leg/rl/observation_contract_sha256", observation_sha_);
        for (int i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            register_input(prefix + "/max_torque", caps_[i]);
            register_output(prefix + "/control_torque", effort_[i], 0.);
            const auto raw = std::string{"/wheel_leg/rl/raw/"} + kMotorNames[i];
            register_input(raw, action_[i]);
            register_input(raw + "/scale", action_scale_[i]);
            register_input(raw + "/clip_min", action_min_[i]);
            register_input(raw + "/clip_max", action_max_[i]);
        }
        register_output("/wheel_leg/enable_request", enable_, false);
        register_output("/wheel_leg/rl/input/enable", policy_enable_, false);
        register_output("/wheel_leg/rl/input/command", policy_command_, Eigen::Vector3d::Zero());
        register_output("/wheel_leg/rl/input/height", policy_height_, .305);
        register_output("/wheel_leg/rl/applied_action", applied_, JointVector{});
        register_output("/wheel_leg/rl/action_applied", action_applied_, false);
        register_output("/wheel_leg/rl/state", state_, 0);
        register_output("/wheel_leg/rl/fault_latched", fault_, false);
        register_output("/wheel_leg/recovery/phase", recovery_phase_, 0);
        register_output("/wheel_leg/recovery/fault", recovery_fault_, 0);
        register_output("/wheel_leg/recovery/capture_ready", recovery_capture_ready_, false);
        register_output("/wheel_leg/recovery/capture_blockers", recovery_capture_blockers_, 0);
        recovery_.configure_speed_protection(
            get_parameter_or("self_righting_overspeed_fault_enabled", true),
            get_parameter_or("self_righting_speed_braking_enabled", true));
        timeout_ = get_parameter_or("inference_timeout", .06);
        drive_loss_debounce_ =
            std::chrono::duration<double>(get_parameter_or("drive_loss_debounce", .1));
        if (!std::isfinite(drive_loss_debounce_.count()) || drive_loss_debounce_.count() < 0.)
            throw std::invalid_argument("Invalid drive loss debounce");
        heading_hold_ = get_parameter_or("heading_hold_enabled", false);
        height_rate_ = get_parameter_or("height_reference_rate_max", .6666666666666666);
        recovery_enabled_ = get_parameter_or("self_righting.enabled", false);
        const bool sim_only = get_parameter_or("self_righting.simulation_only", true);
        const bool hardware_verified =
            get_parameter_or("self_righting.hardware_mapping_verified", false);
        simulation_guarded_ = recovery_enabled_ && (sim_only || !hardware_verified);
        if (simulation_guarded_)
            register_input("/wheel_leg/simulation", simulation_);
        if (!std::isfinite(timeout_) || timeout_ <= 0. || !std::isfinite(height_rate_)
            || height_rate_ <= 0.)
            throw std::invalid_argument("Invalid inference timeout or height rate");
    }
    void before_updating() override {
        if (*policy_rate_ != policy_.policy_hz || *model_spec_ != kModelSpecSha256
            || *observation_sha_ != policy_.observation_sha)
            throw std::invalid_argument("Policy model, cadence or observation contract mismatch");
        for (int i = 0; i < 6; ++i)
            if (*action_scale_[i] != 1. || !std::isinf(*action_min_[i])
                || !std::isinf(*action_max_[i]))
                throw std::invalid_argument(
                    "Wheel-leg adapters require unscaled raw policy actions");
        if (simulation_guarded_ && (!simulation_.ready() || !*simulation_))
            throw std::invalid_argument(
                "Self-righting requires an explicit /wheel_leg/simulation=true input until "
                "hardware mapping is verified");
    }
    void update() override {
        if (!schedule_checked_) {
            if (*rate_ != 1000.)
                throw std::invalid_argument("Wheel-leg RL requires a 1000 Hz executor");
            schedule_checked_ = true;
        }
        if (*reset_ != last_reset_) {
            reset_runtime();
            *fault_ = false;
            last_reset_ = *reset_;
        }
        if (!*requested_) {
            if (*state_ != 1)
                reset_runtime();
            *state_ = 1;
            return;
        }
        if (*fault_)
            return;
        if (*state_ >= 3 && *policy_enable_) {
            const auto now = Clock::now();
            const double since =
                have_target_
                    ? std::chrono::duration<double>(now.time_since_epoch()).count()
                          - accepted_stamp_ * 1e-9
                    : std::chrono::duration<double>(now - wait_started_.value_or(now)).count();
            if (since > .5 && !action_starved_logged_) {
                RCLCPP_WARN(
                    get_logger(),
                    "no policy action for %.2f s: state=%d valid=%d drives_ready=%d "
                    "action_valid=%.0f action_seq=%zu accepted_seq=%zu",
                    since, *state_, static_cast<int>(*valid_), static_cast<int>(*drives_ready_),
                    *action_valid_, *action_sequence_, accepted_sequence_);
                action_starved_logged_ = true;
            } else if (since <= .5)
                action_starved_logged_ = false;
        }
        *enable_ = true;
        if (*state_ <= 1)
            *state_ = 2;
        if (*state_ == 2) {
            if (!*valid_ || !*drives_ready_) {
                drive_loss_started_.reset();
                return;
            }
            *state_ = recovery_enabled_ ? 4 : 3;
            last_pd_.reset();
            last_command_tick_.reset();
        } else if (!*valid_) {
            fail("wheel-leg state estimate invalid");
            return;
        } else if (!*drives_ready_) {
            const auto now = Clock::now();
            if (!drive_loss_started_)
                drive_loss_started_ = now;
            if (std::chrono::duration<double>(now - *drive_loss_started_) < drive_loss_debounce_)
                return;
            fail("DM drive readiness lost");
            return;
        } else {
            drive_loss_started_.reset();
        }
        const bool pd_due = !last_pd_ || *tick_ - *last_pd_ >= 5;
        if (*state_ == 4) {
            if (pd_due) {
                wheel_leg::SelfRightingCore::Feedback input;
                std::copy_n(q_->begin(), 4, input.position.begin());
                std::copy_n(dq_->begin(), 4, input.velocity.begin());
                input.spring_compensation = wheel_leg::recovery_v202::compensation(input.position);
                for (int i = 0; i < 3; ++i) {
                    input.gravity[i] = (*gravity_)[i];
                    input.gyro[i] = (*gyro_)[i];
                }
                input.wheel_velocity = {(*dq_)[4], (*dq_)[5]};
                recovery_output_ = recovery_.step(input, .005, true);
                *recovery_phase_ = recovery_output_.phase;
                *recovery_fault_ = recovery_output_.fault;
                if (recovery_output_.phase == wheel_leg::SelfRightingCore::Fault) {
                    *recovery_capture_ready_ = false;
                    *recovery_capture_blockers_ = capture_blockers();
                    fail("self-righting protection");
                    return;
                }
                JointVector torque;
                std::copy_n(recovery_output_.torque.begin(), 4, torque.begin());
                torque[4] = recovery_output_.wheel_torque[0];
                torque[5] = recovery_output_.wheel_torque[1];
                if (!write_effort(torque))
                    return;
                last_pd_ = *tick_;
            }
            const bool captured = capture_ready();
            *recovery_capture_ready_ = captured;
            *recovery_capture_blockers_ = capture_blockers();
            if (!captured) {
                if (*policy_enable_)
                    fail("capture lost while awaiting policy");
                return;
            }
        }
        if (!*policy_enable_) {
            *policy_enable_ = true;
            wait_started_ = Clock::now();
            accepted_sequence_ = *action_sequence_;
        }
        update_command();
        if (*fault_)
            return;
        if (*healthy_ <= 0.) {
            fail("inference worker failed");
            return;
        }
        if (*action_valid_ > 0. && *action_sequence_ != accepted_sequence_) {
            JointVector raw;
            for (int i = 0; i < 6; ++i) {
                raw[i] = *action_[i];
                if (!std::isfinite(raw[i])) {
                    fail("non-finite policy action");
                    return;
                }
            }
            target_ = action_target(policy_, raw, *q_);
            pending_history_ = executed_history(policy_, raw);
            if (!std::ranges::all_of(target_, [](double v) { return std::isfinite(v); })) {
                fail("invalid action target");
                return;
            }
            accepted_sequence_ = *action_sequence_;
            accepted_stamp_ = *action_stamp_;
            have_target_ = true;
            if (*state_ == 4) {
                *state_ = 3;
                last_pd_.reset();
                *recovery_capture_ready_ = false;
                *recovery_capture_blockers_ = 0;
            }
        }
        const auto now = Clock::now();
        const double age = have_target_
                             ? std::chrono::duration<double>(now.time_since_epoch()).count()
                                   - accepted_stamp_ * 1e-9
                             : std::chrono::duration<double>(now - *wait_started_).count();
        if (age > timeout_ || age < 0.) {
            fail("policy result timeout");
            return;
        }
        if (have_target_ && (!last_pd_ || *tick_ - *last_pd_ >= 5)) {
            const auto torque = JointSolver::pd(
                target_, *q_, *dq_, policy_.leg_kp, policy_.leg_kd, policy_.gas_gain);
            if (!write_effort(torque))
                return;
            *applied_ = pending_history_;
            *action_applied_ = true;
            last_pd_ = *tick_;
        }
    }

private:
    using Clock = std::chrono::steady_clock;
    void reset_runtime() {
        for (auto& output : effort_)
            *output = 0.;
        *enable_ = false;
        *policy_enable_ = false;
        *applied_ = JointVector{};
        *action_applied_ = false;
        *policy_command_ = Eigen::Vector3d::Zero();
        *policy_height_ = .305;
        *state_ = 1;
        *recovery_phase_ = 0;
        *recovery_fault_ = 0;
        *recovery_capture_ready_ = false;
        *recovery_capture_blockers_ = 0;
        have_target_ = false;
        accepted_sequence_ = *action_sequence_;
        accepted_stamp_ = 0;
        last_pd_.reset();
        last_command_tick_.reset();
        wait_started_.reset();
        drive_loss_started_.reset();
        action_starved_logged_ = false;
        heading_.reset();
        recovery_.reset();
    }
    void fail(const char* reason) {
        RCLCPP_ERROR(get_logger(), "Wheel-leg RL stopped: %s", reason);
        const int phase = *recovery_phase_, fault = *recovery_fault_;
        const bool capture_ready = *recovery_capture_ready_;
        const int capture_blockers = *recovery_capture_blockers_;
        reset_runtime();
        *fault_ = true;
        *recovery_phase_ = phase;
        *recovery_fault_ = fault;
        *recovery_capture_ready_ = capture_ready;
        *recovery_capture_blockers_ = capture_blockers;
    }
    bool write_effort(const JointVector& effort) {
        JointVector caps;
        for (int i = 0; i < 6; ++i)
            caps[i] = *caps_[i];
        if (!std::ranges::all_of(effort, [](double v) { return std::isfinite(v); })
            || !std::ranges::all_of(caps, [](double v) { return std::isfinite(v) && v > 0.; })) {
            fail("invalid actuator effort or limits");
            return false;
        }
        const auto api = JointSolver::constrain(effort, *q_, caps);
        for (int i = 0; i < 6; ++i)
            *effort_[i] = api[i];
        return true;
    }
    static constexpr int kCapturePhase = 1 << 0;
    static constexpr int kCaptureGravity = 1 << 1;
    static constexpr int kCaptureGyro = 1 << 2;
    static constexpr int kCaptureWheelVelocity = 1 << 3;
    static constexpr int kCaptureLegVelocity = 1 << 4;
    static constexpr int kCaptureTarget = 1 << 5;
    int capture_blockers() const {
        int blockers = 0;
        if (recovery_output_.phase != wheel_leg::SelfRightingCore::Complete)
            blockers |= kCapturePhase;
        if (gravity_->z() >= -.9903)
            blockers |= kCaptureGravity;
        if (gyro_->head<2>().norm() >= .35)
            blockers |= kCaptureGyro;
        const auto target = action_target(policy_, JointVector{}, *q_);
        for (int i = 0; i < 6; ++i)
            if (std::abs((*dq_)[i]) >= (i < 4 ? .6 : 2.))
                blockers |= i < 4 ? kCaptureLegVelocity : kCaptureWheelVelocity;
        for (int i = 0; i < 4; ++i)
            if (std::abs(target[i] - (*q_)[i]) >= .25)
                blockers |= kCaptureTarget;
        return blockers;
    }
    bool capture_ready() const {
        return capture_blockers() == 0;
    }
    void update_command() {
        const std::size_t divisor = static_cast<std::size_t>(1000. / policy_.policy_hz);
        if (last_command_tick_ && *tick_ - *last_command_tick_ < divisor)
            return;
        last_command_tick_ = *tick_;
        if (*state_ == 4) {
            *policy_command_ = Eigen::Vector3d::Zero();
            *policy_height_ = .305;
            return;
        }
        if (!command_->vector.allFinite() || !std::isfinite(*height_)) {
            fail("invalid chassis command");
            return;
        }
        const bool spin = rmcs_msgs::is_spining(*mode_);
        double vx =
            spin ? 0.
                 : std::clamp(command_->vector.x(), -policy_.forward_limit, policy_.forward_limit);
        double yaw = command_->vector.z();
        if (heading_hold_ && !spin && vx != 0. && yaw == 0.) {
            if (!heading_)
                heading_ = attitude_->z();
            yaw = std::clamp(
                5. * std::remainder(*heading_ - attitude_->z(), 2. * std::numbers::pi),
                -policy_.yaw_limit, policy_.yaw_limit);
        } else
            heading_.reset();
        *policy_command_ =
            Eigen::Vector3d{vx, 0., std::clamp(yaw, -policy_.yaw_limit, policy_.yaw_limit)};
        const double step = height_rate_ / policy_.policy_hz;
        *policy_height_ +=
            std::clamp(std::clamp(*height_, .23, .42) - *policy_height_, -step, step);
    }
    const PolicyParameters& policy_;
    InputInterface<JointVector> q_, dq_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, attitude_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    InputInterface<rmcs_msgs::ChassisMode> mode_;
    InputInterface<double> height_, rate_, action_valid_, healthy_, policy_rate_;
    InputInterface<bool> valid_, drives_ready_, requested_, simulation_;
    InputInterface<std::size_t> reset_, tick_, action_sequence_;
    InputInterface<std::uint64_t> action_stamp_;
    InputInterface<std::string> model_spec_, observation_sha_;
    std::array<InputInterface<double>, 6> caps_, action_, action_scale_, action_min_, action_max_;
    std::array<OutputInterface<double>, 6> effort_;
    OutputInterface<bool> enable_, policy_enable_, fault_, action_applied_;
    OutputInterface<JointVector> applied_;
    OutputInterface<Eigen::Vector3d> policy_command_;
    OutputInterface<double> policy_height_;
    OutputInterface<int> state_, recovery_phase_, recovery_fault_, recovery_capture_blockers_;
    OutputInterface<bool> recovery_capture_ready_;
    wheel_leg::SelfRightingCore recovery_;
    wheel_leg::SelfRightingCore::Output recovery_output_;
    JointVector target_{}, pending_history_{};
    std::optional<std::size_t> last_pd_, last_command_tick_;
    std::optional<Clock::time_point> wait_started_, drive_loss_started_;
    std::optional<double> heading_;
    std::size_t last_reset_ = 0, accepted_sequence_ = 0;
    std::uint64_t accepted_stamp_ = 0;
    std::chrono::duration<double> drive_loss_debounce_{.1};
    bool action_starved_logged_ = false;
    double timeout_, height_rate_;
    bool heading_hold_, recovery_enabled_, simulation_guarded_ = false, have_target_ = false,
        schedule_checked_ = false;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::chassis::WheelLegRlController, rmcs_executor::Component)
