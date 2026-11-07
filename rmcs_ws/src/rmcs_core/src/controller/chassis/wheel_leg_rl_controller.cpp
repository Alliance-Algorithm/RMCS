#include "wheel_leg/capture.hpp"
#include "wheel_leg/joint_solver.hpp"
#include "wheel_leg/policy/selection.hpp"
#include "wheel_leg/recovery_yaw.hpp"
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
static_assert(wheel_leg::kRecoveryModelSpecSha256 == kModelSpecSha256);
class WheelLegRlController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegRlController()
        : Node{get_component_name(), node::options()}
        , policy_(policy_parameters(get_parameter("policy").as_string()))
        , recovery_config_(recovery_configuration())
        , recovery_(recovery_config_)
        , support_observer_(wheel_leg::recovery_leg_geometry(), recovery_config_.dt) {
        register_input("/wheel_leg/state/position", q_);
        register_input("/wheel_leg/state/velocity", dq_);
        register_input("/wheel_leg/state/valid", valid_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/state/attitude", attitude_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/imu/acceleration", acceleration_);
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
        register_output("/wheel_leg/recovery/yaw_align_active", recovery_yaw_active_, false);
        register_output("/wheel_leg/recovery/capture_ready", recovery_capture_ready_, false);
        register_output("/wheel_leg/recovery/capture_blockers", recovery_capture_blockers_, 0);
        timeout_ = get_parameter_or("inference_timeout", .06);
        drive_loss_debounce_ =
            std::chrono::duration<double>(get_parameter_or("drive_loss_debounce", .1));
        if (!std::isfinite(drive_loss_debounce_.count()) || drive_loss_debounce_.count() < 0.)
            throw std::invalid_argument("Invalid drive loss debounce");
        heading_hold_ = get_parameter_or("heading_hold_enabled", false);
        height_rate_ = get_parameter_or("height_reference_rate_max", .6666666666666666);
        recovery_enabled_ = get_parameter_or("self_righting.enabled", false);
        if (recovery_enabled_)
            register_input("/gimbal/yaw/angle", recovery_yaw_);
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
        *recovery_yaw_active_ = recovery_enabled_ && *state_ != 3;
        if (*state_ <= 1)
            *state_ = 2;
        if (*state_ == 2) {
            if (!*valid_ || !*drives_ready_) {
                drive_loss_started_.reset();
                return;
            }
            if (recovery_enabled_ && !wheel_leg::recovery_yaw_aligned(*recovery_yaw_))
                return;
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
        // Run inference throughout recovery, as a shadow policy. Only the script
        // owns the actuators until measured dynamic capture accepts a fresh action.
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
        if (*state_ == 3 && recovery_started_ && -gravity_->z() < std::cos(std::numbers::pi / 4.)) {
            *recovery_fault_ = 5;
            fail("self-righting lost upright after RL takeover");
            return;
        }
        const bool pd_due = !last_pd_ || *tick_ - *last_pd_ >= 5;
        if (!pd_due)
            return;
        if (*state_ == 4) {
            if (!acceleration_->allFinite()) {
                fail("invalid self-righting acceleration");
                return;
            }
            auto feedback = support_observer_.observe(
                Eigen::Map<const wheel_leg::JointReferenceRecoveryVector6>{q_->data()},
                Eigen::Map<const wheel_leg::JointReferenceRecoveryVector6>{dq_->data()}, *gravity_,
                *gyro_, *acceleration_,
                recovery_started_ ? recovery_.command().phase
                                  : wheel_leg::JointReferenceRecoveryPhase::kSelect);
            if (!feedback.q.allFinite()) {
                fail("invalid self-righting feedback");
                return;
            }
            if (!recovery_started_) {
                recovery_.reset(feedback.q);
                recovery_started_ = true;
            }
            *recovery_capture_blockers_ = wheel_leg::recovery_capture_blockers(
                feedback, *acceleration_, recovery_config_.nominal);
            *recovery_capture_ready_ = *recovery_capture_blockers_ == 0;
            feedback.rl_capture_ready = *recovery_capture_ready_ && have_target_;
            const auto& recovery_command = recovery_.update(feedback);
            *recovery_phase_ = static_cast<int>(recovery_command.phase);
            *recovery_fault_ = recovery_command.failure_code;
            if (recovery_command.failed) {
                fail("self-righting script failed");
                return;
            }
            feedback.wheel_probe_torque = support_observer_.probe_command(
                recovery_command.scripted && recovery_command.release_finished);
            if (!recovery_command.pure_rl) {
                const auto native_torque =
                    recovery_.torques(feedback, wheel_leg::JointReferenceRecoveryVector6::Zero());
                JointVector torque;
                std::copy_n(native_torque.data(), 6, torque.begin());
                if (!write_effort(torque))
                    return;
                last_pd_ = *tick_;
                return;
            }
            // The actor owns all six axes in this very PD tick. Its next command
            // update also accepts driver motion, without a static standing dwell.
            *state_ = 3;
            *recovery_yaw_active_ = false;
            last_command_tick_.reset();
            update_command();
            if (*fault_)
                return;
            RCLCPP_INFO(get_logger(), "Self-righting handover: policy owns all six axes");
        }
        if (have_target_) {
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
    wheel_leg::JointReferenceRecoveryConfig recovery_configuration() {
        auto config = wheel_leg::make_recovery_config();
        config.prepare_speed_rad_s = get_parameter_or("self_righting.fold_plant_speed_rad_s", 2.);
        config.insert_speed_rad_s = get_parameter_or("self_righting.insert_speed_rad_s", 2.);
        config.orbit_speed_rad_s = get_parameter_or("self_righting.orbit_speed_rad_s", 2.);
        config.push_speed_rad_s = get_parameter_or("self_righting.push_speed_rad_s", 2.);
        config.capture_speed_rad_s = get_parameter_or("self_righting.capture_speed_rad_s", 2.);
        config.side_speed_rad_s = get_parameter_or("self_righting.side_speed_rad_s", 2.);
        if (config.prepare_speed_rad_s > 2. || config.insert_speed_rad_s > 2.
            || config.orbit_speed_rad_s > 2. || config.push_speed_rad_s > 2.
            || config.capture_speed_rad_s > 2. || config.side_speed_rad_s > 2.)
            throw std::invalid_argument("Recovery reference speed exceeds 2 rad/s");
        return config;
    }
    void reset_runtime() {
        for (auto& output : effort_)
            *output = 0.;
        *enable_ = false;
        *recovery_yaw_active_ = false;
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
        recovery_.reset(wheel_leg::JointReferenceRecoveryVector6::Zero(), false);
        support_observer_.reset();
        recovery_started_ = false;
        target_ = pending_history_ = JointVector{};
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
    InputInterface<Eigen::Vector3d> gravity_, gyro_, attitude_, acceleration_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    InputInterface<rmcs_msgs::ChassisMode> mode_;
    InputInterface<double> height_, rate_, action_valid_, healthy_, policy_rate_;
    InputInterface<double> recovery_yaw_;
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
    OutputInterface<bool> recovery_yaw_active_;
    const wheel_leg::JointReferenceRecoveryConfig recovery_config_;
    wheel_leg::JointReferenceRecoveryController recovery_;
    wheel_leg::ClosedChainSupportObserver support_observer_;
    bool recovery_started_ = false;
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
