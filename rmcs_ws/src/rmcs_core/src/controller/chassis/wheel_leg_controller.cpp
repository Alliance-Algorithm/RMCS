#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Geometry>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/chassis_mode.hpp>
#include <rmcs_msgs/wheel_leg_rl_state.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>

#include "wheel_leg/joint_control.hpp"
#include "wheel_leg/jump.hpp"
#include "wheel_leg/recovery.hpp"
#include "wheel_leg/recovery_yaw.hpp"
#include "wheel_leg/support_observer.hpp"

namespace rmcs_core::controller::chassis {

class WheelLegController
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegController()
        : Node{get_component_name(), node::options()}
        , jump_(
              get_parameter("jump_apex").as_double(),
              node::param_or("jump_request_duration", wheel_leg::jump_policy::kRequestDuration)) {
        moving_jump_vx_max_ =
            node::param_or("moving_jump_vx_max", wheel_leg::jump_policy::kMovingVxMax);
        jump_vx_rate_ = node::param_or("jump_vx_rate", 2.);
        if (!std::isfinite(jump_vx_rate_) || jump_vx_rate_ <= 0.)
            throw std::invalid_argument("Jump velocity rate must be finite and positive");
        if (!std::isfinite(moving_jump_vx_max_) || moving_jump_vx_max_ < 0.
            || moving_jump_vx_max_ > wheel_leg::jump_policy::kForwardLimit)
            throw std::invalid_argument("Moving jump speed limit is outside the policy limits");
        register_input("/wheel_leg/state/position", position_);
        register_input("/wheel_leg/state/velocity", velocity_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/imu/acceleration", acceleration_);
        register_input("/gimbal/yaw/angle", recovery_yaw_);
        register_input("/wheel_leg/control_ready", control_ready_);
        register_input("/chassis/control_enable", control_enable_);
        register_input("/chassis/control_velocity", control_velocity_);
        register_input("/chassis/control_height", control_height_);
        register_input("/chassis/control_mode", control_mode_);
        register_input("/chassis/reset_count", reset_count_);
        register_input("/chassis/jump/request_count", jump_request_count_);
        register_input("/predefined/update_count", update_count_);
        register_input("/predefined/update_rate", update_rate_);
        register_input("/predefined/timestamp", timestamp_);
        register_input("/wheel_leg/rl/valid", policy_valid_);
        register_input("/wheel_leg/rl/healthy", healthy_);
        register_input("/wheel_leg/rl/action_seq", action_sequence_);
        register_input("/wheel_leg/rl/policy_rate", policy_rate_);
        register_input("/wheel_leg/rl/action_target", action_target_);
        for (std::size_t i = 0; i < wheel_leg::kJointNames.size(); ++i) {
            const auto name = std::string{wheel_leg::kJointNames[i]};
            register_input("/wheel_leg/rl/raw/" + name + "/raw", raw_action_[i]);
            register_output(
                "/wheel_leg/" + name + "/control_torque", torque_[i],
                std::numeric_limits<double>::quiet_NaN());
        }

        register_output("/wheel_leg/rl/input/enable", policy_enable_, false);
        register_output("/wheel_leg/rl/input/command", policy_command_, Eigen::Vector3d::Zero());
        register_output(
            "/wheel_leg/rl/input/height", policy_height_, wheel_leg::jump_policy::kStandingHeight);
        register_output("/wheel_leg/jump/active", jump_active_, false);
        register_output("/wheel_leg/jump/apex", jump_apex_, jump_.apex());
        register_output("/wheel_leg/jump/elapsed", jump_elapsed_, 0.);
        register_output("/wheel_leg/rl/applied_action", applied_action_, wheel_leg::JointVector{});
        register_output("/wheel_leg/rl/action_applied", action_applied_, false);
        register_output("/wheel_leg/rl/state", state_, static_cast<int>(State::kIdle));
        register_output("/wheel_leg/recovery/phase", recovery_phase_, 0);
        register_output("/wheel_leg/recovery/fault", recovery_fault_, 0);
        register_output("/wheel_leg/recovery/capture_blockers", recovery_blockers_, 0);
        register_output("/wheel_leg/recovery/capture_ready", recovery_capture_ready_, false);
        register_output("/wheel_leg/recovery/yaw_align_active", recovery_yaw_active_, false);
        register_output("/wheel_leg/recovery/direction_reversals", recovery_reversals_, 0);
        register_output("/wheel_leg/recovery/rotation_direction", recovery_direction_, 0.);
    }

    void before_updating() override {
        if (*update_rate_ != 1000. || *policy_rate_ != 50.)
            throw std::invalid_argument(
                "Wheel-leg recovery requires a 1000 Hz executor and 50 Hz policy");
        if (wheel_leg::jump_policy::kDeploymentContractSha256
                != "8ce02510cc775eb5c8ab1118aa2830d6eb8a5c2cd4c4d30a967b5c329c092102"
            || moving_jump_vx_max_ != 1.0 || jump_vx_rate_ != 2.0)
            throw std::runtime_error(
                "Compiled control parameters do not match the wheel-leg policy");
    }

    void update() override {
        if (*reset_count_ != last_reset_count_) {
            last_reset_count_ = *reset_count_;
            reset();
            fault_ = false;
        }
        const bool jump_requested = *jump_request_count_ != last_jump_request_count_;
        last_jump_request_count_ = *jump_request_count_;
        if (!*control_enable_ || fault_) {
            if (*state_ != static_cast<int>(State::kIdle))
                reset();
            return;
        }
        if (!update_state())
            return;
        update_jump(jump_requested && *state_ == static_cast<int>(State::kRunning));
        update_command();
        if (fault_)
            return;
        if (!update_action())
            return;
        if (*state_ == static_cast<int>(State::kRunning) && recovery_started_
            && -gravity_->z() < std::cos(std::numbers::pi / 4.)) {
            *recovery_fault_ = 5;
            fail("self-righting lost upright after RL takeover");
            return;
        }
        if (!last_pd_tick_ || *update_count_ - *last_pd_tick_ >= kPdDivisor) {
            if (*state_ == static_cast<int>(State::kRecovering))
                update_recovery();
            else if (have_target_)
                update_torque();
        }
    }

private:
    using State = rmcs_msgs::WheelLegRlState;
    static constexpr std::size_t kPdDivisor = 5;
    static constexpr std::size_t kActionTimeoutTicks = 60;
    static constexpr std::size_t kDriveLossTicks = 100;

    void reset() {
        recovery_started_ = false;
        support_observer_.reset();
        recovery_.reset(wheel_leg::JointReferenceRecoveryVector6::Zero(), false);
        *recovery_phase_ = *recovery_fault_ = *recovery_blockers_ = 0;
        *recovery_capture_ready_ = false;
        *recovery_reversals_ = 0;
        *recovery_direction_ = 0.;
        *recovery_yaw_active_ = false;
        last_yaw_warning_.reset();
        reset_jump();
        for (auto& output : torque_)
            *output = std::numeric_limits<double>::quiet_NaN();
        *policy_enable_ = false;
        *applied_action_ = wheel_leg::JointVector{};
        *action_applied_ = false;
        policy_command_->setZero();
        *policy_height_ = wheel_leg::jump_policy::kStandingHeight;
        *state_ = static_cast<int>(State::kIdle);
        have_target_ = false;
        accepted_target_ = pending_history_ = wheel_leg::JointVector{};
        accepted_sequence_ = *action_sequence_;
        last_jump_request_count_ = *jump_request_count_;
        ticks_since_action_ = drive_loss_ticks_ = 0;
        last_pd_tick_.reset();
        last_command_tick_.reset();
    }

    void reset_jump() {
        jump_.reset();
        jump_velocity_ = 0.;
        *jump_active_ = false;
        *jump_elapsed_ = 0.;
    }

    void fail(const char* reason) {
        node::error("Wheel-leg RL stopped: {}", reason);
        const int phase = *recovery_phase_, fault = *recovery_fault_;
        const int blockers = *recovery_blockers_;
        const bool capture_ready = *recovery_capture_ready_;
        const int reversals = *recovery_reversals_;
        const double direction = *recovery_direction_;
        reset();
        *recovery_phase_ = phase;
        *recovery_fault_ = fault;
        *recovery_blockers_ = blockers;
        *recovery_capture_ready_ = capture_ready;
        *recovery_reversals_ = reversals;
        *recovery_direction_ = direction;
        fault_ = true;
    }

    bool update_state() {
        if (*state_ == static_cast<int>(State::kIdle)) {
            *state_ = static_cast<int>(State::kWaiting);
            write_torque(wheel_leg::JointVector{});
        }
        if (*state_ == static_cast<int>(State::kWaiting)) {
            if (!*control_ready_ || !valid_feedback()) {
                *recovery_yaw_active_ = false;
                return false;
            }
            *recovery_yaw_active_ = true;
            if (!wheel_leg::recovery_yaw_aligned(*recovery_yaw_)) {
                if (!last_yaw_warning_ || *update_count_ - *last_yaw_warning_ >= *update_rate_) {
                    node::warn(
                        "Self-righting waiting for gimbal yaw alignment: yaw={} rad",
                        *recovery_yaw_);
                    last_yaw_warning_ = *update_count_;
                }
                return false;
            }
            *policy_enable_ = true;
            ticks_since_action_ = 0;
            accepted_sequence_ = *action_sequence_;
            *state_ = static_cast<int>(State::kRecovering);
            node::info("Self-righting started; RL inference runs in shadow mode");
            last_pd_tick_.reset();
            last_command_tick_.reset();
        } else if (!valid_feedback()) {
            fail("wheel-leg state estimate invalid");
            return false;
        } else if (!*control_ready_) {
            reset_jump();
            write_torque(wheel_leg::JointVector{});
            *action_applied_ = false;
            if (++drive_loss_ticks_ >= kDriveLossTicks)
                fail("drive or IMU readiness lost");
            return false;
        }
        drive_loss_ticks_ = 0;
        return true;
    }

    bool valid_feedback() const {
        return std::ranges::all_of(*position_, [](double v) { return std::isfinite(v); })
            && std::ranges::all_of(*velocity_, [](double v) { return std::isfinite(v); })
            && gravity_->allFinite() && std::abs(gravity_->squaredNorm() - 1.) <= 1e-3
            && gyro_->allFinite();
    }

    double now() const {
        return std::chrono::duration<double>(timestamp_->time_since_epoch()).count();
    }

    void update_jump(bool requested) {
        const double time = now();
        jump_.update(time);
        if (requested && have_target_ && *policy_valid_ && *healthy_
            && std::abs(control_velocity_->vector.x()) <= moving_jump_vx_max_
            && std::abs(control_velocity_->vector.y()) < 1e-6
            && !rmcs_msgs::is_spining(*control_mode_)
            && gravity_->z() < -std::cos(std::numbers::pi / 15.) && gyro_->norm() < .8
            && jump_.request(time)) {
            jump_velocity_ = control_velocity_->vector.x();
        }
        *jump_active_ = jump_.active();
        *jump_elapsed_ = jump_.elapsed(time);
    }

    void update_command() {
        const auto divisor = static_cast<std::size_t>(std::lround(*update_rate_ / *policy_rate_));
        if (last_command_tick_ && *update_count_ - *last_command_tick_ < divisor)
            return;
        last_command_tick_ = *update_count_;
        if (*state_ == static_cast<int>(State::kRecovering)) {
            policy_command_->setZero();
            *policy_height_ = wheel_leg::jump_policy::kStandingHeight;
            return;
        }
        if (!control_velocity_->vector.allFinite() || !std::isfinite(*control_height_)) {
            fail("invalid chassis command");
            return;
        }
        if (jump_.active()) {
            const double requested =
                rmcs_msgs::is_spining(*control_mode_)
                    ? 0.
                    : std::clamp(
                          control_velocity_->vector.x(), -moving_jump_vx_max_, moving_jump_vx_max_);
            const double step = jump_vx_rate_ / *policy_rate_;
            jump_velocity_ += std::clamp(requested - jump_velocity_, -step, step);
            *policy_command_ = Eigen::Vector3d{jump_velocity_, 0., 0.};
            *policy_height_ = jump_.height(now());
            return;
        }
        const double vx =
            rmcs_msgs::is_spining(*control_mode_)
                ? 0.
                : std::clamp(
                      control_velocity_->vector.x(), -wheel_leg::jump_policy::kForwardLimit,
                      wheel_leg::jump_policy::kForwardLimit);
        *policy_command_ = Eigen::Vector3d{
            vx, 0.,
            std::clamp(
                control_velocity_->vector.z(), -wheel_leg::jump_policy::kYawLimit,
                wheel_leg::jump_policy::kYawLimit)};
        const double step = wheel_leg::jump_policy::kHeightRate / *policy_rate_;
        *policy_height_ += std::clamp(
            std::clamp(
                *control_height_, wheel_leg::jump_policy::kHeightMin,
                wheel_leg::jump_policy::kHeightMax)
                - *policy_height_,
            -step, step);
    }

    bool update_action() {
        if (!*healthy_) {
            fail("inference worker failed");
            return false;
        }
        if (*policy_valid_ && *action_sequence_ != accepted_sequence_) {
            accepted_target_ = *action_target_;
            for (std::size_t i = 0; i < pending_history_.size(); ++i)
                pending_history_[i] = *raw_action_[i];
            if (!std::ranges::all_of(accepted_target_, [](double v) { return std::isfinite(v); })
                || !std::ranges::all_of(
                    pending_history_, [](double v) { return std::isfinite(v); })) {
                fail("non-finite policy action or history");
                return false;
            }
            accepted_sequence_ = *action_sequence_;
            have_target_ = true;
            ticks_since_action_ = 0;
        } else if (++ticks_since_action_ > kActionTimeoutTicks) {
            fail("policy result timeout");
            return false;
        }
        return true;
    }

    void update_recovery() {
        if (!acceleration_->allFinite()) {
            fail("invalid recovery acceleration");
            return;
        }
        auto feedback = support_observer_.observe(
            Eigen::Map<const wheel_leg::JointReferenceRecoveryVector6>{position_->data()},
            Eigen::Map<const wheel_leg::JointReferenceRecoveryVector6>{velocity_->data()},
            *gravity_, *gyro_, *acceleration_,
            recovery_started_ ? recovery_.command().phase
                              : wheel_leg::JointReferenceRecoveryPhase::kSelect);
        if (!feedback.q.allFinite()) {
            fail("invalid recovery geometry or feedback");
            return;
        }
        if (!recovery_started_) {
            recovery_.reset(feedback.q);
            recovery_started_ = true;
        }
        *recovery_blockers_ = wheel_leg::recovery_capture_blockers(
            feedback, *acceleration_, recovery_config_.nominal);
        *recovery_capture_ready_ = *recovery_blockers_ == 0;
        feedback.rl_capture_ready = *recovery_capture_ready_ && have_target_;
        const auto& command = recovery_.update(feedback);
        if (*recovery_phase_ != static_cast<int>(command.phase))
            node::info("Self-righting phase: {}", wheel_leg::recovery_phase_name(command.phase));
        *recovery_phase_ = static_cast<int>(command.phase);
        *recovery_fault_ = command.failure_code;
        if (*recovery_reversals_ != command.direction_reversals)
            node::warn(
                "Self-righting overturn stalled; reversing leg rotation (attempt {})",
                command.direction_reversals);
        *recovery_reversals_ = command.direction_reversals;
        *recovery_direction_ = command.rotation_direction;
        if (command.failed) {
            fail("self-righting script failed");
            return;
        }
        feedback.wheel_probe_torque =
            support_observer_.probe_command(command.scripted && command.release_finished);
        if (!command.pure_rl) {
            const auto effort =
                recovery_.torques(feedback, wheel_leg::JointReferenceRecoveryVector6::Zero());
            wheel_leg::JointVector torque;
            std::copy_n(effort.data(), 6, torque.begin());
            write_torque(wheel_leg::limit_torque(torque, *position_, wheel_leg::kMechanicalBounds));
            *action_applied_ = false;
            last_pd_tick_ = *update_count_;
            return;
        }
        *state_ = static_cast<int>(State::kRunning);
        *recovery_yaw_active_ = false;
        node::info("Self-righting handover: policy owns all six axes");
        last_command_tick_.reset();
        update_command();
        if (fault_)
            return;
        update_torque();
    }

    void update_torque() {
        const auto& q = *position_;
        auto target = accepted_target_;
        const auto legs = wheel_leg::paired_targets(
            {target[0], target[1], target[2], target[3]}, {q[0], q[1], q[2], q[3]},
            wheel_leg::jump_target_bounds(jump_.active()));
        std::copy(legs.begin(), legs.end(), target.begin());
        auto torque = wheel_leg::pd_torque(
            target, q, *velocity_, wheel_leg::jump_policy::kLegKp, wheel_leg::jump_policy::kLegKd);
        const auto extra = wheel_leg::tuck_torque(
            {q[0], q[1], q[2], q[3]}, legs, {torque[0], torque[1], torque[2], torque[3]},
            jump_.phase(now()) == wheel_leg::JumpCommand::Phase::kFlight);
        for (std::size_t i = 0; i < extra.size(); ++i)
            torque[i] += extra[i];
        write_torque(wheel_leg::limit_torque(torque, *position_));
        *applied_action_ = pending_history_;
        *action_applied_ = true;
        last_pd_tick_ = *update_count_;
    }

    void write_torque(const wheel_leg::JointVector& torque) {
        for (std::size_t i = 0; i < torque.size(); ++i)
            *torque_[i] = torque[i];
    }

    InputInterface<wheel_leg::JointVector> position_, velocity_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, acceleration_;
    InputInterface<double> recovery_yaw_;
    InputInterface<bool> control_ready_;
    InputInterface<bool> control_enable_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> control_velocity_;
    InputInterface<double> control_height_;
    InputInterface<rmcs_msgs::ChassisMode> control_mode_;
    InputInterface<std::size_t> reset_count_, jump_request_count_, update_count_;
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<bool> policy_valid_, healthy_;
    InputInterface<double> policy_rate_, update_rate_;
    InputInterface<std::size_t> action_sequence_;
    InputInterface<wheel_leg::JointVector> action_target_;
    std::array<InputInterface<double>, 6> raw_action_;

    std::array<OutputInterface<double>, 6> torque_;
    OutputInterface<bool> policy_enable_, action_applied_;
    OutputInterface<Eigen::Vector3d> policy_command_;
    OutputInterface<double> policy_height_;
    OutputInterface<bool> jump_active_;
    OutputInterface<double> jump_apex_, jump_elapsed_;
    OutputInterface<wheel_leg::JointVector> applied_action_;
    OutputInterface<int> state_;
    OutputInterface<bool> recovery_yaw_active_, recovery_capture_ready_;
    OutputInterface<int> recovery_phase_, recovery_fault_, recovery_blockers_;
    OutputInterface<int> recovery_reversals_;
    OutputInterface<double> recovery_direction_;

    bool fault_ = false, have_target_ = false;
    wheel_leg::JointVector accepted_target_{}, pending_history_{};
    std::size_t last_reset_count_ = 0, last_jump_request_count_ = 0, accepted_sequence_ = 0;
    std::size_t ticks_since_action_ = 0, drive_loss_ticks_ = 0;
    std::optional<std::size_t> last_pd_tick_, last_command_tick_;
    wheel_leg::JumpCommand jump_;
    const wheel_leg::JointReferenceRecoveryConfig recovery_config_ =
        wheel_leg::make_recovery_config();
    wheel_leg::JointReferenceRecoveryController recovery_{recovery_config_};
    wheel_leg::ClosedChainSupportObserver support_observer_{
        wheel_leg::recovery_leg_geometry(), .005};
    bool recovery_started_ = false;
    std::optional<std::size_t> last_yaw_warning_;
    double moving_jump_vx_max_ = wheel_leg::jump_policy::kMovingVxMax;
    double jump_velocity_ = 0.;
    double jump_vx_rate_ = 2.;
};

} // namespace rmcs_core::controller::chassis

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)
