#include "wheel_leg/balance_solver.hpp"
#include "wheel_leg/joint_solver.hpp"
#include "wheel_leg/leg_kinematics.hpp"
#include "wheel_leg/model/mechanics.hpp"
#include "wheel_leg/motion_planner.hpp"
#include "wheel_leg/self_righting.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_utility/rclcpp/node_mixin.hpp>
#include <stdexcept>
#include <string>
namespace rmcs_core::controller::chassis {
class WheelLegController final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public rmcs_utility::NodeMixin {
public:
    WheelLegController()
        : Node{get_component_name(), node::options()}
        , leg_model_(
              wheel_leg::mechanics::kThighLength, wheel_leg::mechanics::kShankLength,
              wheel_leg::LegLinkage::knee_limit_min(), wheel_leg::LegLinkage::knee_limit_max(),
              linkage_)
        , recovery_(wheel_leg::make_self_righting()) {
        register_input("/wheel_leg/state/position", q_);
        register_input("/wheel_leg/state/velocity", dq_);
        register_input("/wheel_leg/state/valid", valid_);
        register_input("/wheel_leg/state/gravity", gravity_);
        register_input("/wheel_leg/state/attitude", attitude_);
        register_input("/wheel_leg/imu/angular_velocity", gyro_);
        register_input("/wheel_leg/state/passive_knee", knee_);
        register_input("/wheel_leg/state/passive_knee_rate", knee_rate_);
        register_input("/wheel_leg/state/shank", shank_);
        register_input("/wheel_leg/state/shank_rate", shank_rate_);
        register_input("/wheel_leg/state/leg_length", length_);
        register_input("/wheel_leg/state/forward_velocity", velocity_);
        register_input("/wheel_leg/state/distance", distance_);
        register_input("/wheel_leg/state/com_offset", com_offset_);
        register_input("/wheel_leg/dm_control_ready", drives_ready_);
        register_input("/chassis/control_enable", requested_);
        register_input("/chassis/control_velocity", command_);
        register_input("/chassis/control_height", height_);
        register_input("/chassis/reset_count", reset_);
        register_input("/predefined/update_rate", rate_);
        for (int i = 0; i < 6; ++i) {
            const auto prefix = std::string{"/wheel_leg/"} + kMotorNames[i];
            register_input(prefix + "/max_torque", caps_[i]);
            register_output(prefix + "/control_torque", torque_[i], 0.);
        }
        register_output("/wheel_leg/enable_request", enable_, false);
        register_output("/wheel_leg/control/state", phase_, 0);
        register_output("/wheel_leg/control/fault", fault_, false);
        register_output("/wheel_leg/recovery/phase", recovery_phase_, 0);
        register_output("/wheel_leg/recovery/fault", recovery_fault_, 0);
        recovery_.configure_speed_protection(
            get_parameter_or("self_righting_overspeed_fault_enabled", true),
            get_parameter_or("self_righting_speed_braking_enabled", true));
        dt_ = get_parameter_or("dt", .001);
        leg_kp_ = get_parameter_or("leg_kp", 160.);
        leg_kd_ = get_parameter_or("leg_kd", 2.5);
        gas_gain_ = get_parameter_or("gas_spring_gain", 1.);
        max_tilt_ = get_parameter_or("max_tilt_deg", 60.) * std::numbers::pi / 180.;
        recovery_enabled_ = get_parameter_or("self_righting.enabled", false);
        if (recovery_enabled_
            && (get_parameter_or("self_righting.simulation_only", true)
                || !get_parameter_or("self_righting.hardware_mapping_verified", false)))
            register_input("/wheel_leg/simulation", simulation_);
        trajectory_.config.nominal_leg_length = get_parameter_or(
            "trajectory.nominal_leg_length", trajectory_.config.nominal_leg_length);
        trajectory_.config.transition_time_pitch = get_parameter_or(
            "trajectory.transition_time_pitch", trajectory_.config.transition_time_pitch);
        trajectory_.config.transition_time_shank = get_parameter_or(
            "trajectory.transition_time_shank", trajectory_.config.transition_time_shank);
        trajectory_.config.transition_time_knee = get_parameter_or(
            "trajectory.transition_time_knee", trajectory_.config.transition_time_knee);
        trajectory_.config.singularity_epsilon = get_parameter_or(
            "trajectory.singularity_epsilon", trajectory_.config.singularity_epsilon);
        trajectory_.config.leg_length_epsilon = get_parameter_or(
            "trajectory.leg_length_epsilon", trajectory_.config.leg_length_epsilon);
        trajectory_.config.startup_settle_s =
            get_parameter_or("trajectory.startup_settle_s", trajectory_.config.startup_settle_s);
        trajectory_.config.command_accel_limit = get_parameter_or(
            "trajectory.command_accel_limit", trajectory_.config.command_accel_limit);
        trajectory_.config.command_jerk_limit = get_parameter_or(
            "trajectory.command_jerk_limit", trajectory_.config.command_jerk_limit);
        trajectory_.config.command_vloop_kp =
            get_parameter_or("trajectory.command_vloop_kp", trajectory_.config.command_vloop_kp);
        trajectory_.config.command_vloop_kd =
            get_parameter_or("trajectory.command_vloop_kd", trajectory_.config.command_vloop_kd);
        trajectory_.config.leg_motion_coupling = get_parameter_or(
            "trajectory.leg_motion_coupling", trajectory_.config.leg_motion_coupling);
        trajectory_.config.outer_kx =
            get_parameter_or("trajectory.outer_kx", trajectory_.config.outer_kx);
        trajectory_.config.outer_kv =
            get_parameter_or("trajectory.outer_kv", trajectory_.config.outer_kv);
        trajectory_.config.com_height =
            get_parameter_or("trajectory.com_height", trajectory_.config.com_height);
        trajectory_.config.accel_leg_ff =
            get_parameter_or("trajectory.accel_leg_ff", trajectory_.config.accel_leg_ff);
        trajectory_.config.accel_d_kp =
            get_parameter_or("trajectory.accel_d_kp", trajectory_.config.accel_d_kp);
        trajectory_.config.accel_theta_ff =
            get_parameter_or("trajectory.accel_theta_ff", trajectory_.config.accel_theta_ff);
        trajectory_.config.accel_kv =
            get_parameter_or("trajectory.accel_kv", trajectory_.config.accel_kv);
        trajectory_.config.accel_gate_min =
            get_parameter_or("trajectory.accel_gate_min", trajectory_.config.accel_gate_min);
        trajectory_.config.d_max = get_parameter_or("trajectory.d_max", trajectory_.config.d_max);
        trajectory_.config.balance_velocity_kp = get_parameter_or(
            "trajectory.balance_velocity_kp", trajectory_.config.balance_velocity_kp);
        trajectory_.config.balance_position_kp = get_parameter_or(
            "trajectory.balance_position_kp", trajectory_.config.balance_position_kp);
        trajectory_.config.balance_integral_limit = get_parameter_or(
            "trajectory.balance_integral_limit", trajectory_.config.balance_integral_limit);

        motion_.config.max_velocity =
            get_parameter_or("motion.max_velocity", motion_.config.max_velocity);
        motion_.config.max_yaw_rate =
            get_parameter_or("motion.max_yaw_rate", motion_.config.max_yaw_rate);
        motion_.config.channel_kp =
            get_parameter_or("motion.channel_kp", motion_.config.channel_kp);
        motion_.config.velocity_accel =
            get_parameter_or("motion.velocity_accel", motion_.config.velocity_accel);
        motion_.config.velocity_jerk =
            get_parameter_or("motion.velocity_jerk", motion_.config.velocity_jerk);
        motion_.config.yaw_accel = get_parameter_or("motion.yaw_accel", motion_.config.yaw_accel);
        motion_.config.yaw_jerk = get_parameter_or("motion.yaw_jerk", motion_.config.yaw_jerk);
        motion_.config.velocity_sign =
            get_parameter_or("motion.velocity_sign", motion_.config.velocity_sign);
        motion_.config.gravity = get_parameter_or("motion.gravity", motion_.config.gravity);
        motion_.config.bank_to_roll_bias =
            get_parameter_or("motion.bank_to_roll_bias", motion_.config.bank_to_roll_bias);
        motion_.config.friction_mu =
            get_parameter_or("motion.friction_mu", motion_.config.friction_mu);
        motion_.config.track_width =
            get_parameter_or("motion.track_width", motion_.config.track_width);
        motion_.config.com_height =
            get_parameter_or("motion.com_height", motion_.config.com_height);
        motion_.config.lat_accel_safety =
            get_parameter_or("motion.lat_accel_safety", motion_.config.lat_accel_safety);
        balance_.config.pitch_cutoff_hz =
            get_parameter_or("balance.pitch_cutoff_hz", balance_.config.pitch_cutoff_hz);
        balance_.config.pitch_gain =
            get_parameter_or("balance.pitch_gain", balance_.config.pitch_gain);
        balance_.config.pitch_rate_gain =
            get_parameter_or("balance.pitch_rate_gain", balance_.config.pitch_rate_gain);
        balance_.config.velocity_gain =
            get_parameter_or("balance.velocity_gain", balance_.config.velocity_gain);
        balance_.config.distance_gain =
            get_parameter_or("balance.distance_gain", balance_.config.distance_gain);
        balance_.config.balance_limit =
            get_parameter_or("balance.balance_limit", balance_.config.balance_limit);
        balance_.config.yaw_gain = get_parameter_or("balance.yaw_gain", balance_.config.yaw_gain);
        balance_.config.yaw_rate_gain =
            get_parameter_or("balance.yaw_rate_gain", balance_.config.yaw_rate_gain);
        balance_.config.yaw_limit =
            get_parameter_or("balance.yaw_limit", balance_.config.yaw_limit);
        balance_.config.yaw_sign = get_parameter_or("balance.yaw_sign", balance_.config.yaw_sign);
        balance_.config.wheel_limit =
            get_parameter_or("balance.wheel_limit", balance_.config.wheel_limit);
        balance_.config.roll_kp = get_parameter_or("balance.roll_kp", balance_.config.roll_kp);
        balance_.config.roll_kd = get_parameter_or("balance.roll_kd", balance_.config.roll_kd);
        balance_.config.roll_limit =
            get_parameter_or("balance.roll_limit", balance_.config.roll_limit);
        balance_.config.yaw_enabled =
            get_parameter_or("balance.yaw_enabled", balance_.config.yaw_enabled);
        balance_.config.trim_enabled =
            get_parameter_or("balance.trim_enabled", balance_.config.trim_enabled);
        balance_.config.trim_sign =
            get_parameter_or("balance.trim_sign", balance_.config.trim_sign);
        balance_.config.trim_ki = get_parameter_or("balance.trim_ki", balance_.config.trim_ki);
        balance_.config.trim_tau = get_parameter_or("balance.trim_tau", balance_.config.trim_tau);
        balance_.config.trim_deadband =
            get_parameter_or("balance.trim_deadband", balance_.config.trim_deadband);
        balance_.config.trim_limit =
            get_parameter_or("balance.trim_limit", balance_.config.trim_limit);
        balance_.config.trim_pitch_limit =
            get_parameter_or("balance.trim_pitch_limit", balance_.config.trim_pitch_limit);
        balance_.config.trim_velocity_limit =
            get_parameter_or("balance.trim_velocity_limit", balance_.config.trim_velocity_limit);
        motion_.config.dt = trajectory_.config.dt = balance_.config.dt = dt_;
        if (!std::isfinite(dt_) || dt_ <= 0. || !std::isfinite(leg_kp_) || leg_kp_ <= 0.
            || !std::isfinite(leg_kd_) || leg_kd_ < 0. || !std::isfinite(gas_gain_)
            || gas_gain_ < 0. || gas_gain_ > 1.)
            throw std::invalid_argument("Invalid traditional PD or timing parameters");
        for (double value :
             {max_tilt_,
              motion_.config.max_velocity,
              motion_.config.max_yaw_rate,
              motion_.config.channel_kp,
              motion_.config.velocity_accel,
              motion_.config.velocity_jerk,
              motion_.config.yaw_accel,
              motion_.config.yaw_jerk,
              motion_.config.gravity,
              motion_.config.friction_mu,
              motion_.config.track_width,
              motion_.config.com_height,
              balance_.config.balance_limit,
              balance_.config.yaw_limit,
              balance_.config.wheel_limit,
              balance_.config.roll_limit,
              balance_.config.trim_tau,
              balance_.config.trim_limit,
              trajectory_.config.transition_time_pitch,
              trajectory_.config.transition_time_shank,
              trajectory_.config.transition_time_knee,
              trajectory_.config.leg_length_epsilon,
              trajectory_.config.command_accel_limit,
              trajectory_.config.command_jerk_limit,
              trajectory_.config.gravity,
              trajectory_.config.balance_integral_limit})
            if (!std::isfinite(value) || value <= 0.)
                throw std::invalid_argument("Invalid traditional solver span or limit");
    }
    void before_updating() override {
        if (simulation_.ready() && !*simulation_)
            throw std::invalid_argument("Self-righting remains simulation-only");
    }
    void update() override {
        if (!schedule_checked_) {
            if (std::abs(*rate_ * dt_ - 1.) > 1e-9)
                throw std::invalid_argument("Traditional control dt must match executor rate");
            schedule_checked_ = true;
        }
        if (*reset_ != last_reset_) {
            reset_runtime();
            *fault_ = false;
            last_reset_ = *reset_;
        }
        if (!*requested_) {
            if (*phase_ != 1)
                reset_runtime();
            return;
        }
        if (*fault_)
            return;
        *enable_ = true;
        if (*phase_ <= 1)
            *phase_ = 2;
        if (*phase_ == 2) {
            if (!*valid_ || !*drives_ready_)
                return;
            *phase_ = recovery_enabled_ ? 4 : 3;
        } else if (!*valid_ || !*drives_ready_) {
            fail("feedback or drive readiness lost");
            return;
        }
        if (*phase_ == 4) {
            wheel_leg::SelfRightingCore::Feedback feedback;
            std::copy_n(q_->begin(), 4, feedback.position.begin());
            std::copy_n(dq_->begin(), 4, feedback.velocity.begin());
            feedback.spring_compensation =
                wheel_leg::recovery_v202::compensation(feedback.position);
            for (int i = 0; i < 3; ++i) {
                feedback.gravity[i] = (*gravity_)[i];
                feedback.gyro[i] = (*gyro_)[i];
            }
            feedback.wheel_velocity = {(*dq_)[4], (*dq_)[5]};
            const auto output = recovery_.step(feedback, dt_, true);
            *recovery_phase_ = output.phase;
            *recovery_fault_ = output.fault;
            if (output.phase == wheel_leg::SelfRightingCore::Fault) {
                fail("self-righting protection");
                return;
            }
            JointVector effort;
            std::copy_n(output.torque.begin(), 4, effort.begin());
            effort[4] = output.wheel_torque[0];
            effort[5] = output.wheel_torque[1];
            write_effort(effort);
            if (output.phase == wheel_leg::SelfRightingCore::Complete) {
                *phase_ = 3;
                motion_.reset();
                trajectory_.reset();
                balance_.reset();
                turn_valid_ = false;
            }
            return;
        }
        if (std::abs(attitude_->x()) > max_tilt_) {
            fail("traditional tilt limit");
            return;
        }
        if (!command_->vector.allFinite() || !std::isfinite(*height_)) {
            fail("invalid chassis command");
            return;
        }
        // The main controller owns ordering and the only final torque producer.
        const auto motion = motion_.step(command_->vector.x(), command_->vector.z());
        const TrajectoryPlanner::State state{
            .wheel_velocity = .5 * ((*dq_)[4] + (*dq_)[5]),
            .pitch = attitude_->x(),
            .pitch_velocity = gyro_->y(),
            .com_offset = *com_offset_,
            .leg_length = .5 * ((*length_)[0] + (*length_)[1]),
            .velocity = *velocity_,
            .distance = *distance_,
            .knee = *knee_,
            .shank = *shank_,
            .knee_rate = *knee_rate_,
            .shank_rate = *shank_rate_};
        const auto reference = trajectory_.step(state, motion.velocity, *height_);
        const auto balance = balance_.step(
            state.pitch, gyro_->y(), state.velocity, state.distance, attitude_->z(), gyro_->z(),
            attitude_->y(), motion.yaw, motion.bank, motion.roll_feedforward, true,
            reference.balance_feedforward);
        JointVector target{};
        for (int side = 0; side < 2; ++side) {
            double hip, crank;
            if (!leg_model_.joint_targets(
                    reference.hip[side] + balance.trim,
                    reference.knee[side] + (side == 0 ? balance.roll_bias : -balance.roll_bias),
                    wheel_leg::CanonicalCalibration::side(side), hip, crank)) {
                fail("invalid leg target");
                return;
            }
            target[2 * side] = hip;
            target[2 * side + 1] = crank;
        }
        // Preserve the previous drive bridge's per-axis turn latch and hysteresis.
        JointVector api{};
        for (int i = 0; i < 4; ++i) {
            const double measured = kMotorOffsets[i] - (*q_)[i];
            if (!turn_valid_
                || std::abs(target[i] - measured + 2. * std::numbers::pi * turns_[i])
                       > 1.05 * std::numbers::pi)
                turns_[i] = std::round((measured - target[i]) / (2. * std::numbers::pi));
            api[i] = target[i] + 2. * std::numbers::pi * turns_[i];
        }
        turn_valid_ = true;
        target = model_position(api);
        auto effort = JointSolver::pd(target, *q_, *dq_, leg_kp_, leg_kd_, gas_gain_);
        effort[4] = balance.wheel[0];
        effort[5] = balance.wheel[1];
        write_effort(effort);
    }

private:
    void reset_runtime() {
        *enable_ = false;
        *phase_ = 1;
        *recovery_phase_ = 0;
        *recovery_fault_ = 0;
        for (auto& output : torque_)
            *output = 0.;
        motion_.reset();
        trajectory_.reset();
        balance_.reset();
        recovery_.reset();
        turn_valid_ = false;
    }
    void fail(const char* reason) {
        RCLCPP_ERROR(get_logger(), "Wheel-leg traditional stopped: %s", reason);
        const int p = *recovery_phase_, f = *recovery_fault_;
        reset_runtime();
        *fault_ = true;
        *recovery_phase_ = p;
        *recovery_fault_ = f;
    }
    void write_effort(const JointVector& effort) {
        JointVector caps;
        for (int i = 0; i < 6; ++i)
            caps[i] = *caps_[i];
        if (!std::ranges::all_of(effort, [](double v) { return std::isfinite(v); })
            || !std::ranges::all_of(caps, [](double v) { return std::isfinite(v) && v > 0.; })) {
            fail("invalid actuator effort or limits");
            return;
        }
        const auto api = JointSolver::constrain(effort, *q_, caps);
        for (int i = 0; i < 6; ++i)
            *torque_[i] = api[i];
    }
    InputInterface<JointVector> q_, dq_;
    InputInterface<Eigen::Vector3d> gravity_, gyro_, attitude_;
    InputInterface<std::array<double, 2>> knee_, knee_rate_, shank_, shank_rate_, length_;
    InputInterface<double> velocity_, distance_, com_offset_, height_, rate_;
    InputInterface<bool> valid_, drives_ready_, requested_, simulation_;
    InputInterface<rmcs_description::BaseLink::DirectionVector> command_;
    InputInterface<std::size_t> reset_;
    std::array<InputInterface<double>, 6> caps_;
    std::array<OutputInterface<double>, 6> torque_;
    OutputInterface<bool> enable_, fault_;
    OutputInterface<int> phase_, recovery_phase_, recovery_fault_;
    MotionPlanner motion_;
    TrajectoryPlanner trajectory_;
    BalanceSolver balance_;
    wheel_leg::LegLinkage linkage_;
    wheel_leg::WheelLegLegModel leg_model_;
    wheel_leg::SelfRightingCore recovery_;
    LegVector turns_{};
    double dt_, leg_kp_, leg_kd_, gas_gain_, max_tilt_;
    std::size_t last_reset_ = 0;
    bool recovery_enabled_, turn_valid_ = false, schedule_checked_ = false;
};
} // namespace rmcs_core::controller::chassis
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::chassis::WheelLegController, rmcs_executor::Component)
