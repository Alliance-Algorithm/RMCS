#include "wheel_leg_controller.hpp"

#include <cmath>

#include "gas_spring_compensation.hpp"
#include "leg_target_winding.hpp"

namespace rmcs_core::controller::chassis {

bool WheelLegController::scripted_capture_ready_() const {
    using Core = wheelleg::SelfRightingCore;
    if (recovery_command_.phase != Core::Complete)
        return false;
    const Eigen::Vector3d gravity =
        world_base_orientation_().conjugate() * -Eigen::Vector3d::UnitZ();
    if (gravity.z() >= -.9903 || gyro_->head<2>().norm() >= .35
        || dq_.head<4>().cwiseAbs().maxCoeff() >= .6 || dq_.tail<2>().cwiseAbs().maxCoeff() >= 2.)
        return false;
    std::array<double, 4> nominal{}, current{};
    for (int i = 0; i < 4; ++i) {
        nominal[i] = nominal_[i];
        current[i] = q_[i];
    }
    const auto target = wind_leg_targets(nominal, current, leg_target_winding_);
    for (int i = 0; i < 4; ++i)
        if (std::abs(target[i] - q_[i]) >= .25)
            return false;
    return true;
}

bool WheelLegController::advance_scripted_recovery_(std::size_t tick) {
    using Core = wheelleg::SelfRightingCore;
    const bool pd_due =
        !schedule_.last_pd_tick || tick - *schedule_.last_pd_tick >= schedule_.pd_divisor;
    if (pd_due) {
        const auto start = Clock::now();
        Core::Feedback feedback;
        for (int i = 0; i < 4; ++i) {
            feedback.position[i] = q_[i];
            feedback.velocity[i] = dq_[i];
        }
        const auto gas = gas_spring_compensation(feedback.position, 1., true);
        if (!gas.valid) {
            latch_fault_("invalid scripted recovery spring input");
            return false;
        }
        feedback.spring_compensation = gas.torque;
        const Eigen::Vector3d gravity =
            world_base_orientation_().conjugate() * -Eigen::Vector3d::UnitZ();
        for (int i = 0; i < 3; ++i) {
            feedback.gravity[i] = gravity[i];
            feedback.gyro[i] = (*gyro_)[i];
        }
        feedback.wheel_velocity = {dq_[4], dq_[5]};
        recovery_command_ = scripted_recovery_->step(feedback, 1. / pd_frequency_, true);
        if (recovery_command_.phase == Core::Fault) {
            const int fault = recovery_command_.fault;
            latch_fault_("scripted self-righting failed");
            *recovery_phase_output_ = Core::Fault;
            *recovery_fault_output_ = fault;
            return false;
        }
        *recovery_phase_output_ = recovery_command_.phase;
        const Eigen::Vector4d compensation_api =
            leg_jacobian_.transpose() * Eigen::Map<const Eigen::Vector4d>{gas.torque.data()};
        for (int i = 0; i < 4; ++i)
            *gas_compensation_torque_outputs_[i] = compensation_api[i];
        for (int side = 0; side < 2; ++side) {
            *gas_beta_outputs_[side] = gas.beta_deg[side];
            *gas_force_outputs_[side] = gas.force_n[side];
            *gas_weight_outputs_[side] = gas.weight[side];
            *gas_extrapolated_outputs_[side] = gas.extrapolated[side] ? 1. : 0.;
        }
        write_motor_efforts_(
            Eigen::Map<const Eigen::Vector4d>{recovery_command_.torque.data()},
            Eigen::Map<const Eigen::Vector2d>{recovery_command_.wheel_torque.data()}, true);
        if (fault_latched_)
            return false;
        schedule_.last_pd_tick = tick;
        *pd_time_us_ = std::chrono::duration<double, std::micro>{Clock::now() - start}.count();
    }
    const bool ready = scripted_capture_ready_();
    *recovery_ready_output_ = ready;
    if (!ready) {
        // Once inference starts, losing capture cannot reuse a pending action.
        if (recovery_policy_started_)
            latch_fault_("scripted capture lost while awaiting policy");
        return false;
    }
    if (!recovery_policy_started_) {
        recovery_policy_started_ = true;
        accepted_policy_sequence_ = *policy_sequence_input_;
        policy_wait_started_.reset();
    }
    if (!evaluate_policy_(tick)) {
        latch_fault_("no fresh policy action after scripted capture");
        return false;
    }
    if (!policy_targets_valid_)
        return false; // Continue capture efforts until a fresh inference arrives.
    schedule_.last_pd_tick.reset();
    return true;
}

} // namespace rmcs_core::controller::chassis
