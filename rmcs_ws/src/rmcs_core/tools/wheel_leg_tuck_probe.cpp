// Small C ABI for a simulator to exercise production tuck code without ROS or CAN.
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"
#include "controller/chassis/wheel_leg/recovery_profile.hpp"
#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"

#include <algorithm>
#include <exception>
#include <string>

using namespace rmcs_core::controller::chassis;
namespace {
thread_local std::string error;
JointReferenceRecoveryConfig trajectory_config(const std::filesystem::path& directory) {
    auto config =
        RecoveryProfile::load(
            directory / "v7_recovery_profiles_107_v2.json", directory / "v7_height_lookup_107.json")
            .controller;
    config.dt = 0.001;
    config.blend_seconds = 0.0;
    config.dynamic_takeover = config.tuck_before_takeover = config.trajectory_takeover = true;
    config.release_motion_on_takeover = true;
    config.tuck_speed_rad_s = 3.0;
    config.tuck_rotation_direction = -1.0;
    config.prepare_speed_rad_s = 6.0;
    config.insert_speed_rad_s = 10.0;
    config.orbit_speed_rad_s = 6.0;
    return config;
}
struct TrajectoryProbe {
    RecoveryTuckController tuck;
    JointReferenceRecoveryController trajectory;
    bool initialized = false;
    std::array<double, 4> previous_q{};
    TrajectoryProbe(const std::filesystem::path& directory, double gain)
        : tuck{
              directory / "tuck_geometry_107_v1.json",
              "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997",
              {.integral_gain = gain, .body_fixed = true}}
        , trajectory{trajectory_config(directory)} {}
};
} // namespace
extern "C" {
const char* wheel_leg_tuck_error() { return error.c_str(); }
void* wheel_leg_tuck_create(const char* geometry, double integral_gain) {
    try {
        RecoveryTuckConfig config;
        config.integral_gain = integral_gain;
        return new RecoveryTuckController{
            geometry, "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997", config};
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}
void wheel_leg_tuck_destroy(void* context) { delete static_cast<RecoveryTuckController*>(context); }
void wheel_leg_tuck_reset(void* context) { static_cast<RecoveryTuckController*>(context)->reset(); }
int wheel_leg_tuck_step(
    void* context, const double* q, const double* gravity, double dt, int active, double* output) {
    try {
        const auto state = static_cast<RecoveryTuckController*>(context)->update(
            Eigen::Map<const Eigen::Vector4d>{q}, Eigen::Map<const Eigen::Vector3d>{gravity}, dt,
            active != 0);
        std::copy_n(state.reference.data(), 4, output);
        std::copy_n(state.hold_torque.data(), 4, output + 4);
        std::copy_n(state.knee_deg.data(), 2, output + 8);
        std::copy_n(state.sagittal_error_m.data(), 2, output + 10);
        std::copy_n(state.com_b_m.data(), 3, output + 12);
        return state.valid ? (state.ready ? 2 : 1) : 0;
    } catch (const std::exception& e) {
        error = e.what();
        return -1;
    }
}

void* wheel_leg_trajectory_create(const char* directory, double integral_gain) {
    try {
        return new TrajectoryProbe{directory, integral_gain};
    } catch (const std::exception& e) {
        error = e.what();
        return nullptr;
    }
}
void wheel_leg_trajectory_destroy(void* context) { delete static_cast<TrajectoryProbe*>(context); }
// Output: reference4, I4, knees2, sagittal2, COM3, effort6,
// phase, remaining, tracking, tucking, pure_rl, failure, IK_goal4, canonical_q4.
int wheel_leg_trajectory_step(
    void* context, const double* q, const double* dq, const double* gravity, double dt,
    double* output) {
    try {
        auto& probe = *static_cast<TrajectoryProbe*>(context);
        JointReferenceRecoveryFeedback feedback;
        feedback.q = Eigen::Map<const JointReferenceRecoveryVector6>{q};
        feedback.dq = Eigen::Map<const JointReferenceRecoveryVector6>{dq};
        feedback.gravity = Eigen::Map<const Eigen::Vector3d>{gravity};
        std::array<double, 4> raw;
        std::copy_n(q, 4, raw.begin());
        // PhysX also wraps individual revolute joints on long common turns.
        // Exercise the same input normalization as the native RMCS controller.
        probe.previous_q = wind_leg_feedback(
            raw, probe.initialized ? probe.previous_q : raw, LegTargetWinding::kPairedBranch107);
        feedback.q.head<4>() = Eigen::Map<const Eigen::Vector4d>{probe.previous_q.data()};
        if (!probe.initialized) {
            probe.trajectory.reset(feedback.q);
            probe.initialized = true;
        }
        const auto& previous = probe.trajectory.command();
        // After the handover event, a fixture PD holds the final legs for
        // geometry inspection. It is deliberately not an RL actor test.
        const auto state = probe.tuck.update(
            feedback.q.head<4>(), feedback.gravity, dt, previous.tucking || previous.pure_rl,
            previous.targets.head<4>(), feedback.dq.head<4>());
        feedback.tuck_reference = state.reference;
        feedback.tuck_fold_reference = state.fold_reference;
        feedback.tuck_hold_torque = state.hold_torque;
        feedback.tuck_valid = state.valid;
        feedback.tuck_ready = state.ready;
        feedback.tuck_folded = state.folded;
        feedback.rl_capture_ready = state.knees_at_target && state.valid
                                 && -feedback.gravity.z() >= std::cos(40 * std::numbers::pi / 180);
        if (!previous.pure_rl)
            probe.trajectory.update(feedback, dt);
        const auto& command = probe.trajectory.command();
        auto effort = probe.trajectory.torques(feedback, JointReferenceRecoveryVector6::Zero());
        if (command.pure_rl) {
            effort.head<4>() = (160 * (command.targets.head<4>() - feedback.q.head<4>())
                                - 2.5 * feedback.dq.head<4>() + state.hold_torque)
                                   .cwiseMax(-40.0)
                                   .cwiseMin(40.0);
            effort.tail<2>() = (-0.6 * feedback.dq.tail<2>()).cwiseMax(-4.5).cwiseMin(4.5);
        }
        std::copy_n(command.targets.data(), 4, output);
        std::copy_n(state.hold_torque.data(), 4, output + 4);
        std::copy_n(state.knee_deg.data(), 2, output + 8);
        std::copy_n(state.sagittal_error_m.data(), 2, output + 10);
        std::copy_n(state.com_b_m.data(), 3, output + 12);
        std::copy_n(effort.data(), 6, output + 15);
        output[21] = static_cast<int>(command.phase);
        output[22] = command.trajectory_remaining_rad;
        output[23] = command.tracking_error_rad;
        output[24] = command.tucking;
        output[25] = command.pure_rl;
        output[26] = command.failure_code;
        std::copy_n(state.reference.data(), 4, output + 27);
        std::copy_n(feedback.q.data(), 4, output + 31);
        return state.valid && !command.failed ? (state.ready ? 2 : 1) : 0;
    } catch (const std::exception& e) {
        error = e.what();
        return -1;
    }
}
}
