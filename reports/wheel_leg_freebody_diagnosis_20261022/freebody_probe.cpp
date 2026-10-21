// Offline diagnostic adapter. Compiles the current native trajectory unchanged.
// Dynamic IMU and native support observer are exercised; no ROS, CAN or actor.
#include "controller/chassis/wheel_leg/closed_chain_support_observer.hpp"
#include "controller/chassis/wheel_leg/leg_target_winding.hpp"
#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"
#include <algorithm>
#include <string>
using namespace rmcs_core::controller::chassis;
namespace {
thread_local std::string error;
JointReferenceRecoveryConfig config(const RecoveryProfile& profile) {
    auto c = profile.controller;
    c.dt = .001;
    c.blend_seconds = 0.;
    c.dynamic_takeover = c.tuck_before_takeover = c.trajectory_takeover = true;
    c.release_motion_on_takeover = true;
    c.tuck_speed_rad_s = 3.;
    c.tuck_rotation_direction = -1.;
    c.prepare_speed_rad_s = 6.; c.insert_speed_rad_s = 10.; c.orbit_speed_rad_s = 6.;
    return c;
}
struct Probe {
    RecoveryProfile profile;
    RecoveryTuckController tuck;
    JointReferenceRecoveryController trajectory;
    ClosedChainSupportObserver support;
    std::array<double,4> previous_q{};
    bool initialized = false;
    Probe(const std::filesystem::path& dir)
        : profile{RecoveryProfile::load(dir / "v7_recovery_profiles_107_v2.json", dir / "v7_height_lookup_107.json")},
          tuck{dir / "tuck_geometry_107_v1.json", "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997", {}},
          trajectory{config(profile)}, support{profile.geometry, .001, .04} {}
};
}
extern "C" {
const char* freebody_error() { return error.c_str(); }
void* freebody_create(const char* dir) {
    try { return new Probe{dir}; }
    catch (const std::exception& e) { error=e.what(); return nullptr; }
}
void freebody_destroy(void* p) { delete static_cast<Probe*>(p); }
// Output 0..34 follows wheel_leg_trajectory_step; extras:
// 35 helper.valid, 36 helper.ready, 37 plausible support, 38 confirmed support,
// 39 estimated height, 40..41 probe effort. No actor is substituted after RL.
int freebody_step(void* p, const double* q, const double* dq, const double* g,
                  const double* gyro, const double* acceleration, double dt, double* out) {
    try {
        auto& x=*static_cast<Probe*>(p);
        JointReferenceRecoveryVector6 canonical=Eigen::Map<const JointReferenceRecoveryVector6>{q};
        std::array<double,4> raw; std::copy_n(q,4,raw.begin());
        x.previous_q=wind_leg_feedback(raw, x.initialized ? x.previous_q : raw, LegTargetWinding::kPairedBranch107);
        canonical.head<4>()=Eigen::Map<const Eigen::Vector4d>{x.previous_q.data()};
        const auto phase=x.initialized ? x.trajectory.command().phase : JointReferenceRecoveryPhase::kSelect;
        auto f=x.support.observe({}, canonical, Eigen::Map<const JointReferenceRecoveryVector6>{dq},
            Eigen::Map<const Eigen::Vector3d>{g}, Eigen::Map<const Eigen::Vector3d>{gyro},
            Eigen::Map<const Eigen::Vector3d>{acceleration}, phase, dt);
        if (!x.initialized) { x.trajectory.reset(f.q); x.initialized=true; }
        const auto& old=x.trajectory.command();
        auto s=x.tuck.update(f.q.head<4>(), f.gravity, dt, old.tucking,
                            old.targets.head<4>(), f.dq.head<4>());
        f.tuck_reference=s.reference; f.tuck_hold_torque=s.hold_torque;
        f.tuck_valid=s.valid; f.tuck_ready=s.ready;
        f.rl_capture_ready=s.valid && -f.gravity.z()>=std::cos(40.*std::numbers::pi/180.);
        const auto& c=x.trajectory.update(f,dt);
        f.wheel_probe_torque=x.support.probe_command(c.scripted && c.release_finished,dt);
        auto effort=x.trajectory.torques(f, JointReferenceRecoveryVector6::Zero());
        if (c.pure_rl || c.failed) effort.setZero();
        std::copy_n(c.targets.data(),4,out); std::copy_n(s.hold_torque.data(),4,out+4);
        std::copy_n(s.knee_deg.data(),2,out+8); std::copy_n(s.sagittal_error_m.data(),2,out+10);
        std::copy_n(s.com_b_m.data(),3,out+12); std::copy_n(effort.data(),6,out+15);
        out[21]=static_cast<int>(c.phase); out[22]=c.trajectory_remaining_rad;
        out[23]=c.tracking_error_rad; out[24]=c.tucking; out[25]=c.pure_rl; out[26]=c.failure_code;
        std::copy_n(s.reference.data(),4,out+27); std::copy_n(f.q.data(),4,out+31);
        out[35]=s.valid; out[36]=s.ready; out[37]=f.support; out[38]=f.support_confirmed;
        out[39]=f.estimated_height; std::copy_n(f.wheel_probe_torque.data(),2,out+40);
        return c.failed ? 0 : c.pure_rl ? 2 : 1;
    } catch(const std::exception& e) { error=e.what(); return -1; }
}
}
