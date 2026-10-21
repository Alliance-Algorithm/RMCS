#include "controller/chassis/wheel_leg/recovery_profile.hpp"
#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"
#include <filesystem>
#include <memory>
#include <algorithm>
#include <string>
using namespace rmcs_core::controller::chassis;
namespace {
JointReferenceRecoveryConfig config(const std::filesystem::path& dir) {
    auto c = RecoveryProfile::load(dir / "v7_recovery_profiles_107_v2.json", dir / "v7_height_lookup_107.json").controller;
    c.dt = .001;
    c.dynamic_takeover = c.tuck_before_takeover = c.release_motion_on_takeover = true;
    c.blend_seconds = 0.;
    c.prepare_speed_rad_s = 6.;
    c.insert_speed_rad_s = 10.;
    c.orbit_speed_rad_s = 6.;
    c.dynamic_capture_height_min = .20;
    c.dynamic_capture_height_max = .29;
    return c;
}
struct Context {
    RecoveryTuckController tuck;
    JointReferenceRecoveryController fsm;
    bool initialized = false;
    Context(const std::filesystem::path& dir) :
      tuck(dir/"tuck_geometry_107_v1.json", "0ac82045ab584f20091538aa782d93ea545382dc701faca1f8ae92aeb7cd1997", {}),
      fsm(config(dir)) {}
};
thread_local std::string error;
}
extern "C" {
const char* replay_error() { return error.c_str(); }
void* replay_create(const char* dir) {
    try { return new Context{dir}; } catch (const std::exception& e) { error=e.what(); return nullptr; }
}
void replay_destroy(void* p) { delete static_cast<Context*>(p); }
int replay_step(void* p, const double* q, const double* dq, const double* gravity, const double* gyro,
                double height, int support, double dt, double* out) {
    try {
        auto& c=*static_cast<Context*>(p);
        JointReferenceRecoveryFeedback f;
        f.q=Eigen::Map<const JointReferenceRecoveryVector6>{q};
        f.dq=Eigen::Map<const JointReferenceRecoveryVector6>{dq};
        f.gravity=Eigen::Map<const Eigen::Vector3d>{gravity};
        f.gyro=Eigen::Map<const Eigen::Vector3d>{gyro};
        f.estimated_height=height;
        f.support=support != 0;
        // Actual logs never reach the measured knee gate, so actor capture is false.
        f.rl_capture_ready=false;
        if (!c.initialized) { c.fsm.reset(f.q); c.initialized=true; }
        const auto tuck=c.tuck.update(f.q.head<4>(), f.gravity, dt, c.fsm.command().tucking);
        f.tuck_reference=tuck.reference;
        f.tuck_hold_torque=tuck.hold_torque;
        f.tuck_valid=tuck.valid;
        f.tuck_ready=tuck.ready;
        const auto& cmd=c.fsm.update(f,dt);
        const auto tau=c.fsm.torques(f, JointReferenceRecoveryVector6::Zero());
        std::copy_n(cmd.targets.data(),6,out);
        std::copy_n(tau.data(),6,out+6);
        std::copy_n(tuck.reference.data(),4,out+12);
        std::copy_n(tuck.hold_torque.data(),4,out+16);
        out[20]=cmd.tucking;
        out[21]=static_cast<int>(cmd.phase);
        out[22]=cmd.failure_code;
        out[23]=tuck.ready;
        out[24]=(tuck.reference-f.q.head<4>()).cwiseAbs().maxCoeff();
        return 1;
    } catch (const std::exception& e) { error=e.what(); return 0; }
}
}
