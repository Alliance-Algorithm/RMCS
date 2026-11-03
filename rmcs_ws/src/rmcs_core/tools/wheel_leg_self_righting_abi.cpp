#include "controller/wheelleg/recovery_geometry_v202.hpp"
#include "controller/wheelleg/self_righting_core.hpp"

using Core = rmcs_core::controller::wheelleg::SelfRightingCore;

extern "C" {
void* wheelleg_recovery_create_v202() {
    Core::Settings settings;
    settings.stand_knee = rmcs_core::controller::wheelleg::recovery_v202::stand_knee;
    settings.stand_orbit = rmcs_core::controller::wheelleg::recovery_v202::stand_orbit;
    settings.timeout_seconds = 30.;
    auto* core = new Core(
        rmcs_core::controller::wheelleg::recovery_v202::geometry, settings,
        rmcs_core::controller::wheelleg::recovery_v202::support_compensation);
    core->configure_capture(20., .98);
    return core;
}
int wheelleg_recovery_abi_version() { return 6; }
int wheelleg_recovery_compensation(const double* position, double* effort) {
    if (!position || !effort)
        return -1;
    Core::Axes q{};
    std::copy_n(position, 4, q.begin());
    for (double value : q)
        if (!std::isfinite(value))
            return -1;
    const auto result = rmcs_core::controller::wheelleg::recovery_v202::compensation(q);
    std::copy(result.begin(), result.end(), effort);
    return 0;
}
void wheelleg_recovery_destroy(void* handle) { delete static_cast<Core*>(handle); }
int wheelleg_recovery_step(
    void* handle, const double* in, double dt, int enabled, int fresh, double* out) {
    if (!handle || !in || !out)
        return -1;
    Core::Feedback f;
    std::copy_n(in, 4, f.position.begin());
    std::copy_n(in + 4, 4, f.velocity.begin());
    std::copy_n(in + 8, 4, f.spring_compensation.begin());
    std::copy_n(in + 12, 3, f.gravity.begin());
    std::copy_n(in + 15, 3, f.gyro.begin());
    std::copy_n(in + 18, 2, f.wheel_velocity.begin());
    const auto result = static_cast<Core*>(handle)->step(f, dt, enabled, fresh);
    std::copy(result.position.begin(), result.position.end(), out);
    std::copy(result.velocity.begin(), result.velocity.end(), out + 4);
    std::copy(result.torque.begin(), result.torque.end(), out + 8);
    std::copy(result.wheel_torque.begin(), result.wheel_torque.end(), out + 12);
    out[14] = result.phase;
    out[15] = result.fault;
    std::copy(result.servo_velocity.begin(), result.servo_velocity.end(), out + 16);
    std::copy(result.servo_feedforward.begin(), result.servo_feedforward.end(), out + 20);
    return 0;
}
}
