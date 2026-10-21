// Small C ABI for a simulator to exercise production tuck code without ROS or CAN.
#include "controller/chassis/wheel_leg/recovery_tuck_controller.hpp"

#include <algorithm>
#include <exception>
#include <string>

using namespace rmcs_core::controller::chassis;
namespace {
thread_local std::string error;
}
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
}
