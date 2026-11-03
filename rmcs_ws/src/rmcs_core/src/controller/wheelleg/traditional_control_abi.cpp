#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "traditional_control_core.hpp"

namespace {
using namespace rmcs_core::wheelleg;
constexpr int kParameterCount = 28;
constexpr int kKnotWidth = 27;

bool finite(const double* values, int count) {
    return values && count >= 0
        && std::all_of(values, values + count, [](double value) { return std::isfinite(value); });
}
} // namespace

// The ABI is intentionally independent of ROS and Kit. Both hosts construct
// the same parameter object and call the same control implementation.
extern "C" {

int wheelleg_traditional_abi_version() { return 1; }

const char* wheelleg_traditional_gas_spring_model_sha256() {
    return gas_spring_v202::model_sha256.data();
}

const char* wheelleg_traditional_model_spec_sha256() {
    return gas_spring_v202::model_spec_sha256.data();
}

void* wheelleg_traditional_create(const double* values, int count,
                                 const double* knots, int rows,
                                 const double* curve, int curve_rows) {
    if (count != kParameterCount || rows < 2 || rows > 512 || curve_rows < 2 || curve_rows > 4096
        || !finite(values, count) || !finite(knots, rows*kKnotWidth) || !finite(curve, curve_rows*2))
        return nullptr;
    try {
        TraditionalParameters p;
        p.leg_kp=values[0]; p.leg_kd=values[1]; p.wheel_limit=values[2]; p.leg_limit=values[3];
        p.acceleration_limit=values[4]; p.jerk_limit=values[5];
        p.height_rate=values[6]; p.height_acceleration=values[7];
        p.outer_kv=values[8]; p.outer_kx=values[9];
        p.freefall_threshold=values[10]; p.load_threshold=values[11];
        p.support_confirmation=values[12]; p.feedback_timeout=values[13];
        p.max_tilt=values[14]; p.wheel_inertia=values[15]; p.wheel_radius=values[16];
        p.nominal_length=values[17]; p.power.idle_w=values[18]; p.power.efficiency=values[19];
        std::copy_n(values+20, 6, p.power.copper_w_per_nm2.begin());
        p.torque_filter_s=values[26]; p.velocity_filter_s=values[27];
        for (int i=0; i<rows; ++i) {
            const double* row=knots+i*kKnotWidth;
            PostureKnot knot;
            knot.length=row[0];
            std::copy_n(row+1,4,knot.position.begin());
            std::copy_n(row+5,4,knot.gravity.begin());
            std::copy_n(row+9,8,knot.spring_jacobian.begin());
            std::copy_n(row+17,2,knot.compression.begin());
            std::copy_n(row+19,4,knot.balance_gain.begin());
            std::copy_n(row+23,2,knot.yaw_gain.begin());
            knot.axle_x=row[25]; knot.knee_coordinate=row[26];
            p.postures.push_back(knot);
        }
        for(int i=0;i<curve_rows;++i)
            p.force_curve.push_back({curve[i*2],curve[i*2+1]});
        return new TraditionalControlCore(std::move(p));
    } catch (...) {
        return nullptr;
    }
}

void wheelleg_traditional_destroy(void* handle) {
    delete static_cast<TraditionalControlCore*>(handle);
}

void wheelleg_traditional_reset(void* handle) {
    if (handle)
        static_cast<TraditionalControlCore*>(handle)->reset();
}

int wheelleg_traditional_step(void* handle, std::int64_t time_ns, std::int64_t feedback_ns,
                             unsigned flags, const double* observation, int count,
                             const double* commands, int command_count, double* result, int result_count) {
    if (result && result_count > 0)
        std::fill_n(result, result_count, 0.0);
    if (!handle || !observation || !commands || !result || count!=17 || command_count!=4 || result_count!=16)
        return -1;
    TraditionalSensors s;
    std::copy_n(observation,6,s.position.begin());
    std::copy_n(observation+6,6,s.velocity.begin());
    s.pitch=observation[12]; s.pitch_rate=observation[13];
    s.yaw=observation[14]; s.yaw_rate=observation[15]; s.specific_force_norm=observation[16];
    s.time_ns=time_ns; s.feedback_ns=feedback_ns;
    s.enabled=flags&1u; s.fresh=flags&2u; s.reset=flags&4u;
    s.balance_enabled=flags&8u;
    const TraditionalCommand command{commands[0],commands[1],commands[2],commands[3]};
    const auto out=static_cast<TraditionalControlCore*>(handle)->step(s,command);
    std::copy(out.effort.begin(),out.effort.end(),result);
    result[6]=out.length_reference; result[7]=out.velocity_reference; result[8]=out.pitch_reference;
    result[9]=out.velocity_estimate; result[10]=out.common_torque; result[11]=out.differential_torque;
    result[12]=out.power_scale; result[13]=out.predicted_power;
    result[14]=static_cast<int>(out.support); result[15]=out.failure;
    return out.failure;
}

} // extern "C"
