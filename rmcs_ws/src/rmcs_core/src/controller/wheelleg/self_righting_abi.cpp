#include "self_righting_core.hpp"
#include "recovery_geometry_v202.hpp"

using Core = rmcs_core::controller::wheelleg::SelfRightingCore;

extern "C" {
int wheelleg_recovery_abi_version() { return 6; }
int wheelleg_recovery_compensation(const double* position, double* effort) {
    if (!position || !effort) return -1;
    Core::Axes q{};
    std::copy_n(position,4,q.begin());
    for (double value:q) if (!std::isfinite(value)) return -1;
    const auto result=rmcs_core::controller::wheelleg::recovery_v202::compensation(q);
    std::copy(result.begin(),result.end(),effort);
    return 0;
}
void* wheelleg_recovery_create(const double* rows, int count, const double* parameters, int size) {
    if (!rows || count < 2 || !parameters || size != 22) return nullptr;
    try {
        std::vector<Core::Row> table;
        for (int i = 0; i < count; ++i) {
            Core::Row row{};
            std::copy_n(rows+i*5,5,row.begin()); table.push_back(row);
        }
        Core::Settings s;
        s.cruise_speed=parameters[0]; s.acceleration=parameters[1];
        s.velocity_gain=parameters[2]; s.position_gain=parameters[3];
        s.fold_knee=parameters[4]; s.flip_knee=parameters[5];
        s.stand_knee=parameters[6]; s.stand_orbit=parameters[7];
        s.flip_orbit=parameters[8]; s.sweep_orbit=parameters[9];
        s.balance_kp=parameters[10]; s.balance_kd=parameters[11];
        s.balance_kv=parameters[12]; s.flip_asymmetry=parameters[13];
        s.timeout_seconds=parameters[14]; s.torque_limit=parameters[15];
        s.speed_limit=parameters[16];
        s.pitch_trim_gain=parameters[17]; s.yaw_rate_gain=parameters[18];
        s.integral_gain=parameters[19]; s.integral_limit=parameters[20];
        s.support_gain=parameters[21];
        return new Core(std::move(table),s,rmcs_core::controller::wheelleg::recovery_v202::support_compensation);
    } catch (...) { return nullptr; }
}
void wheelleg_recovery_destroy(void* handle) { delete static_cast<Core*>(handle); }
int wheelleg_recovery_step(void* handle, const double* in, double dt, int enabled, int fresh, double* out) {
    if (!handle || !in || !out) return -1;
    Core::Feedback f;
    std::copy_n(in,4,f.position.begin()); std::copy_n(in+4,4,f.velocity.begin());
    std::copy_n(in+8,4,f.spring_compensation.begin());
    std::copy_n(in+12,3,f.gravity.begin()); std::copy_n(in+15,3,f.gyro.begin());
    std::copy_n(in+18,2,f.wheel_velocity.begin());
    const auto result=static_cast<Core*>(handle)->step(f,dt,enabled,fresh);
    std::copy(result.position.begin(),result.position.end(),out);
    std::copy(result.velocity.begin(),result.velocity.end(),out+4);
    std::copy(result.torque.begin(),result.torque.end(),out+8);
    std::copy(result.wheel_torque.begin(),result.wheel_torque.end(),out+12);
    out[14]=result.phase; out[15]=result.fault;
    std::copy(result.servo_velocity.begin(),result.servo_velocity.end(),out+16);
    std::copy(result.servo_feedforward.begin(),result.servo_feedforward.end(),out+20);
    return 0;
}
}
