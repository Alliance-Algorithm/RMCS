// Isaac Sim test adapter. The controller and observer are the deployment sources.
// This adapter supplies simulated Component inputs; it never connects to hardware.
#include "recovery_controller.hpp"
#include "recovery_observer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>
#include <string>

#include <nlohmann/json.hpp>

namespace {
using namespace rmcs::rl;
using Json = nlohmann::json;
thread_local std::string error;

Eigen::Vector3d vector3(const Json& value) {
    return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}
Eigen::Vector4d vector4(const Json& value) {
    return {
        value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>(),
        value.at(3).get<double>()};
}
RecoveryMechanism mechanism(const Json& data) {
    RecoveryMechanism result;
    result.wheel_radius_m = data.at("wheel_radius_m");
    result.spring_stroke_m = data.at("spring_stroke_m");
    result.spring_force_n = data.at("spring_force_n").get<std::array<double, 4>>();
    for (const auto& point : data.at("shell_points"))
        result.shell_points_body_m.push_back(vector3(point));
    for (int i = 0; i < 2; ++i) {
        const auto& input = data.at("sides").at(i);
        auto& side = result.sides[i];
        side.delta_rad = input.at("delta").get<std::vector<double>>();
        side.inner_knee_deg = input.at("inner").get<std::vector<double>>();
        side.slider_m = input.at("slider").get<std::vector<double>>();
        side.hip_origin_m = vector3(input.at("hip_origin"));
        side.hip_axis = vector3(input.at("hip_axis"));
        side.spring_compression_at_zero_m = input.at("spring_zero");
        for (const auto& point : input.at("point"))
            side.wheel_at_hip_zero_m.push_back(vector3(point));
        if (input.contains("knee_axis")) {
            for (const auto& axis : input.at("knee_axis"))
                side.knee_axis_at_hip_zero.push_back(vector3(axis));
            for (const auto& axis : input.at("wheel_axis"))
                side.wheel_axis_at_hip_zero.push_back(vector3(axis));
            side.passive_knee_sign = input.at("sign");
            side.slider_slope_m_per_rad = input.at("slider_slope").get<std::vector<double>>();
        }
    }
    return result;
}
RecoveryConfig config(const Json& data) {
    RecoveryConfig result;
    const auto& poses = data.at("poses");
    result.fold = vector4(poses.at("fold"));
    result.thrust = vector4(poses.at("thrust"));
    result.side_extended = vector4(poses.at("side_extended"));
    result.plant = vector4(poses.at("plant"));
    result.stand = vector4(poses.at("stand"));
    result.upright = vector4(poses.at("upright"));
    result.support_extended = vector4(poses.at("support_extended"));
    result.upright_support_extended = vector4(poses.at("upright_support_extended"));
    result.capture_extended = vector4(poses.at("capture_extended"));
    if (data.contains("root_axis_y")) {
        result.root_axis_y = vector4(data.at("root_axis_y"));
        result.wheel_axis_y = {
            data.at("wheel_axis_y").at(0).get<double>(),
            data.at("wheel_axis_y").at(1).get<double>()};
    }
    return result;
}
struct Session {
    explicit Session(const Json& data)
        : observer(mechanism(data))
        , controller(config(data))
        , diagnostic_wheel_brake(data.value("diagnostic_wheel_brake", false)) {}
    RecoveryObserver observer;
    RecoveryController controller;
    RecoveryFeedback feedback;
    RecoveryCommand command;
    RecoverySensorData submitted;
    bool started = false;
    bool diagnostic_wheel_brake = false;
    RecoveryFailure fault = RecoveryFailure::kNone;
    double dt = 0.005;
};
} // namespace

extern "C" {
const char* rmcs_sim_error() { return error.c_str(); }
void* rmcs_sim_create(const char* data) {
    try {
        return new Session(Json::parse(data));
    } catch (const std::exception& exception) {
        error = exception.what();
        return nullptr;
    }
}
void rmcs_sim_destroy(void* handle) { delete static_cast<Session*>(handle); }

// Inputs: P-order q[6], dq[6], gravity[3], gyro[3], specific force[3],
// and the two previous PhysX-applied wheel torques. Truth contact is not an input.
void rmcs_sim_step(
    void* handle, const double* input, std::uint64_t now_ns, std::uint64_t sequence, double dt,
    int enabled, double* output) {
    auto& session = *static_cast<Session*>(handle);
    session.dt = dt;
    std::fill_n(output, 39, 0.0);
    if (session.fault != RecoveryFailure::kNone) {
        output[6] = static_cast<int>(RecoveryPhase::kFailed);
        output[7] = static_cast<int>(session.fault);
        return;
    }
    auto sensors = session.submitted;
    sensors.steady_ns = now_ns;
    sensors.imu_feedback_ns = now_ns;
    sensors.imu_feedback_sequence = sequence;
    // Input quaternion uses Isaac's xyzw convention, the same IMU attitude
    // used for gravity projection. There is no contact/height truth input.
    sensors.world_base_orientation = Eigen::Quaterniond{input[28], input[25], input[26], input[27]};
    for (int i = 0; i < 2; ++i) {
        sensors.wheel_feedback_ns[i] = now_ns;
        sensors.wheel_feedback_sequence[i] = sequence;
        sensors.wheel_torque_feedback_nm[i] = input[21 + i];
    }
    const bool baseline = session.observer.sensor_baseline_ready();
    session.feedback = session.observer.update(
        Eigen::Map<const RecoveryVector6>(input), Eigen::Map<const RecoveryVector6>(input + 6),
        Eigen::Map<const Eigen::Vector3d>(input + 12),
        Eigen::Map<const Eigen::Vector3d>(input + 15),
        Eigen::Map<const Eigen::Vector3d>(input + 18), dt, sensors);
    if (session.diagnostic_wheel_brake) {
        session.feedback.world_wheel_omega = Eigen::Map<const Eigen::Vector2d>{input + 23};
        session.feedback.world_wheel_omega_valid = session.feedback.world_wheel_omega.allFinite();
    }
    const auto& feedback = session.feedback;
    if (!enabled)
        return; // Update derivative baselines during the zero-effort release.
    if (!feedback.geometry_valid || !feedback.spring_compensation_valid)
        session.fault = RecoveryFailure::kInvalidFeedback;
    if (session.fault == RecoveryFailure::kNone && baseline) {
        if (!session.started) {
            session.started = session.controller.start(feedback);
            if (!session.started)
                session.fault = RecoveryFailure::kInvalidFeedback;
        }
        if (session.started) {
            session.command = session.controller.step(feedback, dt);
            if (session.command.phase == RecoveryPhase::kFailed)
                session.fault = session.command.failure;
            if (session.command.phase == RecoveryPhase::kComplete
                && -feedback.gravity.z() < std::cos(45.0 * std::numbers::pi / 180.0))
                session.fault = RecoveryFailure::kLostUpright;
        }
    }
    auto pulse = session.observer.probe_command(
        session.command.phase == RecoveryPhase::kPrepare
            || session.command.phase == RecoveryPhase::kCapture,
        feedback);
    for (int side = 0; side < 2; ++side)
        if (pulse[side] != 0)
            session.command.torque[4 + side] = pulse[side];
    if (session.fault != RecoveryFailure::kNone) {
        session.command = {.phase = RecoveryPhase::kFailed, .failure = session.fault};
    }
    Eigen::Map<RecoveryVector6>{output} = session.command.torque;
    output[6] = static_cast<int>(session.command.phase);
    output[7] = static_cast<int>(session.command.failure);
    output[8] = session.command.blend;
    output[9] = feedback.height_if_grounded;
    output[10] = feedback.wheel_height_difference;
    output[11] = feedback.geometry_valid;
    output[12] = feedback.height_valid;
    output[13] = feedback.contact_candidate;
    output[14] = feedback.alignment_candidate;
    output[15] = feedback.settled;
    output[16] = feedback.support_confirmed;
    output[17] = feedback.body_clear;
    output[18] = feedback.world_wheel_omega_valid;
    Eigen::Map<Eigen::Vector4d>{output + 19} = session.controller.reference();
    output[23] = feedback.inner_knee_deg[0];
    output[24] = feedback.inner_knee_deg[1];
    output[25] = pulse[0];
    output[26] = pulse[1];
    output[27] = session.observer.sensor_baseline_ready();
    output[28] = session.started;
    output[30] = feedback.geometrically_supported;
    output[31] = feedback.probe_confirmed;
    output[32] = feedback.probe_evidence_mask;
    output[33] = feedback.height_rate_mps;
    output[34] = feedback.world_wheel_omega[0];
    output[35] = feedback.world_wheel_omega[1];
    output[36] = feedback.gyro_acceleration_rad_s2;
    output[37] = feedback.wheel_acceleration_rad_s2[0];
    output[38] = feedback.wheel_acceleration_rad_s2[1];
}

void rmcs_sim_constrain_goal(void* handle, double* goal) {
    auto& session = *static_cast<Session*>(handle);
    Eigen::Vector4d value = Eigen::Map<Eigen::Vector4d>(goal);
    session.observer.constrain_policy_goal(value, 2.0);
    Eigen::Map<Eigen::Vector4d>{goal} = value;
}

// Mirrors RlController::apply_soft_limits_ and its conditional DM output bound.
// The bench records peak exposure; no measured hardware thermal budget is claimed.
void rmcs_sim_apply(void* handle, double* torque, std::uint64_t now_ns) {
    auto& session = *static_cast<Session*>(handle);
    const auto& feedback = session.feedback;
    if (session.started && feedback.geometry_valid) {
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, auxiliary = hip + 1;
            const double slope = feedback.inner_knee_slope_deg_per_rad[side];
            const double outward = slope * (torque[auxiliary] - torque[hip]);
            const auto limits = session.observer.inner_knee_limits_deg(side);
            if ((feedback.inner_knee_deg[side] >= limits[1] - 2.0 && outward > 0)
                || (feedback.inner_knee_deg[side] <= limits[0] + 2.0 && outward < 0)) {
                torque[hip] += outward / (2 * slope);
                torque[auxiliary] -= outward / (2 * slope);
            }
        }
    }
    for (int i = 0; i < 4; ++i) {
        const double bound = conditional_dm_output_bound(feedback.dq[i], 100.0, 20.0, 40.0);
        torque[i] = std::clamp(torque[i], -bound, bound);
    }
    for (int i = 0; i < 2; ++i) {
        session.submitted.wheel_torque_submitted_nm[i] = torque[4 + i];
        session.submitted.wheel_torque_submitted_ns[i] = now_ns;
        session.submitted.wheel_tx_kind[i] = 1;
    }
}
} // extern "C"
