#pragma once

#include <cstdint>
#include <numbers>
#include <string_view>

#include <eigen3/Eigen/Core>

namespace rmcs::rl {

// Frozen native source: 2778206b5905c2760ccc381f2592b08e1cad611a,
// wheeled_tasks/chassis/{recovery,recovery_training}.py. All public six-vectors
// use policy order LH, LA, RH, RA, LW, RW; angles and efforts are output-side.
using V6RecoveryVector6 = Eigen::Matrix<double, 6, 1>;

enum class V6RecoveryPhase : std::uint8_t {
    kSelect = 0, kFold = 1, kPlant = 2, kOrbit = 3, kThrust = 4, kSide = 5,
    kCapture = 6, kPrepare = 7, kBlend = 8, kRl = 9, kFailed = 10,
};
enum class V6RecoveryRoute : std::uint8_t {
    kUpright = 0, kPlant = 1, kOrbitPositive = 2, kOrbitNegative = 3, kSide = 4,
};
std::string_view v6_recovery_phase_name(V6RecoveryPhase phase);

struct V6RecoveryConfig {
    // Load-corrected PREPARE reference, not the physical episode reset pose.
    Eigen::Vector4d nominal = Eigen::Vector4d::Zero();
    Eigen::Vector4d fold = Eigen::Vector4d::Zero();
    Eigen::Vector4d thrust = Eigen::Vector4d::Zero();
    Eigen::Vector4d support = Eigen::Vector4d::Zero();
    V6RecoveryVector6 rl_nominal = V6RecoveryVector6::Zero();
    Eigen::Vector4d root_axis_signs{1.0, 1.0, -1.0, -1.0};
    Eigen::Vector2d wheel_axis_signs{1.0, -1.0};
    double dt = 0.005;
    // Frozen simulation uses 200 ms; zero selects direct deployment takeover.
    double blend_seconds = 0.2;
    double stable_seconds = 1.0;
    double max_script_seconds = 8.0;
    double prepare_speed_rad_s = 1.0;
    double push_speed_rad_s = 4.0;
    double orbit_speed_rad_s = 4.0;
    double capture_speed_rad_s = 4.0;
    double side_speed_rad_s = 6.0;
    double side_angle_rad = std::numbers::pi / 2.0;
    double plant_angle_rad = std::numbers::pi / 9.0;
    double orbit_turns = 1.0;
    double handover_height_min = 0.27;
    double handover_height_max = 0.36;
    double handover_tilt_rad = 8.0 * std::numbers::pi / 180.0;
    double handover_gyro_max = 0.75;
    double reroute_tilt_rad = 70.0 * std::numbers::pi / 180.0;
    double reroute_support_lost_seconds = 0.15;
    int max_reroutes = 1;
    bool wheel_balance_enabled = true;
    double pitch_kp_nm_rad = 8.0;
    double pitch_kd_nm_s_rad = 1.5;
    double wheel_damping_nm_s_rad = 0.2;
    double prepare_tilt_limit_deg = 55.0;
    double capture_tilt_limit_deg = 85.0;
};

struct V6RecoveryFeedback {
    V6RecoveryVector6 q = V6RecoveryVector6::Zero();
    V6RecoveryVector6 dq = V6RecoveryVector6::Zero();
    Eigen::Vector3d gravity{0.0, 0.0, -1.0};
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    double estimated_height = 0.0;
    // Native plausible conditional support; never true simulator contacts.
    bool support = false;
    bool body_clear = false;
    Eigen::Vector2d wheel_probe_torque = Eigen::Vector2d::Zero();
};

struct V6RecoveryCommand {
    V6RecoveryVector6 targets = V6RecoveryVector6::Zero();
    V6RecoveryVector6 continuous_q = V6RecoveryVector6::Zero();
    V6RecoveryPhase phase = V6RecoveryPhase::kRl;
    V6RecoveryRoute route = V6RecoveryRoute::kUpright;
    double blend = 1.0;
    bool motion_released = true;
    bool motion_hold = false;
    bool scripted = false;
    bool blending = false;
    bool pure_rl = true;
    bool failed = false;
    bool release_finished = true;
    bool wheel_balance_active = false;
    int failure_code = 0;
    std::uint64_t age_ticks = 0;
    std::uint64_t phase_ticks = 0;
    std::uint64_t stable_ticks = 0;
    int reroute_count = 0;
};

class V6RecoveryController {
public:
    explicit V6RecoveryController(const V6RecoveryConfig& config);
    void reset(const V6RecoveryVector6& q, bool active = true,
               double initial_release_seconds = 0.0);
    // Exactly one native 5 ms reference update. Ownership of sensor freshness,
    // DM readiness, geometry guards and support probing stays with the caller.
    const V6RecoveryCommand& update(const V6RecoveryFeedback& feedback);
    const V6RecoveryCommand& command() const { return command_; }
    V6RecoveryVector6 project_feedback(const V6RecoveryVector6& q) const;
    // Reevaluate native PD on current feedback without advancing the reference.
    // rl_torque is output-side actor PD effort; both paths are clipped before
    // the optional 200 ms linear torque blend. Zero takes over in the ready tick.
    // No V5 feedforward or 1.5 Nm wheel clamp.
    V6RecoveryVector6 torques(const V6RecoveryFeedback& feedback,
                             const V6RecoveryVector6& rl_torque) const;
    V6RecoveryVector6 effective_action_history(const V6RecoveryVector6& rl_action) const;
    static Eigen::Vector4d paired_delta(const Eigen::Vector4d& goal,
                                       const Eigen::Vector4d& reference);

private:
    using Vector4f = Eigen::Matrix<float, 4, 1>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;
    static Vector4f paired_delta_(const Vector4f& goal, const Vector4f& reference);
    void enter_(V6RecoveryPhase phase);
    void fail_(int code);
    void reset_fsm_(const Vector6f& q, bool active);
    void publish_();
    std::uint64_t ticks_(double seconds) const;
    V6RecoveryConfig config_;
    V6RecoveryCommand command_;
    Vector6f targets_ = Vector6f::Zero();
    Vector6f continuous_q_ = Vector6f::Zero();
    // External native encoder provider uses integer turns, independently of
    // the FSM's trigonometric accumulation. Zero-turn PD must be an identity.
    Vector6f canonical_q_ = Vector6f::Zero();
    Vector6f last_raw_q_ = Vector6f::Zero();
    Vector4f orbit_start_ = Vector4f::Zero();
    Vector4f thrust_anchor_ = Vector4f::Zero();
    Vector4f side_start_ = Vector4f::Zero();
    Vector4f side_direction_ = Vector4f::Zero();
    V6RecoveryPhase phase_ = V6RecoveryPhase::kRl;
    V6RecoveryRoute route_ = V6RecoveryRoute::kUpright;
    std::uint64_t age_ticks_ = 0, phase_ticks_ = 0, ready_ticks_ = 0;
    std::uint64_t stable_ticks_ = 0, plant_ticks_ = 0, unsupported_ticks_ = 0;
    std::uint64_t elapsed_ticks_ = 0;
    int reroute_count_ = 0, failure_code_ = 0;
    bool enabled_ = false, membership_ = false, motion_released_ = true;
    bool pitch_initialized_ = false, release_finished_ = true;
    bool wheel_balance_active_ = false;
    float continuous_pitch_ = 0.0f, last_pitch_ = 0.0f;
    float orbit_angle_ = 0.0f, plant_pitch_ = 0.0f;
    double initial_release_seconds_ = 0.0;
};

} // namespace rmcs::rl
