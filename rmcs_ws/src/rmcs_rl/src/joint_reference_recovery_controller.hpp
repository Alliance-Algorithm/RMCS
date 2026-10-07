#pragma once

#include <cstdint>
#include <numbers>
#include <string_view>

#include <eigen3/Eigen/Core>

namespace rmcs::rl {

// Native references: 2778206b5905c2760ccc381f2592b08e1cad611a,
// wheeled_tasks/chassis/{recovery,recovery_training}.py. Deployment uses a
// single side stroke and direct measured capture. Public six-vectors use policy
// order LH, LA, RH, RA, LW, RW; angles and efforts are output-side.
using JointReferenceRecoveryVector6 = Eigen::Matrix<double, 6, 1>;

enum class JointReferenceRecoveryPhase : std::uint8_t {
    kSelect = 0,
    kFold = 1,
    kPlant = 2,
    kOrbit = 3,
    kThrust = 4,
    kSide = 5,
    kCapture = 6,
    kPrepare = 7,
    kBlend = 8,
    kRl = 9,
    kFailed = 10,
};
enum class JointReferenceRecoveryRoute : std::uint8_t {
    kUpright = 0,
    kPlant = 1,
    kOrbitPositive = 2,
    kOrbitNegative = 3,
    kSide = 4,
};
std::string_view recovery_phase_name(JointReferenceRecoveryPhase phase);

struct JointReferenceRecoveryConfig {
    // Load-corrected PREPARE reference, not the physical episode reset pose.
    Eigen::Vector4d nominal = Eigen::Vector4d::Zero();
    Eigen::Vector4d fold = Eigen::Vector4d::Zero();
    Eigen::Vector4d thrust = Eigen::Vector4d::Zero();
    Eigen::Vector4d support = Eigen::Vector4d::Zero();
    JointReferenceRecoveryVector6 rl_nominal = JointReferenceRecoveryVector6::Zero();
    Eigen::Vector4d root_axis_signs{1.0, 1.0, -1.0, -1.0};
    Eigen::Vector2d wheel_axis_signs{1.0, -1.0};
    double dt = 0.005;
    // Frozen simulation uses 200 ms; zero selects direct deployment takeover.
    double blend_seconds = 0.2;
    // End any script at the caller's bounded upright capture before stabilization.
    bool dynamic_takeover = false;
    double dynamic_capture_height_min = 0.27;
    // Deployment may accept driver commands as soon as the actor owns all axes.
    bool release_motion_on_takeover = false;
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

struct JointReferenceRecoveryFeedback {
    JointReferenceRecoveryVector6 q = JointReferenceRecoveryVector6::Zero();
    JointReferenceRecoveryVector6 dq = JointReferenceRecoveryVector6::Zero();
    Eigen::Vector3d gravity{0.0, 0.0, -1.0};
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    double estimated_height = 0.0;
    // Native plausible conditional support; never true simulator contacts.
    bool support = false;
    // Wheel-probe evidence does not require the static standing height.
    bool support_confirmed = false;
    bool body_clear = false;
    // Caller validates the mechanism's upright pose/velocity envelope and sensing.
    bool rl_capture_ready = false;
    Eigen::Vector2d wheel_probe_torque = Eigen::Vector2d::Zero();
};

struct JointReferenceRecoveryCommand {
    JointReferenceRecoveryVector6 targets = JointReferenceRecoveryVector6::Zero();
    JointReferenceRecoveryVector6 continuous_q = JointReferenceRecoveryVector6::Zero();
    JointReferenceRecoveryPhase phase = JointReferenceRecoveryPhase::kRl;
    JointReferenceRecoveryRoute route = JointReferenceRecoveryRoute::kUpright;
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

class JointReferenceRecoveryController {
public:
    explicit JointReferenceRecoveryController(const JointReferenceRecoveryConfig& config);
    void reset(
        const JointReferenceRecoveryVector6& q, bool active = true,
        double initial_release_seconds = 0.0);
    // Advance once at the configured period. Sensor freshness, drive readiness,
    // geometry guards and support probing stay with the caller.
    const JointReferenceRecoveryCommand& update(const JointReferenceRecoveryFeedback& feedback);
    // The 1 kHz path integrates actual elapsed time; 200 Hz retains the frozen
    // fixed-period arithmetic for comparison with the training implementation.
    const JointReferenceRecoveryCommand&
        update(const JointReferenceRecoveryFeedback& feedback, double elapsed_seconds);
    const JointReferenceRecoveryCommand& command() const { return command_; }
    JointReferenceRecoveryVector6 project_feedback(const JointReferenceRecoveryVector6& q) const;
    // Reevaluate native PD on current feedback without advancing the reference.
    // rl_torque is output-side actor PD effort; both paths are clipped before
    // the optional 200 ms linear torque blend. Zero takes over in the ready tick.
    // The scripted path uses feedback PD and the mechanism's wheel torque limits.
    JointReferenceRecoveryVector6 torques(
        const JointReferenceRecoveryFeedback& feedback,
        const JointReferenceRecoveryVector6& rl_torque) const;
    JointReferenceRecoveryVector6
        effective_action_history(const JointReferenceRecoveryVector6& rl_action) const;
    static Eigen::Vector4d
        paired_delta(const Eigen::Vector4d& goal, const Eigen::Vector4d& reference);

private:
    using Vector4f = Eigen::Matrix<float, 4, 1>;
    using Vector6f = Eigen::Matrix<float, 6, 1>;
    struct StepContext {
        JointReferenceRecoveryPhase phase;
        JointReferenceRecoveryRoute selected_route;
        Eigen::Vector3f gravity;
        Eigen::Vector3f gyro;
        Vector6f dq;
        float height;
        float tilt;
        float pitch;
        float dt;
        float elapsed;
        double elapsed_seconds;
        bool near_upright;
    };
    enum class Winding { kCoupledNearest, kContinuous };
    struct JointGoal {
        Vector4f position;
        float speed;
        Winding winding;
    };
    struct ReferenceStep {
        Vector4f position;
        float remaining;
        float measured_error;
    };
    struct Durations {
        double age = 0.0;
        double phase = 0.0;
        double ready = 0.0;
        double stable = 0.0;
        double plant = 0.0;
        double unsupported = 0.0;
    };
    static Vector4f paired_delta_(const Vector4f& goal, const Vector4f& reference);
    JointGoal update_goal_(const StepContext& step, bool supported);
    ReferenceStep advance_reference_(const StepContext& step, const JointGoal& goal);
    void update_transitions_(
        const StepContext& step, const ReferenceStep& reference,
        const JointReferenceRecoveryFeedback& feedback, double dt);
    void update_wheels_(const StepContext& step, bool supported);
    bool reached_(std::uint64_t ticks, double elapsed, double seconds) const;
    bool phase_reached_(const StepContext& step, double seconds) const;
    float phase_elapsed_() const;
    void enter_(JointReferenceRecoveryPhase phase);
    void fail_(int code);
    void reset_fsm_(const Vector6f& q, bool active);
    void publish_();
    std::uint64_t ticks_(double seconds) const;
    JointReferenceRecoveryConfig config_;
    JointReferenceRecoveryCommand command_;
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
    int side_lower_leg_ = 0;
    JointReferenceRecoveryPhase phase_ = JointReferenceRecoveryPhase::kRl;
    JointReferenceRecoveryRoute route_ = JointReferenceRecoveryRoute::kUpright;
    std::uint64_t age_ticks_ = 0, phase_ticks_ = 0, ready_ticks_ = 0;
    std::uint64_t stable_ticks_ = 0, plant_ticks_ = 0, unsupported_ticks_ = 0;
    std::uint64_t elapsed_ticks_ = 0;
    Durations durations_;
    double elapsed_seconds_ = 0.0;
    int reroute_count_ = 0, failure_code_ = 0;
    bool enabled_ = false, membership_ = false, motion_released_ = true;
    bool pitch_initialized_ = false, release_finished_ = true;
    bool wheel_balance_active_ = false;
    float continuous_pitch_ = 0.0f, last_pitch_ = 0.0f;
    float orbit_angle_ = 0.0f, plant_pitch_ = 0.0f;
    double initial_release_seconds_ = 0.0;
};

} // namespace rmcs::rl
