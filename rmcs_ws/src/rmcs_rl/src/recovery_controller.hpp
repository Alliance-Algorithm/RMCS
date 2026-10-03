#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string_view>

#include <eigen3/Eigen/Dense>

namespace rmcs::rl {

// Policy P order: left hip, left auxiliary root, right hip, right auxiliary root,
// left wheel, right wheel. All torques/angles are at the model output shafts.
using RecoveryVector6 = Eigen::Matrix<double, 6, 1>;

// Provisional output-shaft torque-speed assumption, not a measured DM curve.
double conditional_dm_output_bound(
    double speed_rad_s, double rated_rpm, double rated_torque_nm, double peak_torque_nm) noexcept;

// A flat-ground recovery request starts with centered chassis controls. The
// operator's AUTO mode is only an acknowledgement, not a terrain measurement.
bool neutral_recovery_request(bool auto_mode, const Eigen::Vector3d& velocity) noexcept;
// Keep the policy at zero commanded velocity/jump across BLEND and the first
// full second after RL takes ownership. Invalid clocks fail closed.
bool hold_recovery_command(
    bool recovery_started, bool preparing, double seconds_since_rl, double stable_seconds) noexcept;

// Conservative cumulative above-rated torque budget; it does not claim to be
// an identified winding-temperature model. A measured limit is mandatory.
class RecoveryPeakBudget {
public:
    RecoveryPeakBudget() = default;
    ~RecoveryPeakBudget() = default;

    void configure(double rated_torque_nm, double available_seconds);
    void reset() noexcept { used_seconds_.fill(0.0); }
    void limit(Eigen::Vector4d& torque, double dt) noexcept;

private:
    double rated_torque_nm_ = 20.0;
    double available_seconds_ = 0.0;
    std::array<double, 4> used_seconds_{};
};

enum class RecoveryPhase : int {
    kIdle = 0,
    kFold = 1,
    kPlant = 2,
    kPrepare = 3,
    kBlend = 4,
    kComplete = 5,
    kFailed = 6,
    kOrbit = 7,
    kThrust = 8,
    kSideSwing = 9,
    kCapture = 10,
    kWaitGround = 11,
};

// Diagnostic IDs are the existing RMCS ABI. The Python reference assigns 10
// to WAIT_GROUND and 11 to CAPTURE; exchange shared phases by name instead.
std::string_view recovery_phase_name(RecoveryPhase phase) noexcept;
std::optional<RecoveryPhase> recovery_phase_from_name(std::string_view name) noexcept;

enum class RecoveryFailure : int {
    kNone,
    kInvalidFeedback,
    kTimeout,
    kNoContact,
    kNoReorientation,
    kOrbitExhausted,
    kLostUpright,
    kCancelled,
    kUnsafeRequest,
};

// A validated mechanism/support observer must provide the fields marked valid.
// Height is conditional on wheel contact; it is not an IMU-derived position.
struct RecoveryFeedback {
    std::chrono::steady_clock::time_point stamp{};
    RecoveryVector6 q = RecoveryVector6::Zero();
    RecoveryVector6 dq = RecoveryVector6::Zero();
    Eigen::Vector3d gravity = -Eigen::Vector3d::UnitZ();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    // Reconstructed from IMU attitude, calibrated joint axes and encoder rates.
    // A missing axis calibration leaves this invalid; wheel dq is not a substitute.
    Eigen::Vector2d world_wheel_omega = Eigen::Vector2d::Zero();
    double specific_force_norm_mps2 = 0.0;
    double height_if_grounded = 0.0;
    double wheel_height_difference = 0.0;           // left minus right, world Z
    double height_rate_mps = 0.0;
    double gyro_acceleration_rad_s2 = 0.0;
    Eigen::Vector2d wheel_acceleration_rad_s2 = Eigen::Vector2d::Zero();
    std::uint8_t probe_evidence_mask = 0;
    std::array<double, 2> spring_compensation_nm{}; // left/right: +hip, -aux
    std::array<double, 2> inner_knee_deg{};
    std::array<double, 2> inner_knee_slope_deg_per_rad{};
    std::array<double, 2> spring_compression_m{};
    bool geometry_valid = false;
    bool height_valid = false;
    bool wheel_heights_valid = false;
    // Legacy PLANT uses its own 30 ms geometric/IMU alignment dwell.
    bool alignment_candidate = false;
    bool contact_candidate = false;
    bool geometrically_supported = false;
    bool probe_confirmed = false;
    bool body_contact_suspected = false;
    bool world_wheel_omega_valid = false;
    bool support_confirmed = false;
    bool body_clear = false;
    bool settled = false;
    bool spring_compensation_valid = false;
};

// No operator motion until the posture remains upright and shell-clear for a
// full second. This is a conditional sensor check, not a contact-force sensor.
bool recovery_upright_for_motion(const RecoveryFeedback& feedback) noexcept;

struct RecoveryConfig {
    Eigen::Vector4d root_axis_y{-1.0, -1.0, 1.0, 1.0};
    Eigen::Vector2d wheel_axis_y{1.0, -1.0};
    // Simulation sensitivity candidate at 24 V. Not a hardware calibration.
    double orbit_speed = 5.0;
    double side_speed = 5.0;
    double rollover_speed = 5.25;
    double capture_speed = 4.0;
    double fold_speed = 2.0;
    double prepare_speed = 6.0;
    double stand_speed = 1.0;
    double active_timeout = 8.0;
    double blend_seconds = 0.2;
    Eigen::Vector4d fold{0.177818, 0.043622, -0.177818, -0.043629};
    Eigen::Vector4d thrust{0.434238, -0.170630, -0.434238, 0.170623};
    Eigen::Vector4d side_extended{0.711288, -0.417986, -0.711288, 0.417978};
    Eigen::Vector4d plant{0.420000, -0.137423, -0.420000, 0.137416};
    Eigen::Vector4d stand{0.326994, -0.071460, -0.332353, 0.073090};
    Eigen::Vector4d upright{0.310734, -0.053952, -0.318772, 0.056400};
    Eigen::Vector4d support_extended{0.673618, -0.377482, -0.678504, 0.379293};
    Eigen::Vector4d upright_support_extended{0.654732, -0.357185, -0.662062, 0.359905};
    Eigen::Vector4d capture_extended{0.486003, -0.208743, -0.491306, 0.210578};
};

struct RecoveryCommand {
    RecoveryVector6 torque = RecoveryVector6::Zero();
    RecoveryPhase phase = RecoveryPhase::kIdle;
    RecoveryFailure failure = RecoveryFailure::kNone;
    double blend = 0.0;
};

class RecoveryController {
public:
    explicit RecoveryController(RecoveryConfig config = {});
    ~RecoveryController() = default;
    RecoveryController(const RecoveryController&) = delete;
    RecoveryController& operator=(const RecoveryController&) = delete;
    RecoveryController(RecoveryController&&) = default;
    RecoveryController& operator=(RecoveryController&&) = default;

    bool start(const RecoveryFeedback& feedback);
    void reset() noexcept;
    RecoveryCommand step(const RecoveryFeedback& feedback, double dt);
    RecoveryPhase phase() const noexcept { return phase_; }
    RecoveryFailure failure() const noexcept { return failure_; }
    const Eigen::Vector4d& reference() const noexcept { return reference_; }

    // Shared winding for a pair of coaxial active outputs; orbit/swing references
    // deliberately use signed continuous differences instead.
    static Eigen::Vector4d
        paired_delta(const Eigen::Vector4d& goal, const Eigen::Vector4d& reference);

private:
    enum class Route { kStand, kPlant, kOrbitPositive, kOrbitNegative, kSide };

    bool valid_(const RecoveryFeedback& feedback) const noexcept;
    void transition_(RecoveryPhase next) noexcept;
    void fail_(RecoveryFailure reason) noexcept;
    void update_pitch_(const RecoveryFeedback& feedback) noexcept;
    bool ready_for_blend_(const RecoveryFeedback& feedback) const noexcept;
    double update_reference_(const RecoveryFeedback& feedback, double dt, double angle);
    void update_phase_(
        const RecoveryFeedback& feedback, double dt, double angle, double reference_error);
    RecoveryVector6 efforts_(const RecoveryFeedback& feedback) const;

    RecoveryConfig config_;
    RecoveryPhase phase_ = RecoveryPhase::kIdle;
    RecoveryFailure failure_ = RecoveryFailure::kNone;
    Route route_ = Route::kStand;
    Eigen::Vector4d reference_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d orbit_start_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d side_start_ = Eigen::Vector4d::Zero();
    Eigen::Vector4d side_direction_ = Eigen::Vector4d::Zero();
    double elapsed_ = 0.0;
    double phase_elapsed_ = 0.0;
    double ready_seconds_ = 0.0;
    double contact_seconds_ = 0.0;
    double inverted_seconds_ = 0.0;
    double orbit_angle_ = 0.0;
    double planted_orbit_ = 0.0;
    double planted_pitch_ = 0.0;
    double pitch_ = 0.0;
    double last_pitch_ = 0.0;
    bool side_started_lateral_ = false;
    int side_attempts_ = 0;
    int reroutes_ = 0;
};

} // namespace rmcs::rl
