#pragma once

#include "recovery_controller.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace rmcs::rl {

// Calibrated tables belong to the installed mechanism, not to the ONNX model.
// One row records the wheel center with the corresponding hip at model angle 0.
struct RecoverySideTable {
    std::vector<double> delta_rad;
    std::vector<double> inner_knee_deg;
    std::vector<double> slider_m;
    std::vector<Eigen::Vector3d> wheel_at_hip_zero_m;
    // FK axes at hip angle zero, sampled on the same relative-root LUT.
    std::vector<Eigen::Vector3d> knee_axis_at_hip_zero;
    std::vector<Eigen::Vector3d> wheel_axis_at_hip_zero;
    // The reference uses interpolated nodal derivatives, not segment secants.
    std::vector<double> slider_slope_m_per_rad;
    double passive_knee_sign = 0.0;
    Eigen::Vector3d hip_origin_m = Eigen::Vector3d::Zero();
    Eigen::Vector3d hip_axis = Eigen::Vector3d::UnitY();
    double spring_compression_at_zero_m = 0.0;
};

struct RecoveryMechanism {
    std::array<RecoverySideTable, 2> sides;
    std::vector<Eigen::Vector3d> shell_points_body_m;
    double wheel_radius_m = 0.06;
    double spring_stroke_m = 0.08;
    // Real-force curve in N: F(u) = c0 + c1*u + c2*u² + c3*u³.
    std::array<double, 4> spring_force_n{};
};

// All wheel efforts use model output-shaft coordinates. Submission means a
// frame was queued in software; only a later CAN current response corroborates it.
struct RecoverySensorData {
    std::uint64_t steady_ns = 0;
    std::array<std::uint64_t, 2> wheel_feedback_ns{};
    std::array<std::uint64_t, 2> wheel_feedback_sequence{};
    Eigen::Vector2d wheel_torque_feedback_nm = Eigen::Vector2d::Zero();
    Eigen::Vector2d wheel_torque_submitted_nm = Eigen::Vector2d::Zero();
    std::array<std::uint64_t, 2> wheel_torque_submitted_ns{};
    std::array<std::uint8_t, 2> wheel_tx_kind{};
    std::uint64_t imu_feedback_ns = 0;
    std::uint64_t imu_feedback_sequence = 0;
    std::optional<Eigen::Quaterniond> world_base_orientation;
};

// Provisional sensor/probe thresholds for offline validation. Hardware profiles
// must validate current response, inertia/friction, CAN delays and probe effort.
struct RecoveryObserverConfig {
    double probe_torque_nm = 0.18;
    double minimum_submitted_torque_nm = 0.14;
    double minimum_feedback_torque_nm = 0.025;
    double feedback_torque_ratio = 0.25;
    double pulse_seconds = 0.015;
    // Minimum age of the corroborated pulse, not consecutive quiet samples.
    double response_seconds = 0.005;
    double quiet_seconds = 0.05;
    double support_seconds = 0.1;
    double evidence_ttl_seconds = 0.6;
    double maximum_sample_age_seconds = 0.02;
    double maximum_imu_sample_age_seconds = 0.02;
    double maximum_submission_age_seconds = 0.02;
    double maximum_feedback_interval_seconds = 0.02;
    double maximum_wheel_acceleration_rad_s2 = 100.0;
    double maximum_gyro_acceleration_rad_s2 = 90.0;
};

class RecoveryObserver {
public:
    explicit RecoveryObserver(RecoveryMechanism mechanism, RecoveryObserverConfig config = {});
    ~RecoveryObserver() = default;
    RecoveryObserver(const RecoveryObserver&) = delete;
    RecoveryObserver& operator=(const RecoveryObserver&) = delete;
    RecoveryObserver(RecoveryObserver&&) = default;
    RecoveryObserver& operator=(RecoveryObserver&&) = default;

    void reset() noexcept;
    bool sensor_baseline_ready() const noexcept {
        return initialized_ && last_imu_sequence_ != 0 && last_wheel_sequence_[0] != 0
            && last_wheel_sequence_[1] != 0;
    }
    RecoveryFeedback update(
        const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
        const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt);
    RecoveryFeedback update(
        const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
        const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt,
        const RecoverySensorData& sensors);
    // The owner sends this through its single wheel-torque output, after bounds.
    Eigen::Vector2d probe_command(bool preparing, const RecoveryFeedback& feedback);
    std::array<double, 2> inner_knee_limits_deg(std::size_t side) const noexcept;
    void constrain_policy_goal(Eigen::Vector4d& goal, double knee_margin_deg) const;

private:
    bool update_wheel_samples_(
        const RecoverySensorData& sensors, const Eigen::Vector2d& velocity,
        std::array<bool, 2>& new_samples, Eigen::Vector2d& acceleration);
    bool update_imu_sample_(
        const RecoverySensorData& sensors, const Eigen::Vector3d& omega, bool& new_sample,
        double& acceleration);
    void clear_sensor_evidence_() noexcept;

    RecoveryMechanism mechanism_;
    RecoveryObserverConfig config_;
    Eigen::Vector2d last_wheel_velocity_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_probe_ = Eigen::Vector2d::Zero();
    Eigen::Vector3d last_omega_ = Eigen::Vector3d::Zero();
    std::array<std::array<bool, 2>, 2> probe_evidence_{};
    std::array<std::array<double, 2>, 2> evidence_age_{};
    std::array<std::array<int, 2>, 2> bad_probe_samples_{};
    std::array<std::uint64_t, 2> last_wheel_sample_ns_{}, last_wheel_sequence_{};
    std::array<std::uint64_t, 2> probe_submission_ns_{};
    std::array<std::uint64_t, 2> probe_request_ns_{};
    std::array<int, 2> probe_submission_direction_{};
    std::uint64_t last_sensor_ns_ = 0;
    std::uint64_t current_sensor_ns_ = 0;
    std::uint64_t last_imu_sample_ns_ = 0;
    std::uint64_t last_imu_sequence_ = 0;
    double last_height_m_ = 0.0;
    double filtered_height_rate_mps_ = 0.0;
    double impact_age_s_ = 10.0;
    double support_seconds_ = 0.0;
    double lost_support_seconds_ = 0.0;
    double quiet_seconds_ = 0.0;
    double alignment_seconds_ = 0.0;
    double probe_elapsed_seconds_ = 0.0;
    double last_update_dt_ = 0.0;
    bool initialized_ = false;
};

} // namespace rmcs::rl
