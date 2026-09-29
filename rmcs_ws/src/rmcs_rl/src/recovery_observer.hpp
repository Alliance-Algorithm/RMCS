#pragma once

#include "recovery_controller.hpp"

#include <array>
#include <vector>

namespace rmcs::rl {

// Calibrated tables belong to the installed mechanism, not to the ONNX model.
// One row records the wheel center with the corresponding hip at model angle 0.
struct RecoverySideTable {
    std::vector<double> delta_rad;
    std::vector<double> inner_knee_deg;
    std::vector<double> slider_m;
    std::vector<Eigen::Vector3d> wheel_at_hip_zero_m;
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

class RecoveryObserver {
public:
    explicit RecoveryObserver(RecoveryMechanism mechanism);
    ~RecoveryObserver() = default;
    RecoveryObserver(const RecoveryObserver&) = delete;
    RecoveryObserver& operator=(const RecoveryObserver&) = delete;
    RecoveryObserver(RecoveryObserver&&) = default;
    RecoveryObserver& operator=(RecoveryObserver&&) = default;

    void reset() noexcept;
    RecoveryFeedback update(
        const RecoveryVector6& q, const RecoveryVector6& dq, const Eigen::Vector3d& gravity,
        const Eigen::Vector3d& omega, const Eigen::Vector3d& acceleration, double dt);
    // The owner sends this through its single wheel-torque output, after bounds.
    Eigen::Vector2d probe_command(bool preparing, const RecoveryFeedback& feedback);
    void constrain_policy_goal(Eigen::Vector4d& goal, double knee_margin_deg) const;

private:
    RecoveryMechanism mechanism_;
    Eigen::Vector2d last_wheel_velocity_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d last_probe_ = Eigen::Vector2d::Zero();
    Eigen::Vector3d last_omega_ = Eigen::Vector3d::Zero();
    std::array<std::array<bool, 2>, 2> probe_evidence_{};
    std::array<std::array<double, 2>, 2> evidence_age_{};
    double last_height_m_ = 0.0;
    double filtered_height_rate_mps_ = 0.0;
    double impact_age_s_ = 10.0;
    double support_seconds_ = 0.0;
    double quiet_seconds_ = 0.0;
    int tick_ = 0;
    bool initialized_ = false;
};

} // namespace rmcs::rl
