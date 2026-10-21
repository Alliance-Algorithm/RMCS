#pragma once

#include <array>
#include <filesystem>
#include <string_view>
#include <vector>

#include <eigen3/Eigen/Core>

namespace rmcs_core::controller::chassis {

struct RecoveryTuckConfig {
    double knee_deg = 40.5;
    double knee_tolerance_deg = 0.5;
    double max_sagittal_error_m = 0.005;
    double integral_gain = 100.0;
    double integral_limit_nm = 20.0;
    bool body_fixed = false;
};

struct RecoveryTuckState {
    Eigen::Vector4d reference = Eigen::Vector4d::Zero();
    Eigen::Vector4d fold_reference = Eigen::Vector4d::Zero();
    Eigen::Vector4d hold_torque = Eigen::Vector4d::Zero();
    Eigen::Vector2d knee_deg = Eigen::Vector2d::Zero();
    Eigen::Vector2d sagittal_error_m = Eigen::Vector2d::Zero();
    Eigen::Vector3d com_b_m = Eigen::Vector3d::Zero();
    std::array<Eigen::Vector3d, 2> wheels_b_m{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    bool valid = false;
    bool ready = false;
    bool folded = false;
    bool knees_at_target = false;
};

// Closed-chain geometry and bounded load rejection for the final recovery tuck.
// Both wheels align fore/aft with the whole robot COM. Trajectory recovery uses
// body down; legacy recovery retains alignment along measured gravity.
// Their lateral separation remains the wheel support span.
class RecoveryTuckController {
public:
    RecoveryTuckController(
        const std::filesystem::path& geometry, std::string_view model_spec_sha256,
        const RecoveryTuckConfig& config);
    void reset();
    RecoveryTuckState
        update(const Eigen::Vector4d& q, const Eigen::Vector3d& gravity, double dt, bool tucking);
    // Integrate against the executed, rate-limited reference, including FOLD.
    RecoveryTuckState update(
        const Eigen::Vector4d& q, const Eigen::Vector3d& gravity, double dt, bool tucking,
        const Eigen::Vector4d& executed_reference, const Eigen::Vector4d& dq);
    RecoveryTuckState measure(const Eigen::Vector4d& q, const Eigen::Vector3d& gravity) const;

private:
    struct Knot {
        double delta = 0.0;
        double knee_deg = 0.0;
        Eigen::Vector3d wheel = Eigen::Vector3d::Zero();
        Eigen::Vector3d moment = Eigen::Vector3d::Zero();
    };
    struct Leg {
        Eigen::Vector3d pivot = Eigen::Vector3d::Zero();
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        double hip_reference = 0.0;
        double mass = 0.0;
        std::vector<Knot> knots;
    };
    Knot interpolate_(int side, double delta) const;
    Eigen::Vector3d rotate_(int side, const Eigen::Vector3d& vector, double angle) const;
    void integrate_(
        const Eigen::Vector4d& q, const Eigen::Vector4d& reference, const Eigen::Vector4d& dq,
        double dt);
    RecoveryTuckConfig config_;
    std::array<Leg, 2> legs_;
    Eigen::Vector3d base_moment_ = Eigen::Vector3d::Zero();
    double mass_ = 0.0;
    Eigen::Vector4d integral_ = Eigen::Vector4d::Zero();
};

} // namespace rmcs_core::controller::chassis
