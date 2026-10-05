#pragma once

#include "recovery_observer.hpp"
#include "recovery_profile.hpp"

#include <cstdint>

namespace rmcs::rl {

// Native conditional FK and wheel-response probe. No contact, force, root
// position or simulator truth is accepted here. Freshness is enforced by the
// component's RecoverySensorGuard before each scheduled observation.
class ClosedChainSupportObserver {
public:
    explicit ClosedChainSupportObserver(
        std::array<ClosedChainLegGeometry, 2> geometry, double period_seconds = 0.005);
    void reset();
    JointReferenceRecoveryFeedback observe(
        const RecoverySensorData& sensors, const JointReferenceRecoveryVector6& q,
        const JointReferenceRecoveryVector6& dq, const Eigen::Vector3d& gravity,
        const Eigen::Vector3d& gyro, const Eigen::Vector3d& specific_acceleration,
        JointReferenceRecoveryPhase phase);
    // The 1 kHz path uses actual elapsed time; 200 Hz preserves fixed-step
    // arithmetic. An invalid interval clears evidence and returns nonfinite q.
    JointReferenceRecoveryFeedback observe(
        const RecoverySensorData& sensors, const JointReferenceRecoveryVector6& q,
        const JointReferenceRecoveryVector6& dq, const Eigen::Vector3d& gravity,
        const Eigen::Vector3d& gyro, const Eigen::Vector3d& specific_acceleration,
        JointReferenceRecoveryPhase phase, double elapsed_seconds);
    Eigen::Vector2d probe_command(bool scripted_and_released);
    Eigen::Vector2d probe_command(bool scripted_and_released, double elapsed_seconds);
    bool geometry_valid() const { return geometry_valid_; }
    bool support_confirmed() const { return confirmed_; }
    Eigen::Vector2d height_candidates() const { return heights_.cast<double>(); }

private:
    std::array<ClosedChainLegGeometry, 2> geometry_;
    Eigen::Matrix<float, 6, 1> q_ = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<float, 6, 1> last_raw_ = Eigen::Matrix<float, 6, 1>::Zero();
    Eigen::Matrix<int, 6, 1> winding_ = Eigen::Matrix<int, 6, 1>::Zero();
    Eigen::Vector2f previous_velocity_ = Eigen::Vector2f::Zero();
    Eigen::Vector3f previous_gyro_ = Eigen::Vector3f::Zero();
    Eigen::Vector2f previous_pulse_ = Eigen::Vector2f::Zero();
    Eigen::Vector2f heights_ = Eigen::Vector2f::Zero();
    std::array<std::array<bool, 2>, 2> evidence_{};
    std::array<std::array<float, 2>, 2> evidence_age_{};
    std::array<std::array<int, 2>, 2> bad_samples_{};
    std::array<std::array<double, 2>, 2> evidence_elapsed_seconds_{};
    std::array<std::array<double, 2>, 2> bad_elapsed_seconds_{};
    double period_seconds_ = 0.005;
    std::uint64_t reference_tick_ = 0;
    float lost_seconds_ = 0.0f;
    double lost_elapsed_seconds_ = 0.0;
    double probe_elapsed_seconds_ = 0.0;
    bool initialized_ = false, geometry_valid_ = false, plausible_ = false, confirmed_ = false;
};

} // namespace rmcs::rl
