#pragma once

#include "recovery_observer.hpp"
#include "v6_recovery_profile.hpp"

namespace rmcs::rl {

// Native conditional FK and wheel-response probe. No contact, force, root
// position or simulator truth is accepted here. Freshness is enforced by the
// component's RecoverySensorGuard before each 200 Hz call.
class V6RecoveryObserver {
public:
    explicit V6RecoveryObserver(std::array<V6RecoverySideGeometry, 2> geometry);
    void reset();
    V6RecoveryFeedback observe(
        const RecoverySensorData& sensors, const V6RecoveryVector6& q, const V6RecoveryVector6& dq,
        const Eigen::Vector3d& gravity, const Eigen::Vector3d& gyro,
        const Eigen::Vector3d& specific_acceleration, V6RecoveryPhase phase);
    Eigen::Vector2d probe_command(bool scripted_and_released);
    bool geometry_valid() const { return geometry_valid_; }
    bool support_confirmed() const { return confirmed_; }
    Eigen::Vector2d height_candidates() const { return heights_.cast<double>(); }

private:
    std::array<V6RecoverySideGeometry, 2> geometry_;
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
    std::uint64_t reference_tick_ = 0;
    float lost_seconds_ = 0.0f;
    bool initialized_ = false, geometry_valid_ = false, plausible_ = false, confirmed_ = false;
};

} // namespace rmcs::rl
