#pragma once

#include <cmath>
#include <limits>
#include <numbers>

namespace rmcs_core::controller::chassis::wheel_leg {

inline constexpr double kYawAlignTolerance = 10. * std::numbers::pi / 180.;

inline double recovery_yaw_error(double yaw) { return -std::remainder(yaw, std::numbers::pi); }

inline bool recovery_yaw_aligned(double yaw) {
    return std::isfinite(yaw) && std::abs(recovery_yaw_error(yaw)) <= kYawAlignTolerance;
}

class RecoveryYawAlignment {
public:
    void reset() { initialized_ = false; }

    double update(double yaw, double now) {
        if (!std::isfinite(yaw) || !std::isfinite(now)) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (!initialized_) {
            // 就近锁存正前或正后作为目标，并从此刻起计超时
            target_ = std::round(yaw / std::numbers::pi) * std::numbers::pi;
            deadline_ = now + kAttemptSeconds;
            initialized_ = true;
        }
        error_ = std::remainder(target_ - yaw, 2. * std::numbers::pi);
        if (std::abs(error_) <= kYawAlignTolerance)
            deadline_ = now + kAttemptSeconds;
        else if (now >= deadline_) {
            // 超时未进 ±10°：换转动方向，改试 ±180° 的另一个正前/正后
            target_ -= std::copysign(std::numbers::pi, error_);
            error_ = std::remainder(target_ - yaw, 2. * std::numbers::pi);
            deadline_ = now + kAttemptSeconds;
        }
        return error_ * kApproachGain;
    }

private:
    // 实测 yaw 轴黏滞 4.5~13 N·m，纯 P 稳态停在 12°，2.5 倍驱动才能压进 ±10° 判定带
    static constexpr double kApproachGain = 2.5;
    static constexpr double kAttemptSeconds = 2.;
    bool initialized_ = false;
    double target_ = 0., error_ = 0., deadline_ = 0.;
};

} // namespace rmcs_core::controller::chassis::wheel_leg
