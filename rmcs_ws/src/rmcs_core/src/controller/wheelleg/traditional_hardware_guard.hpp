#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>

namespace rmcs_core::wheelleg {

inline bool traditional_relative_angles_valid(
    const std::array<double, 6>& position, const std::array<double, 2>& lower,
    const std::array<double, 2>& upper, double tolerance) {
    for (std::size_t side = 0; side < 2; ++side) {
        const double relative = position[side * 2 + 1] - position[side * 2];
        if (!std::isfinite(relative) || relative < lower[side] - tolerance
            || relative > upper[side] + tolerance)
            return false;
    }
    return true;
}

// Check the measured pose before requesting DM enable, including the common
// hip rotation which cannot be checked from leg length alone. Rows use ABI v1.
inline bool traditional_start_pose_valid(
    std::span<const double> rows, const std::array<double, 6>& position, double maximum_error) {
    if (rows.size() < 54 || rows.size() % 27 != 0)
        return false;
    for (std::size_t side = 0; side < 2; ++side) {
        const auto hip = side * 2;
        const auto crank = hip + 1;
        const double relative = position[crank] - position[hip];
        bool found = false;
        for (std::size_t upper = 27; upper < rows.size(); upper += 27) {
            const auto lower = upper - 27;
            const double a = rows[lower + 1 + crank] - rows[lower + 1 + hip];
            const double b = rows[upper + 1 + crank] - rows[upper + 1 + hip];
            if (!std::isfinite(relative) || relative < std::min(a, b) || relative > std::max(a, b))
                continue;
            const double weight = (relative - a) / (b - a);
            for (const auto joint : {hip, crank}) {
                const double target =
                    std::lerp(rows[lower + 1 + joint], rows[upper + 1 + joint], weight);
                if (!std::isfinite(position[joint])
                    || std::abs(std::remainder(position[joint] - target, 2.0 * std::numbers::pi))
                           > maximum_error)
                    return false;
            }
            found = true;
            break;
        }
        if (!found)
            return false;
    }
    return true;
}

// The fixed-step math uses logical 1 ms ticks. This separate monotonic watchdog
// detects a real executor gap; it never synthesizes missing physical samples.
class TraditionalHardwareGuard {
public:
    struct Input {
        bool reset = false;
        bool arm = false;
        bool remote_fresh = false;
        bool control_mode = false;
        bool admitted = false;
        bool feedback_fresh = false;
        bool geometry_valid = true;
        bool attitude_valid = false;
        bool power_valid = false;
        bool startup_pose_valid = false;
        bool drives_ready = false;
        bool schedule_valid = true;
    };

    struct Output {
        bool enable = false;
        bool compute = false;
        int failure = 0;
    };

    Output step(
        std::uint64_t now, const Input& input, std::uint64_t maximum_gap,
        std::uint64_t enable_timeout) {
        const bool time_valid =
            previous_ns_ == 0 || (now > previous_ns_ && now - previous_ns_ <= maximum_gap);
        period_ns_ = previous_ns_ != 0 && now >= previous_ns_ ? now - previous_ns_ : 0;
        previous_ns_ = now;
        if (input.reset) {
            armed_ = running_ = false;
            fault_ = 0;
            enable_since_ = 0;
            return {};
        }
        if (!input.remote_fresh || !input.control_mode) {
            if (running_ && !input.remote_fresh)
                fault_ = 11;
            armed_ = running_ = false;
            enable_since_ = 0;
            return {.failure = fault_};
        }
        if (fault_)
            return {.failure = fault_};
        if (!input.admitted) {
            armed_ = false;
            return {.failure = 10};
        }
        if (input.arm)
            armed_ = true;
        if (!armed_)
            return {};
        const int failure = !input.feedback_fresh                              ? 12
                          : !input.geometry_valid                              ? 16
                          : !input.attitude_valid                              ? 13
                          : !input.power_valid                                 ? 14
                          : running_ && (!time_valid || !input.schedule_valid) ? 17
                                                                               : 0;
        if (failure) {
            if (running_)
                latch(failure);
            enable_since_ = 0;
            return {.failure = failure};
        }
        if (!running_ && !input.startup_pose_valid) {
            enable_since_ = 0;
            return {.failure = 15};
        }
        if (running_ && !input.drives_ready) {
            latch(18);
            return {.failure = fault_};
        }
        if (!input.drives_ready) {
            if (enable_since_ == 0)
                enable_since_ = now;
            if (now - enable_since_ > enable_timeout) {
                latch(18);
                return {.failure = fault_};
            }
            return {.enable = true};
        }
        running_ = true;
        return {.enable = true, .compute = true};
    }

    void latch(int failure) {
        fault_ = failure;
        armed_ = running_ = false;
        enable_since_ = 0;
    }

    [[nodiscard]] bool armed() const { return armed_; }
    [[nodiscard]] bool faulted() const { return fault_ != 0; }
    [[nodiscard]] std::uint64_t period_ns() const { return period_ns_; }

private:
    bool armed_ = false, running_ = false;
    int fault_ = 0;
    std::uint64_t previous_ns_ = 0, enable_since_ = 0, period_ns_ = 0;
};

} // namespace rmcs_core::wheelleg
