#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

namespace rmcs_core::referee::app::ui {

class AnimatedToggle {
public:
    using Clock = std::chrono::steady_clock;

    explicit AnimatedToggle(
        std::chrono::duration<double> duration = std::chrono::duration<double>{0.5})
        : duration_(duration) {}

    void set_duration(std::chrono::duration<double> duration) {
        duration_ = std::max(duration, std::chrono::duration<double>{0.0});
    }

    void reset(bool active) {
        initialized_ = false;
        value_ = active ? 1.0 : 0.0;
        target_ = active;
    }

    double update(Clock::time_point now, bool active) {
        if (!initialized_) {
            initialized_ = true;
            start_time_ = now;
            start_value_ = active ? 1.0 : 0.0;
            end_value_ = start_value_;
            value_ = start_value_;
            target_ = active;
            return value_;
        }

        if (active != target_) {
            target_ = active;
            start_value_ = value_;
            end_value_ = active ? 1.0 : 0.0;
            start_time_ = now;
        }

        if (duration_.count() <= 0.0) {
            value_ = end_value_;
            return value_;
        }

        const double elapsed = std::chrono::duration<double>{now - start_time_}.count();
        const double t = std::clamp(elapsed / duration_.count(), 0.0, 1.0);
        value_ = std::lerp(start_value_, end_value_, ease_in_out_cubic_(t));
        return value_;
    }

    double value() const { return value_; }

private:
    static double ease_in_out_cubic_(double t) {
        t = std::clamp(t, 0.0, 1.0);
        if (t < 0.5)
            return 4.0 * t * t * t;
        return 1.0 - std::pow(-2.0 * t + 2.0, 3.0) / 2.0;
    }

    std::chrono::duration<double> duration_;
    Clock::time_point start_time_{};
    double start_value_ = 0.0;
    double end_value_ = 0.0;
    double value_ = 0.0;
    bool initialized_ = false;
    bool target_ = false;
};

} // namespace rmcs_core::referee::app::ui
