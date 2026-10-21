#pragma once

#include <chrono>
#include <optional>
#include <stdexcept>

namespace rmcs_core::controller::chassis {

// A discontinuity stays invalid until the control session is reset. Use one
// sample for recovery observation/sequence integration and another at actuation
// for the time spent holding the previous torque.
class ControlInterval {
public:
    using Clock = std::chrono::steady_clock;

    void set_maximum_interval(Clock::duration maximum) {
        if (maximum <= Clock::duration::zero() || maximum > std::chrono::milliseconds{50})
            throw std::invalid_argument("Control interval must be in (0, 50 ms]");
        maximum_interval_ = maximum;
        reset();
    }

    void reset() noexcept {
        previous_.reset();
        valid_ = true;
    }

    std::optional<double> sample(
        Clock::time_point now, Clock::duration initial_interval = kInitialInterval) noexcept {
        if (!valid_)
            return std::nullopt;
        const auto elapsed = previous_ ? now - *previous_ : initial_interval;
        if (elapsed <= Clock::duration::zero() || elapsed > maximum_interval_) {
            valid_ = false;
            return std::nullopt;
        }
        previous_ = now;
        return std::chrono::duration<double>(elapsed).count();
    }

    bool fresh(Clock::time_point now) const noexcept {
        return valid_ && previous_ && now >= *previous_ && now - *previous_ <= maximum_interval_;
    }

private:
    static constexpr auto kInitialInterval = std::chrono::milliseconds{5};
    Clock::duration maximum_interval_ = std::chrono::milliseconds{20};
    std::optional<Clock::time_point> previous_;
    bool valid_ = true;
};

} // namespace rmcs_core::controller::chassis
