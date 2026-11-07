#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace rmcs_core::controller::chassis::wheel_leg {

class RequestProtocol {
public:
    enum class Admission { kEstimated, kDirect };
    RequestProtocol() = default;
    explicit RequestProtocol(Admission admission)
        : admission_(admission) {}
    struct Estimate {
        bool valid = false;
        double age = 0., vertical_velocity = 0., tilt = 0., angular_speed = 0.;
        std::array<double, 2> contact{};
    };
    struct Condition {
        bool active = false, safety_required = false;
        double height = 0., clock = 0.;
        int response = 0, end_reason = 0;
        std::size_t active_id = 0;
    };

    void reset(double now) {
        condition_ = {};
        last_time_ = now - .02;
        air_ticks_ = 0;
        flight_seen_ = cancelled_ = false;
        landing_since_.reset();
    }

    bool active() const { return condition_.active; }

    bool can_accept(double now, const Estimate& estimate) const {
        const auto observation = measurement(now, estimate);
        return !condition_.active && observation.consecutive && admitted(estimate, observation);
    }

    const Condition& update(
        double now, const Estimate& estimate, std::size_t request_id, double height,
        std::size_t cancel_id, bool allow_new) {
        const auto [clock_valid, consecutive, fresh, grounded, airborne, stable] =
            measurement(now, estimate);
        condition_.response = clock_valid ? 0 : 6;
        condition_.end_reason = 0;
        if (clock_valid && request_id != 0) {
            if (request_id <= max_seen_id_) {
                condition_.response = 2;
            } else {
                max_seen_id_ = request_id;
                const bool valid_height = std::isfinite(height) && height >= .03 && height <= .40;
                condition_.response = condition_.active ? 3 : !valid_height ? 4 : 5;
                if (cancel_id == request_id) {
                    condition_.response = 7;
                } else if (
                    !condition_.active && valid_height
                    && admitted(estimate, measurement(now, estimate)) && consecutive && allow_new) {
                    condition_ = {true, false, height, 0., 1, 0, request_id};
                    accepted_at_ = now;
                    air_ticks_ = 0;
                    flight_seen_ = cancelled_ = false;
                    landing_since_.reset();
                }
            }
        }
        const bool active = condition_.active;
        const double elapsed = active ? now - accepted_at_ : 0.;
        const bool cancel = active && cancel_id == condition_.active_id && clock_valid;
        const bool before_takeoff = cancel && grounded && !flight_seen_ && elapsed < .10;
        cancelled_ = cancelled_ || cancel;
        if (clock_valid) {
            air_ticks_ = active && airborne ? (consecutive ? air_ticks_ + 1 : 1) : 0;
            flight_seen_ = flight_seen_ || (active && air_ticks_ >= 2);
            if (active && stable && consecutive && (flight_seen_ || cancelled_)) {
                if (!landing_since_)
                    landing_since_ = now;
            } else {
                landing_since_.reset();
            }
        }
        const bool landed = landing_since_ && now - *landing_since_ >= 2. - 1e-9;
        const bool timeout = active && clock_valid && elapsed >= 6.;
        condition_.safety_required =
            !clock_valid || (active && (!fresh || !consecutive)) || timeout;
        if (before_takeoff || landed || timeout) {
            condition_.active = false;
            condition_.active_id = 0;
            condition_.height = 0.;
            condition_.end_reason = timeout ? 4 : cancelled_ ? 2 : 1;
            landing_since_.reset();
        }
        condition_.clock = condition_.active ? std::min(elapsed, 5.) : 0.;
        last_time_ = now;
        return condition_;
    }

private:
    struct Measurement {
        bool clock_valid, consecutive, fresh, grounded, airborne, stable;
    };
    Measurement measurement(double now, const Estimate& estimate) const {
        const bool clock_valid = std::isfinite(now) && (!last_time_ || now > *last_time_);
        const bool consecutive = last_time_ && std::abs(now - *last_time_ - .02) <= 1e-8;
        const bool finite = std::isfinite(estimate.age) && std::isfinite(estimate.vertical_velocity)
                         && std::isfinite(estimate.tilt) && std::isfinite(estimate.angular_speed)
                         && std::ranges::all_of(estimate.contact, [](double p) {
                                return std::isfinite(p) && p >= 0. && p <= 1.;
                            });
        const bool fresh = clock_valid && estimate.valid && finite && estimate.age > 0.
                        && estimate.age <= .03 + 1e-9;
        const bool grounded =
            fresh && std::ranges::all_of(estimate.contact, [](double p) { return p >= .8; });
        const bool airborne =
            fresh && std::ranges::all_of(estimate.contact, [](double p) { return p <= .2; });
        const bool stable = grounded && std::abs(estimate.vertical_velocity) <= .15
                         && estimate.tilt <= .25 && estimate.angular_speed <= .5;
        return {clock_valid, consecutive, fresh, grounded, airborne, stable};
    }
    bool admitted(const Estimate&, const Measurement& observation) const {
        return observation.fresh && (admission_ == Admission::kDirect || observation.stable);
    }
    Admission admission_ = Admission::kEstimated;
    Condition condition_;
    std::optional<double> last_time_, landing_since_;
    double accepted_at_ = 0.;
    std::size_t max_seen_id_ = 0;
    int air_ticks_ = 0;
    bool flight_seen_ = false, cancelled_ = false;
};

} // namespace rmcs_core::controller::chassis::wheel_leg
