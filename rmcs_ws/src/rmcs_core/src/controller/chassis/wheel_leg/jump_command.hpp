#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "model/jump_v4.hpp"

namespace rmcs_core::controller::chassis::wheel_leg {

class JumpCommand {
public:
    enum State { IDLE, CROUCH, PUSH, FLIGHT_REFERENCE, RECOVER_REFERENCE };

    explicit JumpCommand(double apex = .46)
        : apex_(apex) {
        if (!std::isfinite(apex) || apex < wheelbipe_jump_v4::kMinimumApex
            || apex > wheelbipe_jump_v4::kMaximumApex)
            throw std::invalid_argument("Jump apex must be within the v4 training range");
    }

    void reset() { active_ = false; }

    bool request(double now) {
        if (active_ || !std::isfinite(now))
            return false;
        started_ = now;
        active_ = true;
        return true;
    }

    void update(double now) {
        if (active_
            && (!std::isfinite(now) || now < started_
                || now - started_ >= wheelbipe_jump_v4::kRequestDuration))
            reset();
    }

    bool active() const { return active_; }
    double elapsed(double now) const { return active_ ? std::clamp(now - started_, 0., 6.) : 0.; }
    double apex() const { return apex_; }

    State state(double now) const {
        if (!active_)
            return IDLE;
        const double t = elapsed(now);
        if (t < wheelbipe_jump_v4::kCrouchSeconds)
            return CROUCH;
        if (t - wheelbipe_jump_v4::kCrouchSeconds <= push_duration())
            return PUSH;
        if (t - wheelbipe_jump_v4::kCrouchSeconds
            < push_duration() + 2. * release_velocity() / 9.81)
            return FLIGHT_REFERENCE;
        return RECOVER_REFERENCE;
    }

    double height(double now) const {
        const double t = elapsed(now);
        switch (state(now)) {
        case CROUCH: {
            const double p = t / wheelbipe_jump_v4::kCrouchSeconds;
            return wheelbipe_jump_v4::kStandingHeight
                 + (wheelbipe_jump_v4::kCrouchHeight - wheelbipe_jump_v4::kStandingHeight)
                       * (3. * p * p - 2. * p * p * p);
        }
        case PUSH: {
            const double p = t - wheelbipe_jump_v4::kCrouchSeconds;
            return wheelbipe_jump_v4::kCrouchHeight
                 + .5 * release_velocity() / push_duration() * p * p;
        }
        case FLIGHT_REFERENCE: return wheelbipe_jump_v4::kReleaseHeight;
        default: return wheelbipe_jump_v4::kStandingHeight;
        }
    }

    std::array<double, 7> context(double now) const {
        if (!active_)
            return {1., 0., 0., 0., 0., 0., 0.};
        return {0., 0., 0., 0., 1., apex_, elapsed(now)};
    }

private:
    double release_velocity() const {
        return std::sqrt(2. * 9.81 * (apex_ - wheelbipe_jump_v4::kReleaseHeight));
    }
    double push_duration() const {
        return 2. * (wheelbipe_jump_v4::kReleaseHeight - wheelbipe_jump_v4::kCrouchHeight)
             / release_velocity();
    }

    double apex_, started_ = 0.;
    bool active_ = false;
};

} // namespace rmcs_core::controller::chassis::wheel_leg
