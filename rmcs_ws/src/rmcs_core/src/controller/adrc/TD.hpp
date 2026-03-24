#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace rmcs_core::controller::adrc {

class TD {
public:
    struct Config {
        double r = 300.0;
        double h = 0.003;
        double max_vel = std::numeric_limits<double>::infinity();
        double max_acc = std::numeric_limits<double>::infinity();
    };

    struct State {
        double x1 = 0.0;
        double x2 = 0.0;
    };

    struct Output {
        double x1 = 0.0;
        double x2 = 0.0;
        double fh = 0.0;
    };

    TD()
        : TD(Config{}) {}

    explicit TD(const Config& cfg) { set_config(cfg); }

    void set_config(const Config& cfg) {
        cfg_ = cfg;
        sanitize_config();
    }

    const Config& config() const { return cfg_; }
    const State& state() const { return state_; }

    void reset(double init_x1 = 0.0, double init_x2 = 0.0) {
        state_.x1 = init_x1;
        state_.x2 = init_x2;
    }

    Output update(double v) {
        const double fh_raw = fhan(state_.x1 - v, state_.x2, cfg_.r, cfg_.h);
        const double fh = clamp(fh_raw, -cfg_.max_acc, cfg_.max_acc);

        state_.x1 += cfg_.h * state_.x2;
        state_.x2 += cfg_.h * fh;
        state_.x2 = clamp(state_.x2, -cfg_.max_vel, cfg_.max_vel);

        return Output{state_.x1, state_.x2, fh};
    }

private:
    static double sign(double x) {
        if (x > 0.0) return 1.0;
        if (x < 0.0) return -1.0;
        return 0.0;
    }

    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    static double fhan(double x1_minus_v, double x2, double r, double h) {
        const double d = r * h * h;
        const double a0 = h * x2;
        const double y = x1_minus_v + a0;
        const double a1 = std::sqrt(d * (d + 8.0 * std::fabs(y)));

        const double a = (std::fabs(y) > d)
            ? (a0 + sign(y) * (a1 - d) * 0.5)
            : (a0 + y);

        if (std::fabs(a) <= d) {
            return -r * a / d;
        }
        return -r * sign(a);
    }

    void sanitize_config() {
        constexpr double kEps = 1e-9;
        cfg_.r = std::max(cfg_.r, kEps);
        cfg_.h = std::max(cfg_.h, kEps);
        cfg_.max_vel = std::max(cfg_.max_vel, 0.0);
        cfg_.max_acc = std::max(cfg_.max_acc, 0.0);
    }

    Config cfg_;
    State state_;
};

} // namespace rmcs_core::controller::adrc
