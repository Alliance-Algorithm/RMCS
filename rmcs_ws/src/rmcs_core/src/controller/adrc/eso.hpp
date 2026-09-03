#pragma once

#include <algorithm>
#include <cmath>

namespace rmcs_core::controller::adrc {

class ESO {
public:
    struct Config {
        double h = 0.001;
        double b0 = 1.0;
        double w0 = 80.0;
        bool auto_beta = true;
        double beta1 = 240.0;
        double beta2 = 19200.0;
        double beta3 = 512000.0;
        double z3_limit = 1e9;
    };

    struct State {
        double z1 = 0.0;
        double z2 = 0.0;
        double z3 = 0.0;
    };

    struct Output {
        double z1 = 0.0;
        double z2 = 0.0;
        double z3 = 0.0;
        double e = 0.0;
    };

    ESO()
        : ESO(Config{}) {}

    explicit ESO(const Config& cfg) { set_config(cfg); }

    void set_config(const Config& cfg) {
        cfg_ = cfg;
        sanitize_config();
        if (cfg_.auto_beta) {
            cfg_.beta1 = 3.0 * cfg_.w0;
            cfg_.beta2 = 3.0 * cfg_.w0 * cfg_.w0;
            cfg_.beta3 = cfg_.w0 * cfg_.w0 * cfg_.w0;
        }
    }

    const Config& config() const { return cfg_; }
    const State& state() const { return state_; }

    void reset(double init_y = 0.0) {
        state_.z1 = init_y;
        state_.z2 = 0.0;
        state_.z3 = 0.0;
    }

    Output update(double y, double u) {
        const double e = state_.z1 - y;
        state_.z1 += cfg_.h * (state_.z2 - cfg_.beta1 * e);
        state_.z2 += cfg_.h * (state_.z3 + cfg_.b0 * u - cfg_.beta2 * e);
        state_.z3 += cfg_.h * (-cfg_.beta3 * e);
        state_.z3 = clamp(state_.z3, -cfg_.z3_limit, cfg_.z3_limit);

        return Output{state_.z1, state_.z2, state_.z3, e};
    }

private:
    static double clamp(double x, double lo, double hi) {
        return std::max(lo, std::min(x, hi));
    }

    void sanitize_config() {
        constexpr double kEps = 1e-9;
        cfg_.h = std::max(cfg_.h, kEps);
        if (std::fabs(cfg_.b0) < kEps) {
            cfg_.b0 = (cfg_.b0 >= 0.0) ? kEps : -kEps;
        }
        cfg_.w0 = std::max(cfg_.w0, kEps);
        cfg_.beta1 = std::max(cfg_.beta1, kEps);
        cfg_.beta2 = std::max(cfg_.beta2, kEps);
        cfg_.beta3 = std::max(cfg_.beta3, kEps);
        cfg_.z3_limit = std::max(cfg_.z3_limit, 0.0);
    }

    Config cfg_;
    State state_;
};

} // namespace rmcs_core::controller::adrc
