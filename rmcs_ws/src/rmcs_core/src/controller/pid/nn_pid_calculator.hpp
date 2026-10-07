#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace rmcs_core::controller::pid {

// Gains retain RMCS's per-update integral/difference convention, not continuous-time units.
// The plant derivative is an approximation: this is an experimental adaptive controller,
// not a Lyapunov-stable controller or an exact gradient through the physical plant.
class NnPidCalculator {
public:
    using Vector = std::array<double, 3>;

    struct Config {
        Vector base{1.0, 0.0, 0.0};
        Vector delta{0.5, 0.0, 0.0};
        Vector input_scale{1.0, 1000.0, 0.01};
        std::size_t hidden_size = 5;
        std::size_t learning_interval = 10;
        std::uint32_t seed = 6020;
        double learning_rate = 0.001;
        double plant_sensitivity = 1.0; // Approximate dy(k)/du(k-1), including sign.
        double gradient_limit = 1.0;
        double weight_limit = 3.0;
        double learning_error_limit = 2.0;
        double integral_min = -1000.0;
        double integral_max = 1000.0;
        double integral_split_min = -std::numeric_limits<double>::infinity();
        double integral_split_max = std::numeric_limits<double>::infinity();
        double output_min = -1.0;
        double output_max = 1.0;
    };

    explicit NnPidCalculator(const Config& config)
        : config_(config)
        , hidden_(config.hidden_size)
        , w1_(config.hidden_size)
        , b1_(config.hidden_size)
        , w2_(config.hidden_size)
        , gradient_w1_(config.hidden_size)
        , gradient_b1_(config.hidden_size)
        , gradient_w2_(config.hidden_size) {
        validate();
        reset();
    }

    void reset() {
        // A local deterministic generator makes every run/restart reproducible.
        auto state = config_.seed;
        auto random = [&state]() {
            state = state * 1664525U + 1013904223U;
            return 0.2 * (static_cast<double>(state) / 4294967295.0 - 0.5);
        };
        for (std::size_t h = 0; h < w1_.size(); ++h) {
            for (auto& weight : w1_[h])
                weight = random();
            b1_[h] = random();
            w2_[h].fill(0.0);
        }
        b2_.fill(0.0);
        gains_ = config_.base;
        integral_ = 0.0;
        last_error_ = std::numeric_limits<double>::quiet_NaN();
        gradient_ready_ = false;
        saturated_ = false;
        learning_steps_ = 0;
        ticks_ = 0;
    }

    double update(double error, double feedforward = 0.0) {
        if (!std::isfinite(error) || !std::isfinite(feedforward)) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }

        // e(k) is associated with the cached gradient of u(k-1), not u(k).
        // Saturated actuator output carries no valid unconstrained control gradient.
        if (++ticks_ % config_.learning_interval == 0 && gradient_ready_
            && std::abs(error) <= config_.learning_error_limit) {
            learn(-error * config_.plant_sensitivity);
        }

        Vector features{
            error,
            error > config_.integral_split_min && error < config_.integral_split_max ? integral_
                                                                                     : 0.0,
            std::isfinite(last_error_) ? error - last_error_ : 0.0};
        Vector input;
        for (std::size_t i = 0; i < 3; ++i)
            input[i] = std::clamp(features[i] / config_.input_scale[i], -1.0, 1.0);

        for (std::size_t h = 0; h < hidden_.size(); ++h) {
            double value = b1_[h];
            for (std::size_t i = 0; i < 3; ++i)
                value += w1_[h][i] * input[i];
            hidden_[h] = std::tanh(value);
        }
        Vector output = b2_;
        Vector output_gradient;
        double command = feedforward;
        for (std::size_t j = 0; j < 3; ++j) {
            for (std::size_t h = 0; h < hidden_.size(); ++h)
                output[j] += w2_[h][j] * hidden_[h];
            output[j] = std::tanh(output[j]);
            gains_[j] = config_.base[j] + config_.delta[j] * output[j];
            command += gains_[j] * features[j];
            output_gradient[j] = features[j] * config_.delta[j] * (1.0 - output[j] * output[j]);
        }

        if (!std::isfinite(command)) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }
        saturated_ = command < config_.output_min || command > config_.output_max;
        gradient_ready_ = !saturated_;
        gradient_b2_ = output_gradient;
        for (std::size_t h = 0; h < hidden_.size(); ++h) {
            double hidden_gradient = 0.0;
            for (std::size_t j = 0; j < 3; ++j) {
                gradient_w2_[h][j] = output_gradient[j] * hidden_[h];
                hidden_gradient += output_gradient[j] * w2_[h][j];
            }
            hidden_gradient *= 1.0 - hidden_[h] * hidden_[h];
            gradient_b1_[h] = hidden_gradient;
            for (std::size_t i = 0; i < 3; ++i)
                gradient_w1_[h][i] = hidden_gradient * input[i];
        }

        integral_ = error > config_.integral_split_min && error < config_.integral_split_max
                      ? std::clamp(integral_ + error, config_.integral_min, config_.integral_max)
                      : 0.0;
        last_error_ = error;
        return std::clamp(command, config_.output_min, config_.output_max);
    }

    const Vector& gains() const { return gains_; }
    std::size_t learning_steps() const { return learning_steps_; }
    bool saturated() const { return saturated_; }

private:
    void validate() const {
        auto positive = [](double value) { return std::isfinite(value) && value > 0.0; };
        if (config_.hidden_size == 0 || config_.hidden_size > 64 || config_.learning_interval == 0
            || !positive(config_.gradient_limit) || !positive(config_.weight_limit)
            || !positive(config_.learning_error_limit) || !positive(config_.learning_rate)
            || !std::isfinite(config_.plant_sensitivity) || config_.plant_sensitivity == 0.0
            || !std::isfinite(config_.output_min) || !std::isfinite(config_.output_max)
            || config_.output_min >= config_.output_max || !std::isfinite(config_.integral_min)
            || !std::isfinite(config_.integral_max) || config_.integral_min > 0.0
            || config_.integral_max < 0.0
            || !(config_.integral_split_min < config_.integral_split_max))
            throw std::invalid_argument("Invalid NN-PID configuration");
        for (std::size_t i = 0; i < 3; ++i) {
            if (!std::isfinite(config_.base[i]) || !std::isfinite(config_.delta[i])
                || config_.delta[i] < 0.0 || !positive(config_.input_scale[i])
                || !std::isfinite(config_.base[i] + config_.delta[i])
                || !std::isfinite(config_.base[i] - config_.delta[i]))
                throw std::invalid_argument("Invalid NN-PID gain or input scale");
        }
    }

    void learn(double loss_gradient) {
        if (!std::isfinite(loss_gradient))
            return;
        double norm = 0.0;
        auto accumulate = [&norm](double value) { norm = std::hypot(norm, value); };
        for (double g : gradient_b2_)
            accumulate(g * loss_gradient);
        for (std::size_t h = 0; h < hidden_.size(); ++h) {
            accumulate(gradient_b1_[h] * loss_gradient);
            for (std::size_t i = 0; i < 3; ++i) {
                accumulate(gradient_w1_[h][i] * loss_gradient);
                accumulate(gradient_w2_[h][i] * loss_gradient);
            }
        }
        if (!std::isfinite(norm) || norm == 0.0)
            return;
        const double step =
            config_.learning_rate * loss_gradient * std::min(1.0, config_.gradient_limit / norm);
        auto apply = [this, step](double& weight, double gradient) {
            weight =
                std::clamp(weight - step * gradient, -config_.weight_limit, config_.weight_limit);
        };
        for (std::size_t j = 0; j < 3; ++j)
            apply(b2_[j], gradient_b2_[j]);
        for (std::size_t h = 0; h < hidden_.size(); ++h) {
            apply(b1_[h], gradient_b1_[h]);
            for (std::size_t i = 0; i < 3; ++i) {
                apply(w1_[h][i], gradient_w1_[h][i]);
                apply(w2_[h][i], gradient_w2_[h][i]);
            }
        }
        ++learning_steps_;
    }

    Config config_;
    std::vector<double> hidden_;
    std::vector<Vector> w1_;
    std::vector<double> b1_;
    std::vector<Vector> w2_;
    Vector b2_{};
    std::vector<Vector> gradient_w1_;
    std::vector<double> gradient_b1_;
    std::vector<Vector> gradient_w2_;
    Vector gradient_b2_{};
    Vector gains_{};
    double integral_ = 0.0;
    double last_error_ = 0.0;
    bool gradient_ready_ = false;
    bool saturated_ = false;
    std::size_t learning_steps_ = 0;
    std::size_t ticks_ = 0;
};

} // namespace rmcs_core::controller::pid
