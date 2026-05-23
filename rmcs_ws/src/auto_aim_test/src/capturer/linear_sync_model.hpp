#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>

#include <eigen3/Eigen/Dense>

class LinearSyncModel final {
public:
    struct TimestampPair {
        double camera_timestamp_sec = 0.0;
        double board_timestamp_sec = 0.0;
    };

    struct LeastSquaresResult {
        double a = 1.0;
        double b = 0.0;
        Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
        double max_abs_residual_sec = 0.0;
        std::size_t sample_count = 0;
    };

    static constexpr double kDefaultResidualThresholdSec = 1.0 / 249.1 / 2.0;

    explicit LinearSyncModel(
        const double tau_sec = 8.0,
        const double residual_threshold_sec = kDefaultResidualThresholdSec) noexcept
        : tau_sec_{tau_sec > 0.0 ? tau_sec : 8.0}
        , residual_threshold_sec_{
              residual_threshold_sec > 0.0 ? residual_threshold_sec
                                           : kDefaultResidualThresholdSec} {}

    auto reset() noexcept -> void {
        initialized_ = false;
        sample_count_ = 0;
        has_last_timestamp_ = false;
        theta_.setZero();
        covariance_.setZero();
    }

    [[nodiscard]] static auto fit_least_squares(std::span<const TimestampPair> samples) noexcept
        -> std::optional<LeastSquaresResult> {
        if (samples.size() < 2)
            return std::nullopt;

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_xx = 0.0;
        double sum_xy = 0.0;

        for (const auto& sample : samples) {
            const auto x = sample.camera_timestamp_sec;
            const auto y = sample.board_timestamp_sec;

            if (!std::isfinite(x) || !std::isfinite(y))
                return std::nullopt;

            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        const auto n = static_cast<double>(samples.size());
        const auto denominator = n * sum_xx - sum_x * sum_x;
        if (!std::isfinite(denominator) || std::abs(denominator) < 1e-12)
            return std::nullopt;

        const auto a = (n * sum_xy - sum_x * sum_y) / denominator;
        const auto b = (sum_y * sum_xx - sum_x * sum_xy) / denominator;
        if (!std::isfinite(a) || !std::isfinite(b))
            return std::nullopt;

        double max_abs_residual_sec = 0.0;
        double sum_squared_residual = 0.0;
        for (const auto& sample : samples) {
            const auto residual =
                sample.board_timestamp_sec - (a * sample.camera_timestamp_sec + b);
            if (!std::isfinite(residual))
                return std::nullopt;
            max_abs_residual_sec = std::max(max_abs_residual_sec, std::abs(residual));
            sum_squared_residual += residual * residual;
        }

        const auto sigma2 = samples.size() > 2
                              ? sum_squared_residual / static_cast<double>(samples.size() - 2)
                              : 0.0;

        Eigen::Matrix2d covariance;
        covariance << n, -sum_x, -sum_x, sum_xx;
        covariance *= sigma2 / denominator;
        if (!covariance.allFinite())
            return std::nullopt;

        return LeastSquaresResult{
            .a = a,
            .b = b,
            .covariance = covariance,
            .max_abs_residual_sec = max_abs_residual_sec,
            .sample_count = samples.size(),
        };
    }

    [[nodiscard]] auto
        initialize_from_least_squares(std::span<const TimestampPair> samples) noexcept
        -> std::optional<LeastSquaresResult> {
        auto fit = fit_least_squares(samples);
        if (!fit)
            return std::nullopt;

        theta_ << fit->a, fit->b;
        covariance_ = fit->covariance;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());

        constexpr auto kMinPriorVariance = 1e-6;
        if (!covariance_.allFinite() || covariance_(0, 0) <= 0.0 || covariance_(1, 1) <= 0.0
            || covariance_.determinant() <= 0.0) {
            covariance_ = Eigen::Matrix2d::Identity() * kMinPriorVariance;
        }
        covariance_.diagonal().array() += kMinPriorVariance;

        has_last_timestamp_ = false;
        last_camera_timestamp_sec_ = 0.0;
        initialized_ = true;
        sample_count_ = fit->sample_count;
        return fit;
    }

    [[nodiscard]] auto initialized() const noexcept -> bool { return initialized_; }
    [[nodiscard]] auto sample_count() const noexcept -> std::size_t { return sample_count_; }
    [[nodiscard]] auto a() const noexcept -> double { return theta_[0]; }
    [[nodiscard]] auto b() const noexcept -> double { return theta_[1]; }
    [[nodiscard]] auto covariance() const noexcept -> const Eigen::Matrix2d& { return covariance_; }
    [[nodiscard]] auto residual_threshold_sec() const noexcept -> double {
        return residual_threshold_sec_;
    }

    [[nodiscard]] auto predict(const double camera_timestamp_sec) const noexcept -> double {
        return theta_[0] * camera_timestamp_sec + theta_[1];
    }

    [[nodiscard]] auto residual_for(
        const double camera_timestamp_sec, const double board_timestamp_sec) const noexcept
        -> double {
        return board_timestamp_sec - predict(camera_timestamp_sec);
    }

    auto update(const double camera_timestamp_sec, const double board_timestamp_sec) noexcept
        -> double {
        const Eigen::Vector2d regressor{camera_timestamp_sec, 1.0};
        const auto residual = board_timestamp_sec - theta_.dot(regressor);

        if (has_last_timestamp_ && !(camera_timestamp_sec > last_camera_timestamp_sec_)) {
            initialized_ = false;
            return std::numeric_limits<double>::quiet_NaN();
        }

        const double lambda_time = [&] {
            if (!has_last_timestamp_) {
                has_last_timestamp_ = true;
                return 1.0;
            }
            const double dt = camera_timestamp_sec - last_camera_timestamp_sec_;
            return std::clamp(std::exp(-dt / tau_sec_), 0.01, 1.0);
        }();

        const Eigen::Vector2d projected_covariance = covariance_ * regressor;
        const auto denominator = lambda_time + regressor.dot(projected_covariance);
        if (!std::isfinite(denominator) || denominator <= 0.0) {
            initialized_ = false;
            return std::numeric_limits<double>::quiet_NaN();
        }

        const Eigen::Vector2d gain = projected_covariance / denominator;
        theta_ += gain * residual;
        covariance_ = (covariance_ - gain * regressor.transpose() * covariance_) / lambda_time;
        covariance_ = 0.5 * (covariance_ + covariance_.transpose());

        if (!theta_.allFinite() || !covariance_.allFinite()) {
            initialized_ = false;
            return std::numeric_limits<double>::quiet_NaN();
        }

        last_camera_timestamp_sec_ = camera_timestamp_sec;
        ++sample_count_;
        return residual;
    }

private:
    double tau_sec_ = 8.0;
    double residual_threshold_sec_ = kDefaultResidualThresholdSec;
    Eigen::Vector2d theta_ = Eigen::Vector2d::Zero();
    Eigen::Matrix2d covariance_ = Eigen::Matrix2d::Zero();
    double last_camera_timestamp_sec_ = 0.0;
    std::size_t sample_count_ = 0;
    bool initialized_ = false;
    bool has_last_timestamp_ = false;
};
