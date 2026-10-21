#include "recovery_tuck_controller.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>

#include <eigen3/Eigen/Geometry>
#include <nlohmann/json.hpp>
#include <openssl/evp.h>

namespace rmcs_core::controller::chassis {
namespace {
Eigen::Vector3d vector(const nlohmann::json& data) {
    if (!data.is_array() || data.size() != 3)
        throw std::invalid_argument("Invalid tuck geometry vector");
    Eigen::Vector3d result{data[0].get<double>(), data[1].get<double>(), data[2].get<double>()};
    if (!result.allFinite())
        throw std::invalid_argument("Nonfinite tuck geometry");
    return result;
}
} // namespace

RecoveryTuckController::RecoveryTuckController(
    const std::filesystem::path& geometry, std::string_view model_spec_sha256,
    const RecoveryTuckConfig& config)
    : config_(config) {
    if (!std::isfinite(config.knee_deg) || config.knee_deg < 40.25 || config.knee_deg > 42.0
        || !std::isfinite(config.knee_tolerance_deg) || config.knee_tolerance_deg <= 0.0
        || config.knee_tolerance_deg > 1.0 || !std::isfinite(config.max_sagittal_error_m)
        || config.max_sagittal_error_m <= 0.0 || config.max_sagittal_error_m > 0.01
        || !std::isfinite(config.integral_gain) || config.integral_gain < 0.0
        || config.integral_gain > 100.0 || !std::isfinite(config.integral_limit_nm)
        || config.integral_limit_nm < 0.0 || config.integral_limit_nm > 20.0)
        throw std::invalid_argument("Invalid recovery tuck bounds");
    std::ifstream file{geometry};
    if (!file)
        throw std::invalid_argument("Cannot read recovery tuck geometry: " + geometry.string());
    const std::string bytes{std::istreambuf_iterator<char>{file}, {}};
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned int digest_size = 0;
    if (EVP_Digest(bytes.data(), bytes.size(), digest.data(), &digest_size, EVP_sha256(), nullptr)
            != 1
        || digest_size != 32)
        throw std::invalid_argument("Cannot hash tuck geometry");
    constexpr std::string_view hex = "0123456789abcdef";
    std::string actual;
    for (unsigned int i = 0; i < digest_size; ++i) {
        actual += hex[digest[i] >> 4];
        actual += hex[digest[i] & 15];
    }
    if (actual != "1b5b1a2b2c07c3436b916867eca8e86491dd6487d9ae492f7b55976ea311bc9b")
        throw std::invalid_argument("Recovery tuck geometry SHA256 mismatch");
    const auto data = nlohmann::json::parse(bytes);
    if (data.at("schema") != "wheel_leg_tuck_geometry_v1"
        || data.at("model_spec_sha256").get<std::string>() != model_spec_sha256
        || data.at("asset_manifest_sha256")
               != "088e699266fa3dc666379a911b23a30e2038ab3d7f113cab4dcd4bdd0a14279b"
        || data.at("height_lookup_sha256")
               != "91c43e8ea7da828155537ee3e64c3f45ccde7a475d002065da4829205027dfa1"
        || data.at("sides").size() != 2)
        throw std::invalid_argument("Recovery tuck geometry identity mismatch");
    mass_ = data.at("base_mass_kg").get<double>();
    base_moment_ = vector(data.at("base_moment_kg_m"));
    if (!std::isfinite(mass_) || mass_ <= 0.0)
        throw std::invalid_argument("Invalid tuck base mass");
    for (int side = 0; side < 2; ++side) {
        auto& leg = legs_[side];
        const auto& source = data.at("sides").at(side);
        leg.pivot = vector(source.at("hip_origin_b_m"));
        leg.axis = vector(source.at("hip_axis_b"));
        leg.hip_reference = source.at("hip_reference_rad");
        leg.mass = source.at("mass_kg");
        if (std::abs(leg.axis.squaredNorm() - 1.0) > 1e-6 || std::abs(leg.axis.y()) < 0.99
            || !std::isfinite(leg.hip_reference) || !std::isfinite(leg.mass) || leg.mass <= 0.0)
            throw std::invalid_argument("Invalid tuck leg mass or axis");
        for (const auto& row : source.at("knots")) {
            Knot knot{
                row.at("delta_rad"), row.at("knee_deg"), vector(row.at("wheel_b_m")),
                vector(row.at("moment_kg_m"))};
            if (!std::isfinite(knot.delta) || !std::isfinite(knot.knee_deg)
                || knot.knee_deg < 39.999 || knot.knee_deg > 107.001
                || (!leg.knots.empty() && knot.delta <= leg.knots.back().delta))
                throw std::invalid_argument("Invalid tuck closed-chain knots");
            leg.knots.push_back(knot);
        }
        if (leg.knots.size() < 2
            || std::abs(std::min(leg.knots.front().knee_deg, leg.knots.back().knee_deg) - 40.0)
                   > 0.001)
            throw std::invalid_argument("Tuck geometry must include the physical lower stop");
        for (std::size_t i = 1; i < leg.knots.size(); ++i)
            if ((leg.knots[i].knee_deg - leg.knots[i - 1].knee_deg)
                    * (leg.knots.back().knee_deg - leg.knots.front().knee_deg)
                <= 0.0)
                throw std::invalid_argument("Tuck knee mapping must retain one assembly branch");
        mass_ += leg.mass;
    }
}

void RecoveryTuckController::reset() { integral_.setZero(); }

RecoveryTuckController::Knot RecoveryTuckController::interpolate_(int side, double delta) const {
    const auto& knots = legs_[side].knots;
    const auto found =
        std::lower_bound(knots.begin(), knots.end(), delta, [](const Knot& knot, double v) {
            return knot.delta < v;
        });
    const auto upper = std::clamp<std::size_t>(found - knots.begin(), 1, knots.size() - 1);
    const auto& a = knots[upper - 1];
    const auto& b = knots[upper];
    const double t = (delta - a.delta) / (b.delta - a.delta);
    return {
        delta, std::lerp(a.knee_deg, b.knee_deg, t), a.wheel + t * (b.wheel - a.wheel),
        a.moment + t * (b.moment - a.moment)};
}

Eigen::Vector3d
    RecoveryTuckController::rotate_(int side, const Eigen::Vector3d& v, double angle) const {
    const auto& axis = legs_[side].axis;
    return std::cos(angle) * v + std::sin(angle) * axis.cross(v)
         + (1.0 - std::cos(angle)) * axis * axis.dot(v);
}

RecoveryTuckState
    RecoveryTuckController::measure(const Eigen::Vector4d& q, const Eigen::Vector3d& g) const {
    RecoveryTuckState state;
    if (!q.allFinite() || !g.allFinite() || std::abs(g.squaredNorm() - 1.0) > 1e-3)
        return state;
    const Eigen::Vector3d forward = g.cross(Eigen::Vector3d::UnitY());
    if (forward.norm() < 0.5)
        return state;
    Eigen::Vector3d moment = base_moment_;
    for (int side = 0; side < 2; ++side) {
        const auto& leg = legs_[side];
        const double delta = q[2 * side + 1] - q[2 * side];
        // Match the existing measured hinge guard. A small stop/compliance
        // excursion still needs a restoring command; it never qualifies capture.
        constexpr double guard_rad = 0.03;
        if (delta < leg.knots.front().delta - guard_rad
            || delta > leg.knots.back().delta + guard_rad)
            return state;
        const auto knot = interpolate_(side, delta);
        const double angle = q[2 * side] - leg.hip_reference;
        state.knee_deg[side] = knot.knee_deg;
        state.wheels_b_m[side] = leg.pivot + rotate_(side, knot.wheel - leg.pivot, angle);
        moment += leg.mass * leg.pivot + rotate_(side, knot.moment - leg.mass * leg.pivot, angle);
    }
    state.com_b_m = moment / mass_;
    bool below = true;
    for (int side = 0; side < 2; ++side) {
        const Eigen::Vector3d offset = state.wheels_b_m[side] - state.com_b_m;
        state.sagittal_error_m[side] = forward.normalized().dot(offset);
        below &= g.dot(offset) > 0.04;
    }
    state.valid = true;
    state.ready = below && (state.knee_deg.array() >= 40.0).all()
               && (state.knee_deg.array() - config_.knee_deg).abs().maxCoeff()
                      <= config_.knee_tolerance_deg
               && state.sagittal_error_m.cwiseAbs().maxCoeff() <= config_.max_sagittal_error_m;
    return state;
}

RecoveryTuckState RecoveryTuckController::update(
    const Eigen::Vector4d& q, const Eigen::Vector3d& gravity, double dt, bool tucking) {
    auto state = measure(q, gravity);
    if (!state.valid || !std::isfinite(dt) || dt <= 0.0 || dt > 0.04) {
        reset();
        state.valid = state.ready = false;
        return state;
    }
    Eigen::Vector4d goal;
    for (int side = 0; side < 2; ++side) {
        const auto& leg = legs_[side];
        double delta = 0.0;
        for (std::size_t i = 1; i < leg.knots.size(); ++i) {
            const auto& a = leg.knots[i - 1];
            const auto& b = leg.knots[i];
            if (config_.knee_deg >= std::min(a.knee_deg, b.knee_deg)
                && config_.knee_deg <= std::max(a.knee_deg, b.knee_deg)) {
                delta = std::lerp(
                    a.delta, b.delta, (config_.knee_deg - a.knee_deg) / (b.knee_deg - a.knee_deg));
                break;
            }
        }
        const auto knot = interpolate_(side, delta);
        const Eigen::Vector3d v = knot.wheel - leg.pivot;
        const Eigen::Vector3d down = gravity - leg.axis * leg.axis.dot(gravity);
        const double angle = std::atan2(leg.axis.dot(v.cross(down)), v.dot(down));
        goal[2 * side] = leg.hip_reference + angle;
        goal[2 * side + 1] = goal[2 * side] + delta;
    }
    // Two coupled hip rotations: moving either leg also moves the total COM.
    // A small Newton solve avoids a fixed pitch offset or a hip-origin surrogate.
    for (int iteration = 0; iteration < 8; ++iteration) {
        const auto current = measure(goal, gravity);
        if (!current.valid) {
            reset();
            state.valid = state.ready = false;
            return state;
        }
        if (current.sagittal_error_m.cwiseAbs().maxCoeff() < 1e-8)
            break;
        Eigen::Matrix2d jacobian;
        constexpr double epsilon = 1e-5;
        for (int side = 0; side < 2; ++side) {
            Eigen::Vector4d shifted = goal;
            shifted.segment<2>(2 * side).array() += epsilon;
            jacobian.col(side) =
                (measure(shifted, gravity).sagittal_error_m - current.sagittal_error_m) / epsilon;
        }
        const double determinant = jacobian.determinant();
        if (!std::isfinite(determinant) || std::abs(determinant) < 1e-6) {
            reset();
            state.valid = state.ready = false;
            return state;
        }
        const Eigen::Vector2d step = jacobian.inverse() * -current.sagittal_error_m;
        for (int side = 0; side < 2; ++side)
            goal.segment<2>(2 * side).array() += std::clamp(step[side], -0.3, 0.3);
    }
    const auto solved = measure(goal, gravity);
    if (!solved.valid) {
        reset();
        state.valid = state.ready = false;
        return state;
    }
    // Preserve a common turn for each closed-chain motor pair, including front/back starts.
    for (int side = 0; side < 2; ++side) {
        const double turns =
            std::nearbyint((q[2 * side] - goal[2 * side]) / (2 * std::numbers::pi));
        goal.segment<2>(2 * side).array() += turns * 2 * std::numbers::pi;
    }
    state.reference = goal;
    if (tucking) {
        integrate_(q, goal, Eigen::Vector4d::Zero(), dt);
    } else {
        reset();
    }
    state.hold_torque = integral_;
    return state;
}

void RecoveryTuckController::integrate_(
    const Eigen::Vector4d& q, const Eigen::Vector4d& reference, const Eigen::Vector4d& dq,
    double dt) {
    Eigen::Vector4d error = reference - q;
    for (int hip : {0, 2})
        error.segment<2>(hip).array() -=
            2 * std::numbers::pi * std::nearbyint(error[hip] / (2 * std::numbers::pi));
    for (int i = 0; i < 4; ++i) {
        const double effort = 160.0 * error[i] - 2.5 * dq[i] + integral_[i];
        const double increment = config_.integral_gain * dt * error[i];
        // Saturation stops outward integration per axis; unloading may unwind
        // immediately. A loaded axis cannot freeze compensation on another axis.
        if ((effort >= 40.0 && increment > 0.0) || (effort <= -40.0 && increment < 0.0))
            continue;
        integral_[i] = std::clamp(
            integral_[i] + increment, -config_.integral_limit_nm, config_.integral_limit_nm);
    }
}

RecoveryTuckState RecoveryTuckController::update(
    const Eigen::Vector4d& q, const Eigen::Vector3d& gravity, double dt, bool tucking,
    const Eigen::Vector4d& executed_reference, const Eigen::Vector4d& dq) {
    const Eigen::Vector4d previous_integral = integral_;
    auto state = update(q, gravity, dt, false);
    if (state.valid && tucking && executed_reference.allFinite() && dq.allFinite()) {
        integral_ = previous_integral;
        integrate_(q, executed_reference, dq, dt);
        state.hold_torque = integral_;
    } else if (tucking && (!executed_reference.allFinite() || !dq.allFinite())) {
        state.valid = state.ready = false;
    }
    return state;
}

} // namespace rmcs_core::controller::chassis
