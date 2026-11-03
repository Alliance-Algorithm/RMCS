#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "wheelleg_gas_spring_v202.hpp"

namespace rmcs_core::wheelleg {

using Effort = std::array<double, 6>; // left hip/crank, right hip/crank, left/right wheel

struct PowerModel {
    Effort copper_w_per_nm2{};
    double idle_w = 0.0;
    double efficiency = 1.0;

    [[nodiscard]] double predict(const Effort& torque, const Effort& velocity) const {
        double power = idle_w;
        for (std::size_t i = 0; i < 6; ++i)
            power += std::max(0.0, torque[i] * velocity[i]) / efficiency
                   + copper_w_per_nm2[i] * torque[i] * torque[i];
        return power;
    }

    double allocate(Effort& torque, const Effort& velocity, double budget_w) const {
        if (!std::isfinite(budget_w) || budget_w < idle_w) {
            torque.fill(0.0);
            return 0.0;
        }
        if (predict(torque, velocity) <= budget_w)
            return 1.0;
        double low = 0.0, high = 1.0;
        for (int iteration = 0; iteration < 48; ++iteration) {
            const double scale = (low + high) * 0.5;
            Effort candidate{};
            for (std::size_t i = 0; i < 6; ++i)
                candidate[i] = torque[i] * scale;
            (predict(candidate, velocity) <= budget_w ? low : high) = scale;
        }
        for (double& value : torque)
            value *= low;
        return low;
    }
};

struct SlewState {
    double value = 0.0;
    double rate = 0.0;

    void step(double target, double max_rate, double max_acceleration, double dt) {
        const double error = target - value;
        const double stopping_rate = std::sqrt(2.0 * max_acceleration * std::abs(error));
        const double desired = std::copysign(std::min(max_rate, stopping_rate), error);
        rate += std::clamp(desired - rate, -max_acceleration * dt, max_acceleration * dt);
        value += rate * dt;
    }
};

enum class Support { unknown = 0, grounded = 1, airborne = 2 };

struct SupportObserver {
    Support state = Support::unknown;
    double grounded_time = 0.0, flight_time = 0.0;

    Support step(double specific_force, double wheel_load_residual, double dt,
                 double freefall_threshold, double load_threshold, double confirmation_s) {
        const bool freefall = specific_force < freefall_threshold
                           && wheel_load_residual < load_threshold;
        flight_time = freefall ? flight_time + dt : 0.0;
        grounded_time = specific_force > 5.0 && !freefall ? grounded_time + dt : 0.0;
        if (flight_time >= confirmation_s)
            state = Support::airborne;
        else if (grounded_time >= confirmation_s)
            state = Support::grounded;
        else if (!freefall && specific_force <= 5.0)
            state = Support::unknown;
        return state;
    }
};

// Row format is shared with the versioned C ABI and exported RMCS YAML.
struct PostureKnot {
    double length = 0.0;
    std::array<double, 4> position{}, gravity{};
    std::array<double, 8> spring_jacobian{}; // [spring L/R][four active leg axes]
    std::array<double, 2> compression{};
    std::array<double, 4> balance_gain{};
    std::array<double, 2> yaw_gain{};
    double axle_x = 0.0, knee_coordinate = 0.0;
};

struct TraditionalParameters {
    double dt = 0.001;
    double leg_kp = 160.0, leg_kd = 2.5;
    double wheel_limit = 4.5, leg_limit = 40.0;
    double acceleration_limit = 1.5, jerk_limit = 4.0;
    double height_rate = 0.02, height_acceleration = 0.1;
    double outer_kv = 0.5, outer_kx = 0.0;
    double freefall_threshold = 3.0, load_threshold = 0.5, support_confirmation = 0.01;
    double feedback_timeout = 0.05, max_tilt = 0.6;
    double wheel_inertia = 0.00058325, wheel_radius = 0.06;
    double nominal_length = 0.334;
    double torque_filter_s = 0.01, velocity_filter_s = 0.02;
    PowerModel power{};
    std::vector<PostureKnot> postures;
    std::vector<std::array<double, 2>> force_curve; // compression [m], extension force [N]

    void validate() const {
        const std::array scalars{dt, leg_kp, leg_kd, wheel_limit, leg_limit,
            acceleration_limit, jerk_limit, height_rate, height_acceleration, outer_kv, outer_kx,
            freefall_threshold, load_threshold, support_confirmation, feedback_timeout,
            max_tilt, wheel_inertia, wheel_radius, nominal_length, power.idle_w, power.efficiency,
            torque_filter_s, velocity_filter_s};
        for (double value : scalars)
            if (!std::isfinite(value) || value < 0.0)
                throw std::invalid_argument("Nonfinite or negative control parameter");
        if (dt != 0.001 || postures.size() < 2 || force_curve.size() < 2
            || !(power.efficiency > 0.0 && power.efficiency <= 1.0)
            || leg_kp < 0.0 || leg_kd < 0.0 || wheel_limit <= 0.0 || leg_limit <= 0.0
            || height_rate <= 0.0 || height_acceleration <= 0.0 || support_confirmation <= 0.0
            || acceleration_limit <= 0.0 || jerk_limit <= 0.0 || wheel_radius <= 0.0
            || wheel_inertia <= 0.0 || feedback_timeout <= 0.0 || max_tilt <= 0.0
            || nominal_length < postures.front().length || nominal_length > postures.back().length)
            throw std::invalid_argument("Invalid traditional control contract");
        for (std::size_t i = 1; i < postures.size(); ++i)
            if (!(postures[i].length > postures[i - 1].length))
                throw std::invalid_argument("Posture lengths must increase");
        for (std::size_t side = 0; side < 2; ++side) {
            const auto delta = [side](const auto& row) { return row.position[side*2+1] - row.position[side*2]; };
            const double direction = delta(postures.back()) - delta(postures.front());
            for (std::size_t i = 1; i < postures.size(); ++i)
                if (!((delta(postures[i])-delta(postures[i-1]))*direction > 0.0))
                    throw std::invalid_argument("Relative-angle table must be strictly monotone");
        }
        for (const auto& row : force_curve)
            if (!std::isfinite(row[0]) || !std::isfinite(row[1]) || row[1] <= 0.0)
                throw std::invalid_argument("Invalid spring force");
        for (std::size_t i = 1; i < force_curve.size(); ++i)
            if (!(force_curve[i][0] > force_curve[i - 1][0]) || force_curve[i][1] <= 0.0)
                throw std::invalid_argument("Invalid spring force knots");
        for (double value : power.copper_w_per_nm2)
            if (!std::isfinite(value) || value < 0.0)
                throw std::invalid_argument("Invalid copper loss coefficient");
    }
};

struct TraditionalSensors {
    Effort position{}, velocity{};
    double pitch = 0.0, pitch_rate = 0.0, yaw = 0.0, yaw_rate = 0.0;
    double specific_force_norm = 9.81;
    std::int64_t time_ns = 0, feedback_ns = 0;
    bool enabled = false, reset = false, fresh = false;
    bool balance_enabled = true;
};

struct TraditionalCommand {
    double velocity = 0.0, yaw_rate = 0.0, length = 0.334, power_budget = 0.0;
};

struct TraditionalResult {
    Effort effort{};
    double length_reference = 0.0, velocity_reference = 0.0, pitch_reference = 0.0;
    double velocity_estimate = 0.0, common_torque = 0.0, differential_torque = 0.0;
    double power_scale = 0.0, predicted_power = 0.0;
    Support support = Support::unknown;
    int failure = 0;
};

class TraditionalControlCore {
public:
    explicit TraditionalControlCore(TraditionalParameters parameters)
        : parameters_(std::move(parameters)) {
        parameters_.validate();
    }

    void reset() {
        initialized_ = false;
        previous_time_ = -1;
        support_ = {};
        speed_ = {};
        height_ = {};
        yaw_reference_ = distance_ = distance_reference_ = 0.0;
        previous_effort_ = {};
        previous_velocity_ = {};
        previous_axle_x_ = previous_knee_ = 0.0;
        common_filtered_ = differential_filtered_ = velocity_filtered_ = 0.0;
        leg_turn_offset_ = {};
    }

    TraditionalResult step(const TraditionalSensors& sensors, const TraditionalCommand& command) {
        TraditionalResult out;
        const auto& p = parameters_;
        bool finite = std::isfinite(sensors.pitch) && std::isfinite(sensors.pitch_rate)
            && std::isfinite(sensors.yaw) && std::isfinite(sensors.yaw_rate)
            && std::isfinite(sensors.specific_force_norm) && std::isfinite(command.velocity)
            && std::isfinite(command.yaw_rate) && std::isfinite(command.length)
            && std::isfinite(command.power_budget);
        for (std::size_t i = 0; i < 6; ++i)
            finite = finite && std::isfinite(sensors.position[i]) && std::isfinite(sensors.velocity[i]);
        if (!finite || sensors.reset || !sensors.enabled || !sensors.fresh
            || sensors.feedback_ns > sensors.time_ns
            || sensors.time_ns - sensors.feedback_ns > static_cast<std::int64_t>(p.feedback_timeout * 1e9)
            || std::abs(sensors.pitch) > p.max_tilt
            || (previous_time_ >= 0 && sensors.time_ns - previous_time_ != 1000000)) {
            out.failure = !finite ? 1 : 2;
            reset();
            return out;
        }
        previous_time_ = sensors.time_ns;
        if (command.power_budget < p.power.idle_w) {
            out.failure = 4;
            reset();
            return out;
        }
        const double measured_length = length_from_relative(sensors.position[1] - sensors.position[0], 0);
        const double measured_right = length_from_relative(sensors.position[3] - sensors.position[2], 1);
        if (!std::isfinite(measured_length) || !std::isfinite(measured_right)) {
            out.failure = 3;
            reset();
            return out;
        }
        const PostureKnot measured = interpolate((measured_length + measured_right) * 0.5);
        if (!initialized_) {
            const double turn = 2.0 * std::acos(-1.0);
            for (std::size_t side = 0; side < 2; ++side) {
                const auto hip = side * 2, crank = hip + 1;
                // The recovery arc can finish one or more complete turns
                // away from the table. Keep each motor pair on that branch;
                // never ask the standing PD to unwind through the chassis.
                const double offset = 0.5
                                    * (sensors.position[hip] - measured.position[hip]
                                       + sensors.position[crank] - measured.position[crank]);
                leg_turn_offset_[side] = turn * std::round(offset / turn);
            }
        }
        const double hip_error =
            0.5
            * ((sensors.position[0] - measured.position[0] - leg_turn_offset_[0])
               - (sensors.position[2] - measured.position[2] - leg_turn_offset_[1]));
        const double axle_x = measured.axle_x - measured.length * std::sin(hip_error);
        if (!initialized_) {
            height_.value = (measured_length + measured_right) * 0.5;
            yaw_reference_ = sensors.yaw;
            previous_axle_x_ = axle_x;
            previous_knee_ = measured.knee_coordinate;
            previous_velocity_ = sensors.velocity;
            initialized_ = true;
        }
        const double axle_speed = (axle_x - previous_axle_x_) / p.dt;
        const double knee_rate = (measured.knee_coordinate - previous_knee_) / p.dt;
        const double wheel_speed = 0.5 * (sensors.velocity[4] - sensors.velocity[5]);
        out.velocity_estimate = p.wheel_radius * (wheel_speed + sensors.pitch_rate
            + 0.5*(sensors.velocity[0] - sensors.velocity[2]) + knee_rate)
            + measured.length * sensors.pitch_rate - axle_speed;
        velocity_filtered_ += p.dt/(p.velocity_filter_s+p.dt)*(out.velocity_estimate-velocity_filtered_);
        out.velocity_estimate = velocity_filtered_;
        previous_axle_x_ = axle_x;
        previous_knee_ = measured.knee_coordinate;
        double load_residual = 0.0;
        for (std::size_t i = 4; i < 6; ++i) {
            const double acceleration = (sensors.velocity[i] - previous_velocity_[i]) / p.dt;
            load_residual = std::max(load_residual,
                std::abs(previous_effort_[i] - p.wheel_inertia * acceleration));
        }
        previous_velocity_ = sensors.velocity;
        out.support = support_.step(sensors.specific_force_norm, load_residual, p.dt,
            p.freefall_threshold, p.load_threshold, p.support_confirmation);
        const bool grounded = out.support == Support::grounded && sensors.balance_enabled;
        if (grounded) {
            speed_.step(std::clamp(command.velocity, -3.0, 3.0), p.acceleration_limit, p.jerk_limit, p.dt);
            distance_ += out.velocity_estimate * p.dt;
            distance_reference_ += speed_.value * p.dt;
            yaw_reference_ += std::clamp(command.yaw_rate, -4.0, 4.0) * p.dt;
        } else {
            speed_ = {};
            distance_reference_ = distance_;
            yaw_reference_ = sensors.yaw;
        }
        const double requested_length = std::clamp(command.length, p.postures.front().length, p.postures.back().length);
        height_.step(requested_length, p.height_rate, p.height_acceleration, p.dt);
        height_.value = std::clamp(height_.value, p.postures.front().length, p.postures.back().length);
        // Stop every outward reference component at a clipped length endpoint.
        if ((height_.value == p.postures.front().length && height_.rate < 0.0)
            || (height_.value == p.postures.back().length && height_.rate > 0.0))
            height_.rate = 0.0;
        const auto target = interpolate(height_.value);
        const double acceleration = speed_.rate + p.outer_kv*(speed_.value - out.velocity_estimate)
                                  + p.outer_kx*(distance_reference_ - distance_);
        out.pitch_reference = std::clamp(std::atan(acceleration / 9.81), -0.15, 0.15);
        const std::array error{sensors.pitch - out.pitch_reference, sensors.pitch_rate,
                              out.velocity_estimate - speed_.value, distance_ - distance_reference_};
        if (grounded) {
            for (std::size_t i = 0; i < 4; ++i)
                out.common_torque += measured.balance_gain[i] * error[i];
            const double yaw_error = std::remainder(sensors.yaw - yaw_reference_, 2.0 * std::acos(-1.0));
            out.differential_torque = measured.yaw_gain[0]*yaw_error
                + measured.yaw_gain[1]*(sensors.yaw_rate - command.yaw_rate);
        }
        common_filtered_ += p.dt/(p.torque_filter_s+p.dt)*(out.common_torque-common_filtered_);
        differential_filtered_ += p.dt/(p.torque_filter_s+p.dt)*(out.differential_torque-differential_filtered_);
        if (!grounded)
            common_filtered_ = differential_filtered_ = 0.0;
        out.common_torque = common_filtered_;
        out.differential_torque = differential_filtered_;
        const std::array measured_leg{sensors.position[0], sensors.position[1],
                                     sensors.position[2], sensors.position[3]};
        const auto spring_holding = gas_spring_v202::evaluate(measured_leg);
        for (std::size_t i = 0; i < 4; ++i) {
            // Support uncertainty gates wheel feedback and reference integration.
            // Keep bounded leg holding effort continuous across contact changes.
            // The two legs may have different measured shapes. Reuse the RL
            // encoder-only elastic model rather than the average posture row.
            const double feedforward = measured.gravity[i] + spring_holding.torque[i];
            out.effort[i] = std::clamp(
                p.leg_kp * (target.position[i] + leg_turn_offset_[i / 2] - sensors.position[i])
                    - p.leg_kd * sensors.velocity[i] + feedforward,
                -p.leg_limit, p.leg_limit);
        }
        // Remove the outward differential torque near each admitted relative-
        // angle endpoint. Common hip motion remains available for balance.
        for (std::size_t side = 0; side < 2; ++side) {
            const std::size_t hip = side*2, crank = hip+1;
            const double relative = sensors.position[crank]-sensors.position[hip];
            const double a = p.postures.front().position[crank]-p.postures.front().position[hip];
            const double b = p.postures.back().position[crank]-p.postures.back().position[hip];
            const double low = std::min(a,b), high = std::max(a,b);
            const double relative_torque = out.effort[crank]-out.effort[hip];
            const double margin = relative_torque > 0.0 ? high-relative : relative-low;
            const double factor = std::clamp(margin/0.03, 0.0, 1.0);
            const double removed = relative_torque*(1.0-factor)*0.5;
            out.effort[crank] -= removed;
            out.effort[hip] += removed;
        }
        // Common torque keeps balance priority over differential steering.
        out.common_torque = std::clamp(out.common_torque, -p.wheel_limit, p.wheel_limit);
        const double differential_room = p.wheel_limit - std::abs(out.common_torque);
        out.differential_torque = std::clamp(out.differential_torque, -differential_room, differential_room);
        out.effort[4] = out.common_torque - out.differential_torque;
        out.effort[5] = -(out.common_torque + out.differential_torque);
        out.power_scale = p.power.allocate(out.effort, sensors.velocity, command.power_budget);
        out.predicted_power = p.power.predict(out.effort, sensors.velocity);
        if (out.power_scale < 1.0) {
            // Do not accumulate unreachable position/yaw objectives under saturation.
            distance_reference_ = distance_;
            yaw_reference_ = sensors.yaw;
        }
        out.length_reference = height_.value;
        out.velocity_reference = speed_.value;
        previous_effort_ = out.effort;
        return out;
    }

private:
    [[nodiscard]] PostureKnot interpolate(double length) const {
        const auto& rows = parameters_.postures;
        std::size_t upper = 1;
        while (upper + 1 < rows.size() && rows[upper].length < length)
            ++upper;
        const auto& a = rows[upper - 1];
        const auto& b = rows[upper];
        const double fraction = std::clamp((length-a.length)/(b.length-a.length), 0.0, 1.0);
        PostureKnot out;
        out.length = std::lerp(a.length, b.length, fraction);
        const auto mix = [fraction](const auto& x, const auto& y, auto& z) {
            for (std::size_t i = 0; i < z.size(); ++i)
                z[i] = std::lerp(x[i], y[i], fraction);
        };
        mix(a.position, b.position, out.position);
        mix(a.gravity, b.gravity, out.gravity);
        mix(a.spring_jacobian, b.spring_jacobian, out.spring_jacobian);
        mix(a.compression, b.compression, out.compression);
        mix(a.balance_gain, b.balance_gain, out.balance_gain);
        mix(a.yaw_gain, b.yaw_gain, out.yaw_gain);
        out.axle_x = std::lerp(a.axle_x, b.axle_x, fraction);
        out.knee_coordinate = std::lerp(a.knee_coordinate, b.knee_coordinate, fraction);
        return out;
    }

    [[nodiscard]] double length_from_relative(double relative, std::size_t side) const {
        const auto& rows = parameters_.postures;
        const auto delta = [side](const auto& row) { return row.position[side*2+1] - row.position[side*2]; };
        const double low = std::min(delta(rows.front()), delta(rows.back()));
        const double high = std::max(delta(rows.front()), delta(rows.back()));
        if (relative < low - 0.05 || relative > high + 0.05)
            return std::numeric_limits<double>::quiet_NaN();
        relative = std::clamp(relative, low, high);
        for (std::size_t i = 1; i < rows.size(); ++i) {
            const double a = delta(rows[i-1]), b = delta(rows[i]);
            if (relative >= std::min(a,b) && relative <= std::max(a,b))
                return std::lerp(rows[i-1].length, rows[i].length, (relative-a)/(b-a));
        }
        return std::numeric_limits<double>::quiet_NaN();
    }

    [[nodiscard]] double spring_force(double compression) const {
        const auto& rows = parameters_.force_curve;
        // Full-robot research model uses explicit endpoint hold, recorded in
        // its contract. Extrapolation is not new measured force evidence.
        compression = std::clamp(compression, rows.front()[0], rows.back()[0]);
        for (std::size_t i = 1; i < rows.size(); ++i)
            if (compression <= rows[i][0])
                return std::lerp(rows[i-1][1], rows[i][1],
                    (compression-rows[i-1][0])/(rows[i][0]-rows[i-1][0]));
        return rows.back()[1];
    }

    TraditionalParameters parameters_;
    bool initialized_ = false;
    std::int64_t previous_time_ = -1;
    SupportObserver support_;
    SlewState speed_, height_;
    double yaw_reference_ = 0.0, distance_ = 0.0, distance_reference_ = 0.0;
    double previous_axle_x_ = 0.0, previous_knee_ = 0.0;
    double common_filtered_ = 0.0, differential_filtered_ = 0.0, velocity_filtered_ = 0.0;
    Effort previous_effort_{}, previous_velocity_{};
    std::array<double, 2> leg_turn_offset_{};
};

} // namespace rmcs_core::wheelleg
