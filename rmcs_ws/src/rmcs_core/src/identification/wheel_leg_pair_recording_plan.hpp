#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "identification/wheel_leg_pair_identification_planner.hpp"

namespace rmcs_core::controller::identification {

// The versioned calibration binds beta to the installed assembly branch. Shape-preserving
// Hermite interpolation avoids the velocity jumps of a piecewise-linear LUT.
class PairGeometry {
public:
    struct Value {
        double position, first, second;
    };
    PairGeometry(std::vector<double> beta, std::vector<double> delta)
        : beta_(std::move(beta))
        , delta_(std::move(delta)) {
        if (beta_.size() < 3 || beta_.size() != delta_.size())
            throw std::invalid_argument("G0 needs at least three beta/delta pairs");
        std::vector<double> h, d;
        for (std::size_t i = 0; i < beta_.size(); ++i) {
            if (!std::isfinite(beta_[i]) || !std::isfinite(delta_[i]) || beta_[i] < 30 * rad
                || beta_[i] > 120 * rad)
                throw std::invalid_argument("G0 beta/delta values outside V6 domain");
            if (i) {
                if (beta_[i] <= beta_[i - 1] || delta_[i] == delta_[i - 1])
                    throw std::invalid_argument("G0 LUT must be strictly monotone");
                h.push_back(beta_[i] - beta_[i - 1]);
                d.push_back((delta_[i] - delta_[i - 1]) / h.back());
                if (d.size() > 1 && d.back() * d.front() <= 0)
                    throw std::invalid_argument("G0 branch cannot reverse direction");
            }
        }
        slope_.resize(beta_.size());
        slope_.front() = d.front();
        slope_.back() = d.back();
        for (std::size_t i = 1; i + 1 < beta_.size(); ++i) {
            const double w1 = 2 * h[i] + h[i - 1], w2 = h[i] + 2 * h[i - 1];
            slope_[i] = (w1 + w2) / (w1 / d[i - 1] + w2 / d[i]);
        }
    }

    Value at(double beta) const {
        if (!std::isfinite(beta) || beta < beta_.front() - 1e-10 || beta > beta_.back() + 1e-10)
            throw std::out_of_range("Beta outside G0 LUT; extrapolation forbidden");
        beta = std::clamp(beta, beta_.front(), beta_.back());
        auto it = std::upper_bound(beta_.begin(), beta_.end(), beta);
        const auto i = std::min(static_cast<std::size_t>(it - beta_.begin()), beta_.size() - 1) - 1;
        const double h = beta_[i + 1] - beta_[i], t = (beta - beta_[i]) / h;
        const double a = 2 * delta_[i] - 2 * delta_[i + 1] + h * (slope_[i] + slope_[i + 1]);
        const double b = -3 * delta_[i] + 3 * delta_[i + 1] - h * (2 * slope_[i] + slope_[i + 1]);
        const double c = h * slope_[i];
        return {
            ((a * t + b) * t + c) * t + delta_[i], (3 * a * t * t + 2 * b * t + c) / h,
            (6 * a * t + 2 * b) / (h * h)};
    }

    std::optional<double> beta(double delta) const noexcept {
        if (!std::isfinite(delta) || delta < std::min(delta_.front(), delta_.back())
            || delta > std::max(delta_.front(), delta_.back()))
            return std::nullopt;
        double lo = beta_.front(), hi = beta_.back();
        for (int iteration = 0; iteration < 48; ++iteration) {
            const double middle = (lo + hi) / 2;
            if ((at(middle).position < delta) == (delta_.back() > delta_.front()))
                lo = middle;
            else
                hi = middle;
        }
        return (lo + hi) / 2;
    }

    static constexpr double rad = std::numbers::pi / 180.0;

private:
    std::vector<double> beta_, delta_, slope_;
};

struct PairRecordingConfig {
    std::string run = "L01";
    double hip_sign = 1, hip_zero = 0;
    double beta_min = 65 * PairGeometry::rad, beta_max = 100 * PairGeometry::rad;
    std::array<double, 2> max_speed{6, 6}, max_acceleration{80, 80};
    double beta_tolerance = 2 * PairGeometry::rad, arrival_speed = 0.08;
    double arrival_stable_s = 0.5, arrival_timeout_s = 10;
    double center_step_rad = 0.01, center_max_rad = 0.20;
    double move_s = 2, dwell_s = 4, baseline_s = 30;
};

class PairRecordingPlan {
public:
    enum class Kind : int { kMove, kHold, kChirp, kMultisine, kEdge };
    enum class Coordinates : int { kHip, kCommon };
    // Labels describe airborne reference phases, never measured ground contact.
    enum class Jump : int {
        kNone,
        kCompress,
        kLoaded,
        kExtend,
        kTuck,
        kReach,
        kBuffer,
        kSettle,
        kGap
    };
    struct Pose {
        double theta = 0, beta = 85 * PairGeometry::rad;
    };
    struct Segment {
        std::string name;
        Kind kind = Kind::kHold;
        Coordinates coordinates = Coordinates::kHip;
        Pose from{}, to{};
        double start_s = 0, duration_s = 0, requested_s = 0;
        double theta_amplitude = 0, beta_amplitude = 0, scale = 1;
        std::array<double, 2> theta_hz{}, beta_hz{};
        bool holdout = false, arrival = false;
        double stable_s = 0;
        int group = 0, cycle = -1;
        Jump jump = Jump::kNone;
    };

    PairRecordingPlan(
        PairGeometry geometry, PairRecordingConfig config,
        std::optional<std::array<double, 2>> initial = std::nullopt)
        : geometry_(std::move(geometry))
        , config_(std::move(config)) {
        validate_config(config_);
        geometry_.at(config_.beta_min);
        geometry_.at(config_.beta_max);
        current_.beta = std::clamp(current_.beta, config_.beta_min, config_.beta_max);
        if (initial) {
            const auto beta = geometry_.beta((*initial)[1] - (*initial)[0]);
            if (!beta)
                throw std::invalid_argument("Initial feedback outside G0 branch");
            current_ = {((*initial)[0] - config_.hip_zero) / config_.hip_sign, *beta};
        }
        const bool holdout = config_.run.ends_with("03") || config_.run.ends_with("J02");
        move("entry_from_feedback", {0, 85 * rad}, 8, holdout);
        anchor("baseline/start", {0, 85 * rad}, holdout);
        hold("baseline/start/sample", config_.baseline_s, holdout);
        if (config_.run.ends_with("J01") || config_.run.ends_with("J02"))
            jump_suite(holdout);
        else if (config_.run.ends_with("01"))
            static_suite();
        else if (config_.run.ends_with("02"))
            dynamic_suite();
        else
            holdout_suite();
        anchor("baseline/end", {0, 85 * rad}, holdout);
        hold("baseline/end/sample", config_.baseline_s, holdout);
    }

    static void validate_config(const PairRecordingConfig& c) {
        if (c.run != "L01" && c.run != "L02" && c.run != "L03" && c.run != "R01" && c.run != "R02"
            && c.run != "R03" && c.run != "LJ01" && c.run != "LJ02" && c.run != "RJ01"
            && c.run != "RJ02")
            throw std::invalid_argument("Unknown single-side recording run");
        if (std::abs(c.hip_sign) != 1 || !std::isfinite(c.hip_zero) || !std::isfinite(c.beta_min)
            || !std::isfinite(c.beta_max) || c.beta_min < 45 * rad || c.beta_min > 65 * rad
            || c.beta_max < 100 * rad || c.beta_max > 102 * rad)
            throw std::invalid_argument("Invalid G0 hip map/admitted beta domain");
        for (double x :
             {c.beta_tolerance, c.arrival_speed, c.arrival_stable_s, c.arrival_timeout_s,
              c.center_step_rad, c.center_max_rad, c.move_s, c.dwell_s, c.baseline_s,
              c.max_speed[0], c.max_speed[1], c.max_acceleration[0], c.max_acceleration[1]})
            if (!std::isfinite(x) || x <= 0)
                throw std::invalid_argument("Invalid recording budget");
        if (c.arrival_timeout_s < c.dwell_s || c.arrival_stable_s > c.dwell_s
            || c.center_step_rad > c.center_max_rad || c.center_max_rad > 0.2)
            throw std::invalid_argument("Invalid arrival/center correction budget");
    }

    const PairGeometry& geometry() const { return geometry_; }
    const PairRecordingConfig& config() const { return config_; }
    const std::vector<Segment>& segments() const { return segments_; }
    double duration() const { return segments_.back().start_s + segments_.back().duration_s; }
    // Only the entry changes at arm time. The remaining plan and amplitude
    // reductions were compiled before joining the executor's real-time graph.
    void set_initial_feedback(const std::array<double, 2>& initial) {
        const auto beta = geometry_.beta(initial[1] - initial[0]);
        if (!beta || !std::isfinite(initial[0]))
            throw std::invalid_argument("Initial feedback outside calibrated branch");
        segments_.front().from = {(initial[0] - config_.hip_zero) / config_.hip_sign, *beta};
    }

    std::array<double, 2> center_bounds(int group) const {
        const double lower = std::min(
            geometry_.at(config_.beta_min).position, geometry_.at(config_.beta_max).position);
        const double upper = std::max(
            geometry_.at(config_.beta_min).position, geometry_.at(config_.beta_max).position);
        double lo = -config_.center_max_rad, hi = config_.center_max_rad;
        bool after_arrival = false;
        for (std::size_t id = 0; id < segments_.size(); ++id)
            if (segments_[id].group == group) {
                const auto& s = segments_[id];
                after_arrival |= s.arrival;
                if (!after_arrival)
                    continue;
                // Envelopes are bounded by one, including normalized multisines.
                const double amplitude = s.beta_amplitude * s.scale;
                for (double beta :
                     {std::min(s.from.beta, s.to.beta) - amplitude,
                      std::max(s.from.beta, s.to.beta) + amplitude}) {
                    const double delta = geometry_.at(beta).position;
                    lo = std::max(lo, lower - delta);
                    hi = std::min(hi, upper - delta);
                }
            }
        return {lo, hi};
    }
    PairSample at(double t) const {
        if (!std::isfinite(t) || t < 0 || t > duration())
            throw std::out_of_range("Recording time");
        auto it =
            std::upper_bound(segments_.begin(), segments_.end(), t, [](double t, const Segment& s) {
                return t < s.start_s + s.duration_s;
            });
        if (it == segments_.end())
            --it;
        return sample(static_cast<std::size_t>(it - segments_.begin()), t - it->start_s);
    }

    PairSample sample(
        std::size_t id, double t, double center = 0, double center_velocity = 0,
        double center_acceleration = 0) const {
        const auto& s = segments_.at(id);
        t = std::clamp(t, 0.0, s.duration_s);
        std::array<double, 3> theta{s.from.theta, 0, 0}, beta{s.from.beta, 0, 0};
        if (s.kind == Kind::kMove) {
            const auto u = quintic(t / s.duration_s, s.duration_s);
            for (std::size_t j = 0; j < 3; ++j) {
                theta[j] = (s.to.theta - s.from.theta) * u[j] + (j == 0 ? s.from.theta : 0);
                beta[j] = (s.to.beta - s.from.beta) * u[j] + (j == 0 ? s.from.beta : 0);
            }
        } else if (s.kind == Kind::kChirp || s.kind == Kind::kMultisine) {
            const auto a = oscillation(s, t, false), b = oscillation(s, t, true);
            for (std::size_t j = 0; j < 3; ++j) {
                theta[j] += a[j];
                beta[j] += b[j];
            }
        } else if (s.kind == Kind::kEdge) {
            theta[0] = s.to.theta;
            beta[0] = s.to.beta;
        }
        const auto g = geometry_.at(beta[0]);
        const double delta = g.position + center;
        const double vd = g.first * beta[1], ad = g.first * beta[2] + g.second * beta[1] * beta[1];
        double qh = config_.hip_zero + config_.hip_sign * theta[0];
        double vh = config_.hip_sign * theta[1], ah = config_.hip_sign * theta[2];
        if (s.coordinates == Coordinates::kCommon) {
            qh += (geometry_.at(s.from.beta).position - g.position) / 2;
            vh -= vd / 2;
            ah -= ad / 2;
        }
        return {
            {qh, qh + delta},
            {vh, vh + vd + center_velocity},
            {ah, ah + ad + center_acceleration},
            beta[0],
            static_cast<int>(id),
            s.holdout,
            static_cast<std::uint8_t>(
                s.kind == Kind::kChirp       ? 6
                : s.kind == Kind::kMultisine ? 8
                : s.kind == Kind::kEdge      ? 9
                : s.kind == Kind::kMove      ? 1
                                             : 0)};
    }

    std::string manifest_json() const {
        std::ostringstream out;
        out << std::setprecision(17);
        out << "{\"revision\":\"pair_v6_recording_v1\",\"run\":\"" << config_.run
            << "\",\"boundary_condition\":\"fixed_base_airborne_installed_springs\","
            << "\"reference_frequency_hz\":50,\"control_frequency_hz\":1000,"
            << "\"beta_source\":\"calibrated_FK_estimate\",\"duration_s\":" << duration()
            << ",\"arrival_waits_extend_duration\":true,\"entry_initial_pose\":\"fresh_feedback_at_"
               "arm\","
            << "\"entry_from_is_placeholder\":true,\"budget_check_grid_s\":0.001,"
            << "\"uncovered_transition\":\"bounded quintic from last reference; admission=3\","
            << "\"held_edge_derivative\":\"undefined at update; zero between updates\","
            << "\"multisine_common_hz\":[0.17,0.43,0.79,1.31,2.17,3.73],"
            << "\"multisine_beta_hz\":[0.23,0.61,1.07,1.73,2.41,3.91],"
            << "\"multisine_phase_rad\":\"pi*i*(i+1)/6; beta adds 0.47\","
            << "\"chirp_beta_phase_rad\":1.0471975511965976,"
            << "\"multisine_normalization\":\"sum of component amplitude bounds = 1\","
            << "\"hip_map\":[" << config_.hip_sign << ',' << config_.hip_zero << "],"
            << "\"admitted_beta_rad\":[" << config_.beta_min << ',' << config_.beta_max << "],"
            << "\"segments\":[";
        for (std::size_t i = 0; i < segments_.size(); ++i) {
            const auto& s = segments_[i];
            if (i)
                out << ',';
            out << "{\"id\":" << i << ",\"name\":\"" << s.name << "\",\"start_s\":" << s.start_s
                << ",\"duration_s\":" << s.duration_s << ",\"requested_s\":" << s.requested_s
                << ",\"kind\":" << static_cast<int>(s.kind)
                << ",\"coordinates\":" << static_cast<int>(s.coordinates)
                << ",\"role\":" << (s.holdout ? 1 : 0)
                << ",\"arrival_required\":" << (s.arrival ? "true" : "false")
                << ",\"group\":" << s.group << ",\"cycle\":" << s.cycle
                << ",\"jump_phase\":" << static_cast<int>(s.jump)
                << ",\"arrival_stable_s\":" << s.stable_s << ",\"from\":[" << s.from.theta << ','
                << s.from.beta << "],\"to\":[" << s.to.theta << ',' << s.to.beta
                << "],\"requested_amplitude\":[" << s.theta_amplitude << ',' << s.beta_amplitude
                << "],\"amplitude_scale\":" << s.scale << ",\"theta_hz\":[" << s.theta_hz[0] << ','
                << s.theta_hz[1] << "],\"beta_hz\":[" << s.beta_hz[0] << ',' << s.beta_hz[1]
                << "]}";
        }
        return out.str() + "]}";
    }

private:
    static constexpr double rad = PairGeometry::rad;
    static double quantize(double s) { return std::ceil(s * 50 - 1e-9) / 50; }
    static std::array<double, 3> quintic(double u, double t) {
        return {
            u * u * u * (10 + u * (-15 + 6 * u)), 30 * u * u * (1 - u) * (1 - u) / t,
            60 * u * (1 - u) * (1 - 2 * u) / (t * t)};
    }
    static std::array<double, 3> sine(double phase, double w, double a) {
        return {
            std::sin(phase), std::cos(phase) * w, std::cos(phase) * a - std::sin(phase) * w * w};
    }
    static std::array<double, 3> oscillation(const Segment& s, double t, bool relative) {
        constexpr double pi = std::numbers::pi;
        std::array<double, 3> wave{};
        const auto& hz = relative ? s.beta_hz : s.theta_hz;
        const double amplitude = (relative ? s.beta_amplitude : s.theta_amplitude) * s.scale;
        if (amplitude == 0)
            return wave;
        if (s.kind == Kind::kChirp) {
            const double a = 2 * pi * (hz[1] - hz[0]) / s.duration_s;
            wave = sine(
                2 * pi * hz[0] * t + a * t * t / 2 + (relative ? pi / 3 : 0),
                2 * pi * hz[0] + a * t, a);
        } else {
            constexpr std::array<double, 6> common{.17, .43, .79, 1.31, 2.17, 3.73};
            constexpr std::array<double, 6> differential{.23, .61, 1.07, 1.73, 2.41, 3.91};
            for (std::size_t i = 0; i < common.size(); ++i) {
                const double w = 2 * pi * (relative ? differential[i] : common[i]);
                const auto v = sine(w * t + pi * i * (i + 1) / 6 + (relative ? .47 : 0), w, 0);
                for (std::size_t j = 0; j < 3; ++j)
                    wave[j] += v[j] / 6;
            }
        }
        const double taper = std::min(2.0, s.duration_s / 4);
        std::array<double, 3> envelope{1, 0, 0};
        if (t < taper)
            envelope = quintic(t / taper, taper);
        else if (t > s.duration_s - taper) {
            envelope = quintic((s.duration_s - t) / taper, taper);
            envelope[1] *= -1;
        }
        return {
            amplitude * wave[0] * envelope[0],
            amplitude * (wave[1] * envelope[0] + wave[0] * envelope[1]),
            amplitude
                * (wave[2] * envelope[0] + 2 * wave[1] * envelope[1] + wave[0] * envelope[2])};
    }
    bool fits(std::size_t id) const {
        const auto& s = segments_[id];
        for (double t = 0; t <= s.duration_s + 1e-9; t += .001) {
            const auto v = sample(id, std::min(t, s.duration_s));
            if (id != 0 && (v.beta < config_.beta_min - 1e-10 || v.beta > config_.beta_max + 1e-10))
                return false;
            for (std::size_t j = 0; j < 2; ++j)
                if (std::abs(v.velocity[j]) > config_.max_speed[j]
                    || std::abs(v.acceleration[j]) > config_.max_acceleration[j])
                    return false;
        }
        return true;
    }
    void add(Segment s) {
        s.from = current_;
        s.group = group_;
        s.start_s = segments_.empty() ? 0 : duration();
        s.requested_s = s.duration_s;
        s.duration_s = quantize(s.duration_s);
        if (!s.holdout && (config_.run.ends_with("03") || config_.run.ends_with("J02")))
            s.holdout = true;
        segments_.push_back(s);
        const auto id = segments_.size() - 1;
        for (int attempt = 0; !fits(id); ++attempt) {
            if (attempt >= 100)
                throw std::invalid_argument("Cannot fit physical trajectory into per-axis budgets");
            if (s.kind == Kind::kChirp || s.kind == Kind::kMultisine)
                segments_.back().scale *= .9;
            else if (s.kind == Kind::kMove)
                segments_.back().duration_s = quantize(segments_.back().duration_s * 1.05 + .02);
            else
                throw std::invalid_argument("Recording pose outside admitted domain");
        }
        current_ = s.to;
    }
    void move(
        std::string name, Pose to, double duration, bool holdout = false, int cycle = -1,
        Jump jump = Jump::kNone) {
        Segment s;
        s.name = std::move(name);
        s.kind = Kind::kMove;
        s.to = to;
        s.duration_s = duration;
        s.holdout = holdout;
        s.cycle = cycle;
        s.jump = jump;
        add(s);
    }
    void hold(
        std::string name, double duration, bool holdout = false, bool arrival = false,
        int cycle = -1, Jump jump = Jump::kNone, double stable = 0) {
        Segment s;
        s.name = std::move(name);
        s.to = current_;
        s.duration_s = duration;
        s.holdout = holdout;
        s.arrival = arrival;
        s.cycle = cycle;
        s.jump = jump;
        s.stable_s = stable > 0 ? stable : config_.arrival_stable_s;
        add(s);
    }
    void anchor(const std::string& name, Pose pose, bool holdout = false) {
        ++group_;
        move(name + "/move", pose, config_.move_s, holdout);
        hold(name + "/arrival", config_.dwell_s, holdout, true);
    }
    void excite(
        const std::string& name, Coordinates coordinates, double a, double b,
        std::array<double, 2> ahz, std::array<double, 2> bhz, bool holdout = false,
        double seconds = 30) {
        Segment s;
        s.name = name;
        s.kind = holdout ? Kind::kMultisine : Kind::kChirp;
        s.coordinates = coordinates;
        s.to = current_;
        s.duration_s = seconds;
        s.theta_amplitude = a;
        s.beta_amplitude = b;
        s.theta_hz = ahz;
        s.beta_hz = bhz;
        s.holdout = holdout;
        add(s);
    }
    void static_suite() {
        // Start with a reachable interior pose, then collect both approach directions.
        std::vector<double> bins{85., 75., 65., 95., 100.};
        for (double beta : {60., 55., 50., 45.})
            if (beta * rad >= config_.beta_min - 1e-10)
                bins.push_back(beta);
        if (config_.beta_max >= 102 * rad - 1e-10)
            bins.push_back(102);
        for (double beta : bins)
            for (double theta : {0., 90., 180., 270.})
                for (int direction : {-1, 1}) {
                    const auto name = "static/b" + std::to_string(static_cast<int>(beta)) + "/t"
                                    + std::to_string(static_cast<int>(theta)) + "/d"
                                    + std::to_string(direction);
                    ++group_;
                    move(
                        name + "/approach",
                        {theta * rad,
                         std::clamp(
                             (beta + direction * 3) * rad, config_.beta_min, config_.beta_max)},
                        1);
                    move(name + "/target", {theta * rad, beta * rad}, 1);
                    hold(name + "/arrival", config_.dwell_s, false, true, -1, Jump::kNone, 2);
                }
        for (double theta : {0., 180.})
            for (double rate : {3., 8., 15.}) {
                anchor("slow/anchor", {theta * rad, 65 * rad});
                for (int repetition = 0; repetition < 2; ++repetition) {
                    move("slow/extend", {theta * rad, 95 * rad}, 1.875 * 30 / rate);
                    move("slow/flex", {theta * rad, 65 * rad}, 1.875 * 30 / rate);
                }
            }
        for (double beta : {75., 95.}) {
            anchor("orientation/anchor", {0, beta * rad});
            for (int eighth = 1; eighth <= 8; ++eighth) {
                move("orientation/out", {eighth * 45 * rad, beta * rad}, 2);
                hold("orientation/out/dwell", 4);
            }
            for (int eighth = 7; eighth >= 0; --eighth) {
                move("orientation/return", {eighth * 45 * rad, beta * rad}, 2);
                hold("orientation/return/dwell", 4);
            }
        }
    }
    void steps(bool holdout) {
        anchor("steps/anchor", {0, 85 * rad}, holdout);
        for (bool common : {true, false})
            for (double change : {.005, .01, .02, .05, .10})
                for (int direction : {-1, 1}) {
                    const auto center = current_;
                    const bool edge = change <= .02;
                    // Changes are limited in ACTIVE coordinates after applying G0.
                    double beta = center.beta;
                    const auto mapped = geometry_.beta(
                        geometry_.at(beta).position + (common ? 0 : 2 * direction * change));
                    if (!mapped)
                        throw std::invalid_argument("Step outside G0 branch");
                    Segment s;
                    s.name = std::string(common ? "steps/common/" : "steps/differential/")
                           + (edge ? "held_edge" : "rounded");
                    s.kind = edge ? Kind::kEdge : Kind::kMove;
                    s.to = {
                        center.theta + (common ? 1 : -1) * config_.hip_sign * direction * change,
                        *mapped};
                    s.duration_s = edge ? 1.2 : .2;
                    s.holdout = holdout;
                    add(s);
                    if (!edge)
                        hold("steps/settle", 2, holdout);
                    if (edge) {
                        Segment back;
                        back.name = "steps/return_edge";
                        back.kind = Kind::kEdge;
                        back.to = center;
                        back.duration_s = 1.2;
                        back.holdout = holdout;
                        add(back);
                    } else {
                        move("steps/return", center, .2, holdout);
                        hold("steps/return_settle", 1, holdout);
                    }
                }
    }
    void dynamic_suite() {
        for (int center = 0; center < 3; ++center)
            for (int mode = 0; mode < 4; ++mode) {
                const auto name =
                    "dynamic/c" + std::to_string(center) + "/mode" + std::to_string(mode);
                anchor(name, {(center % 2) * 90 * rad, (75 + 10 * center) * rad});
                const bool low = mode < 2;
                excite(
                    name + "/excite", mode == 2 ? Coordinates::kCommon : Coordinates::kHip,
                    mode == 1 || mode == 2 ? 0
                    : low                  ? 45 * rad
                                           : 12 * rad,
                    mode == 0 ? 0
                    : low     ? 5 * rad
                              : 3 * rad,
                    low ? std::array{.1, .6} : std::array{.6, 2.0},
                    low ? std::array{.1, .8} : std::array{.8, 2.5});
            }
        anchor("high/anchor", {90 * rad, 85 * rad});
        excite("high/small", Coordinates::kCommon, .02, .01, {2., 6.}, {2.5, 6.}, false, 20);
        steps(false);
    }
    void holdout_suite() {
        for (double beta : {70., 90.})
            for (int direction : {-1, 1}) {
                ++group_;
                move(
                    "holdout/approach",
                    {(direction > 0 ? 90 : 0) * rad, (beta + direction * 3) * rad}, 2, true);
                move("holdout/center", {(direction > 0 ? 90 : 0) * rad, beta * rad}, 2, true);
                hold("holdout/arrival", 4, true, true);
                excite(
                    "holdout/multisine", Coordinates::kCommon, .03, .012, {.17, 3.73}, {.23, 3.91},
                    true, 30);
            }
        steps(true);
    }
    void jump_suite(bool holdout) {
        int cycle = 0;
        const std::array<double, 6> angles{0, 15, -15, 30, -30, 180};
        for (int combination = 0; combination < (holdout ? 6 : 15); ++combination) {
            // Sweep all orientations at each rise time before advancing speed.
            // Rotate compression depths between sweeps to avoid repeating the
            // same coupled angle/depth/rise tuple every six combinations.
            const int sweep = holdout ? combination % 3 : combination / 6;
            const int depth_index = holdout ? combination % 3 : (combination % 6 + sweep) % 3;
            const double depth = (depth_index == 0   ? 75
                                  : depth_index == 1 ? 65
                                                     : config_.beta_min / rad)
                               + (holdout ? 2 : 0);
            const double rise = (sweep == 0 ? .60 : sweep == 1 ? .35 : .20) + (holdout ? .06 : 0);
            const double angle = angles[holdout ? (5 - combination) : combination % 6] * rad;
            anchor("jump/ready", {angle, (holdout ? 90 : 85) * rad}, holdout);
            for (int repetition = 0; repetition < 3; ++repetition, ++cycle) {
                ++group_;
                const auto prefix = "jump/cycle" + std::to_string(cycle);
                move(
                    prefix + "/compress", {angle, depth * rad}, .8, holdout, cycle,
                    Jump::kCompress);
                hold(prefix + "/loaded", .6, holdout, true, cycle, Jump::kLoaded);
                move(
                    prefix + "/extend", {angle, (holdout ? 98 : 100) * rad}, rise, holdout, cycle,
                    Jump::kExtend);
                move(
                    prefix + "/tuck",
                    {angle + (holdout ? -10 : 10) * rad, (holdout ? 77 : 75) * rad}, .30, holdout,
                    cycle, Jump::kTuck);
                move(prefix + "/reach", {angle, 95 * rad}, .30, holdout, cycle, Jump::kReach);
                move(
                    prefix + "/buffer", {angle, (holdout ? 72 : 75) * rad}, .30, holdout, cycle,
                    Jump::kBuffer);
                move(prefix + "/settle", {angle, 85 * rad}, .8, holdout, cycle, Jump::kSettle);
                hold(
                    prefix + "/gap",
                    repetition == 0   ? 3
                    : repetition == 1 ? 1.5
                                      : 1,
                    holdout, false, cycle, Jump::kGap);
            }
        }
    }
    PairGeometry geometry_;
    PairRecordingConfig config_;
    Pose current_{};
    int group_ = 0;
    std::vector<Segment> segments_;
};

// Arrival corrections occur only in designated holds. After qualification the
// correction is frozen for that group; dynamic excitation has no beta servo.
class PairRecordingRunner {
public:
    explicit PairRecordingRunner(const PairRecordingPlan& plan)
        : plan_(plan) {
        for (const auto& segment : plan.segments()) {
            if (static_cast<std::size_t>(segment.group) >= bounds_.size())
                bounds_.push_back(plan.center_bounds(segment.group));
        }
    }
    PairSample update(
        double dt, const std::array<double, 2>& q, const std::array<double, 2>& dq,
        bool reference_due = true, const std::array<double, 2>* held = nullptr) {
        if (!std::isfinite(dt) || dt < 0 || dt > .02)
            throw std::invalid_argument("Invalid recording interval");
        if (done())
            return {};
        enter_group();
        const auto& all = plan_.segments();
        const auto& s = all[id_];
        elapsed_ += dt;
        if (s.arrival) {
            const auto beta = plan_.geometry().beta(q[1] - q[0]);
            const auto& c = plan_.config();
            const bool settled = beta && std::isfinite(dq[0]) && std::isfinite(dq[1])
                              && std::abs(*beta - s.to.beta) <= c.beta_tolerance
                              && std::max(std::abs(dq[0]), std::abs(dq[1])) <= c.arrival_speed;
            stable_ = settled ? stable_ + dt : 0;
            if (elapsed_ + 1e-9 >= s.duration_s && stable_ + 1e-9 >= s.stable_s) {
                admission_ = 1;
                qualified_segment_ = static_cast<int>(id_);
                advance();
            } else if (elapsed_ + 1e-9 >= c.arrival_timeout_s) {
                const auto previous_reference = held ? *held : sample().position;
                admission_ = 2;
                skipped_segment_ = static_cast<int>(id_);
                const int group = s.group;
                while (id_ < all.size() && all[id_].group == group)
                    ++id_;
                elapsed_ = stable_ = correction_wait_ = 0;
                if (!done()) {
                    rebase_from_ = previous_reference;
                    rebase_id_ = id_;
                    rebase_duration_ = all[id_].duration_s;
                    const auto target = plan_.sample(id_, all[id_].duration_s).position;
                    for (std::size_t axis = 0; axis < 2; ++axis) {
                        const double travel = std::abs(target[axis] - previous_reference[axis]);
                        rebase_duration_ = std::max(
                            {rebase_duration_, 1.875 * travel / c.max_speed[axis],
                             std::sqrt(5.773503 * travel / c.max_acceleration[axis])});
                    }
                    rebase_duration_ = std::ceil(rebase_duration_ * 50) / 50;
                }
            } else if (
                elapsed_ >= s.duration_s && beta && stable_ == 0 && correction_wait_ <= 0
                && reference_due) {
                const double error = plan_.geometry().at(s.to.beta).position - (q[1] - q[0]);
                const auto limits = bounds_.at(static_cast<std::size_t>(s.group));
                // Leave enough beta margin for every subsequent excitation in
                // this group. Unreachable bins are skipped rather than enlarged.
                if (limits[0] <= limits[1])
                    center_ = std::clamp(
                        center_ + std::clamp(error, -c.center_step_rad, c.center_step_rad),
                        limits[0], limits[1]);
                correction_wait_ = .2;
            }
            correction_wait_ = std::max(0.0, correction_wait_ - dt);
        } else if (elapsed_ + 1e-9 >= (rebased() ? rebase_duration_ : s.duration_s))
            advance();
        if (done())
            return {};
        enter_group();
        return sample();
    }
    PairSample sample() const {
        const auto& s = plan_.segments().at(id_);
        if (rebased()) {
            // Skipping a group changes the nominal predecessor. Join from the
            // last requested pair instead of jumping to an unvisited pose.
            auto value = plan_.sample(id_, s.duration_s);
            const double u = std::clamp(elapsed_ / rebase_duration_, 0.0, 1.0);
            const double blend = u * u * u * (10 + u * (-15 + 6 * u));
            for (std::size_t axis = 0; axis < 2; ++axis) {
                const double travel = value.position[axis] - (*rebase_from_)[axis];
                value.position[axis] = (*rebase_from_)[axis] + travel * blend;
                value.velocity[axis] = travel * 30 * u * u * (1 - u) * (1 - u) / rebase_duration_;
                value.acceleration[axis] =
                    travel * 60 * u * (1 - u) * (1 - 2 * u) / (rebase_duration_ * rebase_duration_);
            }
            const auto beta = plan_.geometry().beta(value.position[1] - value.position[0]);
            if (!beta)
                throw std::out_of_range("Rebased transition outside calibrated branch");
            value.beta = *beta;
            return value;
        }
        if (id_ == transition_id_ && s.kind == PairRecordingPlan::Kind::kMove) {
            const double u = std::clamp(elapsed_ / s.duration_s, 0.0, 1.0);
            const double blend = u * u * u * (10 + u * (-15 + 6 * u));
            const double velocity =
                -transition_center_ * 30 * u * u * (1 - u) * (1 - u) / s.duration_s;
            const double acceleration = -transition_center_ * 60 * u * (1 - u) * (1 - 2 * u)
                                      / (s.duration_s * s.duration_s);
            return plan_.sample(
                id_, elapsed_, transition_center_ * (1 - blend), velocity, acceleration);
        }
        return plan_.sample(id_, elapsed_, center_);
    }
    bool done() const { return id_ >= plan_.segments().size(); }
    std::size_t segment_id() const { return id_; }
    double segment_time() const { return elapsed_; }
    double center() const {
        if (done())
            return center_;
        const auto value = sample();
        return value.position[1] - value.position[0] - plan_.geometry().at(value.beta).position;
    }
    int admission() const { return rebased() ? 3 : admission_; }
    int skipped_segment() const { return skipped_segment_; }
    int qualified_segment() const { return qualified_segment_; }

private:
    void enter_group() {
        const auto& s = plan_.segments().at(id_);
        if (s.group == group_)
            return;
        transition_center_ = center_;
        transition_id_ = id_;
        group_ = s.group;
        center_ = stable_ = correction_wait_ = 0;
        admission_ = 0;
    }
    void advance() {
        rebase_from_.reset();
        ++id_;
        elapsed_ = stable_ = 0;
        correction_wait_ = 0;
    }
    const PairRecordingPlan& plan_;
    bool rebased() const { return rebase_from_ && id_ == rebase_id_; }
    std::optional<std::array<double, 2>> rebase_from_;
    std::size_t rebase_id_ = 0;
    double rebase_duration_ = 0;
    std::vector<std::array<double, 2>> bounds_;
    std::size_t id_ = 0, transition_id_ = 0;
    int group_ = -1, admission_ = 0, skipped_segment_ = -1, qualified_segment_ = -1;
    double elapsed_ = 0, stable_ = 0, center_ = 0, correction_wait_ = 0, transition_center_ = 0;
};

} // namespace rmcs_core::controller::identification
