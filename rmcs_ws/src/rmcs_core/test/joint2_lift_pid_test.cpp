#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "hardware/joint2_lift_pid.hpp"
#include "hardware/joint2_encoder_velocity.hpp"

namespace {

using rmcs_core::hardware::Joint2LiftPid;
using rmcs_core::hardware::Joint2LiftPidParameters;
using rmcs_core::hardware::Joint2LiftSafetyGate;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void near(double actual, double expected, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-9)
        throw std::runtime_error(std::string{message} + ": actual=" + std::to_string(actual)
                                 + ", expected=" + std::to_string(expected));
}

void target_is_relative_directional_and_limited() {
    Joint2LiftPidParameters p;
    p.lift_angle_rad = 0.5;
    p.lower_limit_rad = -1.0;
    p.upper_limit_rad = 1.0;
    p.lift_direction = -1.0;
    Joint2LiftPid controller{p};
    require(controller.capture_target(0.25), "finite starting angle must capture a target");
    near(controller.target_angle(), -0.25, "negative direction must lower target angle");

    require(controller.capture_target(-0.9), "target may be recaptured for boundary test");
    near(controller.target_angle(), -1.0, "target must clamp at the joint limit");
}

void output_is_rate_and_torque_limited() {
    Joint2LiftPidParameters p;
    p.kp_nm_per_rad = 100.0;
    p.ki_nm_per_rad_s = 0.0;
    p.kd_nm_s_per_rad = 0.0;
    p.torque_limit_nm = 2.0;
    p.torque_slew_nm_s = 4.0;
    p.reference_speed_rad_s = 10.0;
    p.lift_angle_rad = 0.5;
    Joint2LiftPid controller{p};
    require(controller.capture_target(0.0), "target capture should succeed");
    near(controller.update(0.0, 0.0, 0.001, true), 0.004,
         "the first torque step must obey the slew limit");
    for (int i = 0; i < 1000; ++i)
        controller.update(0.0, 0.0, 0.001, true);
    near(controller.update(0.0, 0.0, 0.001, true), 2.0,
         "sustained request must remain within torque limit");
}

void fast_lift_reaches_24nm_and_stops_immediately() {
    Joint2LiftPidParameters p;
    p.kp_nm_per_rad = 48.0;
    p.torque_limit_nm = 24.0;
    p.torque_slew_nm_s = 48.0;
    p.reference_speed_rad_s = 1.0;
    p.lift_direction = -1.0;
    Joint2LiftPid controller{p};
    constexpr double angle = 1.44;
    constexpr double dt = 0.001;
    require(controller.capture_target(angle), "fast lift must capture the target");
    near(controller.target_angle(), angle - p.lift_angle_rad, "fast lift must keep the 30-degree target");
    double previous = 0.0;
    for (int i = 0; i < 500; ++i) {
        const double torque = controller.update(angle, 0.0, dt, true);
        require(torque <= 0.0 && torque >= -24.0, "fast lift must respect direction and 24 Nm ceiling");
        require(std::abs(torque - previous) <= 48.0 * dt + 1e-9, "fast lift must respect the 48 Nm/s ramp");
        previous = torque;
    }
    near(previous, -24.0, "stationary joint must reach 24 Nm within 0.5 seconds");
    near(controller.update(angle, 0.0, dt, false), 0.0, "permission loss must zero full 24 Nm immediately");
    require(!controller.capture_target(std::numeric_limits<double>::quiet_NaN()), "invalid feedback must not capture a target");
    near(controller.update(std::numeric_limits<double>::quiet_NaN(), 0.0, dt, true), 0.0, "invalid feedback must keep torque zero");
}

void permission_loss_immediately_zeros_and_restarts_ramp() {
    Joint2LiftPidParameters p;
    p.kp_nm_per_rad = 10.0;
    p.ki_nm_per_rad_s = 0.0;
    p.kd_nm_s_per_rad = 0.0;
    p.torque_limit_nm = 3.0;
    p.torque_slew_nm_s = 100.0;
    p.reference_speed_rad_s = 1.0;
    p.lift_angle_rad = 0.5;
    Joint2LiftPid controller{p};
    require(controller.capture_target(0.0), "target capture should succeed");
    require(controller.update(0.0, 0.0, 0.001, true) > 0.0,
            "permitted control should produce torque toward target");
    near(controller.update(0.0, 0.0, 0.001, false), 0.0,
         "lost permission must immediately zero torque");
    near(controller.update(0.0, 0.0, 0.001, true), 0.01,
         "re-arm must ramp the reference from current angle");
}

void integral_compensation_removes_constant_load_position_error() {
    // This synthetic inertia/damping/load model tests PID behavior only. Its
    // constants are not identified arm parameters or a prediction of the robot.
    constexpr double inertia = 1.0;
    constexpr double damping = 4.0;
    constexpr double load_torque = 10.0;
    constexpr double initial_angle = 1.44;
    constexpr double dt = 0.001;
    const auto run_loaded_lift = [](double ki, double integral_limit) {
        Joint2LiftPidParameters p;
        p.kp_nm_per_rad = 48.0;
        p.ki_nm_per_rad_s = ki;
        p.kd_nm_s_per_rad = 3.0;
        p.torque_limit_nm = 24.0;
        p.torque_slew_nm_s = 48.0;
        p.reference_speed_rad_s = 1.0;
        p.integral_limit_rad_s = integral_limit;
        p.lift_direction = -1.0;
        Joint2LiftPid controller{p};
        require(controller.capture_target(initial_angle), "loaded lift must capture target");

        double angle = initial_angle;
        double velocity = 0.0;
        double previous_torque = 0.0;
        for (int i = 0; i < 90000; ++i) {
            const double torque = controller.update(angle, velocity, dt, true);
            require(std::isfinite(torque) && std::abs(torque) <= p.torque_limit_nm,
                    "loaded lift must obey the torque ceiling");
            require(std::abs(torque - previous_torque) <= p.torque_slew_nm_s * dt + 1e-9,
                    "loaded lift must obey the torque slew limit");
            const double acceleration = (torque + load_torque - damping * velocity) / inertia;
            velocity += acceleration * dt;
            angle += velocity * dt;
            require(std::isfinite(angle) && std::isfinite(velocity),
                    "loaded plant state must remain finite");
            previous_torque = torque;
        }

        const double error = controller.position_error(angle);
        require(std::abs(velocity) < 0.001, "loaded lift must settle before comparing residuals");
        require(std::abs(previous_torque + load_torque) < 0.01,
                "settled controller must continue balancing the constant load");
        near(controller.update(angle, velocity, dt, false), 0.0,
             "permission loss must immediately remove the accumulated load compensation");
        near(controller.update(angle, velocity, dt, false), 0.0,
             "permission remaining off must keep torque zero");

        // A resumed controller must behave exactly like a fresh one with the
        // same retained target, even after building a substantial load integral.
        Joint2LiftPid fresh{p};
        require(fresh.capture_target(initial_angle), "fresh controller must capture same target");
        fresh.resume(angle);
        for (int i = 0; i < 100; ++i) {
            near(controller.update(angle, 0.0, dt, true), fresh.update(angle, 0.0, dt, true),
                 "re-permission must not retain the old load integral or output");
        }
        return std::abs(error);
    };

    const double limited_integral_error = run_loaded_lift(0.5, 3.0);
    const double compensated_error = run_loaded_lift(4.0, 4.0);
    const double expected_limited_error = (load_torque - 0.5 * 3.0) / 48.0;
    require(std::abs(limited_integral_error - expected_limited_error) < 0.001,
            "insufficient integral capacity must leave the predicted static position error");
    require(compensated_error < 0.001 && compensated_error < limited_integral_error * 0.01,
            "adequate integral capacity must remove at least 99 percent of the old residual");
}

void fast_release_preserves_drive_ramp_and_crosses_zero_before_reversing() {
    Joint2LiftPidParameters p;
    p.kp_nm_per_rad = 48.0;
    p.ki_nm_per_rad_s = 0.0;
    p.kd_nm_s_per_rad = 10.0;
    p.torque_limit_nm = 24.0;
    p.torque_slew_nm_s = 48.0;
    p.torque_release_nm_s = 240.0;
    p.reference_speed_rad_s = 1.0;
    p.lift_direction = -1.0;
    Joint2LiftPid controller{p};
    constexpr double angle = 1.44, dt = 0.001;
    require(controller.capture_target(angle), "fast-release target must capture");
    double torque = 0.0;
    for (int i = 0; i < 500; ++i) {
        const double previous = torque;
        torque = controller.update(angle, 0.0, dt, true);
        require(std::abs(torque - previous) <= 48.0 * dt + 1e-9,
                "fast release must not accelerate drive buildup");
    }
    near(torque, -24.0, "fast release must retain the 24 Nm drive ceiling");
    near(controller.update(angle, -1.0, dt, true), -23.76,
         "same-direction excess torque must release at 240 Nm/s");
    bool saw_zero = false;
    double previous_sent = -24.0;
    for (int i = 1; i < 150; ++i) {
        const double previous = torque;
        torque = controller.update(angle, -4.0, dt, true);
        require(previous * torque >= 0.0, "a release step must never cross zero");
        if (previous < 0.0)
            require(std::abs(torque) <= std::abs(previous), "reversing request must release old drive");
        if (previous >= 0.0)
            require(torque - previous <= 48.0 * dt + 1e-9,
                    "opposite drive must build at the original slew");
        // Mirror the periodic send and immediate-zero/sign-change paths.
        if (i % 5 == 0 || (previous_sent != 0.0
                           && (torque == 0.0 || previous_sent * torque < 0.0))) {
            require(previous_sent * torque >= 0.0,
                    "the send sequence must include zero before reversing");
            previous_sent = torque;
        }
        saw_zero = saw_zero || torque == 0.0;
    }
    require(saw_zero && torque > 0.0, "reverse drive must start after a zero-torque update");
    near(controller.update(angle, -4.0, dt, false), 0.0,
         "safety revocation must bypass even the fast release ramp");

    p.torque_release_nm_s = -1.0;
    require(!Joint2LiftPid{p}.valid_parameters(), "negative release rate must be rejected");
    p.torque_release_nm_s = 47.0;
    require(!Joint2LiftPid{p}.valid_parameters(), "explicit release slower than drive must be rejected");
    p.torque_release_nm_s = std::numeric_limits<double>::quiet_NaN();
    require(!Joint2LiftPid{p}.valid_parameters(), "nonfinite release rate must be rejected");
}

void stiction_release_and_loaded_hold_with_feedback_and_send_delay() {
    // Synthetic stress cases, not fitted robot parameters. Static friction drops
    // at breakaway; quantized samples use the real 20 ms velocity estimator.
    struct Plant { double inertia, load, static_friction, moving_friction; };
    constexpr std::array plants{
        Plant{1.0, 3.0, 16.0, 2.0}, Plant{0.5, 3.0, 16.0, 2.0},
        Plant{2.0, 8.0, 10.0, 2.0}, Plant{1.0, 10.0, 9.0, 5.0},
        Plant{0.5, 1.0, 18.0, 1.0}};
    struct Result { double overshoot, final_error, final_velocity, final_torque, hold_range; };
    const auto run = [](Plant plant, double kd, double release, int additional_delay_ms) {
        Joint2LiftPidParameters p;
        p.kp_nm_per_rad = 48.0;
        p.ki_nm_per_rad_s = 8.0;
        p.kd_nm_s_per_rad = kd;
        p.torque_limit_nm = 24.0;
        p.torque_slew_nm_s = 48.0;
        p.torque_release_nm_s = release;
        p.reference_speed_rad_s = 1.0;
        p.integral_limit_rad_s = 2.5;
        p.lift_direction = -1.0;
        Joint2LiftPid controller{p};
        rmcs_core::hardware::Joint2EncoderVelocityEstimator estimator;
        constexpr double dt = 0.001, initial_angle = 1.44;
        constexpr double encoder_scale = 65536.0 / (2.0 * std::numbers::pi);
        constexpr double torque_per_count = 42.0 * 0.12 * 33.0 / 4096.0;
        require(controller.capture_target(initial_angle), "stiction target must capture");
        double angle = initial_angle, velocity = 0.0, applied = 0.0, minimum = angle;
        double hold_minimum = std::numeric_limits<double>::infinity();
        double hold_maximum = -std::numeric_limits<double>::infinity();
        std::array<double, 21> velocity_history{};
        for (int step = 0; step < 60000; ++step) {
            const int raw = (static_cast<int>(std::lround(angle * encoder_scale)) % 65536 + 65536) % 65536;
            const double measured_velocity = estimator.update(raw, (step + 1) * int64_t{1000000});
            velocity_history[step % velocity_history.size()] = measured_velocity;
            const double delayed_velocity = velocity_history[
                (step + velocity_history.size() - additional_delay_ms) % velocity_history.size()];
            const double torque = controller.update(angle, delayed_velocity, dt, true);
            require(std::isfinite(torque) && std::abs(torque) <= 24.0,
                    "stiction trial must retain the torque ceiling");
            if (step % 5 == 0 || (applied != 0.0 && (torque == 0.0 || applied * torque < 0.0)))
                applied = std::round(torque / torque_per_count) * torque_per_count;
            const double net = applied + plant.load - 4.0 * velocity;
            if (std::abs(velocity) < 1e-5 && std::abs(net) <= plant.static_friction) {
                velocity = 0.0;
            } else {
                const double friction = std::copysign(
                    plant.moving_friction, std::abs(velocity) > 1e-5 ? velocity : net);
                const double next = velocity + (net - friction) / plant.inertia * dt;
                velocity = velocity * next < 0.0 && std::abs(net) <= plant.static_friction
                    ? 0.0 : next;
            }
            angle += velocity * dt;
            minimum = std::min(minimum, angle);
            if (step >= 50000) {
                hold_minimum = std::min(hold_minimum, angle);
                hold_maximum = std::max(hold_maximum, angle);
            }
            require(std::isfinite(angle) && std::isfinite(velocity), "stress plant must stay finite");
        }
        const Result result{
            std::max(0.0, controller.target_angle() - minimum),
            std::abs(controller.position_error(angle)), velocity, applied,
            hold_maximum - hold_minimum};
        near(controller.update(angle, velocity, dt, false), 0.0,
             "loaded stiction trial must still stop immediately on permission loss");
        return result;
    };
    double old_worst_overshoot = 0.0;
    for (const auto plant : plants) {
        old_worst_overshoot = std::max(old_worst_overshoot, run(plant, 3.0, 0.0, 0).overshoot);
        for (const int extra_delay : {0, 20}) {
            const auto result = run(plant, 10.0, 240.0, extra_delay);
            std::cout << "stiction J=" << plant.inertia << ", load=" << plant.load
                      << ", added_delay_ms=" << extra_delay
                      << ", overshoot_deg=" << result.overshoot * 180.0 / std::numbers::pi
                      << ", final_error_deg=" << result.final_error * 180.0 / std::numbers::pi
                      << ", hold_range_deg=" << result.hold_range * 180.0 / std::numbers::pi << '\n';
            require(result.overshoot < std::numbers::pi / 180.0,
                    "fix must keep synthetic breakaway overshoot below one degree");
            require(result.final_error < std::numbers::pi / 180.0,
                    "fix must hold within one degree in synthetic stress cases");
            require(result.hold_range < std::numbers::pi / 180.0,
                    "settled synthetic hold must not roll back several degrees");
            require(std::abs(result.final_velocity) < 0.01,
                    "loaded hold must settle instead of oscillating");
            require(result.final_torque < -0.1, "arrival must retain nonzero supporting torque");
        }
    }
    require(old_worst_overshoot > 4.0 * std::numbers::pi / 180.0,
            "stress model must expose the old breakaway overshoot regression");
}

void invalid_inputs_fail_closed() {
    Joint2LiftPid controller;
    require(controller.capture_target(0.0), "target capture should succeed");
    const std::array<std::array<double, 3>, 4> cases{{
        {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.001},
        {0.0, std::numeric_limits<double>::quiet_NaN(), 0.001},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.021}}};
    for (const auto [angle, velocity, dt] : cases)
        near(controller.update(angle, velocity, dt, true), 0.0,
             "nonfinite feedback or abnormal time step must fail closed");
}

void safety_gate_requires_double_down_then_centered_release() {
    Joint2LiftSafetyGate gate;
    require(!gate.update(true, true, true, false, true, true, true),
            "startup must wait for double-down");
    require(!gate.update(true, true, true, true, true, true, true),
            "double-down itself must keep torque disabled");
    require(gate.update(true, true, true, false, true, true, true),
            "centered release after confirmation may permit torque");
    require(!gate.update(true, true, true, true, true, true, true),
            "double-down must revoke torque immediately");
    require(!gate.update(true, false, true, false, true, true, true),
            "remote loss must revoke and clear the confirmation");
    require(!gate.update(true, true, true, false, true, true, true),
            "remote recovery must require a new double-down confirmation");
}

} // namespace

int main() {
    const std::array tests{
        std::pair{"direction and position limits", &target_is_relative_directional_and_limited},
        std::pair{"torque and slew limits", &output_is_rate_and_torque_limited},
        std::pair{"24 Nm fast lift and immediate stop", &fast_lift_reaches_24nm_and_stops_immediately},
        std::pair{"permission loss and resume", &permission_loss_immediately_zeros_and_restarts_ramp},
        std::pair{"constant load integral compensation", &integral_compensation_removes_constant_load_position_error},
        std::pair{"fast release and safe reversal", &fast_release_preserves_drive_ramp_and_crosses_zero_before_reversing},
        std::pair{"stiction release and loaded hold", &stiction_release_and_loaded_hold_with_feedback_and_send_delay},
        std::pair{"invalid inputs fail closed", &invalid_inputs_fail_closed},
        std::pair{"double-down safety gate", &safety_gate_requires_double_down_then_centered_release}};
    int failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "PASS: " << name << '\n';
        } catch (const std::exception& error) {
            ++failures;
            std::cerr << "FAIL: " << name << ": " << error.what() << '\n';
        }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " cases passed\n";
    return failures == 0 ? 0 : 1;
}
