#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "controller/chassis/wheel_leg/self_righting.hpp"

using namespace rmcs_core::controller::chassis::wheel_leg;
using Core = SelfRightingCore;

void require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

Core::Axes pose(double knee, std::array<double, 2> angles) {
    const auto& table = recovery_v202::geometry;
    auto b = std::upper_bound(table.begin(), table.end(), knee, [](double value, const auto& row) {
        return value < row[0];
    });
    require(b != table.begin() && b != table.end(), "Test knee outside table");
    const auto& a = *(b - 1);
    const double fraction = (knee - a[0]) / ((*b)[0] - a[0]);
    Core::Axes q;
    for (std::size_t i = 0; i < 4; ++i)
        q[i] = std::lerp(a[i + 1], (*b)[i + 1], fraction)
             + (i < 2 ? -1. : 1.) * angles[i / 2] * std::numbers::pi / 180.;
    return q;
}

Core::Settings settings() {
    Core::Settings s;
    s.stand_knee = recovery_v202::stand_knee;
    s.stand_orbit = recovery_v202::stand_orbit;
    s.cruise_speed = 1.2;
    s.timeout_seconds = 40.;
    return s;
}

void follow(Core::Feedback& f, const Core::Output& out, double fraction = 1.) {
    for (std::size_t axis = 0; axis < 4; ++axis) {
        const double next = f.position[axis] + fraction * (out.position[axis] - f.position[axis]);
        f.velocity[axis] = (next - f.position[axis]) / .001;
        f.position[axis] = next;
    }
}

int main() {
    // Reference/feedback replay tests: no simulated contacts or vehicle motion.
    // Four independent front/rear combinations, asymmetric winding and lag.
    int completed = 0;
    for (int combination = 0; combination < 4; ++combination)
        for (bool winding : {false, true})
            for (double lag : {1., .2}) {
                Core core(recovery_v202::geometry, settings());
                Core::PoseRouteSettings routes;
                routes.enabled = true;
                core.configure_pose_routes(routes);
                Core::Feedback f;
                std::array<double, 2> initial{
                    combination & 1 ? 90. : -90., combination & 2 ? 90. : -90.};
                if (winding) {
                    initial[0] += 720.;
                    initial[1] -= 1080.;
                }
                f.position = pose(75., initial);
                const auto entry = f.position;
                auto previous = entry;
                Core::Route expected = combination == 0 ? Core::Route::ProneRear
                                     : combination == 3 ? Core::Route::ProneFront
                                                        : Core::Route::ProneMixed;
                bool clearance_checked = false, support_checked = false, captured = false;
                bool planting_started = false, landing_flexed = false;
                double planting_shape = 0.;
                for (int tick = 0; tick < 25000; ++tick) {
                    const auto out = core.step(f, .001, true);
                    require(out.fault == 0, "Pose route faulted under reference replay");
                    require(core.route() == expected, "Independent leg route was misclassified");
                    for (std::size_t axis = 0; axis < 4; ++axis) {
                        require(
                            std::abs(out.position[axis] - previous[axis]) <= .001200001,
                            "Route transition jumped motor references");
                        require(
                            std::abs(out.velocity[axis]) <= 1.2000001,
                            "Route trajectory exceeded cruise speed");
                        require(
                            std::isfinite(out.torque[axis]) && std::abs(out.torque[axis]) <= 40.,
                            "Route produced invalid motor effort");
                    }
                    for (double torque : out.wheel_torque)
                        require(
                            std::isfinite(torque) && std::abs(torque) <= 4.5,
                            "Route produced invalid wheel effort");
                    // At waypoint transitions the new reference has moved by
                    // at most 0.5*a*dt^2, so compare with an independent target.
                    if (!clearance_checked && out.phase == Core::Sweep) {
                        auto overhead = initial;
                        for (double& angle : overhead)
                            angle += std::sin(angle * std::numbers::pi / 180.) > 0. ? 90. : -90.;
                        const auto target = pose(45., overhead);
                        for (std::size_t i = 0; i < 4; ++i)
                            require(
                                std::abs(out.position[i] - target[i]) < 1e-5,
                                "Leg did not take the overhead arc with its own winding");
                        clearance_checked = true;
                    }
                    if (!support_checked && out.phase == Core::Thrust) {
                        auto support = initial;
                        const double direction = combination == 3 ? 1. : -1.;
                        for (double& angle : support) {
                            angle += std::sin(angle * std::numbers::pi / 180.) > 0. ? 90. : -90.;
                            angle += direction * 85.;
                        }
                        const auto target = pose(45., support);
                        for (std::size_t i = 0; i < 4; ++i)
                            require(
                                std::abs(out.position[i] - target[i]) < 1e-5,
                                "Front/rear route used the wrong support-side stroke");
                        support_checked = true;
                    }
                    if (out.phase == Core::Plant) {
                        const double shape = out.position[1] - out.position[0];
                        if (core.route_waypoint() == 4) {
                            const auto extended = pose(105., {0., 0.});
                            require(
                                std::abs(shape - (extended[1] - extended[0])) < 1e-5,
                                "Landing retracted before the final supporting arc");
                        }
                        if (!planting_started) {
                            planting_shape = shape;
                            planting_started = true;
                        }
                        landing_flexed = landing_flexed || shape < planting_shape - .3;
                    }
                    previous = out.position;
                    follow(f, out, lag);
                    if (out.phase == Core::Complete) {
                        captured = true;
                        break;
                    }
                }
                require(
                    clearance_checked && support_checked && landing_flexed && captured,
                    "Pose route did not traverse the required stages");
                const auto stopped = core.step(f, .001, false);
                require(
                    stopped.phase == Core::Disabled && core.route() == Core::Route::None,
                    "Disable retained route state");
                for (double torque : stopped.torque)
                    require(torque == 0., "Disable retained effort");
                f.position = entry;
                f.velocity.fill(0.);
                require(
                    core.step(f, .001, true).phase == Core::Tuck, "Reset did not replay selection");
                ++completed;
            }

    // Plant assist has the physical wheel signs for front/rear support. A
    // pitch-rate request cannot spin the wheels before the legs are below.
    for (double angle : {-90., 90.}) {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {angle, angle});
        f.gyro[1] = .1;
        bool initial_checked = false, assist_checked = false;
        for (int tick = 0; tick < 20000; ++tick) {
            const auto out = core.step(f, .001, true);
            require(out.fault == 0, "Plant assist replay faulted");
            if (out.phase == Core::Plant) {
                if (!initial_checked) {
                    require(
                        out.wheel_torque[0] == 0. && out.wheel_torque[1] == 0.,
                        "Wheel balance ran before the legs were below the body");
                    f.gyro.fill(0.);
                    initial_checked = true;
                } else if (std::abs(out.wheel_torque[0]) > .01) {
                    require(
                        out.wheel_torque[0] * angle < 0., "Plant wheel assist has wrong direction");
                    require(
                        std::abs(out.wheel_torque[0] + out.wheel_torque[1]) < 1e-10,
                        "Plant assist changed physical left/right wheel signs");
                    assist_checked = true;
                    break;
                }
            }
            follow(f, out);
        }
        require(
            initial_checked && assist_checked, "Plant assist did not run in its measured window");
    }

    // A leg below the body can still point away from the ground when the
    // chassis is pitched. Wheel permission uses gravity, not body angle alone.
    {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {45., 45.});
        f.gravity = {-.5, 0., -std::sqrt(.75)};
        bool planted = false;
        for (int tick = 0; tick < 3000; ++tick) {
            const auto out = core.step(f, .001, true);
            require(out.fault == 0, "Pitched support replay faulted");
            if (out.phase == Core::Plant) {
                require(
                    out.wheel_torque[0] == 0. && out.wheel_torque[1] == 0.,
                    "Wheels ran while legs still pointed away from gravity");
                planted = true;
                break;
            }
            follow(f, out);
        }
        require(planted, "Pitched support did not enter Plant");
    }

    // One down leg remains a support while the opposite leg is rerouted.
    for (bool left_down : {false, true}) {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., left_down ? std::array{0., 90.} : std::array{-90., 0.});
        const auto entry = f.position;
        const std::size_t down = left_down ? 0 : 1;
        bool checked = false;
        for (int tick = 0; tick < 10000; ++tick) {
            const auto out = core.step(f, .001, true);
            require(out.fault == 0, "Mixed support route faulted");
            // The pair sum encodes orbit; changing length must preserve it.
            require(
                std::abs(
                    out.position[down * 2] + out.position[down * 2 + 1] - entry[down * 2]
                    - entry[down * 2 + 1])
                    < 1e-4,
                "A leg below the body was unnecessarily swept overhead");
            follow(f, out);
            if (out.phase == Core::Thrust) {
                checked = true;
                break;
            }
        }
        require(checked && core.route() == Core::Route::ProneMixed, "Mixed support not selected");
    }

    for (double gy : {-1., 0., 1.}) {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {90., -90.});
        f.gravity = gy == 0. ? std::array{0., 0., 1.} : std::array{0., gy, 0.};
        const auto expected = gy == 0. ? Core::Route::Inverted
                            : gy > 0.  ? Core::Route::LeftSide
                                       : Core::Route::RightSide;
        bool failed = false;
        for (int tick = 0; tick < 25000; ++tick) {
            const auto out = core.step(f, .001, true);
            require(core.route() == expected, "Fallen body route misclassified");
            require(out.phase != Core::Complete, "Waypoint arrival falsely declared body upright");
            for (double torque : out.wheel_torque)
                require(torque == 0., "Inverted/side route spun unsupported wheels");
            if (out.fault) {
                require(out.fault == 8, "Finite fallen stroke failed for an unexpected reason");
                failed = true;
                break;
            }
            follow(f, out);
        }
        require(failed, "A finite fallen stroke repeated indefinitely");
        require(core.step(f, .001, true).fault == 8, "Failed route did not latch");
    }

    // Inclined inversion sweeps toward upright with fixed winding. This is a
    // trajectory check; the fixed IMU must still terminate with failure 8.
    for (double gx : {-std::sqrt(.75), std::sqrt(.75)}) {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {810., -1170.});
        f.gravity = {gx, 0., .5};
        const double ground = std::atan2(gx, -.5) * 180. / std::numbers::pi;
        std::array<double, 2> end{810., -1170.};
        for (double& angle : end)
            angle += std::remainder(ground - angle, 360.) + (gx > 0. ? -180. : 180.);
        const auto target = pose(105., end);
        bool failed = false;
        for (int tick = 0; tick < 25000; ++tick) {
            const auto out = core.step(f, .001, true);
            if (out.fault) {
                require(out.fault == 8, "Inclined inversion failed for unexpected reason");
                failed = true;
                break;
            }
            follow(f, out);
        }
        require(failed, "Inclined inversion did not stop after its finite stroke");
        for (std::size_t axis = 0; axis < 4; ++axis)
            require(
                std::abs(f.position[axis] - target[axis]) < .04,
                "Inclined inversion swept away from upright or lost motor winding");
    }

    // Gravity-aligned inverted wheel support damps rolling; an overhead
    // clearance segment must not apply that damping before reaching support.
    {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {180., 180.});
        f.gravity = {0., 0., 1.};
        f.wheel_velocity = {2., -3.};
        bool damped = false;
        for (int tick = 0; tick < 10000; ++tick) {
            const auto out = core.step(f, .001, true);
            require(out.fault == 0, "Inverted wheel damping replay faulted");
            if (out.phase == Core::Thrust) {
                require(
                    out.wheel_torque[0] == -.5 && out.wheel_torque[1] == .75,
                    "Inverted wheel support did not oppose measured rolling");
                damped = true;
                break;
            }
            require(
                out.wheel_torque[0] == 0. && out.wheel_torque[1] == 0.,
                "Inverted wheel damping ran during clearance");
            follow(f, out);
        }
        require(damped, "Inverted wheel support did not reach damping stage");
    }

    // Measured flip completion reselects the prone route with continuous dwell.
    {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {90., -90.});
        f.gravity = {0., 0., 1.};
        core.step(f, .001, true);
        f.gravity = {0., 0., -1.};
        for (int tick = 0; tick < 79; ++tick) {
            const auto out = core.step(f, .001, true);
            follow(f, out);
            require(core.route() == Core::Route::Inverted, "Flip skipped measured upright dwell");
        }
        f.gyro[2] = 3.;
        core.step(f, .001, true);
        f.gyro.fill(0.);
        for (int tick = 0; tick < 79; ++tick) {
            const auto out = core.step(f, .001, true);
            follow(f, out);
            require(core.route() == Core::Route::Inverted, "Interrupted flip dwell accumulated");
        }
        auto last = f.position;
        for (int tick = 0; tick < 2; ++tick) {
            const auto out = core.step(f, .001, true);
            for (std::size_t axis = 0; axis < 4; ++axis)
                require(
                    std::abs(out.position[axis] - last[axis]) <= .001200001,
                    "Flip-to-prone route jumped references");
            follow(f, out);
            last = out.position;
        }
        require(
            core.route() == Core::Route::ProneMixed,
            "Flip did not reselect independent prone legs");
    }

    // Standing entry still needs actual pose tracking and a fresh 500 ms dwell.
    {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(
            recovery_v202::stand_knee, {recovery_v202::stand_orbit, recovery_v202::stand_orbit});
        auto out = core.step(f, .001, true);
        require(out.phase == Core::Capture, "Standing entry skipped capture");
        f.gyro[2] = .4;
        for (int tick = 0; tick < 600; ++tick)
            require(
                core.step(f, .001, true).phase == Core::Capture,
                "Yaw motion passed stability gate");
        f.gyro.fill(0.);
        f.position[0] += .1;
        f.position[1] += .1;
        for (int tick = 0; tick < 600; ++tick)
            require(
                core.step(f, .001, true).phase == Core::Capture, "Leg error passed stability gate");
        f.position = out.position;
        for (int tick = 0; tick < 499; ++tick)
            require(
                core.step(f, .001, true).phase == Core::Capture,
                "Capture skipped continuous dwell");
        require(
            core.step(f, .001, true).phase == Core::Complete, "Quiet standing did not complete");
        f.gravity = {std::sqrt(1. - .79 * .79), 0., -.79};
        require(
            core.step(f, .001, true).phase == Core::Capture,
            "Completed flag survived exit from the capture tilt envelope");
        f.gravity = {0., 1., 0.};
        for (int tick = 0; tick < 101; ++tick)
            out = core.step(f, .001, true);
        require(out.fault == 6, "Capture loss did not stop recovery");
        for (double torque : out.torque)
            require(torque == 0., "Capture loss retained effort");
    }

    {
        Core core(recovery_v202::geometry, settings());
        Core::PoseRouteSettings routes;
        routes.enabled = true;
        core.configure_pose_routes(routes);
        Core::Feedback f;
        f.position = pose(75., {90., -90.});
        f.position[1] += 2.;
        require(core.step(f, .001, true).fault == 7, "Impossible linkage geometry was accepted");
        core.reset();
        f.position = pose(75., {90., -90.});
        f.velocity[0] = 3.15;
        require(core.step(f, .001, true).fault == 3, "Pose route masked actual overspeed");
        core.reset();
        f.velocity.fill(0.);
        require(core.step(f, .001, true, false).fault == 2, "Pose route accepted stale sensing");
        core.reset();
        for (int parameter = 0; parameter < 5; ++parameter) {
            auto invalid = routes;
            if (parameter == 0)
                invalid.tuck_knee_deg = 40.;
            if (parameter == 1)
                invalid.upright_dwell_seconds = 0.;
            if (parameter == 2)
                invalid.support_orbit_deg = std::numeric_limits<double>::quiet_NaN();
            if (parameter == 3)
                invalid.landing_knee_deg = invalid.support_knee_deg;
            if (parameter == 4)
                invalid.landing_orbit_deg = 46.;
            bool rejected = false;
            try {
                core.configure_pose_routes(invalid);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "Invalid route configuration was accepted");
        }
        core.step(f, .001, true);
        bool rejected = false;
        try {
            core.configure_pose_routes(routes);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Running routes were reconfigured");
    }
    std::cout << "PASS: " << completed
              << " prone path replays, mixed support, finite inverted/side strokes, "
                 "measured reselection, stable capture, reset and faults\n";
}
