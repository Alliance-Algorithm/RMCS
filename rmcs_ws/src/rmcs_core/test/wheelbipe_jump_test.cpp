#include "controller/chassis/wheel_leg/joint_solver.hpp"
#include "controller/chassis/wheel_leg/jump_command.hpp"
#include "controller/chassis/wheel_leg/jump_control.hpp"
#include "controller/chassis/wheel_leg/jump_trigger.hpp"
#include "controller/chassis/wheel_leg/policy/selection.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string_view>

using namespace rmcs_core::controller::chassis;
using namespace rmcs_core::controller::chassis::wheel_leg;

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

int main(int argc, char** argv) {
    std::cout << std::setprecision(17);
    if (argc == 2 && std::string_view{argv[1]} == "--trace") {
        bool requested, flight;
        while (std::cin >> requested >> flight) {
            LegVector desired{}, q{}, dq{};
            for (auto* row : {&desired, &q, &dq})
                for (auto& x : *row)
                    std::cin >> x;
            require(bool(std::cin), "Truncated trace input");
            const auto target = paired_targets(desired, q, jump_target_bounds(requested));
            LegVector pd{};
            for (int i = 0; i < 4; ++i)
                pd[i] = 80. * (target[i] - q[i]) - 1.5 * dq[i];
            const auto holding = jump_spring_holding(q);
            const auto extra = jump_tuck_effort(q, target, pd, flight && requested);
            JointVector effort{}, position{}, caps{40., 40., 40., 40., 4.5, 4.5};
            for (int i = 0; i < 4; ++i) {
                effort[i] = pd[i] + extra.torque[i];
                position[i] = q[i];
            }
            const LegBounds bounds{
                {{wheelbipe_jump_v4::kMechanicalMin, wheelbipe_jump_v4::kMechanicalMax},
                 {wheelbipe_jump_v4::kMechanicalMin, wheelbipe_jump_v4::kMechanicalMax}}};
            const auto final =
                motor_effort(JointSolver::constrain(effort, position, caps, .03, bounds));
            for (const auto& row : {target, pd, holding, extra.torque})
                for (const auto x : row)
                    std::cout << x << ' ';
            for (const auto x : extra.gate)
                std::cout << x << ' ';
            for (int i = 0; i < 4; ++i)
                std::cout << final[i] << ' ';
            std::cout << '\n';
        }
        return 0;
    }
    if (argc == 2 && std::string_view{argv[1]} == "--reference") {
        double apex, t;
        while (std::cin >> apex >> t) {
            JumpCommand command{apex};
            command.request(0.);
            command.update(t);
            std::cout << command.height(t) << ' ' << command.state(t) << ' ';
            for (const auto x : command.context(t))
                std::cout << x << ' ';
            std::cout << '\n';
        }
        return 0;
    }
    require(!jump_requested(false, true, false, false, true, true), "Disarmed trigger");
    require(jump_requested(true, true, false, false, true, false), "Ctrl rising trigger");
    require(!jump_requested(true, true, true, false, true, false), "Held Ctrl repeats");
    require(jump_requested(true, false, false, true, true, true), "Switch rising trigger");
    JumpCommand jump;
    require(!jump.request(std::numeric_limits<double>::quiet_NaN()), "Nonfinite request");
    require(jump.request(10.) && !jump.request(10.1), "Busy request replacement");
    require(jump.state(10.) == JumpCommand::CROUCH, "Initial phase");
    jump.update(9.);
    require(!jump.active(), "Backward clock reset");
    jump.request(0.);
    jump.update(wheelbipe_jump_v4::kRequestDuration);
    require(!jump.active() && jump.context(6.)[0] == 1., "Request expiry");
    jump.request(0.);
    jump.reset();
    require(!jump.active(), "Explicit reset");
    const LegVector q{0., .7, 0., -.7}, target{.1, .6, -.1, -.6};
    const LegVector inward{8., -8., -8., 8.}, braking{-8., 8., 8., -8.};
    const auto tuck = jump_tuck_effort(q, target, inward, true);
    require(tuck.gate[0] == 1. && tuck.gate[1] == 1., "Inward intent gate");
    require(
        tuck.torque[0] > 0. && tuck.torque[1] == -tuck.torque[0] && tuck.torque[2] < 0.
            && tuck.torque[3] == -tuck.torque[2],
        "Bilateral torque direction");
    require(jump_tuck_effort(q, target, braking, true).torque == LegVector{}, "Braking assistance");
    require(
        jump_tuck_effort(q, target, inward, false).torque == LegVector{}, "Ordinary assistance");
    require(jump_tuck_effort(q, q, inward, true).torque == LegVector{}, "No relative intent");
    const auto extended = jump_target_bounds(true), ordinary = jump_target_bounds(false);
    require(std::abs(extended[0][1] - ordinary[0][1] - .2) < 1e-14, "Target authority");
    const LegBounds bounds{{{-.14, 1.42}, {-.14, 1.42}}};
    const JointVector caps{40., 40., 40., 40., 4.5, 4.5};
    const auto protected_effort = motor_effort(
        JointSolver::constrain(
            {-40., 40., 40., -40., 0., 0.}, {0., 1.42, 0., -1.42, 0., 0.}, caps, .03, bounds));
    require(protected_effort == JointVector{}, "Actual angle outward protection");
    require(
        policy_parameters("wheelbipe").leg_kp == 60. && policy_parameters("wheelbipe").leg_kd == 2.
            && !policy_parameters("wheelbipe").jump_control
            && policy_parameters("wheelbipe-jump-v4").leg_kp == 80.,
        "Legacy policy parameters changed");
    std::cout << "Jump request, reset, compensation direction, braking, target authority and "
                 "actual limits passed\n";
}
