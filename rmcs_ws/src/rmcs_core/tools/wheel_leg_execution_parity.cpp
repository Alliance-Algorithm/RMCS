#include "controller/chassis/wheel_leg/joint_solver.hpp"
#include "controller/chassis/wheel_leg/policy/selection.hpp"
#include <iomanip>
#include <iostream>
int main() {
    using namespace rmcs_core::controller::chassis;
    int mode;
    std::cout << std::setprecision(17);
    while (std::cin >> mode) {
        JointVector q, dq, raw, caps;
        for (auto& v : q)
            std::cin >> v;
        for (auto& v : dq)
            std::cin >> v;
        for (auto& v : raw)
            std::cin >> v;
        for (auto& v : caps)
            std::cin >> v;
        if (!std::cin)
            return 2;
        const auto& policy = mode ? kWheelbipe : kProneMotion;
        const auto target = action_target(policy, raw, q), history = executed_history(policy, raw);
        const auto effort = JointSolver::constrain(
            JointSolver::pd(target, q, dq, policy.leg_kp, policy.leg_kd, policy.gas_gain), q, caps);
        for (double v : target)
            std::cout << v << ' ';
        for (double v : history)
            std::cout << v << ' ';
        for (double v : effort)
            std::cout << v << ' ';
        std::cout << '\n';
    }
    return 0;
}
