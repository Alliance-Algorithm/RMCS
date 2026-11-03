#pragma once
#include "gas_spring.hpp"
#include "joint_coordinates.hpp"
#include <algorithm>
#include <array>
#include <cmath>
namespace rmcs_core::controller::chassis {
class JointSolver {
public:
    static JointVector
        pd(const JointVector& target, const JointVector& q, const JointVector& dq, double kp,
           double kd, double gas_gain) {
        const auto gas = gas_spring_compensation({q[0], q[1], q[2], q[3]}, gas_gain);
        JointVector effort;
        for (int i = 0; i < 4; ++i)
            effort[i] = kp * (target[i] - q[i]) - kd * dq[i] + gas.torque[i];
        for (int i = 4; i < 6; ++i)
            effort[i] = std::clamp(.5 * (target[i] - dq[i]), -4.5, 4.5);
        return effort;
    }
    static JointVector constrain(
        JointVector effort, const JointVector& q, const JointVector& caps, double margin = .03) {
        for (int i = 0; i < 4; ++i)
            effort[i] = std::clamp(effort[i], -40., 40.);
        for (int side = 0; side < 2; ++side) {
            const int hip = 2 * side, crank = hip + 1;
            const double sign = side == 0 ? 1. : -1.;
            const double relative = sign * (q[crank] - q[hip]);
            const double outward = sign * (effort[crank] - effort[hip]);
            if ((relative >= kMechanicalBounds[side][1] - margin && outward > 0.)
                || (relative <= kMechanicalBounds[side][0] + margin && outward < 0.)) {
                effort[hip] += sign * outward / 2.;
                effort[crank] -= sign * outward / 2.;
            }
        }
        double scale = 1.;
        for (int i = 0; i < 4; ++i)
            if (std::abs(effort[i]) > std::min(40., caps[i]))
                scale = std::min(scale, std::min(40., caps[i]) / std::abs(effort[i]));
        for (int i = 0; i < 4; ++i)
            effort[i] *= scale;
        for (int i = 4; i < 6; ++i)
            effort[i] = std::clamp(effort[i], -caps[i], caps[i]);
        return motor_effort(effort);
    }
};
} // namespace rmcs_core::controller::chassis
