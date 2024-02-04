#pragma once

#include <cmath>
#include <numbers>

#include "controller/pid/pid_node.hpp"

namespace controller {
namespace pid {

class AnglePidNode : public PidNode {
public:
    using PidNode::PidNode;

    virtual double calculate_err(double measurement) override {
        using namespace std::numbers;
        auto diff = fmod(setpoint - measurement, 2 * pi);
        if (diff > pi)
            diff -= 2 * pi;
        else if (diff < -pi)
            diff += 2 * pi;
        return diff;
    }
};

} // namespace pid
} // namespace controller