#pragma once

#include "test_controller/pid_controller/controller_node.hpp"
#include <cmath>
#include <numbers>

namespace pid_controller {

class AngleNode : public ControllerNode {
public:
    using ControllerNode::ControllerNode;

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

} // namespace pid_controller