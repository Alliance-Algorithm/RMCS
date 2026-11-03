#include "controller/chassis/wheel_leg/policy/selection.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
int main(int argc, char** argv) {
    using namespace rmcs_core::controller::chassis;
    if (argc != 2)
        return 2;
    const std::string mode = argv[1];
    if (mode != "--prone-motion-v4-odom-v2" && mode != "--wheelbipe-v202")
        return 2;
    ProneMotion prone;
    Wheelbipe wheelbipe;
    std::uint64_t tick;
    int reset, enabled;
    auto read = [] {
        std::string token;
        std::cin >> token;
        return std::strtod(token.c_str(), nullptr);
    };
    std::cout << std::setprecision(17);
    while (std::cin >> tick >> reset >> enabled) {
        JointVector q{}, dq{}, applied{};
        std::array<double, 3> gyro{}, gravity{}, command{};
        for (auto& v : q)
            v = read();
        for (auto& v : dq)
            v = read();
        for (auto& v : gyro)
            v = read();
        for (auto& v : gravity)
            v = read();
        for (auto& v : command)
            v = read();
        const double height = read();
        for (auto& v : applied)
            v = read();
        if (!std::cin)
            return 2;
        if (reset) {
            prone.reset();
            wheelbipe.reset();
        }
        const auto frame =
            mode == "--wheelbipe-v202"
                ? wheelbipe.sample(tick, q, dq, gyro, gravity, command, height, applied, enabled)
                : prone.sample(tick, q, dq, gyro, gravity, command, height, applied, enabled);
        if (!frame) {
            std::cout << "0\n";
            continue;
        }
        std::cout << '1';
        for (double v : *frame)
            std::cout << ' ' << v;
        for (double v : measured_policy_history(q))
            std::cout << ' ' << v;
        std::cout << '\n';
    }
}
