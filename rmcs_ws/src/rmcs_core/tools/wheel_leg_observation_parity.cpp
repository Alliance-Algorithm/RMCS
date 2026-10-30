// Offline stdin/stdout fixture driver. No executor, vehicle or transport.
#include "controller/chassis/wheel_leg/unified_observation.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    using namespace rmcs_core::controller::chassis;
    if (argc > 2 || (argc == 2 && std::string{argv[1]} != "--prone-motion-v4"))
        return 2;
    UnifiedObservation observer{argc == 2};
    std::uint64_t tick;
    int reset, enabled;
    std::cout << std::setprecision(17);
    const auto read = [] {
        std::string token;
        std::cin >> token;
        return std::strtod(token.c_str(), nullptr);
    };
    while (std::cin >> tick >> reset >> enabled) {
        std::array<double, 6> q{}, dq{}, applied{};
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
        if (reset)
            observer.reset(tick);
        const auto frame =
            observer.sample(tick, q, dq, gyro, gravity, command, height, applied, enabled);
        if (!frame) {
            std::cout << "0\n";
            continue;
        }
        std::cout << '1';
        for (double v : *frame)
            std::cout << ' ' << v;
        for (double v : measured_policy_history(
                 q, argc == 2 ? kFullVehicleV202PolicyProfile : kV6PolicyProfile))
            std::cout << ' ' << v;
        std::cout << '\n';
    }
}
