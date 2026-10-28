// Offline stdin/stdout probe for comparison to the training controller. No ROS,
// motor transport or policy is initialized.
#include <array>
#include <iomanip>
#include <iostream>

#include "controller/chassis/wheel_leg/gas_spring_compensation.hpp"

int main() {
    std::array<double, 4> q{};
    std::cout << std::setprecision(17);
    while (std::cin >> q[0] >> q[1] >> q[2] >> q[3]) {
        const auto value = rmcs_core::controller::chassis::gas_spring_compensation(q, 1.0);
        if (!value.valid)
            return 1;
        for (double torque : value.torque)
            std::cout << torque << ' ';
        for (double beta : value.beta_deg)
            std::cout << beta << ' ';
        std::cout << '\n';
    }
}
