#include "controller/chassis/wheel_leg/motion_planner.hpp"
#include <iomanip>
#include <iostream>
int main() {
    using namespace rmcs_core::controller::chassis;
    TrajectoryPlanner planner;
    TrajectoryPlanner::State state;
    int reset;
    double command, length, dk, ds;
    std::cout << std::setprecision(17);
    while (std::cin >> reset >> command >> length >> state.wheel_velocity >> state.pitch
           >> state.pitch_velocity >> state.com_offset >> state.leg_length >> state.velocity
           >> state.distance >> state.knee[0] >> state.knee[1] >> state.shank[0] >> state.shank[1]
           >> dk >> ds) {
        if (reset)
            planner.reset();
        state.knee_rate = {dk, dk};
        state.shank_rate = {ds, ds};
        const auto ref = planner.step(state, command, length);
        std::cout << ref.pitch << ' ' << ref.wheel_velocity << ' ' << ref.balance_feedforward;
        for (double value : ref.hip)
            std::cout << ' ' << value;
        for (double value : ref.knee)
            std::cout << ' ' << value;
        for (double value : ref.shank)
            std::cout << ' ' << value;
        std::cout << '\n';
    }
    return std::cin.eof() ? 0 : 2;
}
