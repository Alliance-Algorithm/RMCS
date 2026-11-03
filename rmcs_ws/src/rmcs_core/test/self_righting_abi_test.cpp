#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

#include "controller/chassis/wheel_leg/self_righting.hpp"

extern "C" {
int wheelleg_recovery_abi_version();
void* wheelleg_recovery_create(const double*, int, const double*, int);
void wheelleg_recovery_destroy(void*);
int wheelleg_recovery_configure_pose_routes(void*, int, const double*, int);
int wheelleg_recovery_route(void*);
int wheelleg_recovery_route_waypoint(void*);
int wheelleg_recovery_step(void*, const double*, double, int, int, double*);
}

void require(bool value, const char* message) {
    if (!value)
        throw std::runtime_error(message);
}

int main() {
    namespace geometry = rmcs_core::controller::chassis::wheel_leg::recovery_v202;
    using Core = rmcs_core::controller::chassis::wheel_leg::SelfRightingCore;
    const std::array settings{
        1.5,
        3.,
        5.,
        12.,
        75.,
        105.,
        geometry::stand_knee,
        geometry::stand_orbit,
        0.,
        180.,
        35.,
        5.,
        2.,
        0.,
        30.,
        40.,
        3.14,
        0.,
        .6,
        50.,
        30.,
        1.};
    require(wheelleg_recovery_abi_version() == 6, "Existing ABI version changed");
    auto* handle = wheelleg_recovery_create(
        geometry::geometry.front().data(), geometry::geometry.size(), settings.data(),
        settings.size());
    require(handle, "Existing create contract rejected");
    try {
        const std::array routes{45., 105., 95., 80., .08, 2., .6};
        require(
            wheelleg_recovery_configure_pose_routes(nullptr, 1, routes.data(), 7) == -1,
            "Null route handle accepted");
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, nullptr, 7) == -1,
            "Null route parameters accepted");
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, routes.data(), 6) == -1,
            "Wrong route parameter count accepted");
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, routes.data(), 7) == 0,
            "Route extension rejected");
        const std::array landing{45., 105., 95., 80., .08, 2., .6, 48., 20.};
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, landing.data(), 9) == 0,
            "Landing extension rejected");
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, landing.data(), 8) == -1,
            "Partial landing parameters accepted");
        std::array<double, 20> input{};
        std::copy_n(geometry::geometry[17].begin() + 1, 4, input.begin());
        for (std::size_t axis = 0; axis < 4; ++axis)
            input[axis] -= std::numbers::pi / 2.; // left-front/right-rear in CAD axes
        input[14] = -1.;
        std::array<double, 26> output{};
        output.front() = 123456.;
        output.back() = -123456.;
        require(
            wheelleg_recovery_step(handle, input.data(), .001, 1, 1, output.data() + 1) == 0,
            "Existing step contract failed");
        require(output.front() == 123456. && output.back() == -123456., "ABI output layout grew");
        require(
            wheelleg_recovery_route(handle) == static_cast<int>(Core::Route::ProneMixed),
            "ABI route selection differs from production core");
        require(wheelleg_recovery_route_waypoint(handle) == 0, "Initial ABI waypoint invalid");
        require(
            wheelleg_recovery_configure_pose_routes(handle, 1, routes.data(), 7) == -1,
            "ABI reconfigured a running route");
        wheelleg_recovery_step(handle, input.data(), .001, 1, 0, output.data() + 1);
        require(output[16] == 2., "ABI stale feedback failed to latch fault");
        for (std::size_t i = 9; i < 15; ++i)
            require(output[i] == 0., "ABI fault retained actuator effort");
        wheelleg_recovery_step(handle, input.data(), .001, 0, 1, output.data() + 1);
        require(wheelleg_recovery_route(handle) == 0, "ABI reset retained route state");
    } catch (...) {
        wheelleg_recovery_destroy(handle);
        throw;
    }
    wheelleg_recovery_destroy(handle);
    std::cout
        << "PASS: additive pose-route ABI, existing layouts, selection, reset and rejection\n";
}
