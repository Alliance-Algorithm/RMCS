#include "controller/chassis/wheel_leg_joint_pair_geometry.hpp"

// C ABI for offline PhysX validation of the production C++ algorithm.
extern "C" int wheel_leg_pair_step(
    const double* phases, const double* targets, double* commands, const double* parameters,
    int count, double dt) {
    using namespace rmcs_core::controller::chassis;
    using Geometry = WheelLegJointPairGeometry;
    const WheelLegPairVelocityConfig config{
        parameters[0], parameters[1], parameters[2], parameters[3], parameters[4]};
    int invalid = 0;
    for (int i = 0; i < count; ++i) {
        const auto side = i % 2 == 0 ? Geometry::Side::kLeft : Geometry::Side::kRight;
        const auto measured = Geometry::feedback(
            side, phases[2 * i], phases[2 * i + 1], config.min_difference, config.max_difference,
            parameters[6]);
        if (!measured) {
            commands[2 * i] = commands[2 * i + 1] = 0.0;
            ++invalid;
            continue;
        }
        const auto target = Geometry::target(
            side, targets[2 * i], targets[2 * i + 1], config.min_difference, config.max_difference,
            parameters[5]);
        const auto result = wheel_leg_pair_velocity(
            side, *measured, target, {commands[2 * i], commands[2 * i + 1]}, config, dt);
        commands[2 * i] = result.hip;
        commands[2 * i + 1] = result.knee;
    }
    return invalid;
}
