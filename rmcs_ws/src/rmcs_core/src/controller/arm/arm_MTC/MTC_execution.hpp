#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"

#include <rclcpp/duration.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

struct JointMotion {
    std::vector<double> times;
    std::vector<std::array<double, 6>> positions;

    double duration() const { return times.empty() ? 0.0 : times.back(); }

    // Linear interpolation between waypoints. The cursor advances monotonically for
    // nondecreasing elapsed times and rewinds if time goes backwards.
    std::array<double, 6> sample(double elapsed, std::size_t& cursor) const {
        if (elapsed <= times.front()) {
            cursor = 0;
            return positions.front();
        }
        if (elapsed >= times.back()) {
            cursor = times.size() - 1;
            return positions.back();
        }
        if (cursor >= times.size() || elapsed < times[cursor])
            cursor = 0;
        while (times[cursor + 1] <= elapsed)
            ++cursor;
        const double fraction = (elapsed - times[cursor]) / (times[cursor + 1] - times[cursor]);
        std::array<double, 6> result;
        for (std::size_t joint = 0; joint < result.size(); ++joint)
            result[joint] =
                std::lerp(positions[cursor][joint], positions[cursor + 1][joint], fraction);
        return result;
    }
};

// Reorders the trajectory's joints into joint_1..joint_6 order.
inline JointMotion prepare_motion(const MotionSegment& segment) {
    const auto& trajectory = segment.trajectory.joint_trajectory;
    std::array<std::size_t, 6> source{};
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index)
        source[trajectory.joint_names[index].back() - '1'] = index;

    JointMotion motion;
    motion.times.reserve(trajectory.points.size());
    motion.positions.reserve(trajectory.points.size());
    for (const auto& point : trajectory.points) {
        motion.times.push_back(rclcpp::Duration(point.time_from_start).seconds());
        auto& positions = motion.positions.emplace_back();
        for (std::size_t joint = 0; joint < positions.size(); ++joint)
            positions[joint] = point.positions[source[joint]];
    }
    return motion;
}

struct PreparedExecution {
    ExecutionPlan plan;
    std::vector<std::optional<JointMotion>> motions;
};

// Copying allocates; call this in the worker, not the control loop.
inline PreparedExecution prepare_execution(const ExecutionPlan& plan) {
    PreparedExecution prepared{plan, std::vector<std::optional<JointMotion>>(plan.steps.size())};
    for (std::size_t index = 0; index < plan.steps.size(); ++index)
        if (const auto* motion = std::get_if<MotionSegment>(&plan.steps[index].action))
            prepared.motions[index] = prepare_motion(*motion);
    return prepared;
}

} // namespace rmcs_core::controller::arm::MTC
