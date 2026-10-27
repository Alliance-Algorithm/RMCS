#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"
#include "controller/arm/arm_MTC/MTC_dictionary.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

struct JointMotion {
    std::vector<double> times;
    std::vector<std::array<double, 6>> positions;

    double duration() const { return times.empty() ? 0.0 : times.back(); }

    // Prepared motions use linear interpolation, not the supplied velocities or accelerations.
    // The cursor advances monotonically for nondecreasing elapsed times; rewind resets it.
    std::array<double, 6> sample(double elapsed, std::size_t& cursor) const {
        if (times.empty() || times.size() != positions.size())
            throw std::logic_error("cannot sample an empty or inconsistent joint motion");
        if (std::isnan(elapsed))
            throw std::invalid_argument("motion elapsed time is NaN");
        if (elapsed <= times.front()) {
            cursor = 0;
            return positions.front();
        }
        if (elapsed >= times.back()) {
            cursor = times.size() - 1;
            return positions.back();
        }
        cursor = std::min(cursor, times.size() - 1);
        if (elapsed < times[cursor])
            cursor = 0;
        while (cursor + 1 < times.size() && times[cursor + 1] <= elapsed)
            ++cursor;
        const double fraction = (elapsed - times[cursor]) / (times[cursor + 1] - times[cursor]);
        std::array<double, 6> result;
        for (std::size_t joint = 0; joint < result.size(); ++joint)
            result[joint] = std::lerp(positions[cursor][joint], positions[cursor + 1][joint], fraction);
        return result;
    }
};

inline JointMotion prepare_motion(const MotionSegment& segment) {
    const auto require = [](bool condition, const char* reason) {
        if (!condition)
            throw std::invalid_argument(reason);
    };
    require(segment.group == "alliance_arm", "motion group must be alliance_arm");
    const auto& multi_dof = segment.trajectory.multi_dof_joint_trajectory;
    require(
        multi_dof.joint_names.empty() && multi_dof.points.empty(),
        "multi-DOF motion is not supported");
    const auto& trajectory = segment.trajectory.joint_trajectory;
    require(trajectory.joint_names.size() == 6, "motion must name exactly six joints");
    std::array<std::size_t, 6> mapping{};
    std::array<bool, 6> seen{};
    for (std::size_t source = 0; source < 6; ++source) {
        const auto& name = trajectory.joint_names[source];
        require(
            name.size() == 7 && name.compare(0, 6, "joint_") == 0
                && name[6] >= '1' && name[6] <= '6',
            "motion joint names must be joint_1 through joint_6");
        const auto target = static_cast<std::size_t>(name[6] - '1');
        require(!seen[target], "duplicate motion joint name");
        seen[target] = true;
        mapping[target] = source;
    }
    require(!trajectory.points.empty(), "motion has no points");
    JointMotion motion;
    motion.times.reserve(trajectory.points.size());
    motion.positions.reserve(trajectory.points.size());
    const auto finite_six = [](const std::vector<double>& values) {
        return values.size() == 6
            && std::all_of(values.begin(), values.end(), [](double value) {
                   return std::isfinite(value);
               });
    };
    for (const auto& point : trajectory.points) {
        require(finite_six(point.positions), "positions must contain six finite values");
        require(
            point.velocities.empty() || finite_six(point.velocities),
            "velocities must be empty or contain six finite values");
        require(
            point.accelerations.empty() || finite_six(point.accelerations),
            "accelerations must be empty or contain six finite values");
        const auto& stamp = point.time_from_start;
        require(stamp.sec >= 0 && stamp.nanosec < 1000000000u, "invalid motion timestamp");
        const double time = static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        require(std::isfinite(time), "motion time must be finite");
        require(
            motion.times.empty() ? time == 0.0 : time > motion.times.back(),
            "motion times must start at zero and then strictly increase");
        motion.times.push_back(time);
        std::array<double, 6> positions;
        for (std::size_t joint = 0; joint < positions.size(); ++joint)
            positions[joint] = point.positions[mapping[joint]];
        motion.positions.push_back(positions);
    }
    return motion;
}

struct PreparedExecution {
    ExecutionPlan plan;
    std::vector<std::optional<JointMotion>> motions;
};

// Copying and validation allocate; call this in the worker, not the control loop.
inline PreparedExecution prepare_execution(const ExecutionPlan& plan) {
    PreparedExecution prepared{plan, {}};
    prepared.motions.resize(prepared.plan.steps.size());
    for (std::size_t index = 0; index < prepared.plan.steps.size(); ++index) {
        const auto& step = prepared.plan.steps[index];
        if (const auto* motion = std::get_if<MotionSegment>(&step.action)) {
            prepared.motions[index] = prepare_motion(*motion);
        } else if (const auto* command = std::get_if<GripperCommandConfig>(&step.action)) {
            if (command->action != GripperAction::Open && command->action != GripperAction::Close)
                throw std::invalid_argument(step.stage_path + ": unknown gripper command");
            // Legacy gripper execution holds the command for at least 0.5 seconds.
            if (!std::isfinite(command->timeout) || command->timeout < 0.5)
                throw std::invalid_argument(step.stage_path + ": gripper timeout must be finite and >= 0.5");
        } else {
            const auto& scene = std::get<SceneUpdate>(step.action);
            validate(TaskConfig{
                .name = "execution",
                .children = {Step::makeModifyPlanningScene(
                    step.stage_path.empty() ? "scene" : step.stage_path,
                    ModifyPlanningSceneConfig{scene.operations})}});
        }
    }
    return prepared;
}

} // namespace rmcs_core::controller::arm::MTC
