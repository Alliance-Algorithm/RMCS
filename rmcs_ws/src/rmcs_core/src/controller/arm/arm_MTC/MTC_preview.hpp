#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"
#include "controller/arm/arm_MTC/MTC_scene.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace rmcs_core::controller::arm::MTC {

enum class PreviewStatus { Idle, Running, Succeeded, Failed, Cancelled };

class PreviewPlayer {
public:
    explicit PreviewPlayer(moveit::core::RobotModelConstPtr model, double gripper_duration = 0.25)
        : model_(std::move(model))
        , gripper_duration_(gripper_duration) {
        if (!model_ || !std::isfinite(gripper_duration_) || gripper_duration_ < 0.0)
            throw std::invalid_argument("Invalid preview robot model or gripper duration");
    }

    void start(ExecutionPlan plan) {
        auto scene = make_start_scene(plan);
        plan_ = std::move(plan);
        scene_ = std::move(scene);
        clear_cursor();
        status_ = plan_.steps.empty() ? PreviewStatus::Succeeded : PreviewStatus::Running;
        ++scene_revision_;
    }

    void advance(double seconds) {
        if (!std::isfinite(seconds) || seconds < 0.0)
            throw std::invalid_argument("Preview time increment must be finite and nonnegative");
        while (status_ == PreviewStatus::Running) {
            const auto& step = plan_.steps.at(step_index_);
            try {
                double duration = 0.0;
                if (const auto* motion = std::get_if<MotionSegment>(&step.action)) {
                    if (!trajectory_)
                        load_motion(*motion);
                    duration = trajectory_->getDuration();
                } else if (const auto* gripper = std::get_if<GripperCommandConfig>(&step.action)) {
                    if (gripper->action != GripperAction::Open
                        && gripper->action != GripperAction::Close)
                        throw std::runtime_error("unknown simulated gripper action");
                    if (!std::isfinite(gripper->timeout) || gripper->timeout <= 0.0
                        || gripper_duration_ > gripper->timeout)
                        throw std::runtime_error("simulated gripper duration exceeds timeout");
                    duration = gripper_duration_;
                    gripper_action_ = gripper->action;
                } else {
                    auto candidate = planning_scene::PlanningScene::clone(scene_);
                    detail::apply_scene_operations(
                        candidate,
                        ModifyPlanningSceneConfig{std::get<SceneUpdate>(step.action).operations},
                        step.stage_path);
                    scene_ = std::move(candidate);
                    ++scene_revision_;
                }

                const double consumed = std::min(seconds, std::max(0.0, duration - elapsed_));
                elapsed_ += consumed;
                seconds -= consumed;
                if (trajectory_) {
                    auto state =
                        std::make_shared<moveit::core::RobotState>(scene_->getCurrentState());
                    if (!trajectory_->getStateAtDurationFromStart(elapsed_, state) || !state)
                        throw std::runtime_error("cannot interpolate motion trajectory");
                    scene_->setCurrentState(*state);
                    scene_->getCurrentStateNonConst().update();
                }
                if (elapsed_ < duration)
                    return;
                ++step_index_;
                elapsed_ = 0.0;
                trajectory_.reset();
                if (step_index_ == plan_.steps.size())
                    status_ = PreviewStatus::Succeeded;
            } catch (const std::exception& error) {
                diagnostic_ = step.stage_path + ": " + error.what();
                status_ = PreviewStatus::Failed;
            }
        }
    }

    void cancel() {
        if (status_ == PreviewStatus::Running)
            status_ = PreviewStatus::Cancelled;
    }

    void reset() {
        if (!scene_)
            return;
        scene_ = make_start_scene(plan_);
        clear_cursor();
        status_ = PreviewStatus::Idle;
        ++scene_revision_;
    }

    const ExecutionPlan& plan() const { return plan_; }
    planning_scene::PlanningSceneConstPtr scene() const { return scene_; }
    std::size_t step_index() const { return step_index_; }
    PreviewStatus status() const { return status_; }
    const std::string& diagnostic() const { return diagnostic_; }
    std::size_t scene_revision() const { return scene_revision_; }
    std::optional<GripperAction> gripper_action() const { return gripper_action_; }

private:
    planning_scene::PlanningScenePtr make_start_scene(const ExecutionPlan& plan) const {
        if (plan.start_scene.is_diff || plan.start_scene.robot_model_name != model_->getName())
            throw std::invalid_argument("Preview requires a full starting scene for its robot");
        auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
        if (!scene->setPlanningSceneMsg(plan.start_scene))
            throw std::runtime_error("Cannot load preview starting scene");
        scene->getCurrentStateNonConst().update();
        return scene;
    }

    void clear_cursor() {
        step_index_ = 0;
        elapsed_ = 0.0;
        trajectory_.reset();
        diagnostic_.clear();
        gripper_action_.reset();
    }

    void load_motion(const MotionSegment& motion) {
        const auto* group = model_->getJointModelGroup(motion.group);
        if (!group)
            throw std::runtime_error("unknown motion group " + motion.group);
        if (!motion.trajectory.multi_dof_joint_trajectory.joint_names.empty()
            || !motion.trajectory.multi_dof_joint_trajectory.points.empty())
            throw std::runtime_error("multi-DOF preview is not supported");
        const auto& message = motion.trajectory.joint_trajectory;
        if (message.joint_names.empty() || message.points.empty())
            throw std::runtime_error("empty motion trajectory");
        std::unordered_set<std::string> names;
        for (const auto& name : message.joint_names) {
            const auto* joint = model_->getJointModel(name);
            if (!joint || joint->getVariableCount() != 1 || !group->hasJointModel(name)
                || !names.insert(name).second)
                throw std::runtime_error("invalid or duplicate motion joint " + name);
        }
        double previous_time = -1.0;
        for (const auto& point : message.points) {
            const auto valid_values = [&](const auto& values, bool required) {
                return ((!required && values.empty()) || values.size() == names.size())
                    && std::ranges::all_of(
                           values, [](double value) { return std::isfinite(value); });
            };
            if (!valid_values(point.positions, true) || !valid_values(point.velocities, false)
                || !valid_values(point.accelerations, false) || !valid_values(point.effort, false))
                throw std::runtime_error("invalid motion point dimensions or non-finite values");
            if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u)
                throw std::runtime_error("invalid motion timestamp");
            const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            if (time < previous_time)
                throw std::runtime_error("motion timestamps are not monotonic");
            previous_time = time;
        }
        if (message.points.front().time_from_start.sec != 0
            || message.points.front().time_from_start.nanosec != 0)
            throw std::runtime_error("motion must start at time zero");
        for (std::size_t index = 0; index < message.joint_names.size(); ++index)
            if (std::abs(
                    scene_->getCurrentState().getVariablePosition(message.joint_names[index])
                    - message.points.front().positions[index])
                > 1e-4)
                throw std::runtime_error("motion start does not match preview state");
        trajectory_ = std::make_unique<robot_trajectory::RobotTrajectory>(model_, motion.group);
        trajectory_->setRobotTrajectoryMsg(scene_->getCurrentState(), motion.trajectory);
    }

    moveit::core::RobotModelConstPtr model_;
    double gripper_duration_;
    ExecutionPlan plan_;
    planning_scene::PlanningScenePtr scene_;
    std::unique_ptr<robot_trajectory::RobotTrajectory> trajectory_;
    std::size_t step_index_ = 0;
    std::size_t scene_revision_ = 0;
    double elapsed_ = 0.0;
    std::optional<GripperAction> gripper_action_;
    PreviewStatus status_ = PreviewStatus::Idle;
    std::string diagnostic_;
};

} // namespace rmcs_core::controller::arm::MTC
