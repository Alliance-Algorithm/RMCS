#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"
#include "controller/arm/arm_MTC/MTC_scene.hpp"

#include <moveit/robot_trajectory/robot_trajectory.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace rmcs_core::controller::arm::MTC {

enum class PreviewStatus { Idle, Running, Succeeded, Failed, Cancelled };

class PreviewPlayer {
public:
    explicit PreviewPlayer(moveit::core::RobotModelConstPtr model, double gripper_duration = 0.25)
        : model_(std::move(model))
        , gripper_duration_(gripper_duration) {}

    void start(ExecutionPlan plan) {
        plan_ = std::move(plan);
        scene_ = make_start_scene();
        clear_cursor();
        status_ = plan_.steps.empty() ? PreviewStatus::Succeeded : PreviewStatus::Running;
        ++scene_revision_;
    }

    void advance(double seconds) {
        while (status_ == PreviewStatus::Running) {
            const auto& step = plan_.steps[step_index_];
            try {
                double duration = 0.0;
                if (const auto* motion = std::get_if<MotionSegment>(&step.action)) {
                    if (!trajectory_)
                        load_motion(*motion);
                    duration = trajectory_->getDuration();
                } else if (const auto* gripper = std::get_if<GripperCommandConfig>(&step.action)) {
                    duration = gripper_duration_;
                    gripper_action_ = gripper->action;
                } else {
                    detail::apply_scene_operations(
                        scene_, {std::get<SceneUpdate>(step.action).operations}, step.stage_path);
                    ++scene_revision_;
                }

                const double consumed = std::min(seconds, std::max(0.0, duration - elapsed_));
                elapsed_ += consumed;
                seconds -= consumed;
                if (trajectory_) {
                    auto state =
                        std::make_shared<moveit::core::RobotState>(scene_->getCurrentState());
                    trajectory_->getStateAtDurationFromStart(elapsed_, state);
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
        scene_ = make_start_scene();
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
    planning_scene::PlanningScenePtr make_start_scene() const {
        auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
        if (!scene->setPlanningSceneMsg(plan_.start_scene))
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
