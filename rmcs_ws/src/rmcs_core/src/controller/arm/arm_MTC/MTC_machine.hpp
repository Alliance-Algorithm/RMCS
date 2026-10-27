#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"
#include "controller/arm/arm_MTC/MTC_dictionary.hpp"
#include "controller/arm/arm_MTC/MTC_scene.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

namespace mtc = moveit::task_constructor;

enum class PlanStatus { Success, NotBuilt, InitializationFailed, PlanningFailed };

struct PlanResult {
    PlanStatus status = PlanStatus::NotBuilt;
    std::int32_t error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    std::size_t solution_count = 0;
    std::string diagnostic;

    explicit operator bool() const { return status == PlanStatus::Success; }
};

class Planner {
public:
    explicit Planner(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model = {})
        : node_(std::move(node))
        , model_(std::move(model)) {
        if (!node_)
            throw std::invalid_argument("MTC planner requires a ROS node");
    }

    void build(
        const Dictionary& dictionary, const std::vector<std::string>& composed,
        planning_scene::PlanningSceneConstPtr scene = {}) {
        reset_task();
        build(dictionary.helper_build_chunk(composed), std::move(scene));
    }

    void build(const TaskConfig& config, planning_scene::PlanningSceneConstPtr scene = {}) {
        reset_task();
        validate(config);
        auto task = std::make_unique<mtc::Task>(config.name, false);
        task->setName(config.name);
        if (model_)
            task->setRobotModel(model_);
        else
            task->loadRobotModel(node_);
        const auto& model = task->getRobotModel();
        if (!model || !model->hasJointModelGroup(config.group))
            throw std::invalid_argument(config.name + ": unknown robot group " + config.group);
        if (!model->hasLinkModel(config.ik_frame))
            throw std::invalid_argument(config.name + ": unknown IK link " + config.ik_frame);
        if (scene && scene->getRobotModel() != model)
            throw std::invalid_argument(
                config.name + ": scene and task must share the robot model");
        task->setProperty("group", config.group);
        task->setProperty("ik_frame", config.ik_frame);
        task->setTimeout(config.timeout);
        detail::StageMetadataMap metadata;
        for (const auto& child : config.children)
            task->add(build_node(child, config, scene, model, metadata, config.name));
        max_solutions_ = config.max_solutions;
        stage_metadata_ = std::move(metadata);
        task_ = std::move(task);
    }

    PlanResult plan() {
        planned_successfully_ = false;
        if (!task_)
            return {
                PlanStatus::NotBuilt, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 0,
                "No MTC task has been built"};
        task_->reset();
        try {
            task_->init();
        } catch (const mtc::InitStageException& error) {
            std::ostringstream diagnostic;
            diagnostic << error;
            return {
                PlanStatus::InitializationFailed, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 0,
                diagnostic.str()};
        } catch (const std::exception& error) {
            return {
                PlanStatus::InitializationFailed, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 0,
                error.what()};
        }
        try {
            const auto code = task_->plan(max_solutions_);
            std::ostringstream diagnostic;
            const bool success = bool(code) && task_->numSolutions() > 0;
            if (!success)
                task_->explainFailure(diagnostic);
            planned_successfully_ = success;
            return {
                success ? PlanStatus::Success : PlanStatus::PlanningFailed, code.val,
                task_->numSolutions(), diagnostic.str()};
        } catch (const std::exception& error) {
            return {
                PlanStatus::PlanningFailed, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                task_->numSolutions(), error.what()};
        }
    }

    ExecutionPlan export_plan(std::size_t solution_index = 0) const {
        if (!task_ || !planned_successfully_)
            throw std::logic_error("No successful MTC plan is available for export");
        if (solution_index >= task_->solutions().size())
            throw std::out_of_range("MTC solution index is out of range");
        const auto solution =
            std::next(task_->solutions().begin(), static_cast<std::ptrdiff_t>(solution_index));
        return detail::PlanBridge::convert(task_->name(), **solution, stage_metadata_);
    }

    const mtc::Task& task() const {
        if (!task_)
            throw std::logic_error("No MTC task has been built");
        return *task_;
    }

    void publish_solutions() {
        if (!task_)
            throw std::logic_error("No MTC task has been built");
        task_->enableIntrospection();
        task_->publishAllSolutions(false);
    }

private:
    void reset_task() {
        planned_successfully_ = false;
        stage_metadata_.clear();
        task_.reset();
    }

    mtc::solvers::PlannerInterfacePtr make_solver(const MotionConfig& config) const {
        auto solver = std::visit(
            [&](const auto& options) -> mtc::solvers::PlannerInterfacePtr {
                using Options = std::decay_t<decltype(options)>;
                if constexpr (std::is_same_v<Options, PipelineConfig>) {
                    return std::make_shared<mtc::solvers::PipelinePlanner>(
                        node_, options.pipeline, options.planner);
                } else if constexpr (std::is_same_v<Options, JointInterpolationConfig>) {
                    return std::make_shared<mtc::solvers::JointInterpolationPlanner>();
                } else {
                    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
                    cartesian->setStepSize(options.step_size);
                    cartesian->setMinFraction(1.0);
                    return cartesian;
                }
            },
            config.solver);
        solver->setMaxVelocityScalingFactor(config.velocity_scaling);
        solver->setMaxAccelerationScalingFactor(config.acceleration_scaling);
        solver->setTimeout(config.timeout);
        return solver;
    }

    mtc::Stage::pointer build_node(
        const NodeConfig& node, const TaskConfig& task,
        const planning_scene::PlanningSceneConstPtr& scene,
        const moveit::core::RobotModelConstPtr& model, detail::StageMetadataMap& metadata,
        const std::string& parent_path) const {
        return std::visit(
            [&](const auto& config) -> mtc::Stage::pointer {
                using Config = std::decay_t<decltype(config)>;
                const auto path = parent_path + "/" + config.name;
                if constexpr (std::is_same_v<Config, StageConfig>) {
                    auto stage = std::visit(
                        [&](const auto& stage_config) -> mtc::Stage::pointer {
                            using Stage = std::decay_t<decltype(stage_config)>;
                            if constexpr (std::is_same_v<Stage, CurrentStateConfig>) {
                                if (scene)
                                    return std::make_unique<mtc::stages::FixedState>(
                                        config.name, planning_scene::PlanningScene::clone(scene));
                                auto current =
                                    std::make_unique<mtc::stages::CurrentState>(config.name);
                                current->setTimeout(std::min(stage_config.timeout, task.timeout));
                                return current;
                            } else if constexpr (std::is_same_v<Stage, FixedStateConfig>) {
                                auto fixed_scene =
                                    scene ? planning_scene::PlanningScene::clone(scene)
                                          : std::make_shared<planning_scene::PlanningScene>(model);
                                auto& state = fixed_scene->getCurrentStateNonConst();
                                for (const auto& [variable, value] : stage_config.joints) {
                                    if (std::find(
                                            model->getVariableNames().begin(),
                                            model->getVariableNames().end(), variable)
                                        == model->getVariableNames().end())
                                        throw std::invalid_argument(
                                            config.name + ": unknown start-state variable "
                                            + variable);
                                    state.setVariablePosition(variable, value);
                                }
                                state.update();
                                if (!state.satisfiesBounds())
                                    throw std::invalid_argument(
                                        config.name + ": fixed start state violates joint bounds");
                                for (const auto& configured_object : stage_config.objects) {
                                    if (!fixed_scene->knowsFrameTransform(
                                            configured_object.pose.header.frame_id))
                                        throw std::invalid_argument(
                                            config.name + ": unknown object frame "
                                            + configured_object.pose.header.frame_id);
                                    moveit_msgs::msg::CollisionObject object;
                                    object.header = configured_object.pose.header;
                                    object.id = configured_object.id;
                                    object.operation = moveit_msgs::msg::CollisionObject::ADD;
                                    shape_msgs::msg::SolidPrimitive box;
                                    box.type = shape_msgs::msg::SolidPrimitive::BOX;
                                    box.dimensions = {
                                        configured_object.dimensions.x,
                                        configured_object.dimensions.y,
                                        configured_object.dimensions.z};
                                    object.primitives.push_back(box);
                                    object.primitive_poses.push_back(configured_object.pose.pose);
                                    if (!fixed_scene->processCollisionObjectMsg(object))
                                        throw std::runtime_error(
                                            config.name + ": failed to add object "
                                            + configured_object.id);
                                }
                                return std::make_unique<mtc::stages::FixedState>(
                                    config.name, std::move(fixed_scene));
                            } else {
                                if constexpr (std::is_same_v<Stage, MoveToConfig>) {
                                    const auto solver = make_solver(stage_config.motion);
                                    const auto& group = stage_config.group.empty()
                                                          ? task.group
                                                          : stage_config.group;
                                    if (!model->hasJointModelGroup(group))
                                        throw std::invalid_argument(
                                            config.name + ": unknown robot group " + group);
                                    if (stage_config.use_ik_frame
                                        && !model->hasLinkModel(
                                            stage_config.ik_frame.empty() ? task.ik_frame
                                                                          : stage_config.ik_frame))
                                        throw std::invalid_argument(
                                            config.name + ": unknown IK frame");
                                    auto move =
                                        std::make_unique<mtc::stages::MoveTo>(config.name, solver);
                                    move->setGroup(group);
                                    if (stage_config.use_ik_frame)
                                        move->setIKFrame(
                                            stage_config.ik_frame.empty() ? task.ik_frame
                                                                          : stage_config.ik_frame);
                                    move->setTimeout(stage_config.motion.timeout);
                                    std::visit(
                                        [&](const auto& goal) { move->setGoal(goal); },
                                        stage_config.goal);
                                    move->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
                                    return move;
                                } else if constexpr (std::is_same_v<Stage, MoveRelativeConfig>) {
                                    const auto solver = make_solver(stage_config.motion);
                                    const auto& group = stage_config.group.empty()
                                                          ? task.group
                                                          : stage_config.group;
                                    if (!model->hasJointModelGroup(group))
                                        throw std::invalid_argument(
                                            config.name + ": unknown robot group " + group);
                                    const auto& ik_frame = stage_config.ik_frame.empty()
                                                             ? task.ik_frame
                                                             : stage_config.ik_frame;
                                    if (!model->hasLinkModel(ik_frame))
                                        throw std::invalid_argument(
                                            config.name + ": unknown IK frame " + ik_frame);
                                    auto move = std::make_unique<mtc::stages::MoveRelative>(
                                        config.name, solver);
                                    move->setGroup(group);
                                    move->setIKFrame(ik_frame);
                                    move->setTimeout(stage_config.motion.timeout);
                                    move->setDirection(stage_config.direction);
                                    move->setMinMaxDistance(
                                        stage_config.min_distance, stage_config.max_distance);
                                    move->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
                                    return move;
                                } else if constexpr (std::is_same_v<Stage, GripperCommandConfig>) {
                                    auto command =
                                        std::make_unique<mtc::stages::ModifyPlanningScene>(
                                            config.name);
                                    command->setCallback([](const planning_scene::PlanningScenePtr&,
                                                            const mtc::PropertyMap&) {});
                                    command->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
                                    return command;
                                } else {
                                    detail::validate_scene_operation_links(
                                        stage_config, *model, config.name);
                                    auto modify =
                                        std::make_unique<mtc::stages::ModifyPlanningScene>(
                                            config.name);
                                    modify->setCallback(
                                        [name = config.name, operations = stage_config](
                                            const planning_scene::PlanningScenePtr& updated,
                                            const mtc::PropertyMap&) {
                                            detail::apply_scene_operations(
                                                updated, operations, name);
                                        });
                                    modify->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
                                    return modify;
                                }
                            }
                        },
                        config.config);
                    auto group = task.group;
                    std::visit(
                        [&](const auto& stage_config) {
                            using Stage = std::decay_t<decltype(stage_config)>;
                            if constexpr (
                                std::is_same_v<Stage, MoveToConfig>
                                || std::is_same_v<Stage, MoveRelativeConfig>) {
                                if (!stage_config.group.empty())
                                    group = stage_config.group;
                            }
                        },
                        config.config);
                    metadata.emplace(
                        stage.get(), detail::StageMetadata{path, config, std::move(group)});
                    return stage;
                } else {
                    mtc::ContainerBase::pointer container;
                    switch (config.type) {
                    case ContainerType::Serial:
                        container = std::make_unique<mtc::SerialContainer>(config.name);
                        break;
                    case ContainerType::Alternatives:
                        container = std::make_unique<mtc::Alternatives>(config.name);
                        break;
                    case ContainerType::Fallbacks:
                        container = std::make_unique<mtc::Fallbacks>(config.name);
                        break;
                    }
                    container->setProperty("group", task.group);
                    container->setProperty("ik_frame", task.ik_frame);
                    for (const auto& child : config.children)
                        container->add(build_node(child, task, scene, model, metadata, path));
                    return container;
                }
            },
            node.config);
    }

    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelConstPtr model_;
    std::unique_ptr<mtc::Task> task_;
    detail::StageMetadataMap stage_metadata_;
    bool planned_successfully_ = false;
    std::size_t max_solutions_ = 1;
};

class Machine {
public:
    struct PlannedTrajectory {
        std::uint64_t request_id = 0;
        bool plan_success = false;
        std::string diagnostic;
        std::optional<ExecutionPlan> plan;
    };

    Machine()
        : node_(std::make_shared<rclcpp::Node>(
              "arm_mtc_planner",
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))) {
        model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_);
        model_ = model_loader_->getModel();
        if (!model_ || !model_->hasJointModelGroup("alliance_arm"))
            throw std::runtime_error("MTC machine requires a configured alliance_arm robot model");
        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "alliance_arm");
        planner_ = std::make_unique<Planner>(node_, model_);
        scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
            "get_planning_scene");
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        try {
            spin_thread_ = std::thread([this] {
                while (executor_running_.load(std::memory_order_acquire)
                       && node_->get_node_base_interface()->get_context()->is_valid())
                    executor_->spin_once(std::chrono::milliseconds(20));
            });
            planning_thread_ = std::thread([this] { planning_loop(); });
        } catch (...) {
            stop();
            throw;
        }
    }

    Machine(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model)
        : node_(std::move(node))
        , model_(std::move(model)) {
        if (!node_ || !model_)
            throw std::invalid_argument("MTC machine requires a ROS node and robot model");
        planner_ = std::make_unique<Planner>(node_, model_);
        planning_thread_ = std::thread([this] { planning_loop(); });
    }

    ~Machine() { stop(); }

    Machine(const Machine&) = delete;
    Machine& operator=(const Machine&) = delete;
    Machine(Machine&&) = delete;
    Machine& operator=(Machine&&) = delete;

    std::uint64_t
        process(const TaskConfig& config, planning_scene::PlanningSceneConstPtr scene = {}) {
        auto request = std::make_unique<PlanRequest>();
        request->config = config;
        if (scene)
            request->scene = planning_scene::PlanningScene::clone(scene);
        std::uint64_t request_id;
        {
            std::lock_guard lock(mutex_);
            if (!running_)
                throw std::logic_error("Cannot submit a request to a stopped MTC machine");
            request_id = ++latest_request_id_;
            request->request_id = request_id;
            pending_request_ = std::move(request);
        }
        cv_.notify_one();
        return request_id;
    }

    std::shared_ptr<const PlannedTrajectory> get_trajectory() const {
        return planned_trajectory_.load(std::memory_order_acquire);
    }

    const rclcpp::Node::SharedPtr& node() const { return node_; }
    const moveit::core::RobotModelConstPtr& robot_model() const { return model_; }
    const auto& moveit_group_getter() const { return move_group_; }

    // Call outside node callbacks: the machine's executor services this request.
    planning_scene::PlanningScenePtr fetch_scene() const {
        if (!scene_client_)
            throw std::logic_error("This MTC machine has no planning scene service client");
        if (!scene_client_->wait_for_service(std::chrono::seconds(5)))
            throw std::runtime_error("Planning scene service is unavailable");
        auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        using Components = moveit_msgs::msg::PlanningSceneComponents;
        query->components.components =
            Components::SCENE_SETTINGS | Components::ROBOT_STATE
            | Components::ROBOT_STATE_ATTACHED_OBJECTS
            | Components::WORLD_OBJECT_GEOMETRY | Components::OCTOMAP
            | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX
            | Components::LINK_PADDING_AND_SCALING | Components::OBJECT_COLORS;
        auto response = scene_client_->async_send_request(query);
        if (response.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            scene_client_->remove_pending_request(response);
            throw std::runtime_error("Timed out fetching the planning scene");
        }
        auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
        const auto message = response.get();
        if (message->scene.is_diff || !scene->setPlanningSceneMsg(message->scene))
            throw std::runtime_error("Cannot load the complete planning scene");
        scene->getCurrentStateNonConst().update();
        return scene;
    }

    void stop() {
        std::lock_guard stop_lock(stop_mutex_);
        {
            std::lock_guard lock(mutex_);
            running_ = false;
            pending_request_.reset();
        }
        cv_.notify_all();
        if (planning_thread_.joinable())
            planning_thread_.join();
        executor_running_.store(false, std::memory_order_release);
        if (executor_)
            executor_->cancel();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

private:
    struct PlanRequest {
        std::uint64_t request_id = 0;
        TaskConfig config;
        planning_scene::PlanningSceneConstPtr scene;
    };

    void planning_loop() {
        while (true) {
            std::unique_ptr<PlanRequest> request;
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { return !running_ || pending_request_; });
                if (!running_)
                    return;
                request = std::move(pending_request_);
            }
            auto result = std::make_shared<PlannedTrajectory>();
            result->request_id = request->request_id;
            try {
                // Explicit snapshots are for simulation; otherwise use the latest world.
                if (!request->scene && scene_client_)
                    request->scene = fetch_scene();
                planner_->build(request->config, request->scene);
                const auto planned = planner_->plan();
                result->diagnostic = planned.diagnostic;
                if (planned) {
                    result->plan = planner_->export_plan();
                    result->plan_success = true;
                }
            } catch (const std::exception& error) {
                result->diagnostic = error.what();
            } catch (...) {
                result->diagnostic = "Unknown MTC planning failure";
            }
            {
                std::lock_guard lock(mutex_);
                if (running_ && request->request_id == latest_request_id_)
                    planned_trajectory_.store(std::move(result), std::memory_order_release);
            }
        }
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
    std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
    moveit::core::RobotModelConstPtr model_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<Planner> planner_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::atomic_bool executor_running_{true};
    std::thread spin_thread_;
    std::thread planning_thread_;
    std::mutex stop_mutex_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool running_ = true;
    std::uint64_t latest_request_id_ = 0;
    std::unique_ptr<PlanRequest> pending_request_;
    std::atomic<std::shared_ptr<const PlannedTrajectory>> planned_trajectory_{nullptr};
};

} // namespace rmcs_core::controller::arm::MTC
