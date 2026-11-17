#pragma once

#include "controller/arm/arm_MTC/MTC_bridge.hpp"
#include "controller/arm/arm_MTC/MTC_dictionary.hpp"
#include "controller/arm/arm_MTC/MTC_scene.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
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
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
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
#include <utility>
#include <vector>

namespace rmcs_core::controller::arm::MTC {

namespace mtc = moveit::task_constructor;

namespace detail {

// MoveRelative that rotates about an axis through a fixed point instead of the tool origin.
class RotatingMoveRelative : public mtc::stages::MoveRelative {
public:
    RotatingMoveRelative(
        const std::string& name, const mtc::solvers::PlannerInterfacePtr& solver,
        RotationDirectionTarget direction, std::string link)
        : MoveRelative(name, solver)
        , direction_(std::move(direction))
        , link_(std::move(link)) {}

protected:
    bool compute(
        const mtc::InterfaceState& state, planning_scene::PlanningScenePtr& scene,
        mtc::SubTrajectory& solution, mtc::Interface::Direction dir) override {
        const auto& start = state.scene();
        const auto& reference = start->getFrameTransform(direction_.frame_id);
        const auto& tool = start->getCurrentState().getGlobalLinkTransform(link_);
        const auto& [px, py, pz] = direction_.axis_point;
        Eigen::Isometry3d pivot = tool;
        pivot.translation() = reference * Eigen::Vector3d(px, py, pz);
        setIKFrame(tool.inverse() * pivot, link_);

        // Resolve the axis once at stage entry, even when its frame is the tool link.
        const auto& [ax, ay, az] = direction_.axis_direction;
        const Eigen::Vector3d axis = reference.linear() * Eigen::Vector3d(ax, ay, az).normalized();
        geometry_msgs::msg::TwistStamped twist;
        twist.header.frame_id = start->getRobotModel()->getModelFrame();
        twist.twist.angular.x = axis.x();
        twist.twist.angular.y = axis.y();
        twist.twist.angular.z = axis.z();
        setDirection(twist);
        return MoveRelative::compute(state, scene, solution, dir);
    }

private:
    RotationDirectionTarget direction_;
    std::string link_;
};

} // namespace detail

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
        , model_(std::move(model)) {}

    void build(
        const Dictionary& dictionary, const std::vector<std::string>& composed,
        planning_scene::PlanningSceneConstPtr scene = {}) {
        build(dictionary.helper_build_chunk(composed), std::move(scene));
    }

    void build(TaskConfig config, planning_scene::PlanningSceneConstPtr scene = {}) {
        task_.reset();
        stage_metadata_.clear();
        bind_task_objects(config); // No-op for dictionary output; resolves raw configs.

        auto task = std::make_unique<mtc::Task>(config.name, false);
        if (model_)
            task->setRobotModel(model_);
        else
            task->loadRobotModel(node_);
        task->setProperty("group", config.group);
        task->setProperty("ik_frame", config.ik_frame);
        task->setTimeout(config.timeout);

        BuildContext context{config, scene, task->getRobotModel(), stage_metadata_};
        for (const auto& child : config.children)
            task->add(build_node(child, context, config.name));
        max_solutions_ = config.max_solutions;
        task_ = std::move(task);
    }

    PlanResult plan() {
        if (!task_)
            return {PlanStatus::NotBuilt, moveit_msgs::msg::MoveItErrorCodes::FAILURE, 0,
                    "No MTC task has been built"};
        task_->reset();
        try {
            task_->init();
        } catch (const mtc::InitStageException& error) {
            std::ostringstream diagnostic;
            diagnostic << error;
            return {PlanStatus::InitializationFailed, moveit_msgs::msg::MoveItErrorCodes::FAILURE,
                    0, diagnostic.str()};
        }
        const auto code = task_->plan(max_solutions_);
        const bool success = bool(code) && task_->numSolutions() > 0;
        std::ostringstream diagnostic;
        if (!success)
            task_->explainFailure(diagnostic);
        return {success ? PlanStatus::Success : PlanStatus::PlanningFailed, code.val,
                task_->numSolutions(), diagnostic.str()};
    }

    ExecutionPlan export_plan(std::size_t solution_index = 0) const {
        const auto solution =
            std::next(task_->solutions().begin(), static_cast<std::ptrdiff_t>(solution_index));
        return detail::PlanBridge::convert(task_->name(), **solution, stage_metadata_);
    }

    const mtc::Task& task() const { return *task_; }

    void publish_solutions() {
        task_->enableIntrospection();
        task_->publishAllSolutions(false);
    }

private:
    struct BuildContext {
        const TaskConfig& task;
        const planning_scene::PlanningSceneConstPtr& scene;
        const moveit::core::RobotModelConstPtr& model;
        detail::StageMetadataMap& metadata;
    };

    static const std::string& or_default(const std::string& value, const std::string& fallback) {
        return value.empty() ? fallback : value;
    }

    mtc::solvers::PlannerInterfacePtr make_solver(const MotionConfig& config) const {
        auto solver = std::visit(
            overloaded{
                [&](const PipelineConfig& options) -> mtc::solvers::PlannerInterfacePtr {
                    return std::make_shared<mtc::solvers::PipelinePlanner>(
                        node_, options.pipeline, options.planner);
                },
                [](const JointInterpolationConfig&) -> mtc::solvers::PlannerInterfacePtr {
                    return std::make_shared<mtc::solvers::JointInterpolationPlanner>();
                },
                [](const CartesianPathConfig& options) -> mtc::solvers::PlannerInterfacePtr {
                    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
                    cartesian->setStepSize(options.step_size);
                    cartesian->setMinFraction(1.0);
                    return cartesian;
                },
            },
            config.solver);
        solver->setMaxVelocityScalingFactor(config.velocity_scaling);
        solver->setMaxAccelerationScalingFactor(config.acceleration_scaling);
        solver->setTimeout(config.timeout);
        return solver;
    }

    mtc::Stage::pointer build_node(
        const NodeConfig& node, BuildContext& context, const std::string& parent_path) const {
        if (const auto* container = std::get_if<ContainerConfig>(&node.config))
            return make_container(*container, context, parent_path + "/" + container->name);

        const auto& config = std::get<StageConfig>(node.config);
        auto stage = std::visit(
            [&](const auto& options) { return make_stage(config.name, options, context); },
            config.config);
        std::string group = context.task.group;
        if (const auto* move = std::get_if<MoveToConfig>(&config.config))
            group = or_default(move->group, group);
        else if (const auto* move = std::get_if<MoveRelativeConfig>(&config.config))
            group = or_default(move->group, group);
        context.metadata.emplace(
            stage.get(),
            detail::StageMetadata{parent_path + "/" + config.name, config, std::move(group)});
        return stage;
    }

    mtc::Stage::pointer make_container(
        const ContainerConfig& config, BuildContext& context, const std::string& path) const {
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
        container->setProperty("group", context.task.group);
        container->setProperty("ik_frame", context.task.ik_frame);
        for (const auto& child : config.children)
            container->add(build_node(child, context, path));
        return container;
    }

    // Without an explicit scene, plan from the live robot state.
    mtc::Stage::pointer make_stage(
        const std::string& name, const CurrentStateConfig& config, BuildContext& context) const {
        if (context.scene)
            return std::make_unique<mtc::stages::FixedState>(
                name, planning_scene::PlanningScene::clone(context.scene));
        auto current = std::make_unique<mtc::stages::CurrentState>(name);
        current->setTimeout(std::min(config.timeout, context.task.timeout));
        return current;
    }

    mtc::Stage::pointer make_stage(
        const std::string& name, const FixedStateConfig& config, BuildContext& context) const {
        auto scene = context.scene ? planning_scene::PlanningScene::clone(context.scene)
                                   : std::make_shared<planning_scene::PlanningScene>(context.model);
        auto& state = scene->getCurrentStateNonConst();
        for (const auto& [variable, value] : config.joints)
            state.setVariablePosition(variable, value);
        state.update();
        for (const auto& configured : config.objects) {
            moveit_msgs::msg::CollisionObject object;
            object.header = configured.pose.header;
            object.id = configured.id;
            object.operation = moveit_msgs::msg::CollisionObject::ADD;
            shape_msgs::msg::SolidPrimitive box;
            box.type = shape_msgs::msg::SolidPrimitive::BOX;
            box.dimensions = {configured.dimensions.x, configured.dimensions.y, configured.dimensions.z};
            object.primitives.push_back(box);
            object.primitive_poses.push_back(configured.pose.pose);
            scene->processCollisionObjectMsg(object);
        }
        return std::make_unique<mtc::stages::FixedState>(name, std::move(scene));
    }

    mtc::Stage::pointer make_stage(
        const std::string& name, const MoveToConfig& config, BuildContext& context) const {
        auto move = std::make_unique<mtc::stages::MoveTo>(name, make_solver(config.motion));
        move->setGroup(or_default(config.group, context.task.group));
        if (config.use_ik_frame)
            move->setIKFrame(or_default(config.ik_frame, context.task.ik_frame));
        move->setTimeout(config.motion.timeout);
        std::visit([&](const auto& goal) { move->setGoal(goal); }, config.goal);
        move->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
        return move;
    }

    mtc::Stage::pointer make_stage(
        const std::string& name, const MoveRelativeConfig& config, BuildContext& context) const {
        const auto solver = make_solver(config.motion);
        const auto& ik_frame = or_default(config.ik_frame, context.task.ik_frame);
        std::unique_ptr<mtc::stages::MoveRelative> move;
        if (const auto* rotation = std::get_if<RotationDirectionTarget>(&config.direction)) {
            // A negative angle range rotates the other way: flip the axis, keep distances >= 0.
            auto target = *rotation;
            if (config.max_distance < 0.0)
                for (auto& component : target.axis_direction)
                    component = -component;
            move = std::make_unique<detail::RotatingMoveRelative>(
                name, solver, std::move(target), ik_frame);
            move->setMinMaxDistance(std::abs(config.min_distance), std::abs(config.max_distance));
        } else {
            move = std::make_unique<mtc::stages::MoveRelative>(name, solver);
            move->setDirection(std::get<geometry_msgs::msg::Vector3Stamped>(config.direction));
            move->setMinMaxDistance(config.min_distance, config.max_distance);
        }
        move->setGroup(or_default(config.group, context.task.group));
        move->setIKFrame(ik_frame);
        move->setTimeout(config.motion.timeout);
        move->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
        return move;
    }

    // Planning-time placeholder; the gripper command is carried into the execution plan.
    mtc::Stage::pointer
        make_stage(const std::string& name, const GripperCommandConfig&, BuildContext&) const {
        return make_scene_stage(name, [](const planning_scene::PlanningScenePtr&) {});
    }

    mtc::Stage::pointer
        make_stage(const std::string& name, const LogPoseConfig& config, BuildContext&) const {
        return make_scene_stage(
            name, [logger = node_->get_logger(),
                   link = config.link](const planning_scene::PlanningScenePtr& scene) {
                const auto& pose = scene->getCurrentState().getGlobalLinkTransform(link);
                const Eigen::Quaterniond q(pose.linear());
                double roll = 0.0, pitch = 0.0, yaw = 0.0;
                // Inverse of the setRPY used by makePoseGoal.
                tf2::Matrix3x3(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(roll, pitch, yaw);
                RCLCPP_INFO(
                    logger,
                    ".x = %.3f, .y = %.3f, .z = %.3f, .roll = %.3f, .pitch = %.3f, .yaw = %.3f",
                    pose.translation().x(), pose.translation().y(), pose.translation().z(), roll,
                    pitch, yaw);
            });
    }

    mtc::Stage::pointer make_stage(
        const std::string& name, const ModifyPlanningSceneConfig& config, BuildContext&) const {
        return make_scene_stage(
            name, [name, config](const planning_scene::PlanningScenePtr& scene) {
                detail::apply_scene_operations(scene, config, name);
            });
    }

    template <typename Callback>
    static mtc::Stage::pointer make_scene_stage(const std::string& name, Callback callback) {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(name);
        stage->setCallback(
            [callback = std::move(callback)](
                const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
                callback(scene);
            });
        stage->restrictDirection(mtc::PropagatingEitherWay::FORWARD);
        return stage;
    }

    rclcpp::Node::SharedPtr node_;
    moveit::core::RobotModelConstPtr model_;
    std::unique_ptr<mtc::Task> task_;
    detail::StageMetadataMap stage_metadata_;
    std::size_t max_solutions_ = 1;
};

// Plans on a background thread. Only the latest submitted request is planned and published.
class Machine {
public:
    struct PlannedTrajectory {
        std::uint64_t request_id = 0;
        std::string diagnostic;
        std::optional<ExecutionPlan> plan; // Empty when planning failed.
    };

    Machine()
        : node_(std::make_shared<rclcpp::Node>(
              "arm_mtc_planner",
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))) {
        model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_);
        model_ = model_loader_->getModel();
        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "alliance_arm");
        planner_ = std::make_unique<Planner>(node_, model_);
        scene_client_ =
            node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
        executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_);
        spin_thread_ = std::thread([this] {
            while (executor_running_.load(std::memory_order_acquire)
                   && node_->get_node_base_interface()->get_context()->is_valid())
                executor_->spin_once(std::chrono::milliseconds(20));
        });
        planning_thread_ = std::thread([this] { planning_loop(); });
    }

    // Offline machine: no scene service, so every request must carry its own scene.
    Machine(rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr model)
        : node_(std::move(node))
        , model_(std::move(model))
        , planner_(std::make_unique<Planner>(node_, model_)) {
        planning_thread_ = std::thread([this] { planning_loop(); });
    }

    ~Machine() { stop(); }

    Machine(const Machine&) = delete;
    Machine& operator=(const Machine&) = delete;

    std::uint64_t
        process(const TaskConfig& config, planning_scene::PlanningSceneConstPtr scene = {}) {
        auto request = std::make_unique<PlanRequest>(PlanRequest{
            0, config, scene ? planning_scene::PlanningScene::clone(scene) : nullptr});
        std::uint64_t request_id;
        {
            std::lock_guard lock(mutex_);
            request_id = request->request_id = ++latest_request_id_;
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
        if (!scene_client_->wait_for_service(std::chrono::seconds(5)))
            throw std::runtime_error("Planning scene service is unavailable");
        auto query = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
        using Components = moveit_msgs::msg::PlanningSceneComponents;
        query->components.components =
            Components::SCENE_SETTINGS | Components::ROBOT_STATE
            | Components::ROBOT_STATE_ATTACHED_OBJECTS | Components::WORLD_OBJECT_GEOMETRY
            | Components::OCTOMAP | Components::TRANSFORMS | Components::ALLOWED_COLLISION_MATRIX
            | Components::LINK_PADDING_AND_SCALING | Components::OBJECT_COLORS;
        auto response = scene_client_->async_send_request(query);
        if (response.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            scene_client_->remove_pending_request(response);
            throw std::runtime_error("Timed out fetching the planning scene");
        }
        auto scene = std::make_shared<planning_scene::PlanningScene>(model_);
        if (!scene->setPlanningSceneMsg(response.get()->scene))
            throw std::runtime_error("Cannot load the planning scene");
        scene->getCurrentStateNonConst().update();
        return scene;
    }

    void stop() {
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
                if (planned)
                    result->plan = planner_->export_plan();
            } catch (const std::exception& error) {
                result->diagnostic = error.what();
            }
            std::lock_guard lock(mutex_);
            if (running_ && request->request_id == latest_request_id_)
                planned_trajectory_.store(std::move(result), std::memory_order_release);
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
    std::mutex mutex_;
    std::condition_variable cv_;
    bool running_ = true;
    std::uint64_t latest_request_id_ = 0;
    std::unique_ptr<PlanRequest> pending_request_;
    std::atomic<std::shared_ptr<const PlannedTrajectory>> planned_trajectory_{nullptr};
};

} // namespace rmcs_core::controller::arm::MTC
