#include "controller/arm/Kinematic.hpp"
#include "controller/arm/trajectory.hpp"
#include "hardware/endian_promise.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <limits>
#include <memory>
#include <string>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/crc/dji_crc.hpp>
#include <rmcs_utility/package_receive.hpp>
#include <rmcs_utility/tick_timer.hpp>

namespace rmcs_core::controller::arm {

class ArmController final : public rmcs_executor::Component {
public:
    ArmController()
        : node_(
              std::make_shared<rclcpp::Node>(
                  get_component_name(),
                  rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))) {

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_output("/arm/enable_flag", is_arm_enable, false);

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", theta[i]);
            register_output(joint_prefix + "/target_theta", target_theta[i], NAN);
        }

        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);
        move_group_->startStateMonitor();

        exec_.add_node(node_);
        spin_running_.store(true, std::memory_order_release);
        spin_thread_ = std::thread([this] { spin_loop(); });

        moveit_running_.store(true, std::memory_order_release);
        moveit_thread_ = std::thread([this] { moveit_loop(); });
    }

    ~ArmController() override {
        moveit_running_.store(false, std::memory_order_release);
        spin_running_.store(false, std::memory_order_release);

        exec_.cancel();

        if (moveit_thread_.joinable())
            moveit_thread_.join();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    void update() override {
        static std::size_t idx  = 0;
        auto moveit_result = planned_trajectory_.load(std::memory_order::acquire);
        if (!moveit_result || moveit_result->positions.empty())
            return;
        std::vector<std::vector<double>> target_theta;

        target_theta.reserve(moveit_result->positions.size());
        target_theta = moveit_result->positions;

        if (idx != target_theta.size() - 1) {
            ++idx;
        }

        const auto& q = target_theta[idx];

        RCLCPP_INFO(
            node_->get_logger(), "pt[%zu] q=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]", idx, q[0], q[1],
            q[2], q[3], q[4], q[5]);
    }

private:
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::atomic_bool moveit_running_{false};
    std::thread moveit_thread_;
    struct PlannedTraj {
        std::vector<std::vector<double>> positions;
    };
    std::atomic<std::shared_ptr<const PlannedTraj>> planned_trajectory_{nullptr};
    void moveit_loop() {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // while (state_running_.load(std::memory_order_acquire)) {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.position.x = 0.48;
        target_pose1.position.y = 0.0;
        target_pose1.position.z = 0.4;
        move_group_->setMaxVelocityScalingFactor(0.01);
        move_group_->setMaxAccelerationScalingFactor(0.01);

        move_group_->setPoseTarget(target_pose1);
        move_group_->setPlanningTime(2.0);
        const bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "plan failed");
            return;
        }
        const auto& trajectory_points = my_plan.trajectory.joint_trajectory.points;
        auto result                   = std::make_shared<PlannedTraj>();
        result->positions.reserve(trajectory_points.size());
        for (const auto& pt : trajectory_points) {
            result->positions.push_back(pt.positions);
        }
        planned_trajectory_.store(result, std::memory_order::release);
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6];
    OutputInterface<double> target_theta[6];
    static constexpr const char* PLANNING_GROUP = "alliance_arm";

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor exec_;
    std::atomic_bool spin_running_{false};
    std::thread spin_thread_;
    void spin_loop() {
        while (spin_running_.load(std::memory_order_acquire)) {
            exec_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)
