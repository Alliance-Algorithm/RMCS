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

        exec_.add_node(node_);
        spin_running_.store(true, std::memory_order_release);
        spin_thread_ = std::thread([this] { spin_loop(); });

        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch);

        register_output("/arm/enable_flag", is_arm_enable, false);

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", theta[i]);
            register_output(joint_prefix + "/target_theta", target_theta[i], NAN);
        }

        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "alliance_arm");
        move_group_->startStateMonitor();
        moveit_running_.store(true, std::memory_order_release);
        moveit_thread_ = std::thread([this] {
            moveit_init();
            while (moveit_running_.load(std::memory_order_acquire)) {
                moveit_loop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
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
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        using namespace rmcs_msgs;
        static bool initial_check_done{false};
        if (!initial_check_done) {
            *is_arm_enable = false;
            for (std::size_t i = 0; i < std::size(theta); ++i) {
                *target_theta[i] = *theta[i];
            }
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                initial_check_done = true;
            }
        } else {
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                *is_arm_enable = false;
                for (std::size_t i = 0; i < std::size(theta); ++i) {
                    *target_theta[i] = *theta[i];
                }
                set_arm_mode(rmcs_msgs::ArmMode::None);
            } else {
                *is_arm_enable = true;
                if (switch_left == Switch::UP && switch_right == Switch::UP) {
                    set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Position);
                } else if (switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
                    set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Orientation);
                } else if (switch_left == Switch::MIDDLE && switch_right == Switch::UP) {
                }
            }
        }

        update_plan_request_and_trajectory_step();
    }

private:
    rmcs_msgs::ArmMode last_requested_arm_mode_{rmcs_msgs::ArmMode::None};

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::atomic_bool moveit_running_{false};
    std::thread moveit_thread_;

    struct PlanRequest {
        uint64_t request_id{0};
        rmcs_msgs::ArmMode arm_mode{rmcs_msgs::ArmMode::None};
    };
    struct PlannedTrajectory {
        uint64_t request_id{0};
        bool plan_success{false};
        std::vector<std::vector<double>> positions;
    };
    std::atomic<std::shared_ptr<const PlanRequest>> plan_request{nullptr};
    std::atomic<std::shared_ptr<const PlannedTrajectory>> planned_trajectory_{nullptr};

    void set_arm_mode(rmcs_msgs::ArmMode mode) {
        const auto current = plan_request.load(std::memory_order_acquire);
        auto next          = std::make_shared<PlanRequest>();
        if (current) {
            next->request_id = current->request_id;
        }
        next->arm_mode = mode;
        plan_request.store(next, std::memory_order_release);
    }

    rmcs_msgs::ArmMode get_arm_mode() const {
        const auto current = plan_request.load(std::memory_order_acquire);
        if (!current) {
            return rmcs_msgs::ArmMode::None;
        }
        return current->arm_mode;
    }

    void moveit_init() {};
    void moveit_loop() {
        const auto request = plan_request.load(std::memory_order_acquire);
        static uint64_t last_planned_request_id_{0};

        if (!request || request->request_id == last_planned_request_id_) {
            return;
        }
        last_planned_request_id_ = request->request_id;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        {
            // TODO: use current_mode to choose planning target/constraints.
            geometry_msgs::msg::Pose target_pose1;
            target_pose1.position.x = 0.48;
            target_pose1.position.y = 0.0;
            target_pose1.position.z = 0.4;
            move_group_->setMaxVelocityScalingFactor(0.01);
            move_group_->setMaxAccelerationScalingFactor(0.01);
            move_group_->setPoseTarget(target_pose1);
            move_group_->setPlanningTime(2.0);
        }
        const bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        auto result          = std::make_shared<PlannedTrajectory>();
        result->request_id   = request->request_id;
        result->plan_success = success;
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "plan failed");
            planned_trajectory_.store(result, std::memory_order::release);
            return;
        }
        const auto& trajectory_points = my_plan.trajectory.joint_trajectory.points;
        result->positions.reserve(trajectory_points.size());
        for (const auto& pt : trajectory_points) {
            result->positions.push_back(pt.positions);
        }

        planned_trajectory_.store(result, std::memory_order::release);
    }
    void update_plan_request_and_trajectory_step() {
        static std::size_t trajectory_steps{0};
        const auto current_arm_mode = get_arm_mode();

        if (current_arm_mode != last_requested_arm_mode_) {
            last_requested_arm_mode_   = current_arm_mode;
            trajectory_steps           = 0;
            const auto current_request = plan_request.load(std::memory_order_acquire);
            auto request               = std::make_shared<PlanRequest>();
            request->request_id        = current_request ? (current_request->request_id + 1) : 1;
            request->arm_mode          = current_arm_mode;
            plan_request.store(request, std::memory_order_release);
        }

        const auto moveit_result = planned_trajectory_.load(std::memory_order_acquire);
        if (!moveit_result) {
            return;
        }
        const auto latest_plan_request = plan_request.load(std::memory_order_acquire);
        if (!latest_plan_request) {
            return;
        }

        if (moveit_result->request_id == latest_plan_request->request_id) {
            if (moveit_result->plan_success && !moveit_result->positions.empty()) {

                if (trajectory_steps < moveit_result->positions.size()) {
                    const auto& q = moveit_result->positions[trajectory_steps];
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "req[%llu] pt[%zu] q=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                        static_cast<unsigned long long>(moveit_result->request_id),
                        trajectory_steps, q[0], q[1], q[2], q[3], q[4], q[5]);
                    trajectory_steps++;
                }
            }
        }
    }

    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6];
    OutputInterface<double> target_theta[6];

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
