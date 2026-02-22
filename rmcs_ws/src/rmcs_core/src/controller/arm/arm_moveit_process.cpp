#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rmcs_executor/component.hpp>

static constexpr const char* PLANNING_GROUP = "alliance_arm";

namespace rmcs_core::controller::arm {

class ArmMoveitProcess
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmMoveitProcess()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        target_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/arm/target_joints", rclcpp::QoS(1).best_effort());

        enable_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/arm/enable_cmd", rclcpp::QoS(1).best_effort());

        // ★ 关键：在构造函数中创建独立子节点，供 MoveGroupInterface 使用
        moveit_node_ = std::make_shared<rclcpp::Node>(
            "arm_moveit_mgi_node",
            rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true));

        // ★ 用独立 executor 在后台 spin 这个子节点
        moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        moveit_executor_->add_node(moveit_node_);
        spin_thread_ = std::thread([this]() { moveit_executor_->spin(); });

        RCLCPP_INFO(get_logger(), "ArmMoveitProcess constructed");
    }

    ~ArmMoveitProcess() override {
        moveit_running_.store(false, std::memory_order_release);
        if (moveit_thread_.joinable())
            moveit_thread_.join();
        moveit_executor_->cancel();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    void update() override {
        // ★ 用 moveit_node_ 而非 shared_from_this()
        if (!moveit_initialized_) {
            moveit_initialized_ = true;
            move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
                moveit_node_, PLANNING_GROUP);
            move_group_->startStateMonitor(10.0);

            moveit_running_.store(true, std::memory_order_release);
            moveit_thread_ = std::thread([this] {
                RCLCPP_INFO(get_logger(), "moveit thread started");
                while (moveit_running_.load(std::memory_order_acquire)) {
                    moveit_loop();
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            });

            one_shot_timer_ = create_wall_timer(
                std::chrono::seconds(3),
                [this]() {
                    should_plan_.store(true);
                    one_shot_timer_.reset();
                    RCLCPP_INFO(get_logger(), "Triggering planning...");
                });

            RCLCPP_INFO(get_logger(), "MoveGroupInterface initialized");
            return;
        }

        if (!trajectory_ready_.load()) return;

        double elapsed = (now() - traj_start_time_).seconds();
        double total = rclcpp::Duration(traj_points_.back().time_from_start).seconds();

        if (elapsed >= total) {
            publish_point(traj_points_.back().positions);
            trajectory_ready_.store(false);
            RCLCPP_INFO(get_logger(), "Trajectory finished");
            return;
        }

        for (int i = (int)traj_points_.size() - 1; i >= 0; --i) {
            double t = rclcpp::Duration(traj_points_[i].time_from_start).seconds();
            if (elapsed >= t) {
                publish_point(traj_points_[i].positions);
                return;
            }
        }
    }

private:
    void moveit_loop() {
        if (!should_plan_.exchange(false)) return;

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x    = 0.48;
        target_pose.position.y    = 0.0;
        target_pose.position.z    = 0.4;
        target_pose.orientation.w = 1.0;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.0;

        move_group_->setMaxVelocityScalingFactor(0.05);
        move_group_->setMaxAccelerationScalingFactor(0.05);
        move_group_->setPoseTarget(target_pose);
        move_group_->setPlanningTime(5.0);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success =
            (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            RCLCPP_WARN(get_logger(), "Planning FAILED");
            return;
        }

        traj_points_     = my_plan.trajectory.joint_trajectory.points;
        traj_start_time_ = now();
        trajectory_ready_.store(true);
        RCLCPP_INFO(get_logger(), "Planning SUCCESS: %zu points", traj_points_.size());
    }

    void publish_point(const std::vector<double>& q) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(6);
        for (std::size_t i = 0; i < 5 && i < q.size(); ++i)
            msg.data[i] = q[i];
        msg.data[5] = 0.0;
        target_pub_->publish(msg);

        std_msgs::msg::Bool en;
        en.data = true;
        enable_pub_->publish(en);
    }

    // ★ 独立子节点 + executor + spin线程
    rclcpp::Node::SharedPtr                                         moveit_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>      moveit_executor_;
    std::thread                                                     spin_thread_;

    bool moveit_initialized_{false};
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::atomic_bool         moveit_running_{false};
    std::thread              moveit_thread_;

    std::atomic<bool>        should_plan_{false};
    rclcpp::TimerBase::SharedPtr one_shot_timer_;

    std::atomic<bool>        trajectory_ready_{false};
    std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points_;
    rclcpp::Time             traj_start_time_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              enable_pub_;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::arm::ArmMoveitProcess, rmcs_executor::Component)