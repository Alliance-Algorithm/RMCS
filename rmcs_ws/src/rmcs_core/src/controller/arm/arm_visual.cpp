#include "arm_action/obstacle/obstacle_course.hpp"
#include "controller/arm/arm_action/arm_action_machine.hpp"
#include "controller/arm/arm_action/obstacle/obstacle.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <vector>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rmcs_core::controller::arm {

class ArmVisual final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmVisual()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , arm_action_machine_()
        , move_group_(arm_action_machine_.moveit_group_getter().get())
        , moveit_visual_tools_(
              arm_action_machine_.node(), move_group_->getPlanningFrame(),
              rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_->getRobotModel())
        , obstacle_course_(move_group_) {

        moveit_visual_tools_.loadTrajectoryPub("/display_planned_path", false);

        gui_sub_ = arm_action_machine_.node()->create_subscription<sensor_msgs::msg::Joy>(
            "/rviz_visual_tools_gui", 100,
            [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) { gui_callback(msg); });

        simulated_state_ = std::make_shared<moveit::core::RobotState>(move_group_->getRobotModel());
        simulated_state_->setToDefaultValues();

        joint_states_pub_ =
            arm_action_machine_.node()->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);
        joint_states_timer_ = arm_action_machine_.node()->create_wall_timer(
            std::chrono::milliseconds(20), [this] { publish_simulated_joint_state(); });

        simulated_state_->update();
        pose_text_pub_ =
            arm_action_machine_.node()->create_publisher<visualization_msgs::msg::MarkerArray>(
                "/rviz_visual_tools", 10);
        pose_text_timer_ = arm_action_machine_.node()->create_wall_timer(
            std::chrono::milliseconds(100), [this] { publish_ee_pose_text(); });

        goal_marker_sub_ =
            arm_action_machine_.node()
                ->create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
                    kGoalMarkerTopic, 10,
                    [this](
                        const visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr
                            msg) { goal_marker_callback(msg); });

        running_.store(true, std::memory_order_release);
        visual_thread_ = std::thread([this] { visual_loop(); });
    }

    ~ArmVisual() {
        {
            std::lock_guard lock(mutex_);
            quit_ = true;
        }
        running_.store(false, std::memory_order_release);
        cv_.notify_all();

        if (visual_thread_.joinable())
            visual_thread_.join();
    }

    void update() override {}

private:
    static constexpr const char* kGoalMarkerTopic =
        "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update";
    static constexpr const char* kGoalMarkerPrefix = "EE:goal";

    void gui_callback(const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
        if (msg->buttons.size() > 1 && msg->buttons[1]) {
            std::lock_guard lock(mutex_);
            start_pending_ = true;
            cv_.notify_all();
        } else if (msg->buttons.size() > 4 && msg->buttons[4]) {
            std::lock_guard lock(mutex_);
            reset_pending_ = true;
            cv_.notify_all();
        }
    }

    void visual_loop() {
        while (true) {
            bool start = false;
            bool reset = false;
            bool quit = false;
            {
                std::unique_lock lock(mutex_);
                cv_.wait(lock, [this] { return quit_ || start_pending_ || reset_pending_; });
                start = start_pending_;
                reset = reset_pending_;
                quit = quit_;
                start_pending_ = false;
                reset_pending_ = false;
            }
            if (quit)
                break;
            if (reset) {
                reset_visual();
                continue;
            }
            if (start)
                plan_and_visualize();
        }
    }

    void plan_and_visualize() {
        obstacle_course_.set_operation(
            "energy_unit_left_front", obstacle::CollisionObjectOperation::ADD);

        obstacle_course_.apply_collision();

        const Action::PoseTarget target{-0.295, -0.033, 0.177, 1.972, -0.603, 1.442};
        arm_action_machine_.process({Action::Step::makePose(target, Action::MotionParams{})});

        const auto result = wait_for_plan();
        if (!result || !result->plan_success) {
            RCLCPP_ERROR(get_logger(), "Planning failed");
            moveit_visual_tools_.publishText(
                geometry_msgs::msg::Pose().set__position(
                    geometry_msgs::msg::Point().set__x(0.3).set__y(0.0).set__z(0.6)),
                "Planning failed", rviz_visual_tools::RED, rviz_visual_tools::XXLARGE);
            moveit_visual_tools_.trigger();
            return;
        }

        auto trajectory_msg = build_trajectory_msg(*result);
        if (trajectory_msg.joint_trajectory.points.empty()) {
            RCLCPP_WARN(get_logger(), "Empty trajectory");
            return;
        }

        moveit_visual_tools_.publishTrajectoryLine(
            trajectory_msg, move_group_->getRobotModel()->getLinkModel("link_6"),
            move_group_->getRobotModel()->getJointModelGroup("alliance_arm"),
            rviz_visual_tools::LIME_GREEN);
        moveit_visual_tools_.trigger();

        RCLCPP_INFO(get_logger(), "Visualized trajectory");
    }

    void reset_visual() {
        moveit_visual_tools_.deleteAllMarkers();

        moveit_msgs::msg::DisplayTrajectory reset_msg;
        reset_msg.model_id = move_group_->getRobotModel()->getName();

        moveit_msgs::msg::RobotTrajectory reset_trajectory;
        reset_trajectory.joint_trajectory.joint_names = move_group_->getJointNames();
        {
            std::lock_guard lock(state_mutex_);
            moveit::core::robotStateToRobotStateMsg(*simulated_state_, reset_msg.trajectory_start);
            trajectory_msgs::msg::JointTrajectoryPoint point;
            simulated_state_->copyJointGroupPositions("alliance_arm", point.positions);
            reset_trajectory.joint_trajectory.points.push_back(point);
        }
        reset_msg.trajectory.push_back(reset_trajectory);

        moveit_visual_tools_.publishTrajectoryPath(reset_msg);
        moveit_visual_tools_.trigger();

        RCLCPP_INFO(get_logger(), "Visualization reset");
    }

    void publish_simulated_joint_state() {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = arm_action_machine_.node()->now();
        msg.header.frame_id = "rmcs_arm_visual_sim";
        msg.name = move_group_->getJointNames();
        msg.position.reserve(msg.name.size());
        {
            std::lock_guard lock(state_mutex_);
            for (const auto& name : msg.name)
                msg.position.push_back(simulated_state_->getVariablePosition(name));
        }
        joint_states_pub_->publish(msg);
    }

    void goal_marker_callback(
        const visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr& msg) {
        std::lock_guard lock(goal_mutex_);
        for (const auto& marker : msg->markers)
            if (marker.name.rfind(kGoalMarkerPrefix, 0) == 0) {
                goal_pose_ = marker.pose;
                has_goal_pose_ = true;
            }
        for (const auto& pose : msg->poses)
            if (pose.name.rfind(kGoalMarkerPrefix, 0) == 0) {
                goal_pose_ = pose.pose;
                has_goal_pose_ = true;
            }
    }

    void publish_ee_pose_text() {
        Eigen::Vector3d p;
        double roll, pitch, yaw;

        geometry_msgs::msg::Pose goal_pose;
        bool have_goal = false;
        {
            std::lock_guard lock(goal_mutex_);
            have_goal = has_goal_pose_;
            goal_pose = goal_pose_;
        }

        if (have_goal) {
            p = Eigen::Vector3d(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
            tf2::Matrix3x3(
                tf2::Quaternion(
                    goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z,
                    goal_pose.orientation.w))
                .getRPY(roll, pitch, yaw);
        } else {
            Eigen::Isometry3d tf;
            {
                std::lock_guard lock(state_mutex_);
                tf = simulated_state_->getGlobalLinkTransform("link_6");
            }
            p = tf.translation();
            Eigen::Quaterniond q(tf.rotation());
            tf2::Matrix3x3(tf2::Quaternion(q.x(), q.y(), q.z(), q.w())).getRPY(roll, pitch, yaw);
        }

        char text[160];
        std::snprintf(
            text, sizeof(text), "xyz:%.3f,%.3f,%.3f rpy:%.3f,%.3f,%.3f", p.x(), p.y(), p.z(), roll,
            pitch, yaw);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = arm_action_machine_.node()->now();
        marker.ns = "ee_pose";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = p.x();
        marker.pose.position.y = p.y();
        marker.pose.position.z = p.z() + 0.15;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.05;
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
        marker.text = text;

        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(marker);
        pose_text_pub_->publish(arr);
    }

    std::shared_ptr<const ActionMachine::PlannedTrajectory> wait_for_plan() {
        uint64_t last_id = 0;
        if (const auto current = arm_action_machine_.get_trajectory())
            last_id = current->request_id;

        constexpr auto timeout = std::chrono::seconds(15);
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        while (running_.load(std::memory_order_acquire)
               && std::chrono::steady_clock::now() < deadline) {
            const auto result = arm_action_machine_.get_trajectory();
            if (result && result->request_id != 0 && result->request_id != last_id)
                return result;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return nullptr;
    }

    moveit_msgs::msg::RobotTrajectory
        build_trajectory_msg(const ActionMachine::PlannedTrajectory& result) const {
        moveit_msgs::msg::RobotTrajectory msg;
        msg.joint_trajectory.joint_names = move_group_->getJointNames();

        double time_from_start = 0.0;
        for (const auto& entry : result.step_position_map) {
            const auto& positions = entry.second;
            for (std::size_t i = 0; i < positions.size(); ++i) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = positions[i];
                point.time_from_start = rclcpp::Duration::from_seconds(time_from_start);
                msg.joint_trajectory.points.push_back(std::move(point));
                time_from_start += 0.1;
            }
        }
        return msg;
    }

    ActionMachine arm_action_machine_;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    obstacle::ObstacleCourse obstacle_course_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gui_sub_;
    std::thread visual_thread_;
    std::atomic_bool running_{false};

    std::mutex mutex_;
    std::condition_variable cv_;
    bool start_pending_{false};
    bool reset_pending_{false};
    bool quit_{false};

    moveit::core::RobotStatePtr simulated_state_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::TimerBase::SharedPtr joint_states_timer_;

    std::mutex state_mutex_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_text_pub_;
    rclcpp::TimerBase::SharedPtr pose_text_timer_;

    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr
        goal_marker_sub_;
    std::mutex goal_mutex_;
    geometry_msgs::msg::Pose goal_pose_;
    bool has_goal_pose_{false};
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmVisual, rmcs_executor::Component)
