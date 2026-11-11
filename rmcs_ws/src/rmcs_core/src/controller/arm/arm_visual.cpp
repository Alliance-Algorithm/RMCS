#include "axis_transform.hpp"
#include "controller/arm/arm_MTC/MTC_machine.hpp"
#include "controller/arm/arm_MTC/MTC_parameter_map.hpp"
#include "controller/arm/arm_MTC/MTC_preview.hpp"
#include "controller/arm/arm_MTC/MTC_step.hpp"
#include "controller/arm/obstacle/obstacle_coordinates_map.hpp"
#include "controller/arm/obstacle/obstacle_course.hpp"
#include "controller/arm/obstacle/obstacle_types.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <numbers>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <variant>

namespace rmcs_core::controller::arm {

class ArmVisual final
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmVisual()
        : Node(get_component_name())
        , P_({0, -1, 0}, {-0.118 - 0.051, 0, -0.208})
        , Q_({1, 0, 0}, {-0.118 - 0.051, 0, -0.208 - 0.015})
        , point_place_({1, 0, 0}, {0.11, 0, 0})
        , mtc_dictionary_(MTC::make_default_dictionary())
        , mtc_machine_()
        , preview_(mtc_machine_.robot_model())
        , moveit_visual_tools_(
              mtc_machine_.node(), mtc_machine_.robot_model()->getModelFrame(),
              rviz_visual_tools::RVIZ_MARKER_TOPIC, mtc_machine_.robot_model())
        , obstacle_course_(mtc_machine_.moveit_group_getter().get(), obstacle_coordinates_map_) {
        initial_setting();
        initial_scene_ = mtc_machine_.fetch_scene();
        auto& state = initial_scene_->getCurrentStateNonConst();
        state.setToDefaultValues();
        for (const auto* joint_name :
             {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"})
            state.setVariablePosition(joint_name, 0.0);
        state.update();

        plan_from_preview_ = declare_parameter<bool>("plan_from_preview", true);

        const auto& node = mtc_machine_.node();
        joint_states_pub_ =
            node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        scene_pub_ = node->create_publisher<moveit_msgs::msg::PlanningScene>(
            "/arm_mtc_preview/planning_scene", rclcpp::QoS(1).reliable().transient_local());
        ee_pose_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
            rviz_visual_tools::RVIZ_MARKER_TOPIC, 10);
        publish_current_state(std::chrono::steady_clock::now(), true);
        gui_sub_ = node->create_subscription<sensor_msgs::msg::Joy>(
            "/rviz_visual_tools_gui", 10,
            [this](sensor_msgs::msg::Joy::ConstSharedPtr message) { gui_callback(*message); });
        goal_marker_sub_ =
            node->create_subscription<visualization_msgs::msg::InteractiveMarkerUpdate>(
                kGoalMarkerTopic, 10,
                [this](visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr message) {
                    goal_marker_callback(*message);
                });
        timer_ = node->create_wall_timer(std::chrono::milliseconds(20), [this] { preview_tick(); });
    }

    ~ArmVisual() override {
        if (timer_)
            timer_->cancel();
        gui_sub_.reset();
        goal_marker_sub_.reset();
        mtc_machine_.stop();
    }

    void update() override {}

private:
    static constexpr const char* kGoalMarkerTopic =
        "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update";
    static constexpr const char* kGoalMarkerPrefix = "EE:goal";

    void initial_setting() {
        initial_pose_ = {0.050,
                         -0.200,
                         0.300,
                         -std::numbers::pi / 4 * 3,
                         -std::numbers::pi / 4,
                         std::numbers::pi / 2};
        P_result_ = P_.transform(initial_pose_);
        Q_result_ = Q_.transform(initial_pose_);
        point_result_ = point_place_.transform(initial_pose_);

        obstacle_course_.set_pose("exchange", initial_pose_);
        obstacle_course_.set_pose("pillar", initial_pose_);
    }

    void goal_marker_callback(const visualization_msgs::msg::InteractiveMarkerUpdate& message) {
        std::lock_guard lock(goal_mutex_);
        for (const auto& name : message.erases)
            if (name.rfind(kGoalMarkerPrefix, 0) == 0)
                has_goal_pose_ = false;
        for (const auto& marker : message.markers)
            if (marker.name.rfind(kGoalMarkerPrefix, 0) == 0) {
                goal_pose_ = marker.pose;
                goal_frame_ = marker.header.frame_id;
                has_goal_pose_ = true;
            }
        for (const auto& pose : message.poses)
            if (pose.name.rfind(kGoalMarkerPrefix, 0) == 0) {
                goal_pose_ = pose.pose;
                goal_frame_ = pose.header.frame_id;
                has_goal_pose_ = true;
            }
    }

    void gui_callback(const sensor_msgs::msg::Joy& message) {
        try {
            if (message.buttons.size() > 4 && message.buttons[4]) {
                request_id_ = 0;
                preview_.reset();
                moveit_visual_tools_.deleteAllMarkers();
                moveit_visual_tools_.trigger();
                force_full_scene_ = true;
                publish_current_state(std::chrono::steady_clock::now(), true);
            } else if (message.buttons.size() > 1 && message.buttons[1]) {
                preview_.cancel();
                const bool extract = next_extract_;
                next_extract_ = !next_extract_;
                request_id_ = mtc_machine_.process(
                    mtc_dictionary_.helper_build_chunk(
                        {"current_start", extract ? "extra_right_back" : "store_right_back",
                         extract ? "extra" : "store", "auto_walk"},
                        {{"unit", "energy_unit_right_back"}}),
                    // mtc_dictionary_.helper_build_chunk(
                    // {"current_start", "extra_left_front", "extra", "tech_core_test",
                    //  "tech_core_place", "push", "rotate_P", "rotate_Q"},
                    // {{"unit", "energy_unit_left_front"}},
                    // {
                    //     {"tech_core_test_pre_place", MTC::makePoseGoal(
                    //                                      {.frame_id = "base_link",
                    //                                       .x = point_result_.point.x(),
                    //                                       .y = point_result_.point.y(),
                    //                                       .z = point_result_.point.z(),
                    //                                       .roll = initial_pose_.roll,
                    //                                       .pitch = initial_pose_.pitch,
                    //                                       .yaw = initial_pose_.yaw})},
                    //     {"tech_core_rotate_P",
                    //      MTC::makeRotationDirection({
                    //          .frame_id = "base_link",
                    //          .axis_point =
                    //              {P_result_.point.x(), P_result_.point.y(),
                    //               P_result_.point.z()},
                    //          .axis_direction =
                    //              {P_result_.direction.x(), P_result_.direction.y(),
                    //               P_result_.direction.z()},

                    //      })},
                    //     {"tech_core_rotate_Q",
                    //      MTC::makeRotationDirection({
                    //          .frame_id = "base_link",
                    //          .axis_point =
                    //              {Q_result_.point.x(), Q_result_.point.y(),
                    //               Q_result_.point.z()},
                    //          .axis_direction =
                    //              {Q_result_.direction.x(), Q_result_.direction.y(),
                    //               Q_result_.direction.z()},

                    //      })},
                    // }),
                    plan_from_preview_ ? preview_.scene()
                                       : planning_scene::PlanningSceneConstPtr{});
            }
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "MTC request failed: %s", error.what());
        }
    }

    void preview_tick() {
        try {
            const auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_tick_).count();
            last_tick_ = now;
            bool force_ee_pose = false;

            const auto result = mtc_machine_.get_trajectory();
            if (result && result->request_id == request_id_
                && result->request_id != consumed_request_id_) {
                consumed_request_id_ = result->request_id;
                if (result->plan_success && result->plan) {
                    preview_.start(*result->plan);
                    elapsed = 0.0;
                    force_full_scene_ = true;
                    force_ee_pose = true;
                    publish_plan_trajectory(*result->plan);
                    RCLCPP_INFO(
                        get_logger(), "%s", MTC::describe_execution_plan(*result->plan).c_str());
                } else {
                    RCLCPP_ERROR(
                        get_logger(), "MTC planning failed: %s", result->diagnostic.c_str());
                }
            }

            const auto previous_status = preview_.status();
            preview_.advance(elapsed);
            if (preview_.status() != previous_status) {
                force_full_scene_ = true;
                if (preview_.status() == MTC::PreviewStatus::Failed)
                    RCLCPP_ERROR(
                        get_logger(), "MTC playback failed: %s", preview_.diagnostic().c_str());
            }

            publish_current_state(now, force_ee_pose);
        } catch (const std::exception& error) {
            preview_.cancel();
            RCLCPP_ERROR(get_logger(), "MTC preview failed: %s", error.what());
        }
    }

    void publish_plan_trajectory(const MTC::ExecutionPlan& plan) {
        moveit_visual_tools_.deleteAllMarkers();

        const auto* ee_link = mtc_machine_.robot_model()->getLinkModel("link_6");
        std::size_t line_count = 0;
        for (const auto& step : plan.steps) {
            const auto* motion = std::get_if<MTC::MotionSegment>(&step.action);
            if (!motion)
                continue;
            if (motion->trajectory.joint_trajectory.points.empty()) {
                RCLCPP_WARN(
                    get_logger(), "Skipping empty trajectory line at %s", step.stage_path.c_str());
                continue;
            }
            const auto* group = mtc_machine_.robot_model()->getJointModelGroup(motion->group);
            if (moveit_visual_tools_.publishTrajectoryLine(
                    motion->trajectory, ee_link, group, rviz_visual_tools::LIME_GREEN))
                ++line_count;
            else
                RCLCPP_WARN(
                    get_logger(), "Failed to publish trajectory line at %s",
                    step.stage_path.c_str());
        }
        moveit_visual_tools_.trigger();
        RCLCPP_INFO(get_logger(), "Displayed %zu trajectory line(s) from MTC plan", line_count);
    }

    void publish_current_state(
        std::chrono::steady_clock::time_point now, bool force_ee_pose = false) {
        const auto scene = preview_.scene() ? preview_.scene() : initial_scene_;
        const auto& state = scene->getCurrentState();
        const std::array<double, 6> joint_angles{
            state.getVariablePosition("joint_1"), state.getVariablePosition("joint_2"),
            state.getVariablePosition("joint_3"), state.getVariablePosition("joint_4"),
            state.getVariablePosition("joint_5"), state.getVariablePosition("joint_6"),
        };
        const auto gripper = preview_.gripper_action();

        sensor_msgs::msg::JointState joints;
        joints.header.stamp = mtc_machine_.node()->now();
        joints.header.frame_id = "rmcs_arm_visual_sim";
        joints.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joints.position.assign(joint_angles.begin(), joint_angles.end());
        joint_states_pub_->publish(joints);

        // RCLCPP_INFO_THROTTLE(
        //     get_logger(), *mtc_machine_.node()->get_clock(), 1000,
        //     "MTC preview rad=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] gripper=%s", joint_angles[0],
        //     joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5],
        //     !gripper                               ? "unknown"
        //     : *gripper == MTC::GripperAction::Open ? "open"
        //                                            : "close");

        moveit_msgs::msg::PlanningScene scene_message;
        const bool full_scene = force_full_scene_ || scene_revision_ != preview_.scene_revision()
                             || now - last_full_scene_ >= std::chrono::seconds(1);
        if (full_scene) {
            scene->getPlanningSceneMsg(scene_message);
            last_full_scene_ = now;
            scene_revision_ = preview_.scene_revision();
            force_full_scene_ = false;
        } else {
            scene_message.is_diff = true;
            moveit::core::robotStateToRobotStateMsg(state, scene_message.robot_state, false);
            scene_message.robot_state.is_diff = true;
        }
        scene_message.robot_state.joint_state.header.stamp = joints.header.stamp;
        scene_pub_->publish(scene_message);

        if (force_ee_pose || now - last_ee_pose_ >= std::chrono::milliseconds(100)) {
            geometry_msgs::msg::Pose goal_pose;
            std::string goal_frame;
            bool has_goal = false;
            {
                std::lock_guard lock(goal_mutex_);
                has_goal = has_goal_pose_;
                goal_pose = goal_pose_;
                goal_frame = goal_frame_;
            }

            const bool was_published = goal_text_published_;
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id =
                goal_frame.empty() ? mtc_machine_.robot_model()->getModelFrame() : goal_frame;
            marker.header.stamp = joints.header.stamp;
            marker.ns = "ee_goal_pose";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.05;
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            if (has_goal) {
                double roll = 0.0;
                double pitch = 0.0;
                double yaw = 0.0;
                tf2::Matrix3x3(
                    tf2::Quaternion(
                        goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z,
                        goal_pose.orientation.w))
                    .getRPY(roll, pitch, yaw);
                char text[192];
                std::snprintf(
                    text, sizeof(text), "xyz:%.3f,%.3f,%.3f\nrpy:%.3f,%.3f,%.3f",
                    goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, roll, pitch,
                    yaw);
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = goal_pose;
                marker.pose.position.z += 0.15;
                marker.text = text;
            } else {
                marker.action = visualization_msgs::msg::Marker::DELETE;
            }
            goal_text_published_ = has_goal;

            if (has_goal || was_published) {
                visualization_msgs::msg::MarkerArray markers;
                markers.markers.push_back(marker);
                ee_pose_pub_->publish(markers);
            }
            last_ee_pose_ = now;
        }
    }

    AxisTransform P_;
    AxisTransform Q_;
    AxisTransform point_place_;

    AxisTransform::Result P_result_;
    AxisTransform::Result Q_result_;
    AxisTransform::Result point_result_;
    obstacle::Pose initial_pose_ = {0, 0, 0, 0, 0, 0};

    MTC::Dictionary mtc_dictionary_;
    MTC::Machine mtc_machine_;
    MTC::PreviewPlayer preview_;
    moveit_visual_tools::MoveItVisualTools moveit_visual_tools_;
    obstacle::ObstacleCoordinatesMap obstacle_coordinates_map_;
    obstacle::ObstacleCourse obstacle_course_;
    planning_scene::PlanningScenePtr initial_scene_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ee_pose_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr gui_sub_;
    rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerUpdate>::SharedPtr
        goal_marker_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::uint64_t request_id_ = 0;
    std::uint64_t consumed_request_id_ = 0;
    std::size_t scene_revision_ = 0;
    bool force_full_scene_ = true;
    bool plan_from_preview_ = false;
    bool next_extract_ = true;
    std::chrono::steady_clock::time_point last_tick_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_full_scene_{};
    std::chrono::steady_clock::time_point last_ee_pose_{};
    std::mutex goal_mutex_;
    geometry_msgs::msg::Pose goal_pose_;
    std::string goal_frame_;
    bool has_goal_pose_ = false;
    bool goal_text_published_ = false;
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmVisual, rmcs_executor::Component)
