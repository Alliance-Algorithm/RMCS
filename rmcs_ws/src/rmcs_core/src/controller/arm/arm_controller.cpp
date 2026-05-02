#include "filter/low_pass_filter.hpp"
#include "hardware/endian_promise.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

#include <atomic>
#include <chrono>
#include <mutex>
#include <sys/cdefs.h>
#include <thread>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/arm_mode.hpp>
#include <rmcs_msgs/gripper_mode.hpp>
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
        : custom_joint_filter_(0.2)
        , node_(
              std::make_shared<rclcpp::Node>(
                  get_component_name(),
                  rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))) {

        exec_.add_node(node_);
        spin_thread_ = std::thread([this] { spin_loop(); });
        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "alliance_arm");
        move_group_->startStateMonitor();
        register_input("/remote/joystick/right", joystick_right_);
        register_input("/remote/joystick/left", joystick_left_);
        register_input("/remote/switch/right", switch_right_);
        register_input("/remote/switch/left", switch_left_);
        register_input("/remote/mouse/velocity", mouse_velocity_);
        register_input("/remote/mouse", mouse_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/remote/rotary_knob_switch", rotary_knob_switch);

        register_output("/arm/mode", arm_mode_);
        register_output("/arm/enable_flag", is_arm_enable, false);

        register_input("/referee/image_transmission/custom", custom_data_);

        for (std::size_t i = 0; i < 6; ++i) {
            const std::string joint_prefix = "/arm/joint_" + std::to_string(i + 1);
            register_input(joint_prefix + "/theta", theta[i]);
            register_input(joint_prefix + "/lower_limit", joint_lower_limit_[i]);
            register_input(joint_prefix + "/upper_limit", joint_upper_limit_[i]);
            register_output(joint_prefix + "/target_theta", target_theta[i], NAN);
        }
        register_output("/arm/gripper/target_theta", target_theta[6], NAN);
        register_input("/arm/gripper/motor/angle", theta[6], NAN);
        register_input("/arm/gripper/motor/velocity", gripper_velocity_, NAN);
        register_input("/arm/gripper/motor/torque", gripper_torque_, NAN);
        register_output("/arm/gripper/target_theta_error", gripper_target_theta_error, NAN);
        register_input("/arm/image_pitch/motor/angle", image_pitch_theta_, NAN);
        register_output("/arm/image_pitch/target_theta", image_pitch_target_theta_, NAN);
        register_output("/arm/custom_big_yaw", custom_big_yaw_output_, 0.0);

        moveit_running_.store(true, std::memory_order_release);
        moveit_thread_ = std::thread([this] {
            while (moveit_running_.load(std::memory_order_acquire)) {

                moveit_loop();

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    ~ArmController() override {
        moveit_running_.store(false, std::memory_order_release);
        exec_.cancel();

        if (moveit_thread_.joinable())
            moveit_thread_.join();
        if (spin_thread_.joinable())
            spin_thread_.join();
    }

    void update() override {
        *arm_mode_ = get_arm_mode();

        auto knob         = *rotary_knob_switch;
        auto switch_right = *switch_right_;
        auto switch_left  = *switch_left_;
        auto keyboard     = *keyboard_;
        auto mouse        = *mouse_;
        using namespace rmcs_msgs;
        static bool initial_check_done{false};

        if (!initial_check_done) {
            *is_arm_enable = false;
            if (switch_left == Switch::DOWN && switch_right == Switch::DOWN) {
                initial_check_done = true;
            }
            reset();
            return;
        }

        if ((switch_left == Switch::DOWN && switch_right == Switch::DOWN)
            || switch_left == Switch::UNKNOWN) {
            reset();
            return;
        } else {
            *is_arm_enable = true;
        }

        if (switch_left == Switch::UP && switch_right == Switch::UP) {
            if (last_switch_left_ != Switch::UP || last_switch_right_ != Switch::UP) {
                set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Position);
            }
        } else if (switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
            if (last_switch_left_ != Switch::UP || last_switch_right_ != Switch::MIDDLE) {
                set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Orientation);
            }
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (knob != last_rotary_knob_switch_) {
                if (knob == Switch::UP) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_One_Stairs);
                } else if (knob == Switch::DOWN) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Down_Stairs);
                }
            }
            if (keyboard.g && !last_keyboard_.g) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Walk);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Down_Stairs);
                }
            }
            if (keyboard.b && !last_keyboard_.b) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Two_Stairs);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_One_Stairs);
                }
            }
            if (keyboard.f && !last_keyboard_.f) {
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LB);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RB);
                }
            }
            if (keyboard.s && !last_keyboard_.s) {
                set_arm_mode(rmcs_msgs::ArmMode::Auto_Spin);
            }
            if (keyboard.r && !last_keyboard_.r) {
                if (!keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Custome);
                }
            }
            if (keyboard.w && !last_keyboard_.w) {
                if (!keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Linear);
                }
            }
            if (keyboard.a && !last_keyboard_.a) {
                set_arm_mode(rmcs_msgs::ArmMode::None);
            }
            if (mouse.left && !last_mouse_.left) {
                image_pitch_theta1_offset_ += 0.05;
            }
            if (mouse.right && !last_mouse_.right) {
                image_pitch_theta1_offset_ -= 0.05;
            }
        } else {
            set_arm_mode(rmcs_msgs::ArmMode::None);
        }

        switch (get_arm_mode()) {
            using namespace rmcs_msgs;
        case ArmMode::DT7_Control_Position: {
            execute_dt7_position();
            break;
        }
        case ArmMode::DT7_Control_Orientation: {
            execute_dt7_orientation();
            break;
        }
        case ArmMode::Custome: {
            execute_custom();
            break;
        }
        case ArmMode::Auto_Walk:
        case ArmMode::Auto_Linear:
        case ArmMode::Auto_Extract_LB:
        case ArmMode::Auto_Extract_RB:
        case ArmMode::Auto_Storage_RB:
        case ArmMode::Auto_Storage_LB: {
            execute_plan_request_and_trajectory_step();
            break;
        }
        case ArmMode::None: {
            break;
        }
        default: {
            break;
        }
        }
        last_switch_left_        = switch_left;
        last_switch_right_       = switch_right;
        last_rotary_knob_switch_ = knob;
        last_keyboard_           = keyboard;
        last_mouse_              = mouse;
        gripper_control();
        image_pitch_control();
    }

private:
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

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
        std::vector<int> steps;
    };
    std::atomic<std::shared_ptr<const PlanRequest>> plan_request{nullptr};
    std::atomic<std::shared_ptr<const PlannedTrajectory>> planned_trajectory_{nullptr};

    void set_arm_mode(rmcs_msgs::ArmMode mode) {
        const auto current = plan_request.load(std::memory_order_acquire);
        auto next          = std::make_shared<PlanRequest>();
        if (current) {
            next->request_id = current->request_id;
        }
        if (mode != rmcs_msgs::ArmMode::None) {
            ++next->request_id;
        }
        next->arm_mode = mode;
        plan_request.store(next, std::memory_order_release);
    }

    void set_gripper_mode(rmcs_msgs::GripperMode mode) { gripper_mode_ = mode; }
    rmcs_msgs::GripperMode get_gripper_mode() const { return gripper_mode_; }

    rmcs_msgs::ArmMode get_arm_mode() const {
        const auto current = plan_request.load(std::memory_order_acquire);
        if (!current) {
            return rmcs_msgs::ArmMode::None;
        }
        return current->arm_mode;
    }

    void moveit_loop() {

        static std::vector<double> auto_walk_joint_target;
        static constexpr std::size_t AutoStepCount                   = 5;
        const std::array<std::string, AutoStepCount> auto_step_names = {
            "step1_pose", "step2_pose", "step3_lin", "step4_lin", "step5_pose"};
        struct AutoMineConfig {
            std::array<std::array<double, 6>, AutoStepCount> step_rpy{};
            std::array<bool, AutoStepCount> lin_mask{};
            std::array<double, AutoStepCount> velocity_scaling{};
            std::array<double, AutoStepCount> acceleration_scaling{};
        };
        static AutoMineConfig auto_storage_lb_config;
        static AutoMineConfig auto_storage_rb_config;
        static AutoMineConfig auto_extract_lb_config;
        static AutoMineConfig auto_extract_rb_config;
        static bool parameter_initialized{false};

        const auto load_auto_mine_config =
            [this, &auto_step_names](const std::string& mine_name, AutoMineConfig& config) {
                for (std::size_t i = 0; i < auto_step_names.size(); ++i) {
                    const auto& step_name = auto_step_names[i];
                    config.lin_mask[i]    = step_name.size() >= 4
                                      && step_name.compare(step_name.size() - 4, 4, "_lin") == 0;

                    const std::string prefix = mine_name + ".params." + step_name + ".";
                    std::vector<double> target_pose_values;

                    node_->get_parameter(prefix + "target_pose", target_pose_values);
                    node_->get_parameter(prefix + "velocity_scaling", config.velocity_scaling[i]);
                    node_->get_parameter(
                        prefix + "acceleration_scaling", config.acceleration_scaling[i]);

                    for (std::size_t j = 0; j < 6; ++j) {
                        config.step_rpy[i][j] = target_pose_values[j];
                    }
                }
            };

        const auto get_auto_mine_config = [&](rmcs_msgs::ArmMode mode) -> AutoMineConfig* {
            switch (mode) {
            case rmcs_msgs::ArmMode::Auto_Storage_LB: return &auto_storage_lb_config;
            case rmcs_msgs::ArmMode::Auto_Storage_RB: return &auto_storage_rb_config;
            case rmcs_msgs::ArmMode::Auto_Extract_LB: return &auto_extract_lb_config;
            case rmcs_msgs::ArmMode::Auto_Extract_RB: return &auto_extract_rb_config;
            default: return nullptr;
            }
        };
        const auto linear_point_transformer = [](const geometry_msgs::msg::Pose& start_pose,
                                                 const Eigen::Vector3d& local_dir,
                                                 double distance) {
            Eigen::Isometry3d T;
            tf2::fromMsg(start_pose, T);
            Eigen::Vector3d p_local = local_dir.normalized() * distance;
            T.translation() += T.linear() * p_local;
            return tf2::toMsg(T);
        };

        const auto auto_between_pose_builder = [this, &get_auto_mine_config]() {
            double b             = node_->get_parameter("b").as_double();
            double k_in          = node_->get_parameter("k_in").as_double();
            double k_out         = node_->get_parameter("k_out").as_double();
            double k_out_reverse = node_->get_parameter("k_out_reverse").as_double();
            double k{0.5};

            auto* config = get_auto_mine_config(this->get_arm_mode());
            if (!config) {
                return std::array<double, 6>{};
            }

            std::array<double, 6> target          = config->step_rpy[1];
            std::array<double, 6> between_pose    = config->step_rpy[0];
            geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose("link_6").pose;

            const Eigen::Vector3d current_xyz(
                current_pose.position.x, current_pose.position.y, current_pose.position.z);
            const Eigen::Vector3d target_xyz(target[0], target[1], target[2]);

            const Eigen::Vector3d delta_xyz = target_xyz - current_xyz;
            switch (this->get_arm_mode()) {
            case rmcs_msgs::ArmMode::Auto_Storage_LB:
            case rmcs_msgs::ArmMode::Auto_Extract_LB: {

                if (delta_xyz.x() > 0 && delta_xyz.x() < b) {
                    k = k_out_reverse;
                } else if (delta_xyz.x() < 0 && delta_xyz.x() > -b) {
                    k = k_out;
                } else {
                    k = k_in;
                }
                between_pose[0] = k * (current_xyz.x()) + (1 - k) * (target_xyz.x());

                if (delta_xyz.y() < 0 && delta_xyz.y() > -b) {
                    k = k_out_reverse;
                } else if (delta_xyz.y() > 0 && delta_xyz.y() < b) {
                    k = k_out;
                } else {
                    k = k_in;
                }
                between_pose[1] = k * (current_xyz.y()) + (1 - k) * (target_xyz.y());

                if (delta_xyz.z() > 0 && delta_xyz.z() < b) {
                    k = k_out_reverse;
                } else if (delta_xyz.z() < 0 && delta_xyz.z() > -b) {
                    k = k_out;
                } else {
                    k = k_in;
                }
                between_pose[2] = k * (current_xyz.z()) + (1 - k) * (target_xyz.z());

                break;
            }
            case rmcs_msgs::ArmMode::Auto_Storage_RB:
            case rmcs_msgs::ArmMode::Auto_Extract_RB: {
                if (delta_xyz.x() > 0 && delta_xyz.x() < b) {
                    k = k_out_reverse;
                } else if (delta_xyz.x() < 0 && delta_xyz.x() > -b) {
                    k = k_out;
                } else {
                    k = k_in;
                }
                between_pose[0] = k * (current_xyz.x()) + (1 - k) * (target_xyz.x());

                if (delta_xyz.y() < 0 && delta_xyz.y() > -b) {
                    k = k_out;
                } else if (delta_xyz.y() > 0 && delta_xyz.y() < b) {
                    k = k_out_reverse;
                } else {
                    k = k_in;
                }
                between_pose[1] = k * (current_xyz.y()) + (1 - k) * (target_xyz.y());

                if (delta_xyz.z() > 0 && delta_xyz.z() <b) {
                    k = k_out_reverse;
                } else if (delta_xyz.z() < 0 && delta_xyz.z() > -b) {
                    k = k_out;
                } else {
                    k = k_in;
                }
                between_pose[2] = k * (current_xyz.z()) + (1 - k) * (target_xyz.z());

                break;
            }
            }

            tf2::Quaternion current_q;
            tf2::fromMsg(current_pose.orientation, current_q);
            tf2::Quaternion target_q;
            target_q.setRPY(target[3], target[4], target[5]);
            tf2::Quaternion q_mid = current_q.slerp(target_q, k_in);
            tf2::Matrix3x3(q_mid).getRPY(between_pose[3], between_pose[4], between_pose[5]);
            return between_pose;
        };

        if (!parameter_initialized) {
            node_->get_parameter("auto_walk_joint_target", auto_walk_joint_target);

            load_auto_mine_config("auto_storage_lb", auto_storage_lb_config);
            load_auto_mine_config("auto_storage_rb", auto_storage_rb_config);
            load_auto_mine_config("auto_extract_lb", auto_extract_lb_config);
            load_auto_mine_config("auto_extract_rb", auto_extract_rb_config);

            parameter_initialized = true;
        }

        const auto request = plan_request.load(std::memory_order_acquire);
        static uint64_t last_planned_request_id_{0};
        if (!request || request->request_id == last_planned_request_id_) {
            return;
        }

        auto result          = std::make_shared<PlannedTrajectory>();
        result->request_id   = request->request_id;
        result->plan_success = false;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        move_group_->clearPathConstraints();
        move_group_->setMaxAccelerationScalingFactor(0.05);
        move_group_->setMaxVelocityScalingFactor(0.03);
        move_group_->setPlanningTime(5.0);
        switch (request->arm_mode) {
        case rmcs_msgs::ArmMode::Auto_Walk: {
            move_group_->setPlanningPipelineId("ompl");
            move_group_->setPlannerId("");
            move_group_->setJointValueTarget(
                std::map<std::string, double>{
                    {"joint_1", auto_walk_joint_target[0]},
                    {"joint_2", auto_walk_joint_target[1]},
                    {"joint_3", auto_walk_joint_target[2]},
                    {"joint_4", auto_walk_joint_target[3]},
                    {"joint_5", auto_walk_joint_target[4]},
                    {"joint_6", auto_walk_joint_target[5]},
            });
            break;
        }
        case rmcs_msgs::ArmMode::Auto_Linear: {
            const static double distance        = 0.1;
            geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
            const auto target_pose = linear_point_transformer(start_pose, {1, 0, 0}, distance);
            move_group_->setGoalOrientationTolerance(0.2);
            move_group_->setGoalPositionTolerance(0.01);
            move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
            move_group_->setPlannerId("LIN");
            move_group_->setPoseTarget(target_pose, "link_6");
            break;
        }
        case rmcs_msgs::ArmMode::Auto_Storage_LB:
        case rmcs_msgs::ArmMode::Auto_Storage_RB:
        case rmcs_msgs::ArmMode::Auto_Extract_LB:
        case rmcs_msgs::ArmMode::Auto_Extract_RB: {
            auto* config = get_auto_mine_config(request->arm_mode);
            if (!config) {
                last_planned_request_id_ = request->request_id;
                planned_trajectory_.store(result, std::memory_order::release);
                return;
            }
            auto current_state = move_group_->getCurrentState();

            result->positions.clear();
            result->steps.clear();
            result->plan_success = true;

            for (std::size_t i = 0; i < auto_step_names.size(); ++i) {
                move_group_->clearPoseTargets();
                move_group_->clearPathConstraints();
                move_group_->setStartState(*current_state);
                move_group_->setPlanningTime(5.0);
                move_group_->setGoalOrientationTolerance(0.2);
                move_group_->setGoalPositionTolerance(0.01);
                move_group_->setMaxVelocityScalingFactor(config->velocity_scaling[i]);
                move_group_->setMaxAccelerationScalingFactor(config->acceleration_scaling[i]);
                if (i == 0) {
                    config->step_rpy[0] = auto_between_pose_builder();
                }
                if (config->lin_mask[i]) {
                    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
                    move_group_->setPlannerId("LIN");
                } else {
                    move_group_->setPlanningPipelineId("ompl");
                    move_group_->setPlannerId("");
                }

                move_group_->setPositionTarget(
                    config->step_rpy[i][0], config->step_rpy[i][1], config->step_rpy[i][2],
                    "link_6");
                move_group_->setRPYTarget(
                    config->step_rpy[i][3], config->step_rpy[i][4], config->step_rpy[i][5],
                    "link_6");

                moveit::planning_interface::MoveGroupInterface::Plan segment_plan;
                if (move_group_->plan(segment_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_WARN(
                        node_->get_logger(), "Auto sequence segment %s plan failed",
                        auto_step_names[i].c_str());
                    result->plan_success = false;
                    break;
                }

                const auto& trajectory_points = segment_plan.trajectory.joint_trajectory.points;
                const std::size_t start_index = (i == 0) ? 0 : 1;
                for (std::size_t j = start_index; j < trajectory_points.size(); ++j) {
                    result->positions.push_back(trajectory_points[j].positions);
                }
                result->steps.push_back(static_cast<int>(result->positions.size()) - 1);
                const auto& joint_names = segment_plan.trajectory.joint_trajectory.joint_names;
                const auto& last_point  = trajectory_points.back();
                auto next_state = std::make_shared<moveit::core::RobotState>(*current_state);
                next_state->setVariablePositions(joint_names, last_point.positions);
                next_state->update();
                current_state = next_state;
            }

            if (!result->plan_success) {
                RCLCPP_WARN(node_->get_logger(), "Auto_Mine composite plan failed");
            } else {
                RCLCPP_INFO(
                    node_->get_logger(), "Auto_Mine stored size:%d",
                    static_cast<int>(result->positions.size()));
            }
            last_planned_request_id_ = request->request_id;
            planned_trajectory_.store(result, std::memory_order::release);
            return;
        }
        default: {
            last_planned_request_id_ = request->request_id;
            planned_trajectory_.store(result, std::memory_order::release);
            return;
        }
        }
        result->plan_success =
            (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!(result->plan_success)) {
            RCLCPP_WARN(node_->get_logger(), "plan failed");
        } else {
            const auto& trajectory_points = my_plan.trajectory.joint_trajectory.points;
            result->positions.reserve(trajectory_points.size());
            for (const auto& pt : trajectory_points) {
                result->positions.push_back(pt.positions);
            }
            RCLCPP_INFO(
                node_->get_logger(), "plan stored size:%d",
                static_cast<int>(trajectory_points.size()));
        }

        last_planned_request_id_ = request->request_id;
        planned_trajectory_.store(result, std::memory_order::release);
    }
    void execute_plan_request_and_trajectory_step() {
        static std::size_t trajectory_steps{0};
        static uint64_t last_executed_request_id{0};
        const auto current_request = plan_request.load(std::memory_order_acquire);
        if (!current_request) {
            return;
        }
        if (current_request->request_id != last_executed_request_id) {
            trajectory_steps         = 0;
            last_executed_request_id = current_request->request_id;
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
                    for (std::size_t i = 0; i < q.size(); ++i) {
                        *target_theta[i] = q[i];
                    }
                    switch (get_arm_mode()) {
                    case rmcs_msgs::ArmMode::Auto_Extract_LB:
                    case rmcs_msgs::ArmMode::Auto_Extract_RB: {
                        if (trajectory_steps == moveit_result->steps[2] && !is_gripper_complete) {
                            set_gripper_mode(rmcs_msgs::GripperMode::Close);
                            trajectory_steps--;
                        }
                        break;
                    }
                    case rmcs_msgs::ArmMode::Auto_Storage_RB:
                    case rmcs_msgs::ArmMode::Auto_Storage_LB: {
                        if (trajectory_steps == moveit_result->steps[2] && !is_gripper_complete) {
                            set_gripper_mode(rmcs_msgs::GripperMode::Open);
                            trajectory_steps--;
                        }
                        break;
                    }
                    default: break;
                    }
                    trajectory_steps++;
                }
            }
        }
    }
    void execute_custom() {
        struct __attribute__((packed)) CustomFrame {
            uint16_t big_yaw;
            uint16_t joint[6];
            uint16_t gripper;
        };
        const auto& custom_data = *custom_data_;
        std::array<std::uint8_t, sizeof(CustomFrame)> raw{};
        std::copy_n(custom_data.begin(), raw.size(), raw.begin());
        const auto frame  = std::bit_cast<CustomFrame>(raw);
        constexpr auto pi = std::numbers::pi;

        const auto raw_to_angle = [](std::uint16_t raw, double divisor) {
            auto angle = static_cast<double>(raw) / divisor * 2.0 * pi;
            if (angle > pi) {
                angle -= 2.0 * pi;
            }
            return angle;
        };
        Eigen::Vector<double, 6> angles;
        for (Eigen::Index i = 0; i < angles.size(); ++i) {
            const auto joint_index = static_cast<std::size_t>(i);
            double divisor         = (joint_index == 3 || joint_index == 5) ? 32768.0 : 65536.0;
            angles[i]              = raw_to_angle(frame.joint[joint_index], divisor);
            if (i == 2 || i == 5) {
                angles[i] = -angles[i];
            }
        }
        const auto filtered_angles = custom_joint_filter_.update(angles);
        for (Eigen::Index i = 0; i < filtered_angles.size(); ++i) {
            *target_theta[static_cast<std::size_t>(i)] = filtered_angles[i];
        }

        *custom_big_yaw_output_ = raw_to_angle(frame.big_yaw, 65536.0);
    }
    void execute_dt7_orientation() {
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[5] += 0.003 * joystick_left_->y();
            *target_theta[5] =
                std::clamp(*target_theta[5], *joint_lower_limit_[5], *joint_upper_limit_[5]);
        }
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[4] += 0.003 * joystick_left_->x();
            *target_theta[4] =
                std::clamp(*target_theta[4], *joint_lower_limit_[4], *joint_upper_limit_[4]);
        }
        if (fabs(joystick_right_->y()) > 0.01) {
            *target_theta[3] += 0.003 * joystick_right_->y();
            *target_theta[3] =
                std::clamp(*target_theta[3], *joint_lower_limit_[3], *joint_upper_limit_[3]);
        }
    }
    void execute_dt7_position() {
        if (fabs(joystick_left_->x()) > 0.01) {
            *target_theta[2] += 0.001 * joystick_left_->x();
            *target_theta[2] =
                std::clamp(*target_theta[2], *joint_lower_limit_[2], *joint_upper_limit_[2]);
        }
        if (fabs(joystick_right_->x()) > 0.01) {
            *target_theta[1] += 0.001 * joystick_right_->x();
            *target_theta[1] =
                std::clamp(*target_theta[1], *joint_lower_limit_[1], *joint_upper_limit_[1]);
        }
        if (fabs(joystick_left_->y()) > 0.01) {
            *target_theta[0] += 0.001 * joystick_left_->y();
            *target_theta[0] =
                std::clamp(*target_theta[0], *joint_lower_limit_[0], *joint_upper_limit_[0]);
        }
    }
    void gripper_control() {
        auto unwrap = [](double angle) -> double { return angle < 0 ? angle + 2.0 * M_PI : angle; };

        switch (get_gripper_mode()) {
        case rmcs_msgs::GripperMode::Open: {
            if (*gripper_velocity_ < 0.05 && *gripper_velocity_ > 0 && *gripper_torque_ > 30) {
                *target_theta[6] = *theta[6];
                set_gripper_mode(rmcs_msgs::GripperMode::None);
                is_gripper_complete = true;
            } else {
                is_gripper_complete = false;
                *target_theta[6]    = *theta[6] + 0.05;
            }
            break;
        }
        case rmcs_msgs::GripperMode::Close: {
            if (*gripper_velocity_ < 0 && *gripper_velocity_ > -0.05 && *gripper_torque_ < -30) {
                *target_theta[6] = *theta[6];
                set_gripper_mode(rmcs_msgs::GripperMode::None);
                is_gripper_complete = true;
            } else {
                is_gripper_complete = false;
                *target_theta[6]    = *theta[6] - 0.05;
            }

            break;
        }

        default: {
            is_gripper_complete = false;
        } break;
        }

        *gripper_target_theta_error = unwrap(*target_theta[6]) - unwrap(*theta[6]);
    }
    // bool gripper_judgement() {
    //     if (*gripper_velocity_ < 0.05 && *gripper_torque_ > 30)
    //         return true;
    //     else
    //         return false;
    // }
    void image_pitch_control() {

        double qmin = -3;
        double qmax = 3;
        node_->get_parameter("image_pitch_qmin", qmin);
        node_->get_parameter("image_pitch_qmax", qmax);
        double target_theta        = *theta[1] + image_pitch_theta1_offset_;
        target_theta               = std::clamp(target_theta, qmin, qmax);
        *image_pitch_target_theta_ = target_theta;
    }
    void reset() {

        *is_arm_enable = false;
        for (std::size_t i = 0; i < std::size(theta); ++i) {
            *target_theta[i] = *theta[i];
        }
        *gripper_target_theta_error = NAN;
        *image_pitch_target_theta_  = NAN;
        image_pitch_theta1_offset_  = 0.0;
        last_switch_left_           = *switch_left_;
        last_switch_right_          = *switch_right_;
        last_rotary_knob_switch_    = *rotary_knob_switch;
        last_keyboard_              = *keyboard_;
        last_mouse_                 = *mouse_;
        custom_joint_filter_.reset();
        set_gripper_mode(rmcs_msgs::GripperMode::None);
        set_arm_mode(rmcs_msgs::ArmMode::None);
    }

    rmcs_msgs::Switch last_switch_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_switch_right_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch last_rotary_knob_switch_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Keyboard last_keyboard_{rmcs_msgs::Keyboard::zero()};
    rmcs_msgs::Mouse last_mouse_{rmcs_msgs::Mouse::zero()};
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> rotary_knob_switch;
    InputInterface<Eigen::Vector2d> mouse_velocity_;
    InputInterface<rmcs_msgs::Mouse> mouse_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;

    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[7];
    InputInterface<double> gripper_velocity_;
    InputInterface<double> gripper_torque_;
    bool is_gripper_complete{false};
    std::array<InputInterface<double>, 6> joint_lower_limit_;
    std::array<InputInterface<double>, 6> joint_upper_limit_;
    OutputInterface<double> target_theta[7];
    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;

    rmcs_msgs::GripperMode gripper_mode_{rmcs_msgs::GripperMode::None};
    OutputInterface<double> gripper_target_theta_error;

    InputInterface<double> image_pitch_theta_;
    double image_pitch_theta1_offset_{0.0};
    OutputInterface<double> image_pitch_target_theta_;

    InputInterface<std::array<uint8_t, 30>> custom_data_;
    OutputInterface<double> custom_big_yaw_output_;
    filter::LowPassFilter<6> custom_joint_filter_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor exec_;
    std::thread spin_thread_;
    void spin_loop() { exec_.spin(); }
};

} // namespace rmcs_core::controller::arm

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::arm::ArmController, rmcs_executor::Component)
