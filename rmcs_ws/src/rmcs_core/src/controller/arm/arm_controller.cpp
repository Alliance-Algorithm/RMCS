#include "filter/low_pass_filter.hpp"
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
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <string>

#include <atomic>
#include <chrono>
#include <mutex>
#include <sys/cdefs.h>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
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
        register_output("/arm/gripper/target_theta_error", gripper_target_theta_error, NAN);
        register_input("/arm/image_pitch/motor/angle", image_pitch_theta_, NAN);
        register_output("/arm/image_pitch/target_theta", image_pitch_target_theta_, NAN);
        register_output("/arm/custom_big_yaw", custom_big_yaw_output_, 0.0);

        move_group_ =
            std::make_unique<moveit::planning_interface::MoveGroupInterface>(node_, "alliance_arm");
        move_group_->startStateMonitor();
        moveit_running_.store(true, std::memory_order_release);
        moveit_thread_ = std::thread([this] {
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

        if (knob == Switch::UP) {
            set_gripper_mode(rmcs_msgs::GripperMode::Open);
        } else if (knob == Switch::DOWN) {
            set_gripper_mode(rmcs_msgs::GripperMode::Close);
        }

        if (switch_left == Switch::UP && switch_right == Switch::UP) {
            set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Position);
        } else if (switch_left == Switch::UP && switch_right == Switch::MIDDLE) {
            set_arm_mode(rmcs_msgs::ArmMode::DT7_Control_Orientation);
        } else if (switch_left == Switch::DOWN && switch_right == Switch::UP) {
            if (keyboard.g) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Walk);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Down_Stairs);
                }
            }
            if (keyboard.b) {
                if (!keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_Two_Stairs);
                } else if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Up_One_Stairs);
                }
            }
            if (keyboard.f) {
                if (keyboard.shift && !keyboard.ctrl) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_LB);
                } else if (keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Auto_Storage_RB);
                }
            }
            if (keyboard.s) {
                set_arm_mode(rmcs_msgs::ArmMode::Auto_Spin);
            }
            if (keyboard.r) {
                if (!keyboard.ctrl && !keyboard.shift) {
                    set_arm_mode(rmcs_msgs::ArmMode::Custome);
                }
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
        case ArmMode::Auto_Walk: {
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

        gripper_control();
        image_pitch_control();
        last_mouse_ = mouse;
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
        if (mode != rmcs_msgs::ArmMode::Auto_Walk) {
            last_requested_arm_mode_ = mode;
        }
        const auto current = plan_request.load(std::memory_order_acquire);
        auto next          = std::make_shared<PlanRequest>();
        if (current) {
            next->request_id = current->request_id;
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
        static bool moveit_parameter_initialized{false};

        if (!moveit_parameter_initialized) {
            node_->get_parameter("auto_walk_joint_target", auto_walk_joint_target);
            moveit_parameter_initialized = true;
        }

        const auto request = plan_request.load(std::memory_order_acquire);
        static uint64_t last_planned_request_id_{0};
        if (!request || request->request_id == last_planned_request_id_) {
            return;
        }
        last_planned_request_id_ = request->request_id;

        auto result          = std::make_shared<PlannedTrajectory>();
        result->request_id   = request->request_id;
        result->plan_success = false;

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        move_group_->setStartStateToCurrentState();
        move_group_->clearPoseTargets();
        move_group_->setMaxVelocityScalingFactor(0.005);
        move_group_->setMaxAccelerationScalingFactor(0.005);
        move_group_->setPlanningTime(2.0);
        switch (request->arm_mode) {
        case rmcs_msgs::ArmMode::Auto_Walk: {
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
        default: {
            break;
        }
        }
        const bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        result->plan_success = success;
        if (!success) {
            RCLCPP_WARN(node_->get_logger(), "plan failed");
        } else {
            const auto& trajectory_points = my_plan.trajectory.joint_trajectory.points;
            result->positions.reserve(trajectory_points.size());
            for (const auto& pt : trajectory_points) {
                result->positions.push_back(pt.positions);
            }
        }

        planned_trajectory_.store(result, std::memory_order::release);
    }
    void execute_plan_request_and_trajectory_step() {
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
                    for (std::size_t i = 0; i < std::min<std::size_t>(6, q.size()); ++i) {
                        // *target_theta[i] = q[i];
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

        constexpr double closed_angle = 0.0;
        constexpr double open_angle   = -2.1;

        switch (get_gripper_mode()) {
        case rmcs_msgs::GripperMode::Open: *target_theta[6] = open_angle; break;
        case rmcs_msgs::GripperMode::Close: *target_theta[6] = closed_angle; break;
        case rmcs_msgs::GripperMode::Custom:
        case rmcs_msgs::GripperMode::None: break;
        }

        *gripper_target_theta_error = unwrap(*target_theta[6]) - unwrap(*theta[6]);
    }

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
        last_mouse_                 = *mouse_;
        custom_joint_filter_.reset();
        set_gripper_mode(rmcs_msgs::GripperMode::None);
        set_arm_mode(rmcs_msgs::ArmMode::None);
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
    InputInterface<double> theta[7];
    std::array<InputInterface<double>, 6> joint_lower_limit_;
    std::array<InputInterface<double>, 6> joint_upper_limit_;
    OutputInterface<double> target_theta[7];
    OutputInterface<rmcs_msgs::ArmMode> arm_mode_;

    rmcs_msgs::GripperMode gripper_mode_{rmcs_msgs::GripperMode::None};
    OutputInterface<double> gripper_target_theta_error;

    InputInterface<double> image_pitch_theta_;
    double image_pitch_theta1_offset_{0.0};
    rmcs_msgs::Mouse last_mouse_{rmcs_msgs::Mouse::zero()};
    OutputInterface<double> image_pitch_target_theta_;

    InputInterface<std::array<uint8_t, 30>> custom_data_;
    OutputInterface<double> custom_big_yaw_output_;
    filter::LowPassFilter<6> custom_joint_filter_;

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
