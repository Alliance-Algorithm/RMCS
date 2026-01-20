#include "hardware/device/Kinematic.hpp"
#include "hardware/device/drag_teach.hpp"
#include "hardware/device/trajectory.hpp"
#include "hardware/endian_promise.hpp"
#include "hardware/fsm/FSM.hpp"
#include "hardware/fsm/FSM_gold_l.hpp"
#include "hardware/fsm/FSM_gold_m.hpp"
#include "hardware/fsm/FSM_gold_r.hpp"
#include "hardware/fsm/FSM_sliver.hpp"
#include "hardware/fsm/FSM_storage_lb.hpp"
#include "hardware/fsm/FSM_storage_rb.hpp"
#include "hardware/fsm/FSM_up_stairs.hpp"
#include "hardware/fsm/FSM_walk.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Core/util/Meta.h>
#include <fstream>
#include <numbers>
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

namespace rmcs_core::hardware::arm {
class ArmController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ArmController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        // register_input("/remote/joystick/right", joystick_right_);
        // register_input("/remote/joystick/left", joystick_left_);
        // register_input("/remote/switch/right", switch_right_);
        // register_input("/remote/switch/left", switch_left_);
        // register_input("/remote/mouse/velocity", mouse_velocity_);
        // register_input("/remote/mouse", mouse_);
        // register_input("/remote/keyboard", keyboard_);

        // register_input("/referee/vision/custom", custom_controller);

        register_input("/arm/Joint6/theta", theta[5]);
        register_input("/arm/Joint5/theta", theta[4]);
        register_input("/arm/Joint4/theta", theta[3]);
        register_input("/arm/Joint3/theta", theta[2]);
        register_input("/arm/Joint2/theta", theta[1]);
        register_input("/arm/Joint1/theta", theta[0]);
        register_output("/arm/Joint6/target_theta", target_theta[5], nan);
        register_output("/arm/Joint5/target_theta", target_theta[4], nan);
        register_output("/arm/Joint4/target_theta", target_theta[3], nan);
        register_output("/arm/Joint3/target_theta", target_theta[2], nan);
        register_output("/arm/Joint2/target_theta", target_theta[1], nan);
        register_output("/arm/Joint1/target_theta", target_theta[0], nan);

        register_output("/arm/Joint6/control_torque", control_torque[5], nan);
        register_output("/arm/Joint5/control_torque", control_torque[4], nan);
        register_output("/arm/Joint4/control_torque", control_torque[3], nan);
        register_output("/arm/Joint3/control_torque", control_torque[2], nan);
        register_output("/arm/Joint2/control_torque", control_torque[1], nan);
        register_output("/arm/Joint1/control_torque", control_torque[0], nan);

        register_input("/arm/Joint1/raw_angle", joint1_raw_angle);
        register_output("/arm/Joint1/zero_point", joint1_zero_point, 14381);

        register_output("/arm/enable_flag", is_arm_enable, true);
        register_output("/arm/pump/target_vel", arm_pump_target_vel, NAN);
        register_output("/mine/pump/target_vel", mine_pump_target_vel, NAN);
        register_output("/arm/pump/relay/CH", arm_pump_relay, 0b00000000);
        publisher_ =
            create_publisher<std_msgs::msg::Float32MultiArray>("/engineer/joint/measure", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/engineer/joint/control", 10,
            [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                if (msg->data.size() == 6) {
                    *target_theta[0] = static_cast<double>(msg->data[0]);
                    *target_theta[1] = -static_cast<double>(msg->data[1]);
                    *target_theta[2] = -static_cast<double>(msg->data[2]) + std::numbers::pi / 2;
                    *target_theta[3] = static_cast<double>(msg->data[3]);
                    *target_theta[4] = static_cast<double>(msg->data[4]);
                    *target_theta[5] = static_cast<double>(msg->data[5]);
                }
            });
        *arm_mode = rmcs_msgs::ArmMode::None;
        register_output("/arm/mode", arm_mode, rmcs_msgs::ArmMode::None);
    }
    void update() override {

    }

private:

    // auto_mode_Fsm
    Auto_Gold_Left fsm_gold_l;
    Auto_Gold_Mid fsm_gold_m;
    Auto_Gold_Right fsm_gold_r;
    Auto_Sliver fsm_sliver;
    Auto_Set_Walk_Arm fsm_walk;
    Auto_Up_Stairs fsm_up_stairs;
    Auto_Storage_LB fsm_storage_lb;
    Auto_Storage_RB fsm_storage_rb;

    OutputInterface<rmcs_msgs::ArmMode> arm_mode;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    static constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    // InputInterface<std::array<uint8_t, 30>> custom_controller;

    // InputInterface<Eigen::Vector2d> joystick_right_;
    // InputInterface<Eigen::Vector2d> joystick_left_;
    // InputInterface<rmcs_msgs::Switch> switch_right_;
    // InputInterface<rmcs_msgs::Switch> switch_left_;
    // InputInterface<Eigen::Vector2d> mouse_velocity_;
    // InputInterface<rmcs_msgs::Mouse> mouse_;
    // InputInterface<rmcs_msgs::Keyboard> keyboard_;
    OutputInterface<bool> is_arm_enable;
    InputInterface<double> theta[6]; // motor_current_angle
    OutputInterface<double> target_theta[6];
    OutputInterface<double> control_torque[6];

    InputInterface<int> joint1_raw_angle;
    OutputInterface<double> joint1_zero_point;

    OutputInterface<double> arm_pump_target_vel;
    OutputInterface<double> mine_pump_target_vel;
    OutputInterface<uint8_t> arm_pump_relay;
};
} // namespace rmcs_core::controller::arm
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::arm::ArmController, rmcs_executor::Component)